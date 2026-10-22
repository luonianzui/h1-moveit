#include "h1_2_moveit_demo/door_handoff_path_certificate.hpp"

#include "h1_2_moveit_demo/door_handoff_binding_fields.hpp"

#include <rcutils/sha256.h>

#include <dirent.h>
#include <fcntl.h>
#include <linux/fs.h>
#include <limits.h>
#include <sys/random.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace h1_2_moveit_demo::door_handoff_path_certificate
{
namespace
{

constexpr char kManifestMagic[] = "H1_DOOR_HANDOFF_PATH_CERTIFICATE_V1";
constexpr char kManifestFilename[] = "manifest.txt";
constexpr char kPayloadFilename[] = "states.bin";
constexpr std::array<std::uint8_t, 8> kPayloadMagic{
  'H', '1', 'D', 'H', 'P', 'T', 'H', '1'};
constexpr std::uint32_t kSchemaVersion = 1U;
constexpr std::uint32_t kEndianTag = 0x01020304U;
constexpr std::size_t kPayloadHeaderBytes = 44U;
constexpr std::size_t kMaximumManifestBytes = 64U * 1024U;
constexpr std::size_t kMaximumPayloadBytes = 1024U * 1024U;
constexpr double kPi = 3.14159265358979323846;
constexpr double kIngressContinuityLimit = 4.9 * kPi / 180.0;
constexpr double kCoarseContinuityLimit = 4.9 * kPi / 180.0;
constexpr double kExactContinuityLimit = 0.49 * kPi / 180.0;
constexpr double kContinuityRoundoff = 1e-12;
constexpr double kExactIdentityTolerance = 1e-10;
constexpr char kOriginalHandleSha256[] =
  "ad6670185fa5756ba5dabdee7e542db2df3346973fea4eb887288f54a6b6f59e";
static_assert(
  sizeof(double) == 8U && std::numeric_limits<double>::is_iec559,
  "door handoff certificates require IEEE-754 binary64");

const std::map<std::string, std::string> kExactManifestFields{
  {"schema_version", "1"},
  {"byte_order", "little"},
  {"scalar_format", "ieee754-binary64"},
  {"joint_count", "7"},
  {"outer_anchor_state_count", "1"},
  {"ingress_state_count", "45"},
  {"coarse_state_count", "73"},
  {"exact_state_count", "613"},
  {"execution_moving_target_count", "601"},
  {"dense_interpolation_sample_count", "2943"},
  {"joint_names",
    "right_shoulder_pitch_joint,right_shoulder_roll_joint,"
    "right_shoulder_yaw_joint,right_elbow_joint,right_wrist_roll_joint,"
    "right_wrist_pitch_joint,right_wrist_yaw_joint"},
  {"ingress_continuity_limit_degrees", "4.9"},
  {"coarse_continuity_limit_degrees", "4.9"},
  {"exact_continuity_limit_degrees", "0.49"},
  {"environment_clearance_m", "0.010"},
  {"cartesian_minimum_fraction", "0.995"},
  {"partial_allowed", "false"},
  {"full_reverse_required", "true"},
  {"original_stl_contact_required", "true"},
  {"timing_cached", "false"},
  {"door_start_degrees", "0"},
  {"door_end_degrees", "60"},
  {"handle_start_degrees", "90"},
  {"handle_local_z_mm", "35"},
  {"handle_phase_degrees", "21"},
  {"palm_down_tilt_degrees", "7"},
  {"handle_local_x_mm", "0"},
  {"outward_offset_mm", "16"},
  {"clamp_q1_mm", "11.35"},
  {"clamp_q2_mm", "12.35"},
  {"pregrasp_q1_mm", "10.35"},
  {"pregrasp_q2_mm", "11.35"},
  {"pad_station_limit_mm", "26"},
  {"nonpad_clearance_limit_mm", "0.6"},
  {"dual_hand_clearance_limit_mm", "5"},
};

const std::set<std::string> kHashManifestFields = []() {
    std::set<std::string> fields;
    for (const auto & field : kBindingFieldRegistry) {
      fields.emplace(field.name);
    }
    fields.emplace("payload_sha256");
    return fields;
  }();

class UniqueFd
{
public:
  explicit UniqueFd(const int descriptor = -1)
  : descriptor_(descriptor) {}

  ~UniqueFd()
  {
    if (descriptor_ >= 0) {
      ::close(descriptor_);
    }
  }

  UniqueFd(const UniqueFd &) = delete;
  UniqueFd & operator=(const UniqueFd &) = delete;

  UniqueFd(UniqueFd && other) noexcept
  : descriptor_(other.release()) {}

  UniqueFd & operator=(UniqueFd && other) noexcept
  {
    if (this != &other) {
      reset(other.release());
    }
    return *this;
  }

  int get() const {return descriptor_;}

  int release()
  {
    const int descriptor = descriptor_;
    descriptor_ = -1;
    return descriptor;
  }

  void reset(const int descriptor = -1)
  {
    if (descriptor_ >= 0) {
      ::close(descriptor_);
    }
    descriptor_ = descriptor;
  }

  explicit operator bool() const {return descriptor_ >= 0;}

private:
  int descriptor_{-1};
};

bool valid_path_component(const std::string & component)
{
  return !component.empty() && component != "." && component != ".." &&
         component.size() <= NAME_MAX &&
         component.find('/') == std::string::npos &&
         component.find('\0') == std::string::npos;
}

int open_absolute_directory_no_symlinks(
  const std::string & path,
  std::string * error)
{
  if (!error || path.empty() || path.front() != '/' ||
    path.find('\0') != std::string::npos ||
    (path.size() > 1U && path.back() == '/'))
  {
    if (error) {
      *error = "directory path is not canonical and absolute";
    }
    return -1;
  }
  UniqueFd current(::open("/", O_RDONLY | O_DIRECTORY | O_CLOEXEC));
  if (!current) {
    *error = "cannot open filesystem root: " + std::string(std::strerror(errno));
    return -1;
  }
  if (path == "/") {
    return current.release();
  }

  std::size_t begin = 1U;
  while (begin < path.size()) {
    const std::size_t slash = path.find('/', begin);
    const std::size_t end = slash == std::string::npos ? path.size() : slash;
    const std::string component = path.substr(begin, end - begin);
    if (!valid_path_component(component)) {
      *error = "directory path contains an unsafe component";
      return -1;
    }
    const int next = ::openat(
      current.get(), component.c_str(),
      O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    if (next < 0) {
      *error = "cannot safely open directory component '" + component +
        "': " + std::strerror(errno);
      return -1;
    }
    current.reset(next);
    if (slash == std::string::npos) {
      break;
    }
    begin = slash + 1U;
  }
  return current.release();
}

bool split_absolute_target(
  const std::string & path,
  std::string * parent,
  std::string * leaf,
  std::string * error)
{
  if (!parent || !leaf || !error || path.empty() || path.front() != '/' ||
    path.back() == '/' || path.find('\0') != std::string::npos)
  {
    if (error) {
      *error = "target path is not canonical and absolute";
    }
    return false;
  }
  const std::size_t slash = path.rfind('/');
  if (slash == std::string::npos) {
    *error = "target path has no absolute parent";
    return false;
  }
  *parent = slash == 0U ? "/" : path.substr(0U, slash);
  *leaf = path.substr(slash + 1U);
  if (!valid_path_component(*leaf)) {
    *error = "target leaf name is unsafe";
    return false;
  }
  return true;
}

bool random_hex_suffix(std::string * suffix, std::string * error)
{
  if (!suffix || !error) {
    return false;
  }
  std::array<std::uint8_t, 16U> random_bytes{};
  std::size_t offset = 0U;
  while (offset < random_bytes.size()) {
    const ssize_t count = ::getrandom(
      random_bytes.data() + offset, random_bytes.size() - offset, 0U);
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count <= 0) {
      *error = "cannot obtain a strong random temporary-directory suffix";
      if (count < 0) {
        *error += ": " + std::string(std::strerror(errno));
      }
      return false;
    }
    offset += static_cast<std::size_t>(count);
  }
  std::ostringstream hexadecimal;
  hexadecimal << std::hex << std::setfill('0');
  for (const std::uint8_t byte : random_bytes) {
    hexadecimal << std::setw(2) << static_cast<unsigned int>(byte);
  }
  *suffix = hexadecimal.str();
  return true;
}

VerificationResult failure(const std::string & message)
{
  VerificationResult result;
  result.message = message;
  return result;
}

bool is_lower_hex_sha256(const std::string & value)
{
  if (value.size() != 64U) {
    return false;
  }
  bool any_nonzero = false;
  for (const char character : value) {
    const bool accepted =
      (character >= '0' && character <= '9') ||
      (character >= 'a' && character <= 'f');
    if (!accepted) {
      return false;
    }
    any_nonzero = any_nonzero || character != '0';
  }
  return any_nonzero;
}

bool parse_unsigned(const std::string & text, std::size_t * value)
{
  if (!value || text.empty()) {
    return false;
  }
  std::size_t parsed = 0U;
  for (const char character : text) {
    if (character < '0' || character > '9') {
      return false;
    }
    const std::size_t digit = static_cast<std::size_t>(character - '0');
    if (parsed > (std::numeric_limits<std::size_t>::max() - digit) / 10U) {
      return false;
    }
    parsed = parsed * 10U + digit;
  }
  *value = parsed;
  return true;
}

VerificationResult validate_expected_bindings_shape(
  const ExpectedBindings & expected_bindings)
{
  const std::size_t expected_count = kHashManifestFields.size() - 1U;
  if (expected_bindings.sha256_by_field.size() != expected_count) {
    return failure("expected binding field count is invalid");
  }
  for (const auto & key : kHashManifestFields) {
    if (key == "payload_sha256") {
      continue;
    }
    const auto binding = expected_bindings.sha256_by_field.find(key);
    if (binding == expected_bindings.sha256_by_field.end()) {
      return failure("expected binding is missing: " + key);
    }
    if (!is_lower_hex_sha256(binding->second)) {
      return failure("expected binding is non-canonical: " + key);
    }
  }
  for (const auto & [key, unused] : expected_bindings.sha256_by_field) {
    (void)unused;
    if (key == "payload_sha256" ||
      kHashManifestFields.find(key) == kHashManifestFields.end())
    {
      return failure("expected binding field is unknown: " + key);
    }
  }
  if (expected_bindings.sha256_by_field.at("original_handle_stl_sha256") !=
    kOriginalHandleSha256)
  {
    return failure(
      "expected binding mismatch: original_handle_stl_sha256 does not "
      "select the original handle STL");
  }
  VerificationResult result;
  result.passed = true;
  result.expected_bindings_checked = true;
  result.message = "expected binding shape passed";
  return result;
}

void append_u32(
  std::vector<std::uint8_t> * bytes,
  const std::uint32_t value)
{
  for (std::size_t byte = 0U; byte < 4U; ++byte) {
    bytes->push_back(
      static_cast<std::uint8_t>(value >> (8U * byte)));
  }
}

void append_double(
  std::vector<std::uint8_t> * bytes,
  const double value)
{
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  for (std::size_t byte = 0U; byte < 8U; ++byte) {
    bytes->push_back(
      static_cast<std::uint8_t>(bits >> (8U * byte)));
  }
}

bool encode_payload(
  const PathCertificate & certificate,
  std::vector<std::uint8_t> * payload,
  std::string * error)
{
  if (!payload || !error) {
    return false;
  }
  if (certificate.selected_anchor_index > kRedundancyPrepLayerCount ||
    certificate.ingress.size() != kIngressStateCount ||
    certificate.coarse.size() != kCoarseStateCount ||
    certificate.exact.size() != kExactStateCount)
  {
    *error = "certificate state counts or selected anchor are invalid";
    return false;
  }
  constexpr std::size_t kStateCount =
    kOuterAnchorStateCount + kIngressStateCount + kCoarseStateCount +
    kExactStateCount;
  constexpr std::size_t kExpectedBytes =
    kPayloadHeaderBytes + kStateCount * kJointCount * sizeof(double);
  payload->clear();
  payload->reserve(kExpectedBytes);
  payload->insert(payload->end(), kPayloadMagic.begin(), kPayloadMagic.end());
  append_u32(payload, kSchemaVersion);
  append_u32(payload, kEndianTag);
  append_u32(payload, sizeof(double));
  append_u32(payload, kJointCount);
  append_u32(payload, kOuterAnchorStateCount);
  append_u32(payload, kIngressStateCount);
  append_u32(payload, kCoarseStateCount);
  append_u32(payload, kExactStateCount);
  append_u32(
    payload, static_cast<std::uint32_t>(certificate.selected_anchor_index));
  const auto append_state = [&payload](const ArmState & state) {
      for (const double value : state) {
        append_double(payload, value);
      }
    };
  append_state(certificate.outer_anchor);
  for (const auto & state : certificate.ingress) {
    append_state(state);
  }
  for (const auto & state : certificate.coarse) {
    append_state(state);
  }
  for (const auto & state : certificate.exact) {
    append_state(state);
  }
  if (payload->size() != kExpectedBytes) {
    *error = "encoded payload size invariant failed";
    return false;
  }
  return true;
}

bool encode_manifest(
  const PathCertificate & certificate,
  const ExpectedBindings & expected_bindings,
  const std::string & payload_sha256,
  std::string * manifest,
  std::string * error)
{
  if (!manifest || !error) {
    return false;
  }
  const auto bindings = validate_expected_bindings_shape(expected_bindings);
  if (!bindings.passed) {
    *error = bindings.message;
    return false;
  }
  if (!is_lower_hex_sha256(payload_sha256)) {
    *error = "encoded payload SHA-256 is invalid";
    return false;
  }
  std::ostringstream stream;
  stream << kManifestMagic << '\n';
  for (const auto & [key, value] : kExactManifestFields) {
    stream << key << '=' << value << '\n';
  }
  stream << "selected_anchor_index=" << certificate.selected_anchor_index << '\n';
  for (const auto & key : kHashManifestFields) {
    if (key == "payload_sha256") {
      continue;
    }
    stream << key << '=' << expected_bindings.sha256_by_field.at(key) << '\n';
  }
  stream << "payload_sha256=" << payload_sha256 << '\n';
  *manifest = stream.str();
  if (manifest->empty() || manifest->size() > kMaximumManifestBytes) {
    *error = "encoded manifest size is invalid";
    return false;
  }
  return true;
}

VerificationResult parse_manifest(
  const std::string & manifest,
  std::map<std::string, std::string> * fields,
  std::size_t * selected_anchor_index)
{
  if (!fields || !selected_anchor_index) {
    return failure("internal manifest output is null");
  }
  if (manifest.empty() || manifest.size() > kMaximumManifestBytes) {
    return failure("manifest size is invalid");
  }
  for (const unsigned char character : manifest) {
    if (character == '\n' || character == '\r') {
      continue;
    }
    if (character < 0x20U || character > 0x7eU) {
      return failure("manifest contains a non-canonical byte");
    }
  }

  std::istringstream stream(manifest);
  std::string line;
  if (!std::getline(stream, line)) {
    return failure("manifest is empty");
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  if (line != kManifestMagic) {
    return failure("manifest magic is invalid");
  }

  fields->clear();
  std::size_t line_number = 1U;
  while (std::getline(stream, line)) {
    ++line_number;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      return failure("manifest contains an empty line");
    }
    if (line.find_first_of(" \t") != std::string::npos) {
      return failure("manifest contains non-canonical whitespace");
    }
    const std::size_t equals = line.find('=');
    if (equals == std::string::npos || equals == 0U ||
      equals + 1U >= line.size() || line.find('=', equals + 1U) !=
      std::string::npos)
    {
      return failure(
        "manifest line " + std::to_string(line_number) + " is malformed");
    }
    const std::string key = line.substr(0U, equals);
    const std::string value = line.substr(equals + 1U);
    if (!fields->emplace(key, value).second) {
      return failure("manifest field is duplicated: " + key);
    }
  }

  const std::size_t expected_field_count =
    kExactManifestFields.size() + kHashManifestFields.size() + 1U;
  if (fields->size() != expected_field_count) {
    return failure("manifest field count is invalid");
  }
  for (const auto & [key, expected] : kExactManifestFields) {
    const auto found = fields->find(key);
    if (found == fields->end()) {
      return failure("manifest field is missing: " + key);
    }
    if (found->second != expected) {
      return failure("manifest contract mismatch: " + key);
    }
  }
  for (const auto & key : kHashManifestFields) {
    const auto found = fields->find(key);
    if (found == fields->end()) {
      return failure("manifest hash field is missing: " + key);
    }
    if (!is_lower_hex_sha256(found->second)) {
      return failure("manifest hash is non-canonical: " + key);
    }
  }
  if (fields->at("original_handle_stl_sha256") != kOriginalHandleSha256) {
    return failure("manifest does not bind the verified original handle STL");
  }

  const auto anchor = fields->find("selected_anchor_index");
  if (anchor == fields->end() ||
    !parse_unsigned(anchor->second, selected_anchor_index) ||
    *selected_anchor_index > kRedundancyPrepLayerCount)
  {
    return failure("manifest selected anchor index is invalid");
  }

  for (const auto & [key, unused] : *fields) {
    (void)unused;
    if (key != "selected_anchor_index" &&
      kExactManifestFields.find(key) == kExactManifestFields.end() &&
      kHashManifestFields.find(key) == kHashManifestFields.end())
    {
      return failure("manifest field is unknown: " + key);
    }
  }

  VerificationResult result;
  result.passed = true;
  result.message = "manifest contract passed";
  return result;
}

bool read_u32(
  const std::vector<std::uint8_t> & bytes,
  std::size_t * offset,
  std::uint32_t * value)
{
  if (!offset || !value || *offset > bytes.size() ||
    bytes.size() - *offset < 4U)
  {
    return false;
  }
  const std::size_t index = *offset;
  *value = static_cast<std::uint32_t>(bytes[index]) |
    (static_cast<std::uint32_t>(bytes[index + 1U]) << 8U) |
    (static_cast<std::uint32_t>(bytes[index + 2U]) << 16U) |
    (static_cast<std::uint32_t>(bytes[index + 3U]) << 24U);
  *offset += 4U;
  return true;
}

bool read_double(
  const std::vector<std::uint8_t> & bytes,
  std::size_t * offset,
  double * value)
{
  if (!offset || !value || *offset > bytes.size() ||
    bytes.size() - *offset < 8U)
  {
    return false;
  }
  std::uint64_t bits = 0U;
  for (std::size_t byte = 0U; byte < 8U; ++byte) {
    bits |= static_cast<std::uint64_t>(bytes[*offset + byte]) << (8U * byte);
  }
  static_assert(sizeof(bits) == sizeof(*value));
  std::memcpy(value, &bits, sizeof(bits));
  *offset += 8U;
  return true;
}

bool read_states(
  const std::vector<std::uint8_t> & payload,
  std::size_t * offset,
  const std::size_t count,
  std::vector<ArmState> * states,
  std::string * error)
{
  if (!offset || !states || !error) {
    return false;
  }
  states->clear();
  states->reserve(count);
  for (std::size_t state_index = 0U; state_index < count; ++state_index) {
    ArmState state{};
    for (std::size_t joint = 0U; joint < state.size(); ++joint) {
      if (!read_double(payload, offset, &state[joint])) {
        *error = "payload ended inside a state array";
        return false;
      }
      if (!std::isfinite(state[joint])) {
        *error = "payload contains a non-finite joint value";
        return false;
      }
    }
    states->push_back(state);
  }
  return true;
}

VerificationResult audit_continuity(
  const std::vector<ArmState> & states,
  const double limit,
  const std::string & label)
{
  for (std::size_t state = 1U; state < states.size(); ++state) {
    for (std::size_t joint = 0U; joint < kJointCount; ++joint) {
      const double step = std::abs(states[state][joint] - states[state - 1U][joint]);
      if (!std::isfinite(step) || step > limit + kContinuityRoundoff) {
        return failure(
          label + " continuity exceeds its fixed per-joint limit at state " +
          std::to_string(state) + ", joint " + std::to_string(joint));
      }
    }
  }
  VerificationResult result;
  result.passed = true;
  result.message = label + " continuity passed";
  return result;
}

VerificationResult audit_edge(
  const ArmState & first,
  const ArmState & second,
  const double limit,
  const std::string & label)
{
  for (std::size_t joint = 0U; joint < kJointCount; ++joint) {
    const double step = std::abs(second[joint] - first[joint]);
    if (!std::isfinite(step) || step > limit + kContinuityRoundoff) {
      return failure(
        label + " exceeds its fixed per-joint junction limit at joint " +
        std::to_string(joint));
    }
  }
  VerificationResult result;
  result.passed = true;
  result.message = label + " passed";
  return result;
}

VerificationResult parse_payload(
  const std::vector<std::uint8_t> & payload,
  const std::size_t manifest_anchor_index)
{
  constexpr std::size_t kStateCount =
    kOuterAnchorStateCount + kIngressStateCount + kCoarseStateCount +
    kExactStateCount;
  constexpr std::size_t kExpectedPayloadBytes =
    kPayloadHeaderBytes + kStateCount * kJointCount * sizeof(double);
  static_assert(kExpectedPayloadBytes == 41036U);
  if (payload.size() != kExpectedPayloadBytes ||
    payload.size() > kMaximumPayloadBytes)
  {
    return failure("payload length is invalid or has trailing bytes");
  }
  if (!std::equal(kPayloadMagic.begin(), kPayloadMagic.end(), payload.begin())) {
    return failure("payload magic is invalid");
  }

  std::size_t offset = kPayloadMagic.size();
  std::array<std::uint32_t, 9> header{};
  for (auto & value : header) {
    if (!read_u32(payload, &offset, &value)) {
      return failure("payload header is truncated");
    }
  }
  if (header[0] != kSchemaVersion || header[1] != kEndianTag ||
    header[2] != sizeof(double) || header[3] != kJointCount ||
    header[4] != kOuterAnchorStateCount ||
    header[5] != kIngressStateCount || header[6] != kCoarseStateCount ||
    header[7] != kExactStateCount || header[8] != manifest_anchor_index)
  {
    return failure("payload header does not match the fixed contract");
  }
  if (offset != kPayloadHeaderBytes) {
    return failure("payload header size invariant failed");
  }

  PathCertificate certificate;
  certificate.selected_anchor_index = manifest_anchor_index;
  double outer_value = 0.0;
  for (std::size_t joint = 0U; joint < kJointCount; ++joint) {
    if (!read_double(payload, &offset, &outer_value) ||
      !std::isfinite(outer_value))
    {
      return failure("payload outer anchor is truncated or non-finite");
    }
    certificate.outer_anchor[joint] = outer_value;
  }
  std::string state_error;
  if (!read_states(
      payload, &offset, kIngressStateCount, &certificate.ingress, &state_error) ||
    !read_states(
      payload, &offset, kCoarseStateCount, &certificate.coarse, &state_error) ||
    !read_states(
      payload, &offset, kExactStateCount, &certificate.exact, &state_error))
  {
    return failure(state_error);
  }
  if (offset != payload.size()) {
    return failure("payload was not consumed exactly");
  }

  const auto ingress = audit_continuity(
    certificate.ingress, kIngressContinuityLimit, "ingress");
  if (!ingress.passed) {
    return ingress;
  }
  const auto coarse = audit_continuity(
    certificate.coarse, kCoarseContinuityLimit, "coarse");
  if (!coarse.passed) {
    return coarse;
  }
  const auto exact = audit_continuity(
    certificate.exact, kExactContinuityLimit, "exact");
  if (!exact.passed) {
    return exact;
  }
  const auto outer_identity = audit_edge(
    certificate.outer_anchor, certificate.ingress.back(),
    kExactIdentityTolerance, "outer anchor to ingress endpoint identity");
  if (!outer_identity.passed) {
    return outer_identity;
  }
  const auto ingress_to_coarse = audit_edge(
    certificate.ingress.front(), certificate.coarse.front(),
    kCoarseContinuityLimit, "ingress contact to coarse start");
  if (!ingress_to_coarse.passed) {
    return ingress_to_coarse;
  }
  const auto ingress_to_exact = audit_edge(
    certificate.ingress.front(), certificate.exact.front(),
    kExactContinuityLimit, "ingress contact to exact start");
  if (!ingress_to_exact.passed) {
    return ingress_to_exact;
  }
  const auto coarse_to_exact = audit_edge(
    certificate.coarse.front(), certificate.exact.front(),
    kCoarseContinuityLimit, "coarse start to exact start");
  if (!coarse_to_exact.passed) {
    return coarse_to_exact;
  }

  VerificationResult result;
  result.passed = true;
  result.message = "certificate format, hash, finite-value, count, and continuity gates passed";
  result.certificate = std::move(certificate);
  return result;
}

bool read_file(
  const std::string & path,
  const std::size_t maximum_size,
  std::vector<std::uint8_t> * contents,
  std::string * error)
{
  if (!contents || !error || path.empty()) {
    return false;
  }
  const int descriptor =
    ::open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (descriptor < 0) {
    *error = "cannot safely open regular file: " + path + ": " +
      std::strerror(errno);
    return false;
  }
  struct stat status {};
  if (::fstat(descriptor, &status) != 0 || !S_ISREG(status.st_mode) ||
    status.st_size <= 0 ||
    static_cast<std::uintmax_t>(status.st_size) > maximum_size)
  {
    const int saved_errno = errno;
    ::close(descriptor);
    *error = "file size is invalid: " + path;
    if (saved_errno != 0) {
      *error += ": " + std::string(std::strerror(saved_errno));
    }
    return false;
  }
  contents->resize(static_cast<std::size_t>(status.st_size));
  std::size_t offset = 0U;
  while (offset < contents->size()) {
    const ssize_t count = ::read(
      descriptor, contents->data() + offset, contents->size() - offset);
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count <= 0) {
      const int saved_errno = errno;
      ::close(descriptor);
      *error = "file read was incomplete: " + path;
      if (count < 0) {
        *error += ": " + std::string(std::strerror(saved_errno));
      }
      return false;
    }
    offset += static_cast<std::size_t>(count);
  }
  ::close(descriptor);
  return true;
}

VerificationResult verify_expected_bindings(
  const std::map<std::string, std::string> & manifest_fields,
  const ExpectedBindings & expected_bindings)
{
  const auto shape = validate_expected_bindings_shape(expected_bindings);
  if (!shape.passed) {
    return shape;
  }
  for (const auto & key : kHashManifestFields) {
    if (key == "payload_sha256") {
      continue;
    }
    const auto expected = expected_bindings.sha256_by_field.find(key);
    const auto actual = manifest_fields.find(key);
    if (actual == manifest_fields.end() || actual->second != expected->second) {
      return failure("manifest binding mismatch: " + key);
    }
  }
  VerificationResult result;
  result.passed = true;
  result.expected_bindings_checked = true;
  result.message = "all expected bindings match";
  return result;
}

VerificationResult verify_impl(
  const std::string & manifest,
  const std::vector<std::uint8_t> & payload,
  const ExpectedBindings * expected_bindings)
{
  std::map<std::string, std::string> fields;
  std::size_t selected_anchor_index = 0U;
  const auto parsed_manifest = parse_manifest(
    manifest, &fields, &selected_anchor_index);
  if (!parsed_manifest.passed) {
    return parsed_manifest;
  }
  if (sha256_hex(payload) != fields.at("payload_sha256")) {
    return failure("payload SHA-256 does not match the manifest");
  }
  if (expected_bindings) {
    const auto bindings = verify_expected_bindings(fields, *expected_bindings);
    if (!bindings.passed) {
      return bindings;
    }
  }
  auto result = parse_payload(payload, selected_anchor_index);
  if (result.passed && expected_bindings) {
    result.expected_bindings_checked = true;
    result.message =
      "certificate format and all caller-supplied expected bindings passed";
  }
  return result;
}

bool same_file_snapshot(const struct stat & first, const struct stat & second)
{
  return first.st_dev == second.st_dev && first.st_ino == second.st_ino &&
         first.st_mode == second.st_mode && first.st_nlink == second.st_nlink &&
         first.st_size == second.st_size &&
         first.st_mtim.tv_sec == second.st_mtim.tv_sec &&
         first.st_mtim.tv_nsec == second.st_mtim.tv_nsec &&
         first.st_ctim.tv_sec == second.st_ctim.tv_sec &&
         first.st_ctim.tv_nsec == second.st_ctim.tv_nsec;
}

enum class OwnedPathState
{
  kAbsent,
  kOwned,
  kOther,
  kError,
};

OwnedPathState classify_owned_path(
  const int parent_fd,
  const std::string & name,
  const struct stat & expected,
  std::string * error)
{
  struct stat actual {};
  if (::fstatat(
      parent_fd, name.c_str(), &actual, AT_SYMLINK_NOFOLLOW) != 0)
  {
    if (errno == ENOENT) {
      return OwnedPathState::kAbsent;
    }
    if (error) {
      *error = "cannot classify path '" + name + "': " +
        std::strerror(errno);
    }
    return OwnedPathState::kError;
  }
  if (S_ISDIR(actual.st_mode) && actual.st_dev == expected.st_dev &&
    actual.st_ino == expected.st_ino)
  {
    return OwnedPathState::kOwned;
  }
  return OwnedPathState::kOther;
}

bool absolute_directory_matches(
  const std::string & path,
  const struct stat & expected,
  std::string * error)
{
  UniqueFd current(open_absolute_directory_no_symlinks(path, error));
  if (!current) {
    return false;
  }
  struct stat actual {};
  if (::fstat(current.get(), &actual) != 0) {
    if (error) {
      *error = "cannot restat absolute parent directory: " +
        std::string(std::strerror(errno));
    }
    return false;
  }
  if (actual.st_dev != expected.st_dev || actual.st_ino != expected.st_ino ||
    actual.st_uid != expected.st_uid ||
    (actual.st_mode & (S_IWGRP | S_IWOTH)) != 0)
  {
    if (error) {
      *error = "absolute parent directory identity or permissions changed";
    }
    return false;
  }
  return true;
}

bool directory_has_exact_certificate_entries(
  const int directory_fd,
  std::string * error)
{
  if (!error) {
    return false;
  }
  const int duplicate = ::openat(
    directory_fd, ".",
    O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
  if (duplicate < 0) {
    *error = "cannot duplicate certificate directory descriptor: " +
      std::string(std::strerror(errno));
    return false;
  }
  DIR * stream = ::fdopendir(duplicate);
  if (!stream) {
    const int saved_errno = errno;
    ::close(duplicate);
    *error = "cannot enumerate certificate directory: " +
      std::string(std::strerror(saved_errno));
    return false;
  }

  bool saw_manifest = false;
  bool saw_payload = false;
  bool valid = true;
  errno = 0;
  while (const dirent * entry = ::readdir(stream)) {
    const std::string name(entry->d_name);
    if (name == "." || name == "..") {
      continue;
    }
    if (name == kManifestFilename && !saw_manifest) {
      saw_manifest = true;
    } else if (name == kPayloadFilename && !saw_payload) {
      saw_payload = true;
    } else {
      *error = "certificate directory contains an unexpected entry: " + name;
      valid = false;
      break;
    }
  }
  if (valid && errno != 0) {
    *error = "cannot finish enumerating certificate directory: " +
      std::string(std::strerror(errno));
    valid = false;
  }
  if (::closedir(stream) != 0 && valid) {
    *error = "cannot close certificate directory stream: " +
      std::string(std::strerror(errno));
    valid = false;
  }
  if (valid && (!saw_manifest || !saw_payload)) {
    *error = "certificate directory does not contain both fixed files";
    valid = false;
  }
  return valid;
}

bool read_regular_at(
  const int directory_fd,
  const char * filename,
  const std::size_t maximum_size,
  std::vector<std::uint8_t> * contents,
  std::string * error)
{
  if (!filename || !contents || !error) {
    return false;
  }
  UniqueFd descriptor(::openat(
      directory_fd, filename,
      O_RDONLY | O_NONBLOCK | O_NOFOLLOW | O_CLOEXEC));
  if (!descriptor) {
    *error = "cannot safely open certificate file '" +
      std::string(filename) + "': " + std::strerror(errno);
    return false;
  }

  struct stat before {};
  if (::fstat(descriptor.get(), &before) != 0) {
    *error = "cannot stat certificate file '" + std::string(filename) +
      "': " + std::strerror(errno);
    return false;
  }
  if (!S_ISREG(before.st_mode) || before.st_nlink != 1 ||
    (before.st_mode & 0222) != 0 || before.st_size <= 0 ||
    static_cast<std::uintmax_t>(before.st_size) > maximum_size)
  {
    *error = "certificate file type, link count, permissions, or size is invalid: " +
      std::string(filename);
    return false;
  }

  contents->assign(static_cast<std::size_t>(before.st_size), 0U);
  std::size_t offset = 0U;
  while (offset < contents->size()) {
    const ssize_t count = ::read(
      descriptor.get(), contents->data() + offset, contents->size() - offset);
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count <= 0) {
      *error = "certificate file read was incomplete: " +
        std::string(filename);
      if (count < 0) {
        *error += ": " + std::string(std::strerror(errno));
      }
      return false;
    }
    offset += static_cast<std::size_t>(count);
  }

  std::uint8_t trailing = 0U;
  ssize_t trailing_count = -1;
  do {
    trailing_count = ::read(descriptor.get(), &trailing, 1U);
  } while (trailing_count < 0 && errno == EINTR);
  if (trailing_count != 0) {
    *error = "certificate file changed size while being read: " +
      std::string(filename);
    return false;
  }

  struct stat after {};
  struct stat path_after {};
  if (::fstat(descriptor.get(), &after) != 0 ||
    ::fstatat(
      directory_fd, filename, &path_after, AT_SYMLINK_NOFOLLOW) != 0 ||
    !same_file_snapshot(before, after) ||
    !same_file_snapshot(before, path_after))
  {
    *error = "certificate file changed while being read: " +
      std::string(filename);
    return false;
  }
  return true;
}

bool read_certificate_directory_fd(
  const int directory_fd,
  const bool require_read_only_directory,
  std::string * manifest,
  std::vector<std::uint8_t> * payload,
  std::string * error)
{
  if (!manifest || !payload || !error) {
    return false;
  }
  struct stat before {};
  if (::fstat(directory_fd, &before) != 0 || !S_ISDIR(before.st_mode)) {
    *error = "certificate path is not a readable directory";
    return false;
  }
  if (require_read_only_directory && (before.st_mode & 0222) != 0) {
    *error = "published certificate directory is writable";
    return false;
  }
  if (!directory_has_exact_certificate_entries(directory_fd, error)) {
    return false;
  }

  std::vector<std::uint8_t> manifest_bytes;
  if (!read_regular_at(
      directory_fd, kManifestFilename, kMaximumManifestBytes,
      &manifest_bytes, error) ||
    !read_regular_at(
      directory_fd, kPayloadFilename, kMaximumPayloadBytes, payload, error))
  {
    return false;
  }
  if (!directory_has_exact_certificate_entries(directory_fd, error)) {
    return false;
  }

  struct stat after {};
  if (::fstat(directory_fd, &after) != 0 ||
    !same_file_snapshot(before, after))
  {
    *error = "certificate directory changed while being read";
    return false;
  }
  manifest->assign(
    reinterpret_cast<const char *>(manifest_bytes.data()),
    manifest_bytes.size());
  return true;
}

bool write_all_and_seal(
  const int directory_fd,
  const char * filename,
  const std::uint8_t * data,
  const std::size_t size,
  std::string * error)
{
  if (!filename || (!data && size != 0U) || !error) {
    return false;
  }
  UniqueFd descriptor(::openat(
      directory_fd, filename,
      O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600));
  if (!descriptor) {
    *error = "cannot create temporary certificate file '" +
      std::string(filename) + "': " + std::strerror(errno);
    return false;
  }
  std::size_t offset = 0U;
  while (offset < size) {
    const ssize_t count = ::write(descriptor.get(), data + offset, size - offset);
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count <= 0) {
      *error = "cannot completely write temporary certificate file '" +
        std::string(filename) + "'";
      if (count < 0) {
        *error += ": " + std::string(std::strerror(errno));
      }
      return false;
    }
    offset += static_cast<std::size_t>(count);
  }
  if (::fchmod(descriptor.get(), 0400) != 0) {
    *error = "cannot seal temporary certificate file '" +
      std::string(filename) + "': " + std::strerror(errno);
    return false;
  }
  if (::fsync(descriptor.get()) != 0) {
    *error = "cannot sync temporary certificate file '" +
      std::string(filename) + "': " + std::strerror(errno);
    return false;
  }
  const int raw_descriptor = descriptor.release();
  if (::close(raw_descriptor) != 0) {
    *error = "cannot close temporary certificate file '" +
      std::string(filename) + "': " + std::strerror(errno);
    return false;
  }
  return true;
}

}  // namespace

std::string sha256_hex(const std::uint8_t * data, const std::size_t size)
{
  if (!data && size != 0U) {
    throw std::invalid_argument("sha256 input pointer is null");
  }
  rcutils_sha256_ctx_t context;
  rcutils_sha256_init(&context);
  if (size != 0U) {
    rcutils_sha256_update(&context, data, size);
  }
  std::array<std::uint8_t, RCUTILS_SHA256_BLOCK_SIZE> digest{};
  rcutils_sha256_final(&context, digest.data());
  std::ostringstream result;
  result << std::hex << std::setfill('0');
  for (const std::uint8_t byte : digest) {
    result << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return result.str();
}

VerificationResult inspect_format_bytes(
  const std::string & manifest,
  const std::vector<std::uint8_t> & payload)
{
  return verify_impl(manifest, payload, nullptr);
}

VerificationResult inspect_format_files(
  const std::string & manifest_path,
  const std::string & payload_path)
{
  std::vector<std::uint8_t> manifest_bytes;
  std::vector<std::uint8_t> payload;
  std::string error;
  if (!read_file(
      manifest_path, kMaximumManifestBytes, &manifest_bytes, &error))
  {
    return failure(error);
  }
  if (!read_file(payload_path, kMaximumPayloadBytes, &payload, &error)) {
    return failure(error);
  }
  const std::string manifest(
    reinterpret_cast<const char *>(manifest_bytes.data()),
    manifest_bytes.size());
  return inspect_format_bytes(manifest, payload);
}

VerificationResult verify_bytes(
  const std::string & manifest,
  const std::vector<std::uint8_t> & payload,
  const ExpectedBindings & expected_bindings)
{
  return verify_impl(manifest, payload, &expected_bindings);
}

VerificationResult verify_files(
  const std::string & manifest_path,
  const std::string & payload_path,
  const ExpectedBindings & expected_bindings)
{
  std::vector<std::uint8_t> manifest_bytes;
  std::vector<std::uint8_t> payload;
  std::string error;
  if (!read_file(
      manifest_path, kMaximumManifestBytes, &manifest_bytes, &error))
  {
    return failure(error);
  }
  if (!read_file(payload_path, kMaximumPayloadBytes, &payload, &error)) {
    return failure(error);
  }
  const std::string manifest(
    reinterpret_cast<const char *>(manifest_bytes.data()),
    manifest_bytes.size());
  return verify_bytes(manifest, payload, expected_bindings);
}

VerificationResult inspect_format_directory(
  const std::string & certificate_directory)
{
  std::string error;
  UniqueFd directory(
    open_absolute_directory_no_symlinks(certificate_directory, &error));
  if (!directory) {
    return failure(error);
  }
  std::string manifest;
  std::vector<std::uint8_t> payload;
  if (!read_certificate_directory_fd(
      directory.get(), true, &manifest, &payload, &error))
  {
    return failure(error);
  }
  return inspect_format_bytes(manifest, payload);
}

VerificationResult verify_directory(
  const std::string & certificate_directory,
  const ExpectedBindings & expected_bindings)
{
  std::string error;
  UniqueFd directory(
    open_absolute_directory_no_symlinks(certificate_directory, &error));
  if (!directory) {
    return failure(error);
  }
  std::string manifest;
  std::vector<std::uint8_t> payload;
  if (!read_certificate_directory_fd(
      directory.get(), true, &manifest, &payload, &error))
  {
    return failure(error);
  }
  return verify_bytes(manifest, payload, expected_bindings);
}

FileHashResult sha256_regular_file(
  const std::string & path,
  const std::uint64_t maximum_size_bytes)
{
  FileHashResult result;
  std::string parent_path;
  std::string leaf;
  std::string error;
  if (!split_absolute_target(path, &parent_path, &leaf, &error)) {
    result.message = error;
    return result;
  }
  UniqueFd parent(open_absolute_directory_no_symlinks(parent_path, &error));
  if (!parent) {
    result.message = error;
    return result;
  }
  UniqueFd descriptor(::openat(
      parent.get(), leaf.c_str(),
      O_RDONLY | O_NONBLOCK | O_NOFOLLOW | O_CLOEXEC));
  if (!descriptor) {
    result.message = "cannot safely open regular file: " +
      std::string(std::strerror(errno));
    return result;
  }

  struct stat before {};
  if (::fstat(descriptor.get(), &before) != 0 || !S_ISREG(before.st_mode) ||
    before.st_size < 0 ||
    static_cast<std::uintmax_t>(before.st_size) > maximum_size_bytes)
  {
    result.message = "regular file type or initial size is invalid";
    return result;
  }

  rcutils_sha256_ctx_t context;
  rcutils_sha256_init(&context);
  std::array<std::uint8_t, 64U * 1024U> buffer{};
  std::uint64_t total = 0U;
  while (true) {
    const ssize_t count = ::read(descriptor.get(), buffer.data(), buffer.size());
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count < 0) {
      result.message = "cannot hash regular file: " +
        std::string(std::strerror(errno));
      return result;
    }
    if (count == 0) {
      break;
    }
    const auto unsigned_count = static_cast<std::uint64_t>(count);
    if (unsigned_count > maximum_size_bytes - total) {
      result.message = "regular file exceeded the maximum size while hashing";
      return result;
    }
    rcutils_sha256_update(
      &context, buffer.data(), static_cast<std::size_t>(count));
    total += unsigned_count;
  }

  struct stat after {};
  struct stat path_after {};
  if (::fstat(descriptor.get(), &after) != 0 ||
    ::fstatat(
      parent.get(), leaf.c_str(), &path_after, AT_SYMLINK_NOFOLLOW) != 0 ||
    !same_file_snapshot(before, after) ||
    !same_file_snapshot(before, path_after) ||
    total != static_cast<std::uint64_t>(before.st_size))
  {
    result.message = "regular file changed while being hashed";
    return result;
  }

  std::array<std::uint8_t, RCUTILS_SHA256_BLOCK_SIZE> digest{};
  rcutils_sha256_final(&context, digest.data());
  std::ostringstream hexadecimal;
  hexadecimal << std::hex << std::setfill('0');
  for (const std::uint8_t byte : digest) {
    hexadecimal << std::setw(2) << static_cast<unsigned int>(byte);
  }
  result.passed = true;
  result.message = "regular file SHA-256 passed stable-descriptor checks";
  result.sha256 = hexadecimal.str();
  result.size_bytes = total;
  return result;
}

FileHashResult sha256_self_executable(
  const std::uint64_t maximum_size_bytes)
{
  FileHashResult result;
  constexpr char kSelfExecutableLink[] = "/proc/self/exe";
  UniqueFd descriptor(::open(
      kSelfExecutableLink, O_RDONLY | O_NONBLOCK | O_CLOEXEC));
  if (!descriptor) {
    result.message = "cannot open the running executable inode: " +
      std::string(std::strerror(errno));
    return result;
  }

  struct stat before {};
  struct stat proc_before {};
  if (::fstat(descriptor.get(), &before) != 0 ||
    ::stat(kSelfExecutableLink, &proc_before) != 0 ||
    !S_ISREG(before.st_mode) || before.st_size < 0 ||
    static_cast<std::uintmax_t>(before.st_size) > maximum_size_bytes ||
    !same_file_snapshot(before, proc_before))
  {
    result.message = "running executable type, size, or initial inode is invalid";
    return result;
  }

  rcutils_sha256_ctx_t context;
  rcutils_sha256_init(&context);
  std::array<std::uint8_t, 64U * 1024U> buffer{};
  std::uint64_t total = 0U;
  while (true) {
    const ssize_t count = ::read(descriptor.get(), buffer.data(), buffer.size());
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count < 0) {
      result.message = "cannot hash the running executable inode: " +
        std::string(std::strerror(errno));
      return result;
    }
    if (count == 0) {
      break;
    }
    const auto unsigned_count = static_cast<std::uint64_t>(count);
    if (unsigned_count > maximum_size_bytes - total) {
      result.message =
        "running executable exceeded the maximum size while hashing";
      return result;
    }
    rcutils_sha256_update(
      &context, buffer.data(), static_cast<std::size_t>(count));
    total += unsigned_count;
  }

  struct stat after {};
  struct stat proc_after {};
  if (::fstat(descriptor.get(), &after) != 0 ||
    ::stat(kSelfExecutableLink, &proc_after) != 0 ||
    !same_file_snapshot(before, after) ||
    !same_file_snapshot(before, proc_after) ||
    total != static_cast<std::uint64_t>(before.st_size))
  {
    result.message = "running executable changed while being hashed";
    return result;
  }

  std::array<std::uint8_t, RCUTILS_SHA256_BLOCK_SIZE> digest{};
  rcutils_sha256_final(&context, digest.data());
  std::ostringstream hexadecimal;
  hexadecimal << std::hex << std::setfill('0');
  for (const std::uint8_t byte : digest) {
    hexadecimal << std::setw(2) << static_cast<unsigned int>(byte);
  }
  result.passed = true;
  result.message =
    "running executable SHA-256 passed stable-descriptor checks";
  result.sha256 = hexadecimal.str();
  result.size_bytes = total;
  return result;
}

WriteResult write_new_directory(
  const std::string & certificate_directory,
  const PathCertificate & certificate,
  const ExpectedBindings & expected_bindings)
{
  WriteResult result;
  std::vector<std::uint8_t> payload;
  std::string manifest;
  std::string error;
  if (!encode_payload(certificate, &payload, &error) ||
    !encode_manifest(
      certificate, expected_bindings, sha256_hex(payload), &manifest, &error))
  {
    result.message = "certificate encoding rejected: " + error;
    return result;
  }
  const auto encoded_verification = verify_bytes(
    manifest, payload, expected_bindings);
  if (!encoded_verification.passed ||
    !encoded_verification.expected_bindings_checked)
  {
    result.message = "encoded certificate failed strict verification: " +
      encoded_verification.message;
    return result;
  }

  std::string parent_path;
  std::string target_name;
  if (!split_absolute_target(
      certificate_directory, &parent_path, &target_name, &error))
  {
    result.message = error;
    return result;
  }
  UniqueFd parent(open_absolute_directory_no_symlinks(parent_path, &error));
  if (!parent) {
    result.message = error;
    return result;
  }
  struct stat parent_identity {};
  if (::fstat(parent.get(), &parent_identity) != 0 ||
    !S_ISDIR(parent_identity.st_mode) ||
    parent_identity.st_uid != ::geteuid() ||
    (parent_identity.st_mode & (S_IWGRP | S_IWOTH)) != 0)
  {
    result.message =
      "certificate parent must be an owned directory without group/other "
      "write permission";
    return result;
  }

  struct stat existing {};
  if (::fstatat(
      parent.get(), target_name.c_str(), &existing,
      AT_SYMLINK_NOFOLLOW) == 0)
  {
    result.message = "certificate target already exists; overwrite is forbidden";
    return result;
  }
  if (errno != ENOENT) {
    result.message = "cannot establish that certificate target is absent: " +
      std::string(std::strerror(errno));
    return result;
  }

  std::string temporary_name;
  bool created = false;
  for (std::size_t attempt = 0U; attempt < 128U; ++attempt) {
    std::string random_suffix;
    if (!random_hex_suffix(&random_suffix, &error)) {
      result.message = error;
      return result;
    }
    temporary_name = ".h1cert.tmp." + random_suffix;
    if (::mkdirat(parent.get(), temporary_name.c_str(), 0700) == 0) {
      created = true;
      break;
    }
    if (errno != EEXIST) {
      result.message = "cannot create sibling temporary directory: " +
        std::string(std::strerror(errno));
      return result;
    }
  }
  if (!created) {
    result.message = "cannot allocate a unique sibling temporary directory";
    return result;
  }

  struct stat owned {};
  if (::fstatat(
      parent.get(), temporary_name.c_str(), &owned,
      AT_SYMLINK_NOFOLLOW) != 0 || !S_ISDIR(owned.st_mode))
  {
    result.message =
      "cannot identify newly created temporary directory; cleanup was not "
      "attempted";
    return result;
  }

  UniqueFd temporary(::openat(
      parent.get(), temporary_name.c_str(),
      O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC));
  if (!temporary) {
    const int saved_errno = errno;
    result.message = "cannot open owned temporary directory: " +
      std::string(std::strerror(saved_errno)) +
      "; random temporary path was preserved without destructive cleanup: " +
      temporary_name;
    return result;
  }
  struct stat opened {};
  struct stat opened_path {};
  if (::fstat(temporary.get(), &opened) != 0 || !S_ISDIR(opened.st_mode) ||
    ::fstatat(
      parent.get(), temporary_name.c_str(), &opened_path,
      AT_SYMLINK_NOFOLLOW) != 0 ||
    opened.st_dev != owned.st_dev || opened.st_ino != owned.st_ino ||
    opened_path.st_dev != owned.st_dev || opened_path.st_ino != owned.st_ino)
  {
    result.message =
      "temporary directory identity changed while being opened; cleanup was "
      "not attempted";
    return result;
  }
  owned = opened;

  const auto fail_before_publish = [&](const std::string & message) {
      WriteResult failed;
      failed.message = message +
        "; random temporary path was preserved without destructive cleanup: " +
        temporary_name;
      return failed;
    };

  if (!write_all_and_seal(
      temporary.get(), kPayloadFilename, payload.data(), payload.size(),
      &error))
  {
    return fail_before_publish(error);
  }
  if (!write_all_and_seal(
      temporary.get(), kManifestFilename,
      reinterpret_cast<const std::uint8_t *>(manifest.data()), manifest.size(),
      &error))
  {
    return fail_before_publish(error);
  }

  std::string readback_manifest;
  std::vector<std::uint8_t> readback_payload;
  if (!read_certificate_directory_fd(
      temporary.get(), false, &readback_manifest, &readback_payload, &error))
  {
    return fail_before_publish(
      "temporary certificate readback failed: " + error);
  }
  const auto readback_verification = verify_bytes(
    readback_manifest, readback_payload, expected_bindings);
  if (!readback_verification.passed ||
    !readback_verification.expected_bindings_checked ||
    readback_manifest != manifest || readback_payload != payload)
  {
    return fail_before_publish(
      "temporary certificate deterministic readback verification failed: " +
      readback_verification.message);
  }
  if (::fchmod(temporary.get(), 0500) != 0) {
    return fail_before_publish(
      "cannot seal temporary certificate directory: " +
      std::string(std::strerror(errno)));
  }
  if (::fsync(temporary.get()) != 0) {
    return fail_before_publish(
      "cannot sync temporary certificate directory: " +
      std::string(std::strerror(errno)));
  }
  if (!absolute_directory_matches(parent_path, parent_identity, &error)) {
    return fail_before_publish(
      "certificate absolute parent changed before publication: " + error);
  }

#ifdef SYS_renameat2
  const long rename_result = ::syscall(
    SYS_renameat2, parent.get(), temporary_name.c_str(), parent.get(),
    target_name.c_str(), RENAME_NOREPLACE);
#else
  errno = ENOSYS;
  const long rename_result = -1;
#endif
  if (rename_result != 0) {
    const int saved_errno = errno;
    std::string message;
    if (saved_errno == EEXIST) {
      message = "certificate target appeared concurrently; overwrite is forbidden";
    } else if (saved_errno == ENOSYS || saved_errno == EINVAL) {
      message = "atomic no-replace directory publication is unsupported; no fallback allowed";
    } else {
      message = "atomic no-replace directory publication failed: " +
        std::string(std::strerror(saved_errno));
    }
    std::string identity_error;
    const auto temporary_state = classify_owned_path(
      parent.get(), temporary_name, owned, &identity_error);
    const auto target_state = classify_owned_path(
      parent.get(), target_name, owned, &identity_error);
    if (target_state == OwnedPathState::kOwned) {
      result.state = PublishState::kPublishedDurabilityUncertain;
      result.message = message +
        "; target now names the complete owned certificate, so it was "
        "preserved with uncertain durability";
      (void)::fsync(parent.get());
      return result;
    }
    result.message = message;
    if (temporary_state == OwnedPathState::kOwned) {
      result.message +=
        "; complete random temporary path was preserved without destructive "
        "cleanup: " + temporary_name;
    } else {
      result.message +=
        "; temporary/target identity is uncertain, so destructive cleanup "
        "was not attempted";
    }
    if (!identity_error.empty()) {
      result.message += ": " + identity_error;
    }
    return result;
  }

  result.state = PublishState::kPublishedDurabilityUncertain;
  std::string identity_error;
  auto target_state = classify_owned_path(
    parent.get(), target_name, owned, &identity_error);
  auto temporary_state = classify_owned_path(
    parent.get(), temporary_name, owned, &identity_error);
  if (target_state != OwnedPathState::kOwned ||
    temporary_state != OwnedPathState::kAbsent ||
    !absolute_directory_matches(parent_path, parent_identity, &identity_error))
  {
    result.message =
      "atomic rename reported success, but target or absolute-parent identity "
      "is uncertain; published data was preserved";
    if (!identity_error.empty()) {
      result.message += ": " + identity_error;
    }
    return result;
  }
  if (::fsync(parent.get()) != 0) {
    result.message =
      "certificate is visible and complete, but parent-directory durability "
      "could not be confirmed: " + std::string(std::strerror(errno));
    return result;
  }
  identity_error.clear();
  target_state = classify_owned_path(
    parent.get(), target_name, owned, &identity_error);
  temporary_state = classify_owned_path(
    parent.get(), temporary_name, owned, &identity_error);
  if (target_state != OwnedPathState::kOwned ||
    temporary_state != OwnedPathState::kAbsent ||
    !absolute_directory_matches(parent_path, parent_identity, &identity_error))
  {
    result.message =
      "certificate parent was synced, but final target or absolute-parent "
      "identity is uncertain; published data was preserved";
    if (!identity_error.empty()) {
      result.message += ": " + identity_error;
    }
    return result;
  }
  result.passed = true;
  result.state = PublishState::kDurablyPublished;
  result.message =
    "new read-only-mode sealed certificate directory was atomically and "
    "durably published";
  return result;
}

}  // namespace h1_2_moveit_demo::door_handoff_path_certificate
