#include "h1_2_moveit_demo/door_handoff_path_certificate.hpp"

#include "h1_2_moveit_demo/door_handoff_binding_fields.hpp"

#include <gtest/gtest.h>

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace certificate =
  h1_2_moveit_demo::door_handoff_path_certificate;

namespace
{

constexpr std::size_t kHeaderBytes = 44U;
constexpr std::size_t kStateBytes = certificate::kJointCount * sizeof(double);
constexpr double kPi = 3.14159265358979323846;
const std::vector<std::string> kGenericHashFields = []() {
    std::vector<std::string> fields;
    fields.reserve(certificate::kBindingFieldRegistry.size() - 1U);
    for (const auto & field : certificate::kBindingFieldRegistry) {
      if (field.name != "original_handle_stl_sha256") {
        fields.emplace_back(field.name);
      }
    }
    return fields;
  }();

void append_u32(std::vector<std::uint8_t> * bytes, const std::uint32_t value)
{
  ASSERT_NE(bytes, nullptr);
  for (std::size_t shift = 0U; shift < 4U; ++shift) {
    bytes->push_back(static_cast<std::uint8_t>(value >> (8U * shift)));
  }
}

void append_double(std::vector<std::uint8_t> * bytes, const double value)
{
  ASSERT_NE(bytes, nullptr);
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  for (std::size_t shift = 0U; shift < 8U; ++shift) {
    bytes->push_back(static_cast<std::uint8_t>(bits >> (8U * shift)));
  }
}

void set_u32(
  std::vector<std::uint8_t> * bytes,
  const std::size_t offset,
  const std::uint32_t value)
{
  ASSERT_NE(bytes, nullptr);
  ASSERT_LE(offset + 4U, bytes->size());
  for (std::size_t shift = 0U; shift < 4U; ++shift) {
    (*bytes)[offset + shift] =
      static_cast<std::uint8_t>(value >> (8U * shift));
  }
}

void set_double(
  std::vector<std::uint8_t> * bytes,
  const std::size_t offset,
  const double value)
{
  ASSERT_NE(bytes, nullptr);
  ASSERT_LE(offset + 8U, bytes->size());
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  for (std::size_t shift = 0U; shift < 8U; ++shift) {
    (*bytes)[offset + shift] =
      static_cast<std::uint8_t>(bits >> (8U * shift));
  }
}

std::vector<std::uint8_t> valid_payload(const std::uint32_t anchor = 0U)
{
  std::vector<std::uint8_t> payload{
    'H', '1', 'D', 'H', 'P', 'T', 'H', '1'};
  append_u32(&payload, 1U);
  append_u32(&payload, 0x01020304U);
  append_u32(&payload, 8U);
  append_u32(&payload, 7U);
  append_u32(&payload, 1U);
  append_u32(&payload, 45U);
  append_u32(&payload, 73U);
  append_u32(&payload, 613U);
  append_u32(&payload, anchor);
  EXPECT_EQ(payload.size(), kHeaderBytes);
  constexpr std::size_t kTotalStates = 1U + 45U + 73U + 613U;
  for (std::size_t state = 0U; state < kTotalStates; ++state) {
    for (std::size_t joint = 0U; joint < certificate::kJointCount; ++joint) {
      append_double(&payload, 0.0);
    }
  }
  EXPECT_EQ(payload.size(), 41036U);
  return payload;
}

std::string valid_manifest(
  const std::vector<std::uint8_t> & payload,
  const std::size_t anchor = 0U)
{
  const std::array<std::pair<const char *, const char *>, 35> fixed{
    std::pair{"schema_version", "1"},
    std::pair{"byte_order", "little"},
    std::pair{"scalar_format", "ieee754-binary64"},
    std::pair{"joint_count", "7"},
    std::pair{"outer_anchor_state_count", "1"},
    std::pair{"ingress_state_count", "45"},
    std::pair{"coarse_state_count", "73"},
    std::pair{"exact_state_count", "613"},
    std::pair{"execution_moving_target_count", "601"},
    std::pair{"dense_interpolation_sample_count", "2943"},
    std::pair{"joint_names",
      "right_shoulder_pitch_joint,right_shoulder_roll_joint,"
      "right_shoulder_yaw_joint,right_elbow_joint,right_wrist_roll_joint,"
      "right_wrist_pitch_joint,right_wrist_yaw_joint"},
    std::pair{"ingress_continuity_limit_degrees", "4.9"},
    std::pair{"coarse_continuity_limit_degrees", "4.9"},
    std::pair{"exact_continuity_limit_degrees", "0.49"},
    std::pair{"environment_clearance_m", "0.010"},
    std::pair{"cartesian_minimum_fraction", "0.995"},
    std::pair{"partial_allowed", "false"},
    std::pair{"full_reverse_required", "true"},
    std::pair{"original_stl_contact_required", "true"},
    std::pair{"timing_cached", "false"},
    std::pair{"door_start_degrees", "0"},
    std::pair{"door_end_degrees", "60"},
    std::pair{"handle_start_degrees", "90"},
    std::pair{"handle_local_z_mm", "35"},
    std::pair{"handle_phase_degrees", "21"},
    std::pair{"palm_down_tilt_degrees", "7"},
    std::pair{"handle_local_x_mm", "0"},
    std::pair{"outward_offset_mm", "16"},
    std::pair{"clamp_q1_mm", "11.35"},
    std::pair{"clamp_q2_mm", "12.35"},
    std::pair{"pregrasp_q1_mm", "10.35"},
    std::pair{"pregrasp_q2_mm", "11.35"},
    std::pair{"pad_station_limit_mm", "26"},
    std::pair{"nonpad_clearance_limit_mm", "0.6"},
    std::pair{"dual_hand_clearance_limit_mm", "5"},
  };
  std::string manifest = "H1_DOOR_HANDOFF_PATH_CERTIFICATE_V1\n";
  for (const auto & [key, value] : fixed) {
    manifest += std::string(key) + "=" + value + "\n";
  }
  manifest += "selected_anchor_index=" + std::to_string(anchor) + "\n";
  for (std::size_t index = 0U; index < kGenericHashFields.size(); ++index) {
    manifest += kGenericHashFields[index] + "=" +
      std::string(64U, static_cast<char>('1' + index % 8U)) + "\n";
  }
  manifest +=
    "original_handle_stl_sha256="
    "ad6670185fa5756ba5dabdee7e542db2df3346973fea4eb887288f54a6b6f59e\n";
  manifest += "payload_sha256=" + certificate::sha256_hex(payload) + "\n";
  return manifest;
}

certificate::ExpectedBindings valid_expected_bindings()
{
  certificate::ExpectedBindings bindings;
  for (std::size_t index = 0U; index < kGenericHashFields.size(); ++index) {
    bindings.sha256_by_field.emplace(
      kGenericHashFields[index],
      std::string(64U, static_cast<char>('1' + index % 8U)));
  }
  bindings.sha256_by_field.emplace(
    "original_handle_stl_sha256",
    "ad6670185fa5756ba5dabdee7e542db2df3346973fea4eb887288f54a6b6f59e");
  return bindings;
}

certificate::VerificationResult verify_with_expected(
  const std::string & manifest,
  const std::vector<std::uint8_t> & payload)
{
  return certificate::verify_bytes(
    manifest, payload, valid_expected_bindings());
}

std::string replace_value(
  std::string manifest,
  const std::string & key,
  const std::string & value)
{
  const std::string prefix = key + "=";
  const std::size_t begin = manifest.find(prefix);
  EXPECT_NE(begin, std::string::npos);
  if (begin == std::string::npos) {
    return manifest;
  }
  const std::size_t end = manifest.find('\n', begin);
  EXPECT_NE(end, std::string::npos);
  manifest.replace(begin + prefix.size(), end - begin - prefix.size(), value);
  return manifest;
}

std::size_t ingress_state_offset(const std::size_t state)
{
  return kHeaderBytes + kStateBytes + state * kStateBytes;
}

std::size_t coarse_state_offset(const std::size_t state)
{
  return ingress_state_offset(certificate::kIngressStateCount) +
         state * kStateBytes;
}

std::size_t exact_state_offset(const std::size_t state)
{
  return coarse_state_offset(certificate::kCoarseStateCount) +
         state * kStateBytes;
}

certificate::PathCertificate synthetic_path_certificate()
{
  certificate::PathCertificate result;
  result.selected_anchor_index = 4U;
  result.ingress.resize(certificate::kIngressStateCount);
  result.coarse.resize(certificate::kCoarseStateCount);
  result.exact.resize(certificate::kExactStateCount);

  certificate::ArmState contact{};
  for (std::size_t joint = 0U; joint < contact.size(); ++joint) {
    const double direction = joint % 2U == 0U ? 1.0 : -1.0;
    contact[joint] = direction * (0.02 + 0.005 * static_cast<double>(joint));
  }
  for (std::size_t state = 0U; state < result.ingress.size(); ++state) {
    for (std::size_t joint = 0U; joint < contact.size(); ++joint) {
      const double direction = joint % 2U == 0U ? 1.0 : -1.0;
      result.ingress[state][joint] = contact[joint] + direction * 0.0001 *
        static_cast<double>(state);
    }
  }
  result.outer_anchor = result.ingress.back();
  for (std::size_t state = 0U; state < result.coarse.size(); ++state) {
    for (std::size_t joint = 0U; joint < contact.size(); ++joint) {
      const double direction = joint % 2U == 0U ? 1.0 : -1.0;
      result.coarse[state][joint] = contact[joint] + direction * 0.0002 *
        static_cast<double>(state);
    }
  }
  for (std::size_t state = 0U; state < result.exact.size(); ++state) {
    for (std::size_t joint = 0U; joint < contact.size(); ++joint) {
      const double direction = joint % 2U == 0U ? 1.0 : -1.0;
      result.exact[state][joint] = contact[joint] + direction * 0.00002 *
        static_cast<double>(state);
    }
  }
  return result;
}

std::vector<std::uint8_t> expected_payload_bytes(
  const certificate::PathCertificate & path)
{
  std::vector<std::uint8_t> payload{
    'H', '1', 'D', 'H', 'P', 'T', 'H', '1'};
  append_u32(&payload, 1U);
  append_u32(&payload, 0x01020304U);
  append_u32(&payload, 8U);
  append_u32(&payload, 7U);
  append_u32(&payload, 1U);
  append_u32(&payload, 45U);
  append_u32(&payload, 73U);
  append_u32(&payload, 613U);
  append_u32(&payload, static_cast<std::uint32_t>(path.selected_anchor_index));
  const auto append_state = [&payload](const certificate::ArmState & state) {
      for (const double value : state) {
        append_double(&payload, value);
      }
    };
  append_state(path.outer_anchor);
  for (const auto & state : path.ingress) {
    append_state(state);
  }
  for (const auto & state : path.coarse) {
    append_state(state);
  }
  for (const auto & state : path.exact) {
    append_state(state);
  }
  EXPECT_EQ(payload.size(), 41036U);
  return payload;
}

std::vector<std::uint8_t> read_binary_file(const std::string & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    ADD_FAILURE() << "could not read " << path;
    return {};
  }
  return std::vector<std::uint8_t>(
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}

std::string read_text_file(const std::string & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    ADD_FAILURE() << "could not read " << path;
    return {};
  }
  return std::string(
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}

void write_text_file(
  const std::string & path,
  const std::string & contents,
  const mode_t mode = 0600)
{
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(stream) << path;
  stream.write(contents.data(), static_cast<std::streamsize>(contents.size()));
  ASSERT_TRUE(stream) << path;
  stream.close();
  ASSERT_TRUE(stream) << path;
  ASSERT_EQ(::chmod(path.c_str(), mode), 0) << path;
}

std::vector<std::string> directory_entries(const std::string & path)
{
  std::vector<std::string> entries;
  DIR * directory = ::opendir(path.c_str());
  if (!directory) {
    ADD_FAILURE() << "could not open directory " << path;
    return entries;
  }
  errno = 0;
  while (const dirent * entry = ::readdir(directory)) {
    const std::string name = entry->d_name;
    if (name != "." && name != "..") {
      entries.push_back(name);
    }
  }
  const int read_errno = errno;
  EXPECT_EQ(::closedir(directory), 0);
  EXPECT_EQ(read_errno, 0);
  std::sort(entries.begin(), entries.end());
  return entries;
}

void remove_tree_no_follow(const std::string & path)
{
  struct stat status {};
  if (::lstat(path.c_str(), &status) != 0) {
    return;
  }
  if (!S_ISDIR(status.st_mode)) {
    if (S_ISREG(status.st_mode)) {
      (void)::chmod(path.c_str(), 0600);
    }
    (void)::unlink(path.c_str());
    return;
  }

  (void)::chmod(path.c_str(), 0700);
  DIR * directory = ::opendir(path.c_str());
  if (directory) {
    while (const dirent * entry = ::readdir(directory)) {
      const std::string name = entry->d_name;
      if (name != "." && name != "..") {
        remove_tree_no_follow(path + "/" + name);
      }
    }
    (void)::closedir(directory);
  }
  (void)::rmdir(path.c_str());
}

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    char path[] = "/tmp/h1_door_certificate_directory_test.XXXXXX";
    const char * created = ::mkdtemp(path);
    if (created) {
      root = created;
    }
  }

  ~TemporaryDirectory()
  {
    if (!root.empty()) {
      remove_tree_no_follow(root);
    }
  }

  TemporaryDirectory(const TemporaryDirectory &) = delete;
  TemporaryDirectory & operator=(const TemporaryDirectory &) = delete;

  std::string child(const std::string & name) const
  {
    return root + "/" + name;
  }

  std::string root;
};

void expect_certificate_bitwise_equal(
  const certificate::PathCertificate & expected,
  const certificate::PathCertificate & actual)
{
  EXPECT_EQ(actual.selected_anchor_index, expected.selected_anchor_index);
  EXPECT_EQ(
    std::memcmp(
      actual.outer_anchor.data(), expected.outer_anchor.data(),
      sizeof(certificate::ArmState)),
    0);
  ASSERT_EQ(actual.ingress.size(), expected.ingress.size());
  ASSERT_EQ(actual.coarse.size(), expected.coarse.size());
  ASSERT_EQ(actual.exact.size(), expected.exact.size());
  const auto expect_states = [](
    const std::vector<certificate::ArmState> & expected_states,
    const std::vector<certificate::ArmState> & actual_states) {
      for (std::size_t index = 0U; index < expected_states.size(); ++index) {
        EXPECT_EQ(
          std::memcmp(
            actual_states[index].data(), expected_states[index].data(),
            sizeof(certificate::ArmState)),
          0) << index;
      }
    };
  expect_states(expected.ingress, actual.ingress);
  expect_states(expected.coarse, actual.coarse);
  expect_states(expected.exact, actual.exact);
}

class TemporaryCertificateFiles
{
public:
  TemporaryCertificateFiles()
  {
    char path[] = "/tmp/h1_door_certificate_test.XXXXXX";
    const char * created = ::mkdtemp(path);
    if (created) {
      directory = created;
      manifest = directory + "/manifest.txt";
      payload = directory + "/states.bin";
      manifest_link = directory + "/manifest-link.txt";
    }
  }

  ~TemporaryCertificateFiles()
  {
    if (!manifest_link.empty()) {
      ::unlink(manifest_link.c_str());
    }
    if (!manifest.empty()) {
      ::unlink(manifest.c_str());
    }
    if (!payload.empty()) {
      ::unlink(payload.c_str());
    }
    if (!directory.empty()) {
      ::rmdir(directory.c_str());
    }
  }

  std::string directory;
  std::string manifest;
  std::string payload;
  std::string manifest_link;
};

TEST(DoorHandoffPathCertificate, Sha256KnownVectors)
{
  const std::vector<std::uint8_t> empty;
  const std::vector<std::uint8_t> abc{'a', 'b', 'c'};
  EXPECT_EQ(
    certificate::sha256_hex(empty),
    "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(
    certificate::sha256_hex(abc),
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST(DoorHandoffPathCertificate, ValidSyntheticCertificatePasses)
{
  const auto payload = valid_payload(4U);
  const auto result = verify_with_expected(valid_manifest(payload, 4U), payload);
  ASSERT_TRUE(result.passed) << result.message;
  EXPECT_TRUE(result.expected_bindings_checked);
  EXPECT_EQ(result.certificate.selected_anchor_index, 4U);
  EXPECT_EQ(result.certificate.ingress.size(), 45U);
  EXPECT_EQ(result.certificate.coarse.size(), 73U);
  EXPECT_EQ(result.certificate.exact.size(), 613U);
}

TEST(DoorHandoffPathCertificate, FormatInspectionNeverClaimsBindings)
{
  const auto payload = valid_payload();
  const auto result =
    certificate::inspect_format_bytes(valid_manifest(payload), payload);
  ASSERT_TRUE(result.passed) << result.message;
  EXPECT_FALSE(result.expected_bindings_checked);
}

TEST(DoorHandoffPathCertificate, PayloadHashMismatchFailsClosed)
{
  auto payload = valid_payload();
  const std::string manifest = valid_manifest(payload);
  payload.back() ^= 0x01U;
  const auto result = verify_with_expected(manifest, payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("SHA-256"), std::string::npos);
}

TEST(DoorHandoffPathCertificate, MagicAndHeaderMismatchFailClosed)
{
  auto payload = valid_payload();
  payload.front() = 'X';
  auto result = verify_with_expected(valid_manifest(payload), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("magic"), std::string::npos);

  payload = valid_payload();
  set_u32(&payload, 8U + 5U * 4U, 44U);
  result = verify_with_expected(valid_manifest(payload), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("header"), std::string::npos);
}

TEST(DoorHandoffPathCertificate, TruncationAndTrailingBytesFailClosed)
{
  auto payload = valid_payload();
  payload.pop_back();
  auto result = verify_with_expected(valid_manifest(payload), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("length"), std::string::npos);

  payload = valid_payload();
  payload.push_back(0U);
  result = verify_with_expected(valid_manifest(payload), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("length"), std::string::npos);
}

TEST(DoorHandoffPathCertificate, NonFiniteValuesFailClosed)
{
  auto payload = valid_payload();
  set_double(&payload, kHeaderBytes, std::numeric_limits<double>::quiet_NaN());
  auto result = verify_with_expected(valid_manifest(payload), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("non-finite"), std::string::npos);

  payload = valid_payload();
  set_double(
    &payload, exact_state_offset(2U),
    std::numeric_limits<double>::infinity());
  result = verify_with_expected(valid_manifest(payload), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("non-finite"), std::string::npos);
}

TEST(DoorHandoffPathCertificate, ExactContinuityBoundariesPass)
{
  auto payload = valid_payload();
  set_double(&payload, ingress_state_offset(1U), 4.9 * kPi / 180.0);
  set_double(&payload, coarse_state_offset(1U), 4.9 * kPi / 180.0);
  set_double(&payload, exact_state_offset(1U), 0.49 * kPi / 180.0);
  const auto result = verify_with_expected(valid_manifest(payload), payload);
  EXPECT_TRUE(result.passed) << result.message;
}

TEST(DoorHandoffPathCertificate, EachContinuityGateFailsClosed)
{
  auto payload = valid_payload();
  set_double(&payload, ingress_state_offset(1U), 4.9001 * kPi / 180.0);
  auto result = verify_with_expected(valid_manifest(payload), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("ingress continuity"), std::string::npos);

  payload = valid_payload();
  set_double(&payload, coarse_state_offset(1U), 4.9001 * kPi / 180.0);
  result = verify_with_expected(valid_manifest(payload), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("coarse continuity"), std::string::npos);

  payload = valid_payload();
  set_double(&payload, exact_state_offset(1U), 0.4901 * kPi / 180.0);
  result = verify_with_expected(valid_manifest(payload), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("exact continuity"), std::string::npos);
}

TEST(DoorHandoffPathCertificate, SegmentJunctionsFailClosed)
{
  auto payload = valid_payload();
  set_double(&payload, kHeaderBytes, 0.001);
  auto result = verify_with_expected(valid_manifest(payload), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("outer anchor"), std::string::npos);

  payload = valid_payload();
  for (std::size_t state = 0U; state < certificate::kCoarseStateCount; ++state) {
    set_double(&payload, coarse_state_offset(state), 4.9001 * kPi / 180.0);
  }
  result = verify_with_expected(valid_manifest(payload), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("ingress contact to coarse"), std::string::npos);

  payload = valid_payload();
  for (std::size_t state = 0U; state < certificate::kExactStateCount; ++state) {
    set_double(&payload, exact_state_offset(state), 0.4901 * kPi / 180.0);
  }
  result = verify_with_expected(valid_manifest(payload), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("ingress contact to exact"), std::string::npos);

  payload = valid_payload();
  for (std::size_t state = 0U; state < certificate::kCoarseStateCount; ++state) {
    set_double(&payload, coarse_state_offset(state), 4.9 * kPi / 180.0);
  }
  for (std::size_t state = 0U; state < certificate::kExactStateCount; ++state) {
    set_double(&payload, exact_state_offset(state), -0.49 * kPi / 180.0);
  }
  result = verify_with_expected(valid_manifest(payload), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("coarse start to exact"), std::string::npos);
}

TEST(DoorHandoffPathCertificate, ManifestShapeAndContractFailClosed)
{
  const auto payload = valid_payload();
  const std::string valid = valid_manifest(payload);

  auto result = verify_with_expected(
    replace_value(valid, "joint_count", "8"), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("contract mismatch"), std::string::npos);

  result = verify_with_expected(
    replace_value(
      valid, "joint_names",
      "right_shoulder_roll_joint,right_shoulder_pitch_joint,x,x,x,x,x"),
    payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("joint_names"), std::string::npos);

  result = verify_with_expected(
    replace_value(valid, "selected_anchor_index", "13"), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("anchor"), std::string::npos);

  result = verify_with_expected(
    replace_value(valid, "partial_allowed", "true"), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("partial_allowed"), std::string::npos);
}

TEST(DoorHandoffPathCertificate, ManifestHashesFailClosed)
{
  const auto payload = valid_payload();
  const std::string valid = valid_manifest(payload);
  auto result = verify_with_expected(
    replace_value(valid, "source_sha256", std::string(64U, '0')), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("non-canonical"), std::string::npos);

  result = verify_with_expected(
    replace_value(valid, "source_sha256", std::string(64U, 'f')), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("binding mismatch"), std::string::npos);

  result = verify_with_expected(
    replace_value(valid, "original_handle_stl_sha256", std::string(64U, 'a')),
    payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("original handle STL"), std::string::npos);
}

TEST(DoorHandoffPathCertificate, ExpectedBindingsMustBeExactAndComplete)
{
  const auto payload = valid_payload();
  const std::string manifest = valid_manifest(payload);
  auto bindings = valid_expected_bindings();
  bindings.sha256_by_field.erase("urdf_sha256");
  auto result = certificate::verify_bytes(manifest, payload, bindings);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("field count"), std::string::npos);

  const auto valid_bindings = valid_expected_bindings();
  for (const auto & [field, expected] : valid_bindings.sha256_by_field) {
    bindings = valid_bindings;
    const char replacement = expected.front() == 'f' ? 'e' : 'f';
    bindings.sha256_by_field.at(field) = std::string(64U, replacement);
    result = certificate::verify_bytes(manifest, payload, bindings);
    EXPECT_FALSE(result.passed) << field;
    EXPECT_NE(result.message.find("binding mismatch"), std::string::npos) << field;
  }

  bindings = valid_expected_bindings();
  bindings.sha256_by_field.erase("urdf_sha256");
  bindings.sha256_by_field.emplace("payload_sha256", std::string(64U, 'a'));
  result = certificate::verify_bytes(manifest, payload, bindings);
  EXPECT_FALSE(result.passed);
}

TEST(DoorHandoffPathCertificate, DuplicateUnknownAndMissingFieldsFailClosed)
{
  const auto payload = valid_payload();
  const std::string valid = valid_manifest(payload);
  auto result = verify_with_expected(valid + "joint_count=7\n", payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("duplicated"), std::string::npos);

  result = verify_with_expected(valid + "unknown=1\n", payload);
  EXPECT_FALSE(result.passed);

  std::string missing = valid;
  const std::string line = "full_reverse_required=true\n";
  const std::size_t position = missing.find(line);
  ASSERT_NE(position, std::string::npos);
  missing.erase(position, line.size());
  result = verify_with_expected(missing, payload);
  EXPECT_FALSE(result.passed);
}

TEST(DoorHandoffPathCertificate, PayloadAndManifestAnchorMustMatch)
{
  const auto payload = valid_payload(7U);
  const auto result = verify_with_expected(valid_manifest(payload, 6U), payload);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("header"), std::string::npos);
}

TEST(DoorHandoffPathCertificate, FileVerifierRejectsSymlinksAndDirectories)
{
  TemporaryCertificateFiles files;
  ASSERT_FALSE(files.directory.empty());
  const auto payload = valid_payload(2U);
  const std::string manifest = valid_manifest(payload, 2U);
  {
    std::ofstream stream(files.manifest, std::ios::binary);
    ASSERT_TRUE(stream);
    stream.write(manifest.data(), static_cast<std::streamsize>(manifest.size()));
    ASSERT_TRUE(stream);
  }
  {
    std::ofstream stream(files.payload, std::ios::binary);
    ASSERT_TRUE(stream);
    stream.write(
      reinterpret_cast<const char *>(payload.data()),
      static_cast<std::streamsize>(payload.size()));
    ASSERT_TRUE(stream);
  }

  auto result = certificate::verify_files(
    files.manifest, files.payload, valid_expected_bindings());
  ASSERT_TRUE(result.passed) << result.message;

  ASSERT_EQ(::symlink(files.manifest.c_str(), files.manifest_link.c_str()), 0);
  result = certificate::verify_files(
    files.manifest_link, files.payload, valid_expected_bindings());
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.message.find("safely open"), std::string::npos);

  result = certificate::verify_files(
    files.directory, files.payload, valid_expected_bindings());
  EXPECT_FALSE(result.passed);
}

TEST(
  DoorHandoffPathCertificate,
  WriteNewDirectoryRoundTripsBitExactlyAndDeterministically)
{
  TemporaryDirectory temporary;
  ASSERT_FALSE(temporary.root.empty());
  const auto path = synthetic_path_certificate();
  const auto bindings = valid_expected_bindings();
  const std::string first_directory = temporary.child("certificate-a");
  const std::string second_directory = temporary.child("certificate-b");

  const auto first_write =
    certificate::write_new_directory(first_directory, path, bindings);
  ASSERT_TRUE(first_write.passed) << first_write.message;
  EXPECT_EQ(
    first_write.state, certificate::PublishState::kDurablyPublished);
  const auto first_verification =
    certificate::verify_directory(first_directory, bindings);
  ASSERT_TRUE(first_verification.passed) << first_verification.message;
  EXPECT_TRUE(first_verification.expected_bindings_checked);
  expect_certificate_bitwise_equal(path, first_verification.certificate);

  const auto expected_payload = expected_payload_bytes(path);
  const auto first_payload =
    read_binary_file(first_directory + "/states.bin");
  EXPECT_EQ(first_payload, expected_payload);
  const std::string first_manifest =
    read_text_file(first_directory + "/manifest.txt");
  ASSERT_FALSE(first_manifest.empty());

  const auto second_write =
    certificate::write_new_directory(second_directory, path, bindings);
  ASSERT_TRUE(second_write.passed) << second_write.message;
  EXPECT_EQ(
    second_write.state, certificate::PublishState::kDurablyPublished);
  EXPECT_EQ(
    read_binary_file(second_directory + "/states.bin"), first_payload);
  EXPECT_EQ(
    read_text_file(second_directory + "/manifest.txt"), first_manifest);

  EXPECT_EQ(
    directory_entries(first_directory),
    (std::vector<std::string>{"manifest.txt", "states.bin"}));
  EXPECT_EQ(
    directory_entries(second_directory),
    (std::vector<std::string>{"manifest.txt", "states.bin"}));
  EXPECT_EQ(
    directory_entries(temporary.root),
    (std::vector<std::string>{"certificate-a", "certificate-b"}));

  for (const std::string & immutable_path : {
      first_directory,
      first_directory + "/manifest.txt",
      first_directory + "/states.bin"})
  {
    struct stat status {};
    ASSERT_EQ(::lstat(immutable_path.c_str(), &status), 0) << immutable_path;
    EXPECT_EQ(status.st_mode & (S_IWUSR | S_IWGRP | S_IWOTH), 0U) <<
      immutable_path;
  }
}

TEST(
  DoorHandoffPathCertificate,
  WriteNewDirectoryRejectsExistingFileDirectoryAndSymlink)
{
  TemporaryDirectory temporary;
  ASSERT_FALSE(temporary.root.empty());
  const auto path = synthetic_path_certificate();
  const auto bindings = valid_expected_bindings();

  const std::string file_target = temporary.child("existing-file");
  write_text_file(file_target, "sentinel");
  auto result = certificate::write_new_directory(file_target, path, bindings);
  EXPECT_FALSE(result.passed);
  EXPECT_EQ(result.state, certificate::PublishState::kNotPublished);
  EXPECT_EQ(read_text_file(file_target), "sentinel");

  const std::string directory_target = temporary.child("existing-directory");
  ASSERT_EQ(::mkdir(directory_target.c_str(), 0700), 0);
  result = certificate::write_new_directory(directory_target, path, bindings);
  EXPECT_FALSE(result.passed);
  EXPECT_EQ(result.state, certificate::PublishState::kNotPublished);
  struct stat directory_status {};
  ASSERT_EQ(::lstat(directory_target.c_str(), &directory_status), 0);
  EXPECT_TRUE(S_ISDIR(directory_status.st_mode));
  EXPECT_TRUE(directory_entries(directory_target).empty());

  const std::string link_target = temporary.child("existing-link");
  ASSERT_EQ(::symlink(directory_target.c_str(), link_target.c_str()), 0);
  result = certificate::write_new_directory(link_target, path, bindings);
  EXPECT_FALSE(result.passed);
  EXPECT_EQ(result.state, certificate::PublishState::kNotPublished);
  struct stat link_status {};
  ASSERT_EQ(::lstat(link_target.c_str(), &link_status), 0);
  EXPECT_TRUE(S_ISLNK(link_status.st_mode));
}

TEST(DoorHandoffPathCertificate, WriteNewDirectoryRejectsSymlinkedParent)
{
  TemporaryDirectory temporary;
  ASSERT_FALSE(temporary.root.empty());
  const std::string real_parent = temporary.child("real-parent");
  const std::string linked_parent = temporary.child("linked-parent");
  ASSERT_EQ(::mkdir(real_parent.c_str(), 0700), 0);
  ASSERT_EQ(::symlink(real_parent.c_str(), linked_parent.c_str()), 0);

  const std::string target = linked_parent + "/certificate";
  const auto result = certificate::write_new_directory(
    target, synthetic_path_certificate(), valid_expected_bindings());
  EXPECT_FALSE(result.passed);
  EXPECT_EQ(result.state, certificate::PublishState::kNotPublished);
  struct stat status {};
  errno = 0;
  EXPECT_EQ(::lstat((real_parent + "/certificate").c_str(), &status), -1);
  EXPECT_EQ(errno, ENOENT);
}

TEST(DoorHandoffPathCertificate, WriteNewDirectoryRejectsUnsafePathTable)
{
  enum class PathKind
  {
    kRelative,
    kTrailingSlash,
    kDotComponent,
    kDotDotComponent,
    kDoubleSlash,
  };
  struct PathCase
  {
    const char * label;
    PathKind kind;
    const char * expected_message;
  };
  const std::array<PathCase, 5> cases{{
    {"relative", PathKind::kRelative, "canonical and absolute"},
    {"trailing slash", PathKind::kTrailingSlash, "canonical and absolute"},
    {"dot component", PathKind::kDotComponent, "unsafe component"},
    {"dot-dot component", PathKind::kDotDotComponent, "unsafe component"},
    {"double slash", PathKind::kDoubleSlash, "canonical and absolute"},
  }};

  for (const auto & test_case : cases) {
    SCOPED_TRACE(test_case.label);
    TemporaryDirectory temporary;
    ASSERT_FALSE(temporary.root.empty());
    const std::string subdirectory = temporary.child("subdirectory");
    ASSERT_EQ(::mkdir(subdirectory.c_str(), 0700), 0);

    std::string target;
    switch (test_case.kind) {
      case PathKind::kRelative:
        target = temporary.root.substr(1U) + "/relative-certificate";
        break;
      case PathKind::kTrailingSlash:
        target = temporary.child("trailing-certificate") + "/";
        break;
      case PathKind::kDotComponent:
        target = temporary.root + "/./dot-certificate";
        break;
      case PathKind::kDotDotComponent:
        target = subdirectory + "/../dot-dot-certificate";
        break;
      case PathKind::kDoubleSlash:
        target = temporary.root + "//double-slash-certificate";
        break;
    }

    const auto result = certificate::write_new_directory(
      target, synthetic_path_certificate(), valid_expected_bindings());
    EXPECT_FALSE(result.passed);
    EXPECT_EQ(result.state, certificate::PublishState::kNotPublished);
    EXPECT_NE(result.message.find(test_case.expected_message), std::string::npos);
    EXPECT_EQ(
      directory_entries(temporary.root),
      (std::vector<std::string>{"subdirectory"}));
    EXPECT_TRUE(directory_entries(subdirectory).empty());
  }
}

TEST(DoorHandoffPathCertificate, WriteNewDirectoryRejectsWritableParentTable)
{
  struct PermissionCase
  {
    const char * label;
    mode_t mode;
  };
  const std::array<PermissionCase, 2> cases{{
    {"group writable", 0720},
    {"other writable", 0702},
  }};

  for (const auto & test_case : cases) {
    SCOPED_TRACE(test_case.label);
    TemporaryDirectory temporary;
    ASSERT_FALSE(temporary.root.empty());
    const std::string parent = temporary.child(test_case.label);
    ASSERT_EQ(::mkdir(parent.c_str(), 0700), 0);
    ASSERT_EQ(::chmod(parent.c_str(), test_case.mode), 0);

    const auto result = certificate::write_new_directory(
      parent + "/certificate", synthetic_path_certificate(),
      valid_expected_bindings());
    EXPECT_FALSE(result.passed);
    EXPECT_EQ(result.state, certificate::PublishState::kNotPublished);
    EXPECT_TRUE(directory_entries(parent).empty());
    ASSERT_EQ(::chmod(parent.c_str(), 0700), 0);
  }
}

TEST(DoorHandoffPathCertificate, WriteNewDirectoryInvalidCertificateDoesNotPublish)
{
  TemporaryDirectory temporary;
  ASSERT_FALSE(temporary.root.empty());
  auto invalid = synthetic_path_certificate();
  invalid.exact.pop_back();
  const std::string target = temporary.child("must-not-exist");

  const auto result = certificate::write_new_directory(
    target, invalid, valid_expected_bindings());
  EXPECT_FALSE(result.passed);
  EXPECT_EQ(result.state, certificate::PublishState::kNotPublished);
  struct stat status {};
  errno = 0;
  EXPECT_EQ(::lstat(target.c_str(), &status), -1);
  EXPECT_EQ(errno, ENOENT);
  EXPECT_TRUE(directory_entries(temporary.root).empty());
}

TEST(
  DoorHandoffPathCertificate,
  VerifyDirectoryRejectsExtraWritableAndSymlinkEntries)
{
  TemporaryDirectory temporary;
  ASSERT_FALSE(temporary.root.empty());
  const auto bindings = valid_expected_bindings();
  const std::string target = temporary.child("certificate");
  const auto write = certificate::write_new_directory(
    target, synthetic_path_certificate(), bindings);
  ASSERT_TRUE(write.passed) << write.message;

  ASSERT_EQ(::chmod(target.c_str(), 0700), 0);
  const std::string extra = target + "/unexpected";
  write_text_file(extra, "unexpected", 0400);
  ASSERT_EQ(::chmod(target.c_str(), 0500), 0);
  auto verification = certificate::verify_directory(target, bindings);
  EXPECT_FALSE(verification.passed);
  ASSERT_EQ(::chmod(target.c_str(), 0700), 0);
  ASSERT_EQ(::unlink(extra.c_str()), 0);
  ASSERT_EQ(::chmod(target.c_str(), 0500), 0);
  verification = certificate::verify_directory(target, bindings);
  ASSERT_TRUE(verification.passed) << verification.message;

  const std::string manifest = target + "/manifest.txt";
  ASSERT_EQ(::chmod(manifest.c_str(), 0600), 0);
  verification = certificate::verify_directory(target, bindings);
  EXPECT_FALSE(verification.passed);
  ASSERT_EQ(::chmod(manifest.c_str(), 0400), 0);
  verification = certificate::verify_directory(target, bindings);
  ASSERT_TRUE(verification.passed) << verification.message;

  const std::string manifest_copy = temporary.child("manifest-copy.txt");
  write_text_file(manifest_copy, read_text_file(manifest), 0400);
  ASSERT_EQ(::chmod(target.c_str(), 0700), 0);
  ASSERT_EQ(::unlink(manifest.c_str()), 0);
  ASSERT_EQ(::symlink(manifest_copy.c_str(), manifest.c_str()), 0);
  ASSERT_EQ(::chmod(target.c_str(), 0500), 0);
  verification = certificate::verify_directory(target, bindings);
  EXPECT_FALSE(verification.passed);
}

TEST(DoorHandoffPathCertificate, VerifyDirectoryRejectsUnsafeMetadataTable)
{
  enum class Mutation
  {
    kWritablePayload,
    kExternalManifestHardLink,
  };
  struct MutationCase
  {
    const char * label;
    Mutation mutation;
  };
  const std::array<MutationCase, 2> cases{{
    {"writable states.bin", Mutation::kWritablePayload},
    {"external manifest hard link", Mutation::kExternalManifestHardLink},
  }};

  for (const auto & test_case : cases) {
    SCOPED_TRACE(test_case.label);
    TemporaryDirectory temporary;
    ASSERT_FALSE(temporary.root.empty());
    const auto bindings = valid_expected_bindings();
    const std::string target = temporary.child("certificate");
    const auto write = certificate::write_new_directory(
      target, synthetic_path_certificate(), bindings);
    ASSERT_TRUE(write.passed) << write.message;

    const std::string manifest = target + "/manifest.txt";
    const std::string payload = target + "/states.bin";
    const std::string external_link = temporary.child("manifest-hard-link.txt");
    switch (test_case.mutation) {
      case Mutation::kWritablePayload:
        ASSERT_EQ(::chmod(payload.c_str(), 0600), 0);
        break;
      case Mutation::kExternalManifestHardLink:
        ASSERT_EQ(::link(manifest.c_str(), external_link.c_str()), 0);
        struct stat status {};
        ASSERT_EQ(::lstat(manifest.c_str(), &status), 0);
        ASSERT_EQ(status.st_nlink, 2);
        break;
    }

    const auto verification = certificate::verify_directory(target, bindings);
    EXPECT_FALSE(verification.passed);

    switch (test_case.mutation) {
      case Mutation::kWritablePayload:
        ASSERT_EQ(::chmod(payload.c_str(), 0400), 0);
        break;
      case Mutation::kExternalManifestHardLink:
        ASSERT_EQ(::unlink(external_link.c_str()), 0);
        break;
    }
    const auto restored = certificate::verify_directory(target, bindings);
    ASSERT_TRUE(restored.passed) << restored.message;
  }
}

TEST(DoorHandoffPathCertificate, Sha256RegularFileKnownVectorAndRejectsSymlink)
{
  TemporaryDirectory temporary;
  ASSERT_FALSE(temporary.root.empty());
  const std::string regular = temporary.child("abc.txt");
  const std::string link = temporary.child("abc-link.txt");
  write_text_file(regular, "abc", 0400);

  const auto hash = certificate::sha256_regular_file(regular);
  ASSERT_TRUE(hash.passed) << hash.message;
  EXPECT_EQ(hash.size_bytes, 3U);
  EXPECT_EQ(
    hash.sha256,
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");

  ASSERT_EQ(::symlink(regular.c_str(), link.c_str()), 0);
  const auto linked_hash = certificate::sha256_regular_file(link);
  EXPECT_FALSE(linked_hash.passed);
}

TEST(DoorHandoffPathCertificate, Sha256SelfExecutableUsesStableRunningInode)
{
  const auto hash = certificate::sha256_self_executable();
  ASSERT_TRUE(hash.passed) << hash.message;
  ASSERT_GT(hash.size_bytes, 0U);
  EXPECT_EQ(hash.sha256.size(), 64U);

  std::ifstream stream("/proc/self/exe", std::ios::binary);
  ASSERT_TRUE(stream.good());
  const std::vector<std::uint8_t> bytes{
    std::istreambuf_iterator<char>(stream),
    std::istreambuf_iterator<char>()};
  ASSERT_EQ(bytes.size(), hash.size_bytes);
  EXPECT_EQ(certificate::sha256_hex(bytes), hash.sha256);

  const auto repeat = certificate::sha256_self_executable();
  ASSERT_TRUE(repeat.passed) << repeat.message;
  EXPECT_EQ(repeat.size_bytes, hash.size_bytes);
  EXPECT_EQ(repeat.sha256, hash.sha256);

  const auto exact_limit = certificate::sha256_self_executable(hash.size_bytes);
  ASSERT_TRUE(exact_limit.passed) << exact_limit.message;
  EXPECT_EQ(exact_limit.sha256, hash.sha256);

  const auto too_small = certificate::sha256_self_executable(
    hash.size_bytes - 1U);
  EXPECT_FALSE(too_small.passed);
}

}  // namespace
