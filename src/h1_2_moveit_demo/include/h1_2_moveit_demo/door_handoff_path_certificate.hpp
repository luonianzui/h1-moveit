#ifndef H1_2_MOVEIT_DEMO__DOOR_HANDOFF_PATH_CERTIFICATE_HPP_
#define H1_2_MOVEIT_DEMO__DOOR_HANDOFF_PATH_CERTIFICATE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace h1_2_moveit_demo::door_handoff_path_certificate
{

constexpr std::size_t kJointCount = 7U;
constexpr std::size_t kOuterAnchorStateCount = 1U;
constexpr std::size_t kIngressStateCount = 45U;
constexpr std::size_t kCoarseStateCount = 73U;
constexpr std::size_t kExactStateCount = 613U;
constexpr std::size_t kExecutionMovingTargetCount = 601U;
constexpr std::size_t kDenseInterpolationSampleCount = 2943U;
constexpr std::size_t kRedundancyPrepLayerCount = 12U;

using ArmState = std::array<double, kJointCount>;

struct PathCertificate
{
  std::size_t selected_anchor_index{0U};
  ArmState outer_anchor{};
  std::vector<ArmState> ingress;
  std::vector<ArmState> coarse;
  std::vector<ArmState> exact;
};

// The caller must compute these bindings from the live source/model/scene/
// parameter/start-state inputs.  The format library never invents trusted
// values and never promotes a format-only inspection to execution admission.
struct ExpectedBindings
{
  std::map<std::string, std::string> sha256_by_field;
};

struct VerificationResult
{
  bool passed{false};
  bool expected_bindings_checked{false};
  std::string message;
  PathCertificate certificate;
};

struct FileHashResult
{
  bool passed{false};
  std::string message;
  std::string sha256;
  std::uint64_t size_bytes{0U};
};

enum class PublishState
{
  kNotPublished,
  kPublishedDurabilityUncertain,
  kDurablyPublished,
};

struct WriteResult
{
  bool passed{false};
  PublishState state{PublishState::kNotPublished};
  std::string message;
};

std::string sha256_hex(const std::uint8_t * data, std::size_t size);

inline std::string sha256_hex(const std::vector<std::uint8_t> & data)
{
  return sha256_hex(data.data(), data.size());
}

VerificationResult inspect_format_bytes(
  const std::string & manifest,
  const std::vector<std::uint8_t> & payload);

// Legacy two-path helpers are format/test utilities only.  Execution loaders
// must use verify_directory() so both fixed files share one directory snapshot.
VerificationResult inspect_format_files(
  const std::string & manifest_path,
  const std::string & payload_path);

VerificationResult verify_bytes(
  const std::string & manifest,
  const std::vector<std::uint8_t> & payload,
  const ExpectedBindings & expected_bindings);

VerificationResult verify_files(
  const std::string & manifest_path,
  const std::string & payload_path,
  const ExpectedBindings & expected_bindings);

VerificationResult inspect_format_directory(
  const std::string & certificate_directory);

VerificationResult verify_directory(
  const std::string & certificate_directory,
  const ExpectedBindings & expected_bindings);

FileHashResult sha256_regular_file(
  const std::string & path,
  std::uint64_t maximum_size_bytes = 1024U * 1024U * 1024U);

// Hash the inode that this process is executing.  Content is opened exactly
// once through the Linux /proc/self/exe magic link and then read only from that
// descriptor; stat() before and after only confirms the link still names the
// same inode.  Callers must not resolve argv[0] or reopen a realpath because an
// on-disk replacement may differ from the running image.
FileHashResult sha256_self_executable(
  std::uint64_t maximum_size_bytes = 1024U * 1024U * 1024U);

// Publish a new read-only-mode sealed certificate directory containing exactly
// manifest.txt and states.bin.  Existing targets are never replaced.
WriteResult write_new_directory(
  const std::string & certificate_directory,
  const PathCertificate & certificate,
  const ExpectedBindings & expected_bindings);

}  // namespace h1_2_moveit_demo::door_handoff_path_certificate

#endif  // H1_2_MOVEIT_DEMO__DOOR_HANDOFF_PATH_CERTIFICATE_HPP_
