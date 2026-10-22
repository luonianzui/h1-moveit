#ifndef H1_2_MOVEIT_DEMO__DOOR_HANDOFF_BINDING_HASH_HPP_
#define H1_2_MOVEIT_DEMO__DOOR_HANDOFF_BINDING_HASH_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace h1_2_moveit_demo::door_handoff_path_certificate
{

constexpr std::size_t kMaximumBindingBundleEntries = 4096U;
constexpr std::size_t kMaximumBindingLogicalIdBytes = 4096U;
constexpr std::uint64_t kMaximumBindingEntryBytes = 256U * 1024U * 1024U;
constexpr std::uint64_t kMaximumBindingEnvelopeBytes = 1024U * 1024U * 1024U;

struct BindingHashEntry
{
  // A stable semantic identifier, such as a package URI or canonical
  // parameter name.  It is part of the hash; filesystem aliases are not.
  std::string logical_id;
  // Exact bytes used to construct the model, scene, state, or policy.
  std::vector<std::uint8_t> contents;
};

struct BindingBundleHashResult
{
  bool passed{false};
  std::string message;
  std::string sha256;
  std::uint64_t raw_size_bytes{0U};
  std::uint64_t envelope_size_bytes{0U};
  std::size_t entry_count{0U};
};

// Hash one of the composite certificate binding fields using a fixed,
// domain-separated, length-prefixed binary envelope.  Entries are sorted by
// logical_id before hashing, so caller iteration order cannot affect the
// digest.  The six direct-file fields (source, executable, and the four
// original STL files) intentionally remain raw-byte SHA-256 values; ordinary
// files use sha256_regular_file(), while the running executable requires
// sha256_self_executable().  This preserves the pinned original-handle hash.
BindingBundleHashResult sha256_binding_bundle(
  const std::string & binding_field,
  const std::vector<BindingHashEntry> & entries);

}  // namespace h1_2_moveit_demo::door_handoff_path_certificate

#endif  // H1_2_MOVEIT_DEMO__DOOR_HANDOFF_BINDING_HASH_HPP_
