#include "h1_2_moveit_demo/door_handoff_binding_hash.hpp"

#include "h1_2_moveit_demo/door_handoff_binding_fields.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace binding =
  h1_2_moveit_demo::door_handoff_path_certificate;

namespace
{

binding::BindingHashEntry entry(
  const std::string & logical_id,
  const std::vector<std::uint8_t> & contents)
{
  return binding::BindingHashEntry{logical_id, contents};
}

std::vector<binding::BindingHashEntry> known_entries()
{
  return {
    entry("move_group/robot_description", {'<', 'r', '/', '>'}),
    entry(
      "robot_state_publisher/robot_description",
      {0x00U, 0xffU, 0x10U}),
  };
}

TEST(DoorHandoffBindingHash, KnownEnvelopeVector)
{
  const auto result = binding::sha256_binding_bundle(
    "urdf_sha256", known_entries());
  ASSERT_TRUE(result.passed) << result.message;
  EXPECT_EQ(result.entry_count, 2U);
  EXPECT_EQ(result.raw_size_bytes, 7U);
  EXPECT_EQ(result.envelope_size_bytes, 141U);
  EXPECT_EQ(
    result.sha256,
    "136989c79c59b23757c53aa3eae2860dd898601469d16b1b225b9b3ebdf62e17");
}

TEST(DoorHandoffBindingHash, CallerOrderDoesNotAffectDigest)
{
  auto reversed = known_entries();
  std::reverse(reversed.begin(), reversed.end());
  const auto first = binding::sha256_binding_bundle(
    "urdf_sha256", known_entries());
  const auto second = binding::sha256_binding_bundle(
    "urdf_sha256", reversed);
  ASSERT_TRUE(first.passed) << first.message;
  ASSERT_TRUE(second.passed) << second.message;
  EXPECT_EQ(first.sha256, second.sha256);
  EXPECT_EQ(first.envelope_size_bytes, second.envelope_size_bytes);
}

TEST(DoorHandoffBindingHash, FieldAndEntryBoundariesAreSeparated)
{
  const auto urdf = binding::sha256_binding_bundle(
    "urdf_sha256", known_entries());
  const auto srdf = binding::sha256_binding_bundle(
    "srdf_sha256", known_entries());
  const auto first_boundary = binding::sha256_binding_bundle(
    "urdf_sha256", {entry("a", {'b', 'c'})});
  const auto second_boundary = binding::sha256_binding_bundle(
    "urdf_sha256", {entry("ab", {'c'})});
  ASSERT_TRUE(urdf.passed);
  ASSERT_TRUE(srdf.passed);
  ASSERT_TRUE(first_boundary.passed);
  ASSERT_TRUE(second_boundary.passed);
  EXPECT_NE(urdf.sha256, srdf.sha256);
  EXPECT_NE(first_boundary.sha256, second_boundary.sha256);
}

TEST(DoorHandoffBindingHash, IdentifierAndContentMutationsChangeDigest)
{
  const auto baseline = binding::sha256_binding_bundle(
    "urdf_sha256", {entry("component-a", {0x00U, 0xffU})});
  const auto changed_identifier = binding::sha256_binding_bundle(
    "urdf_sha256", {entry("component-b", {0x00U, 0xffU})});
  const auto changed_content = binding::sha256_binding_bundle(
    "urdf_sha256", {entry("component-a", {0x00U, 0xfeU})});
  ASSERT_TRUE(baseline.passed);
  ASSERT_TRUE(changed_identifier.passed);
  ASSERT_TRUE(changed_content.passed);
  EXPECT_NE(baseline.sha256, changed_identifier.sha256);
  EXPECT_NE(baseline.sha256, changed_content.sha256);
}

TEST(DoorHandoffBindingHash, EntryCountAndFramingChangeDigest)
{
  const auto one_entry = binding::sha256_binding_bundle(
    "urdf_sha256", {entry("a", {'x', 'y'})});
  const auto two_entries = binding::sha256_binding_bundle(
    "urdf_sha256", {entry("a", {'x'}), entry("b", {'y'})});
  ASSERT_TRUE(one_entry.passed);
  ASSERT_TRUE(two_entries.passed);
  EXPECT_NE(one_entry.sha256, two_entries.sha256);
}

TEST(DoorHandoffBindingHash, AllFixedCompositeFieldsAreAccepted)
{
  std::vector<std::string> digests;
  digests.reserve(binding::binding_field_count(
      binding::BindingFieldKind::kCompositeBundle));
  for (const auto & field : binding::kBindingFieldRegistry) {
    if (field.kind != binding::BindingFieldKind::kCompositeBundle) {
      continue;
    }
    const auto result = binding::sha256_binding_bundle(
      std::string(field.name), {entry("canonical", {0x01U})});
    EXPECT_TRUE(result.passed) << field.name << ": " << result.message;
    digests.push_back(result.sha256);
  }
  ASSERT_EQ(digests.size(), 12U);
  for (std::size_t left = 0U; left < digests.size(); ++left) {
    for (std::size_t right = left + 1U; right < digests.size(); ++right) {
      EXPECT_NE(digests[left], digests[right]);
    }
  }
}

TEST(DoorHandoffBindingHash, DirectFileAndUnknownFieldsAreRejected)
{
  std::size_t direct_count = 0U;
  for (const auto & field : binding::kBindingFieldRegistry) {
    if (field.kind != binding::BindingFieldKind::kDirectFile) {
      continue;
    }
    ++direct_count;
    const auto result = binding::sha256_binding_bundle(
      std::string(field.name), {entry("canonical", {0x01U})});
    EXPECT_FALSE(result.passed) << field.name;
  }
  EXPECT_EQ(direct_count, 6U);
  for (const char * field : {"payload_sha256", "unknown_sha256"}) {
    const auto result = binding::sha256_binding_bundle(
      field, {entry("canonical", {0x01U})});
    EXPECT_FALSE(result.passed) << field;
  }
}

TEST(DoorHandoffBindingHash, EmptyDuplicateAndNonCanonicalInputsFailClosed)
{
  EXPECT_FALSE(binding::sha256_binding_bundle("urdf_sha256", {}).passed);

  const auto duplicate = binding::sha256_binding_bundle(
    "urdf_sha256",
    {entry("same", {0x01U}), entry("same", {0x02U})});
  EXPECT_FALSE(duplicate.passed);

  for (const std::string & logical_id : {
      std::string(), std::string("has space"), std::string("line\nbreak"),
      std::string("nul\0byte", 8U),
      std::string(binding::kMaximumBindingLogicalIdBytes + 1U, 'x')})
  {
    const auto result = binding::sha256_binding_bundle(
      "urdf_sha256", {entry(logical_id, {0x01U})});
    EXPECT_FALSE(result.passed) << logical_id.size();
  }

  const auto empty_contents = binding::sha256_binding_bundle(
    "urdf_sha256", {entry("canonical", {})});
  EXPECT_FALSE(empty_contents.passed);
}

TEST(DoorHandoffBindingHash, EntryCountLimitFailsBeforeHashing)
{
  std::vector<binding::BindingHashEntry> entries;
  entries.reserve(binding::kMaximumBindingBundleEntries + 1U);
  for (std::size_t index = 0U;
    index < binding::kMaximumBindingBundleEntries + 1U; ++index)
  {
    entries.push_back(entry("entry-" + std::to_string(index), {0x01U}));
  }
  const auto result = binding::sha256_binding_bundle(
    "urdf_sha256", entries);
  EXPECT_FALSE(result.passed);
}

TEST(DoorHandoffBindingHash, ExactIdentifierAndEntryCountLimitsPass)
{
  const auto maximum_identifier = binding::sha256_binding_bundle(
    "urdf_sha256",
    {entry(
        std::string(binding::kMaximumBindingLogicalIdBytes, 'x'),
        {0x01U})});
  ASSERT_TRUE(maximum_identifier.passed) << maximum_identifier.message;

  std::vector<binding::BindingHashEntry> entries;
  entries.reserve(binding::kMaximumBindingBundleEntries);
  for (std::size_t index = 0U;
    index < binding::kMaximumBindingBundleEntries; ++index)
  {
    entries.push_back(entry("entry-" + std::to_string(index), {0x01U}));
  }
  const auto maximum_entry_count = binding::sha256_binding_bundle(
    "urdf_sha256", entries);
  ASSERT_TRUE(maximum_entry_count.passed) << maximum_entry_count.message;
  EXPECT_EQ(
    maximum_entry_count.entry_count,
    binding::kMaximumBindingBundleEntries);
}

TEST(DoorHandoffBindingHash, LogicalIdentifierAsciiBoundariesAreCanonical)
{
  for (const std::string & logical_id : {
      std::string(1U, static_cast<char>(0x21U)),
      std::string(1U, static_cast<char>(0x7eU))})
  {
    const auto result = binding::sha256_binding_bundle(
      "urdf_sha256", {entry(logical_id, {0x01U})});
    EXPECT_TRUE(result.passed) << result.message;
  }
  for (const std::string & logical_id : {
      std::string(1U, static_cast<char>(0x20U)),
      std::string(1U, static_cast<char>(0x7fU)),
      std::string(1U, static_cast<char>(0x80U))})
  {
    const auto result = binding::sha256_binding_bundle(
      "urdf_sha256", {entry(logical_id, {0x01U})});
    EXPECT_FALSE(result.passed);
  }
}

}  // namespace
