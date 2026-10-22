#include "h1_2_moveit_demo/door_handoff_binding_hash.hpp"

#include "h1_2_moveit_demo/door_handoff_binding_fields.hpp"

#include <rcutils/sha256.h>

#include <algorithm>
#include <array>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace h1_2_moveit_demo::door_handoff_path_certificate
{
namespace
{

constexpr std::array<std::uint8_t, 8U> kEnvelopeMagic{
  'H', '1', 'D', 'B', 'N', 'D', 'L', '1'};
constexpr std::uint32_t kEnvelopeSchemaVersion = 1U;
constexpr std::uint32_t kEntryMarker = 0x52544e45U;  // "ENTR" little-endian.
constexpr std::uint32_t kEndMarker = 0x21444e45U;  // "END!" little-endian.

bool is_composite_binding_field(const std::string & field)
{
  return binding_field_has_kind(field, BindingFieldKind::kCompositeBundle);
}

bool is_canonical_logical_id(const std::string & logical_id)
{
  if (logical_id.empty() ||
    logical_id.size() > kMaximumBindingLogicalIdBytes)
  {
    return false;
  }
  return std::all_of(
    logical_id.begin(), logical_id.end(), [](const unsigned char byte) {
      return byte >= 0x21U && byte <= 0x7eU;
    });
}

bool checked_add(
  std::uint64_t * total,
  const std::uint64_t amount)
{
  if (!total || amount > kMaximumBindingEnvelopeBytes ||
    *total > kMaximumBindingEnvelopeBytes - amount)
  {
    return false;
  }
  *total += amount;
  return true;
}

void update_u32(
  rcutils_sha256_ctx_t * context,
  const std::uint32_t value)
{
  std::array<std::uint8_t, 4U> bytes{};
  for (std::size_t index = 0U; index < bytes.size(); ++index) {
    bytes[index] = static_cast<std::uint8_t>(value >> (8U * index));
  }
  rcutils_sha256_update(context, bytes.data(), bytes.size());
}

void update_u64(
  rcutils_sha256_ctx_t * context,
  const std::uint64_t value)
{
  std::array<std::uint8_t, 8U> bytes{};
  for (std::size_t index = 0U; index < bytes.size(); ++index) {
    bytes[index] = static_cast<std::uint8_t>(value >> (8U * index));
  }
  rcutils_sha256_update(context, bytes.data(), bytes.size());
}

void update_bytes(
  rcutils_sha256_ctx_t * context,
  const std::uint8_t * bytes,
  const std::size_t size)
{
  if (size != 0U) {
    rcutils_sha256_update(context, bytes, size);
  }
}

std::string final_hex(rcutils_sha256_ctx_t * context)
{
  std::array<std::uint8_t, RCUTILS_SHA256_BLOCK_SIZE> digest{};
  rcutils_sha256_final(context, digest.data());
  std::ostringstream hexadecimal;
  hexadecimal << std::hex << std::setfill('0');
  for (const std::uint8_t byte : digest) {
    hexadecimal << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return hexadecimal.str();
}

BindingBundleHashResult failure(const std::string & message)
{
  BindingBundleHashResult result;
  result.message = message;
  return result;
}

}  // namespace

BindingBundleHashResult sha256_binding_bundle(
  const std::string & binding_field,
  const std::vector<BindingHashEntry> & entries)
{
  if (!is_composite_binding_field(binding_field)) {
    return failure(
      "binding field is not one of the fixed composite certificate fields");
  }
  if (entries.empty() || entries.size() > kMaximumBindingBundleEntries) {
    return failure("binding bundle entry count is invalid");
  }

  std::vector<const BindingHashEntry *> ordered;
  ordered.reserve(entries.size());
  std::uint64_t raw_size = 0U;
  std::uint64_t envelope_size =
    kEnvelopeMagic.size() + sizeof(std::uint32_t) + sizeof(std::uint32_t) +
    binding_field.size() + sizeof(std::uint32_t) + sizeof(std::uint32_t);
  for (const auto & entry : entries) {
    if (!is_canonical_logical_id(entry.logical_id)) {
      return failure("binding bundle logical id is empty or non-canonical");
    }
    if (entry.contents.empty() ||
      entry.contents.size() > kMaximumBindingEntryBytes)
    {
      return failure("binding bundle entry contents size is invalid");
    }
    const std::uint64_t entry_overhead =
      sizeof(std::uint32_t) + sizeof(std::uint32_t) +
      entry.logical_id.size() + sizeof(std::uint64_t);
    if (!checked_add(&envelope_size, entry_overhead) ||
      !checked_add(&envelope_size, entry.contents.size()))
    {
      return failure("binding bundle envelope exceeds its fixed size limit");
    }
    raw_size += static_cast<std::uint64_t>(entry.contents.size());
    ordered.push_back(&entry);
  }
  std::sort(
    ordered.begin(), ordered.end(),
    [](const BindingHashEntry * left, const BindingHashEntry * right) {
      return left->logical_id < right->logical_id;
    });
  for (std::size_t index = 1U; index < ordered.size(); ++index) {
    if (ordered[index - 1U]->logical_id == ordered[index]->logical_id) {
      return failure("binding bundle contains a duplicate logical id");
    }
  }

  rcutils_sha256_ctx_t context;
  rcutils_sha256_init(&context);
  update_bytes(&context, kEnvelopeMagic.data(), kEnvelopeMagic.size());
  update_u32(&context, kEnvelopeSchemaVersion);
  update_u32(&context, static_cast<std::uint32_t>(binding_field.size()));
  update_bytes(
    &context,
    reinterpret_cast<const std::uint8_t *>(binding_field.data()),
    binding_field.size());
  update_u32(&context, static_cast<std::uint32_t>(ordered.size()));
  for (const auto * entry : ordered) {
    update_u32(&context, kEntryMarker);
    update_u32(
      &context, static_cast<std::uint32_t>(entry->logical_id.size()));
    update_bytes(
      &context,
      reinterpret_cast<const std::uint8_t *>(entry->logical_id.data()),
      entry->logical_id.size());
    update_u64(&context, static_cast<std::uint64_t>(entry->contents.size()));
    update_bytes(
      &context, entry->contents.data(), entry->contents.size());
  }
  update_u32(&context, kEndMarker);

  BindingBundleHashResult result;
  result.passed = true;
  result.message =
    "composite binding SHA-256 passed fixed-envelope validation";
  result.sha256 = final_hex(&context);
  result.raw_size_bytes = raw_size;
  result.envelope_size_bytes = envelope_size;
  result.entry_count = ordered.size();
  return result;
}

}  // namespace h1_2_moveit_demo::door_handoff_path_certificate
