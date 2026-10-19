// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

#include "ird_ethercat_hardware_interface/pdo.hpp"

#include <cstdint>

namespace ird_ethercat_hardware_interface
{

PdoLayout compute_layout(const std::vector<PdoEntry> & entries)
{
  // Entries are packed in mapping order with no padding: every supported
  // type is a whole number of bytes.
  PdoLayout layout;
  layout.offsets.reserve(entries.size());
  for (const PdoEntry & entry : entries)
  {
    layout.offsets.push_back(layout.size_bytes);
    layout.size_bytes += size_of(entry.type);
  }
  return layout;
}

uint32_t mapping_value(const PdoEntry & entry)
{
  return (static_cast<uint32_t>(entry.index) << 16) | (static_cast<uint32_t>(entry.subindex) << 8) |
         static_cast<uint32_t>(size_of(entry.type) * 8);
}

RoleLocation find_role(const std::vector<PdoEntry> & entries, const PdoLayout & layout, PdoRole role)
{
  for (size_t i = 0; i < entries.size(); ++i)
  {
    if (entries[i].role == role)
    {
      return {true, layout.offsets[i], entries[i].type};
    }
  }
  return {};
}

int64_t read_value(const uint8_t * p, DataType type)
{
  uint32_t raw = 0;
  for (size_t i = 0; i < size_of(type); ++i)
  {
    raw |= static_cast<uint32_t>(p[i]) << (8 * i);
  }
  switch (type)
  {
    case DataType::INT8: return static_cast<int8_t>(raw);
    case DataType::UINT8: return static_cast<uint8_t>(raw);
    case DataType::INT16: return static_cast<int16_t>(raw);
    case DataType::UINT16: return static_cast<uint16_t>(raw);
    case DataType::INT32: return static_cast<int32_t>(raw);
    case DataType::UINT32: return raw;
  }
  return 0;
}

int64_t saturate(DataType type, int64_t value)
{
  int64_t low = 0;
  int64_t high = 0;
  switch (type)
  {
    case DataType::INT8: low = INT8_MIN; high = INT8_MAX; break;
    case DataType::UINT8: low = 0; high = UINT8_MAX; break;
    case DataType::INT16: low = INT16_MIN; high = INT16_MAX; break;
    case DataType::UINT16: low = 0; high = UINT16_MAX; break;
    case DataType::INT32: low = INT32_MIN; high = INT32_MAX; break;
    case DataType::UINT32: low = 0; high = UINT32_MAX; break;
  }
  return value < low ? low : (value > high ? high : value);
}

void write_value(uint8_t * p, DataType type, int64_t value)
{
  const uint64_t raw = static_cast<uint64_t>(value);
  for (size_t i = 0; i < size_of(type); ++i)
  {
    p[i] = static_cast<uint8_t>((raw >> (8 * i)) & 0xFF);
  }
}

}  // namespace ird_ethercat_hardware_interface
