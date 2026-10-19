// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

// PDO layout (where each mapped object sits in a drive's process data) and
// little-endian encoding/decoding of values. EtherCAT process data is always
// little-endian, independent of the host.

#ifndef IRD_ETHERCAT_HARDWARE_INTERFACE__PDO_HPP_
#define IRD_ETHERCAT_HARDWARE_INTERFACE__PDO_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "ird_ethercat_hardware_interface/config.hpp"

namespace ird_ethercat_hardware_interface
{

/// Byte offset of every entry of a PDO, in mapping order, and its total size.
struct PdoLayout
{
  std::vector<size_t> offsets;
  size_t size_bytes{0};
};

PdoLayout compute_layout(const std::vector<PdoEntry> & entries);

/// Value written to a PDO mapping object subindex: 0xIIIISSLL
/// (object index, subindex, length in bits).
uint32_t mapping_value(const PdoEntry & entry);

/// Location of the entry that has a given role, if any.
struct RoleLocation
{
  bool mapped{false};
  size_t offset{0};
  DataType type{DataType::UINT8};
};

RoleLocation find_role(const std::vector<PdoEntry> & entries, const PdoLayout & layout, PdoRole role);

/// Reads a value of `type` at `p` (sign-extended for signed types).
int64_t read_value(const uint8_t * p, DataType type);

/// Clamps `value` to the range of `type`.
int64_t saturate(DataType type, int64_t value);

/// Writes the low size_of(type) bytes of `value` at `p`.
void write_value(uint8_t * p, DataType type, int64_t value);

}  // namespace ird_ethercat_hardware_interface

#endif  // IRD_ETHERCAT_HARDWARE_INTERFACE__PDO_HPP_
