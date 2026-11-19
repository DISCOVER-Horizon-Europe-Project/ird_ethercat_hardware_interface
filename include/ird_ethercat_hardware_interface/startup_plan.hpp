// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

// The list of SDO writes that configures one drive in PRE-OP. Building it as
// data (instead of issuing the writes inline) lets the unit tests check the
// exact sequence sent to the drives without any hardware.

#ifndef IRD_ETHERCAT_HARDWARE_INTERFACE__STARTUP_PLAN_HPP_
#define IRD_ETHERCAT_HARDWARE_INTERFACE__STARTUP_PLAN_HPP_

#include <vector>

#include "ird_ethercat_hardware_interface/config.hpp"

namespace ird_ethercat_hardware_interface
{

// IRD synchronisation objects.
constexpr uint16_t OBJ_CYCLE_PERIOD = 0x1006;  // :00 communication cycle period [us]
constexpr uint16_t OBJ_SYNC_MODE = 0x2012;     // :01 synchronisation type
constexpr int64_t IRD_SYNC_TYPE_SOFT = 2;
constexpr int64_t IRD_SYNC_TYPE_HARD = 4;

/// SDO writes for one drive, in order. The mode of operation (0x6060) is not
/// included: it is read first and written only if different (see
/// EthercatMaster::configure_drive).
///
/// Order:
///  1. the drive's own startup_sdos from the configuration,
///  2. PDO mapping: clear the assignments (0x1C12/0x1C13) and mapping
///     objects, write each entry and the entry count, assign again. The
///     entries are split over several mapping objects when the drive's
///     pdo_limits require it (split_into_pdos),
///  3. synchronisation objects (0x1006, 0x2012), which the IRD latches at
///     the PRE-OP -> SAFE-OP transition, so they must be written now.
std::vector<SdoWrite> build_startup_plan(const DriveConfig & drive, const BusConfig & bus);

}  // namespace ird_ethercat_hardware_interface

#endif  // IRD_ETHERCAT_HARDWARE_INTERFACE__STARTUP_PLAN_HPP_
