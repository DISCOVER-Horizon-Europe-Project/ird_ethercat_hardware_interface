// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

#include "ird_ethercat_hardware_interface/startup_plan.hpp"

#include "ird_ethercat_hardware_interface/pdo.hpp"

namespace ird_ethercat_hardware_interface
{

namespace
{
constexpr uint16_t OBJ_RXPDO_ASSIGN = 0x1C12;  // sync manager 2 (outputs) PDO assignment
constexpr uint16_t OBJ_TXPDO_ASSIGN = 0x1C13;  // sync manager 3 (inputs) PDO assignment
}  // namespace

std::vector<SdoWrite> build_startup_plan(const DriveConfig & drive, const BusConfig & bus)
{
  std::vector<SdoWrite> plan = drive.startup_sdos;

  // ---- PDO mapping ----
  // 1) Disable the assignments and clear the mapping objects (count = 0),
  //    as the count subindex must be 0 while entries are changed.
  plan.push_back({OBJ_RXPDO_ASSIGN, 0x00, DataType::UINT8, 0});
  plan.push_back({OBJ_TXPDO_ASSIGN, 0x00, DataType::UINT8, 0});
  plan.push_back({drive.rx_pdo_index, 0x00, DataType::UINT8, 0});
  plan.push_back({drive.tx_pdo_index, 0x00, DataType::UINT8, 0});

  // 2) Write the entries (subindex 1..n), then the entry count.
  auto map_entries = [&](uint16_t pdo_index, const std::vector<PdoEntry> & entries)
  {
    for (size_t i = 0; i < entries.size(); ++i)
    {
      plan.push_back(
          {pdo_index, static_cast<uint8_t>(i + 1), DataType::UINT32, static_cast<int64_t>(mapping_value(entries[i]))});
    }
    plan.push_back({pdo_index, 0x00, DataType::UINT8, static_cast<int64_t>(entries.size())});
  };
  map_entries(drive.rx_pdo_index, drive.rx_pdo);
  map_entries(drive.tx_pdo_index, drive.tx_pdo);

  // 3) Assign the mapping objects to the sync managers again.
  plan.push_back({OBJ_RXPDO_ASSIGN, 0x01, DataType::UINT16, drive.rx_pdo_index});
  plan.push_back({OBJ_RXPDO_ASSIGN, 0x00, DataType::UINT8, 1});
  plan.push_back({OBJ_TXPDO_ASSIGN, 0x01, DataType::UINT16, drive.tx_pdo_index});
  plan.push_back({OBJ_TXPDO_ASSIGN, 0x00, DataType::UINT8, 1});

  // ---- Synchronisation ----
  // The IRD latches its sync mode at the PRE-OP -> SAFE-OP transition
  // (EtherCAT manual ch. 5: 0x1006/0x2012 "need to be carefully setup before
  // entering the Safe-Op state"), so both objects are written here. A failed
  // write is not fatal: the sync mode actually latched is checked and
  // reported once the drives are in SAFE-OP.
  if (bus.distributed_clocks)
  {
    if (bus.sync_mode == SyncMode::HARD_SYNC)
    {
      plan.push_back({OBJ_CYCLE_PERIOD, 0x00, DataType::UINT32, bus.cycle_time_us, /*required=*/false});
      plan.push_back({OBJ_SYNC_MODE, 0x01, DataType::UINT16, IRD_SYNC_TYPE_HARD, /*required=*/false});
    }
    else
    {
      // Clears a 4 left over from an earlier HardSync session. Not sufficient
      // on its own: SoftSync also needs SAFE-OP before SYNC0 starts.
      plan.push_back({OBJ_SYNC_MODE, 0x01, DataType::UINT16, IRD_SYNC_TYPE_SOFT, /*required=*/false});
    }
  }

  return plan;
}

}  // namespace ird_ethercat_hardware_interface
