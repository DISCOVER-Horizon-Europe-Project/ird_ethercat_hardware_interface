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
  // The entries of each direction are split over as many mapping objects as
  // the drive's PDO limits require (one for the default IRD mapping).
  const std::vector<PdoObject> rx_objects = split_into_pdos(drive.rx_pdo, drive.rx_pdo_index, drive.pdo_limits);
  const std::vector<PdoObject> tx_objects = split_into_pdos(drive.tx_pdo, drive.tx_pdo_index, drive.pdo_limits);

  // 1) Disable the assignments and clear the mapping objects (count = 0),
  //    as the count subindex must be 0 while entries are changed.
  plan.push_back({OBJ_RXPDO_ASSIGN, 0x00, DataType::UINT8, 0});
  plan.push_back({OBJ_TXPDO_ASSIGN, 0x00, DataType::UINT8, 0});
  for (const auto * objects : {&rx_objects, &tx_objects})
  {
    for (const PdoObject & object : *objects)
    {
      plan.push_back({object.index, 0x00, DataType::UINT8, 0});
    }
  }

  // 2) Write each object's entries (subindex 1..n), then its entry count.
  auto map_entries = [&](const std::vector<PdoObject> & objects, const std::vector<PdoEntry> & entries)
  {
    for (const PdoObject & object : objects)
    {
      for (size_t i = 0; i < object.entry_count; ++i)
      {
        plan.push_back(
            {object.index, static_cast<uint8_t>(i + 1), DataType::UINT32,
             static_cast<int64_t>(mapping_value(entries[object.first_entry + i]))});
      }
      plan.push_back({object.index, 0x00, DataType::UINT8, static_cast<int64_t>(object.entry_count)});
    }
  };
  map_entries(rx_objects, drive.rx_pdo);
  map_entries(tx_objects, drive.tx_pdo);

  // 3) Assign the mapping objects to the sync managers again, in order (the
  //    process image follows the assignment order).
  auto assign = [&](uint16_t assign_index, const std::vector<PdoObject> & objects)
  {
    for (size_t i = 0; i < objects.size(); ++i)
    {
      plan.push_back({assign_index, static_cast<uint8_t>(i + 1), DataType::UINT16, objects[i].index});
    }
    plan.push_back({assign_index, 0x00, DataType::UINT8, static_cast<int64_t>(objects.size())});
  };
  assign(OBJ_RXPDO_ASSIGN, rx_objects);
  assign(OBJ_TXPDO_ASSIGN, tx_objects);

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
