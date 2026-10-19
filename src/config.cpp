// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

#include "ird_ethercat_hardware_interface/config.hpp"

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <map>
#include <set>

namespace ird_ethercat_hardware_interface
{

size_t size_of(DataType type)
{
  switch (type)
  {
    case DataType::INT8:
    case DataType::UINT8: return 1;
    case DataType::INT16:
    case DataType::UINT16: return 2;
    case DataType::INT32:
    case DataType::UINT32: return 4;
  }
  return 0;
}

bool is_signed(DataType type)
{
  return type == DataType::INT8 || type == DataType::INT16 || type == DataType::INT32;
}

const char * to_string(DataType type)
{
  switch (type)
  {
    case DataType::INT8: return "int8";
    case DataType::UINT8: return "uint8";
    case DataType::INT16: return "int16";
    case DataType::UINT16: return "uint16";
    case DataType::INT32: return "int32";
    case DataType::UINT32: return "uint32";
  }
  return "?";
}

bool parse_data_type(const std::string & text, DataType & out)
{
  static const std::map<std::string, DataType> names{
      {"int8", DataType::INT8}, {"uint8", DataType::UINT8}, {"int16", DataType::INT16},
      {"uint16", DataType::UINT16}, {"int32", DataType::INT32}, {"uint32", DataType::UINT32}};
  const auto it = names.find(text);
  if (it == names.end()) return false;
  out = it->second;
  return true;
}

const char * to_string(PdoRole role)
{
  switch (role)
  {
    case PdoRole::NONE: return "none";
    case PdoRole::CONTROLWORD: return "controlword";
    case PdoRole::MODE_OF_OPERATION: return "mode_of_operation";
    case PdoRole::TARGET_VELOCITY: return "target_velocity";
    case PdoRole::STATUSWORD: return "statusword";
    case PdoRole::MODE_DISPLAY: return "mode_display";
    case PdoRole::VELOCITY_ACTUAL: return "velocity_actual";
  }
  return "?";
}

bool parse_pdo_role(const std::string & text, PdoRole & out)
{
  static const std::map<std::string, PdoRole> names{
      {"none", PdoRole::NONE},
      {"controlword", PdoRole::CONTROLWORD},
      {"mode_of_operation", PdoRole::MODE_OF_OPERATION},
      {"target_velocity", PdoRole::TARGET_VELOCITY},
      {"statusword", PdoRole::STATUSWORD},
      {"mode_display", PdoRole::MODE_DISPLAY},
      {"velocity_actual", PdoRole::VELOCITY_ACTUAL}};
  const auto it = names.find(text);
  if (it == names.end()) return false;
  out = it->second;
  return true;
}

DriveConfig make_ird_csv_drive()
{
  DriveConfig drive;
  drive.mode = OperationMode::CYCLIC_SYNC_VELOCITY;
  drive.rx_pdo_index = 0x1600;
  drive.rx_pdo = {
      {0x6040, 0x00, DataType::UINT16, PdoRole::CONTROLWORD},
      {0x6060, 0x00, DataType::INT8, PdoRole::MODE_OF_OPERATION},
      {0x60FF, 0x00, DataType::INT32, PdoRole::TARGET_VELOCITY},
  };
  drive.tx_pdo_index = 0x1A00;
  drive.tx_pdo = {
      {0x6041, 0x00, DataType::UINT16, PdoRole::STATUSWORD},
      {0x6061, 0x00, DataType::INT8, PdoRole::MODE_DISPLAY},
      {0x606C, 0x00, DataType::INT32, PdoRole::VELOCITY_ACTUAL},
  };
  drive.fault_snapshot = {
      {0x603F, 0x00, DataType::UINT16, "error_code"},
      {0x1001, 0x00, DataType::UINT8, "error_register"},
      {0x3022, 0x01, DataType::UINT32, "error_bits_1"},
      {0x3022, 0x02, DataType::UINT32, "error_bits_2"},
      {0x3024, 0x00, DataType::UINT32, "warning_code"},
  };
  return drive;
}

uint32_t ms_to_cycles(uint32_t ms, uint32_t cycle_time_us)
{
  if (cycle_time_us == 0) return 1;
  const uint64_t cycles = (static_cast<uint64_t>(ms) * 1000 + cycle_time_us / 2) / cycle_time_us;
  return cycles < 1 ? 1 : static_cast<uint32_t>(cycles);
}

namespace
{

std::string format(const char * fmt, ...) __attribute__((format(printf, 1, 2)));

std::string format(const char * fmt, ...)
{
  char buffer[512];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  return buffer;
}

// Checks one PDO: required roles present once, with the expected type.
void validate_pdo(
    const DriveConfig & drive, const char * which, const std::vector<PdoEntry> & entries,
    const std::vector<std::pair<PdoRole, DataType>> & required,
    const std::vector<std::pair<PdoRole, DataType>> & optional, std::vector<std::string> & errors)
{
  if (entries.empty() || entries.size() > 254)
  {
    errors.push_back(format("drive '%s': %s must have between 1 and 254 entries", drive.name.c_str(), which));
  }

  std::map<PdoRole, int> count;
  for (const auto & entry : entries)
  {
    if (entry.role == PdoRole::NONE) continue;
    ++count[entry.role];

    bool allowed = false;
    for (const auto & list : {required, optional})
    {
      for (const auto & [role, type] : list)
      {
        if (role != entry.role) continue;
        allowed = true;
        if (entry.type != type)
        {
          errors.push_back(format(
              "drive '%s': %s entry 0x%04X:%02X has role %s, which must be of type %s (got %s)",
              drive.name.c_str(), which, entry.index, entry.subindex, to_string(entry.role), to_string(type),
              to_string(entry.type)));
        }
      }
    }
    if (!allowed)
    {
      errors.push_back(format(
          "drive '%s': role %s is not allowed in the %s", drive.name.c_str(), to_string(entry.role), which));
    }
  }
  for (const auto & [role, n] : count)
  {
    if (n > 1)
    {
      errors.push_back(format("drive '%s': role %s appears %d times in the %s", drive.name.c_str(), to_string(role), n, which));
    }
  }
  for (const auto & [role, type] : required)
  {
    if (count.count(role) == 0)
    {
      errors.push_back(format(
          "drive '%s': the %s needs an entry with role %s (%s)", drive.name.c_str(), which, to_string(role),
          to_string(type)));
    }
  }
}

bool fits(DataType type, int64_t value)
{
  switch (type)
  {
    case DataType::INT8: return value >= INT8_MIN && value <= INT8_MAX;
    case DataType::UINT8: return value >= 0 && value <= UINT8_MAX;
    case DataType::INT16: return value >= INT16_MIN && value <= INT16_MAX;
    case DataType::UINT16: return value >= 0 && value <= UINT16_MAX;
    case DataType::INT32: return value >= INT32_MIN && value <= INT32_MAX;
    case DataType::UINT32: return value >= 0 && value <= UINT32_MAX;
  }
  return false;
}

}  // namespace

std::vector<std::string> validate(const SystemConfig & config)
{
  std::vector<std::string> errors;
  const BusConfig & bus = config.bus;

  // ---- Bus ----
  if (bus.interface.empty())
  {
    errors.push_back("bus: interface must not be empty");
  }
  if (bus.cycle_time_us < 250 || bus.cycle_time_us > 100'000)
  {
    errors.push_back(format("bus: cycle_time_us must be between 250 and 100000 (got %u)", bus.cycle_time_us));
  }
  if (bus.rt_priority < 1 || bus.rt_priority > 99)
  {
    errors.push_back(format("bus: rt_priority must be between 1 and 99 (got %d)", bus.rt_priority));
  }
  if (bus.distributed_clocks && bus.sync0_shift_us >= bus.cycle_time_us)
  {
    errors.push_back(format(
        "bus: sync0_shift_us (%u) must be smaller than cycle_time_us (%u)", bus.sync0_shift_us, bus.cycle_time_us));
  }

  // ---- Drives ----
  if (config.drives.empty())
  {
    errors.push_back("at least one drive must be configured");
  }
  if (config.drives.size() > MAX_DRIVES)
  {
    errors.push_back(format("at most %zu drives are supported (got %zu)", MAX_DRIVES, config.drives.size()));
  }

  std::set<std::string> names;
  std::set<std::string> joints;
  std::set<int> positions;
  for (const DriveConfig & drive : config.drives)
  {
    const char * name = drive.name.c_str();
    if (drive.name.empty())
    {
      errors.push_back("every drive needs a name");
    }
    else if (!names.insert(drive.name).second)
    {
      errors.push_back(format("drive name '%s' is used more than once", name));
    }
    if (drive.joint.empty())
    {
      errors.push_back(format("drive '%s': joint must not be empty", name));
    }
    else if (!joints.insert(drive.joint).second)
    {
      errors.push_back(format("drive '%s': joint '%s' is driven by more than one drive", name, drive.joint.c_str()));
    }
    if (drive.slave_position < 1)
    {
      errors.push_back(format("drive '%s': slave_position must be >= 1 (got %d)", name, drive.slave_position));
    }
    else if (!positions.insert(drive.slave_position).second)
    {
      errors.push_back(format("drive '%s': slave_position %d is used by another drive", name, drive.slave_position));
    }
    if (drive.mode != OperationMode::CYCLIC_SYNC_VELOCITY)
    {
      errors.push_back(format(
          "drive '%s': only mode CSV (9) is supported in this version (got %d)", name, static_cast<int>(drive.mode)));
    }
    if (!(drive.encoder_ticks_per_rev > 0.0))
    {
      errors.push_back(format("drive '%s': encoder_ticks_per_rev must be > 0", name));
    }
    if (!(drive.gear_ratio > 0.0))
    {
      errors.push_back(format("drive '%s': gear_ratio must be > 0", name));
    }
    if (!(drive.max_motor_rpm > 0.0))
    {
      errors.push_back(format("drive '%s': max_motor_rpm must be > 0", name));
    }
    if (!(drive.enable_ramp_motor_rad_s2 >= 0.0))
    {
      errors.push_back(format("drive '%s': enable_ramp_motor_rad_s2 must be >= 0", name));
    }

    validate_pdo(
        drive, "RxPDO", drive.rx_pdo,
        {{PdoRole::CONTROLWORD, DataType::UINT16}, {PdoRole::TARGET_VELOCITY, DataType::INT32}},
        {{PdoRole::MODE_OF_OPERATION, DataType::INT8}}, errors);
    validate_pdo(
        drive, "TxPDO", drive.tx_pdo,
        {{PdoRole::STATUSWORD, DataType::UINT16}, {PdoRole::VELOCITY_ACTUAL, DataType::INT32}},
        {{PdoRole::MODE_DISPLAY, DataType::INT8}}, errors);

    for (const SdoWrite & sdo : drive.startup_sdos)
    {
      if (!fits(sdo.type, sdo.value))
      {
        errors.push_back(format(
            "drive '%s': startup SDO 0x%04X:%02X value %lld does not fit in %s", name, sdo.index, sdo.subindex,
            static_cast<long long>(sdo.value), to_string(sdo.type)));
      }
    }
  }

  return errors;
}

}  // namespace ird_ethercat_hardware_interface
