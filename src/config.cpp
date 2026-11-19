// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

#include "ird_ethercat_hardware_interface/config.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <map>
#include <set>
#include <tuple>

#include "ird_ethercat_hardware_interface/pdo.hpp"

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
    case PdoRole::TARGET_POSITION: return "target_position";
    case PdoRole::TARGET_VELOCITY: return "target_velocity";
    case PdoRole::TARGET_TORQUE: return "target_torque";
    case PdoRole::STATUSWORD: return "statusword";
    case PdoRole::MODE_DISPLAY: return "mode_display";
    case PdoRole::POSITION_ACTUAL: return "position_actual";
    case PdoRole::VELOCITY_ACTUAL: return "velocity_actual";
    case PdoRole::TORQUE_ACTUAL: return "torque_actual";
  }
  return "?";
}

bool parse_pdo_role(const std::string & text, PdoRole & out)
{
  static const std::map<std::string, PdoRole> names{
      {"none", PdoRole::NONE},
      {"controlword", PdoRole::CONTROLWORD},
      {"mode_of_operation", PdoRole::MODE_OF_OPERATION},
      {"target_position", PdoRole::TARGET_POSITION},
      {"target_velocity", PdoRole::TARGET_VELOCITY},
      {"target_torque", PdoRole::TARGET_TORQUE},
      {"statusword", PdoRole::STATUSWORD},
      {"mode_display", PdoRole::MODE_DISPLAY},
      {"position_actual", PdoRole::POSITION_ACTUAL},
      {"velocity_actual", PdoRole::VELOCITY_ACTUAL},
      {"torque_actual", PdoRole::TORQUE_ACTUAL}};
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

const char * command_interface_name(OperationMode mode)
{
  switch (mode)
  {
    case OperationMode::CYCLIC_SYNC_POSITION: return "position";
    case OperationMode::CYCLIC_SYNC_VELOCITY: return "velocity";
    case OperationMode::CYCLIC_SYNC_TORQUE: return "effort";
  }
  return "?";
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

  const bool is_rx = std::string(which) == "RxPDO";
  std::map<PdoRole, int> count;
  for (const auto & entry : entries)
  {
    if (entry.role == PdoRole::NONE && entry.interface.empty())
    {
      errors.push_back(format(
          "drive '%s': %s entry 0x%04X:%02X has neither a role nor an interface name, so nothing would use it",
          drive.name.c_str(), which, entry.index, entry.subindex));
    }
    if (is_rx && entry.role != PdoRole::NONE && !entry.interface.empty())
    {
      errors.push_back(format(
          "drive '%s': RxPDO entry 0x%04X:%02X is written by the driver (role %s), so it cannot also be a "
          "command interface", drive.name.c_str(), entry.index, entry.subindex, to_string(entry.role)));
    }
    if (!std::isfinite(entry.scale) || entry.scale == 0.0)
    {
      errors.push_back(format(
          "drive '%s': %s entry 0x%04X:%02X: scale must be a non-zero number", drive.name.c_str(), which,
          entry.index, entry.subindex));
    }
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
  if (bus.rt_cpu < -2)
  {
    errors.push_back(format("bus: rt_cpu must be a CPU number, -1 (not pinned) or -2 (last CPU) (got %d)", bus.rt_cpu));
  }
  if (bus.dc_setup_attempts < 1)
  {
    errors.push_back("bus: dc_setup_attempts must be at least 1");
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

    // ---- PDO roles: what each mode needs ----
    // RxPDO: the controlword and the target of the selected mode (only that
    // one: a target of another mode would never be written).
    // TxPDO: the statusword, the actual value of the selected mode, and
    // position or velocity (the joint states are derived from them).
    std::vector<std::pair<PdoRole, DataType>> rx_required{{PdoRole::CONTROLWORD, DataType::UINT16}};
    std::vector<std::pair<PdoRole, DataType>> tx_required{{PdoRole::STATUSWORD, DataType::UINT16}};
    switch (drive.mode)
    {
      case OperationMode::CYCLIC_SYNC_POSITION:
        rx_required.push_back({PdoRole::TARGET_POSITION, DataType::INT32});
        tx_required.push_back({PdoRole::POSITION_ACTUAL, DataType::INT32});
        break;
      case OperationMode::CYCLIC_SYNC_VELOCITY:
        rx_required.push_back({PdoRole::TARGET_VELOCITY, DataType::INT32});
        tx_required.push_back({PdoRole::VELOCITY_ACTUAL, DataType::INT32});
        break;
      case OperationMode::CYCLIC_SYNC_TORQUE:
        rx_required.push_back({PdoRole::TARGET_TORQUE, DataType::INT16});
        break;
    }
    validate_pdo(drive, "RxPDO", drive.rx_pdo, rx_required, {{PdoRole::MODE_OF_OPERATION, DataType::INT8}}, errors);
    validate_pdo(
        drive, "TxPDO", drive.tx_pdo, tx_required,
        {{PdoRole::MODE_DISPLAY, DataType::INT8}, {PdoRole::POSITION_ACTUAL, DataType::INT32},
         {PdoRole::VELOCITY_ACTUAL, DataType::INT32}, {PdoRole::TORQUE_ACTUAL, DataType::INT16}},
        errors);

    // ---- PDO limits: the entries must fit in the drive's mapping objects ----
    const PdoLimits & limits = drive.pdo_limits;
    for (const auto & [which, first_index, entries] :
         {std::make_tuple("RxPDO", drive.rx_pdo_index, &drive.rx_pdo),
          std::make_tuple("TxPDO", drive.tx_pdo_index, &drive.tx_pdo)})
    {
      for (const PdoEntry & entry : *entries)
      {
        if (limits.max_bytes > 0 && size_of(entry.type) > limits.max_bytes)
        {
          errors.push_back(format(
              "drive '%s': %s entry 0x%04X:%02X (%zu bytes) is larger than pdo_limits.max_bytes (%u)", name, which,
              entry.index, entry.subindex, size_of(entry.type), limits.max_bytes));
        }
      }
      const std::vector<PdoObject> objects = split_into_pdos(*entries, first_index, limits);
      if (limits.max_pdos > 0 && objects.size() > limits.max_pdos)
      {
        errors.push_back(format(
            "drive '%s': the %s needs %zu mapping objects (0x%04X..0x%04X) with at most %u bytes and %u entries "
            "each, but the drive accepts at most %u (pdo_limits); map fewer objects",
            name, which, objects.size(), objects.front().index, objects.back().index, limits.max_bytes,
            limits.max_entries, limits.max_pdos));
      }
    }

    auto has_role = [](const std::vector<PdoEntry> & entries, PdoRole role)
    {
      return std::any_of(entries.begin(), entries.end(), [role](const PdoEntry & e) { return e.role == role; });
    };
    if (!has_role(drive.tx_pdo, PdoRole::POSITION_ACTUAL) && !has_role(drive.tx_pdo, PdoRole::VELOCITY_ACTUAL))
    {
      errors.push_back(format(
          "drive '%s': the TxPDO needs position_actual or velocity_actual to provide the joint states", name));
    }
    const bool uses_torque =
        has_role(drive.rx_pdo, PdoRole::TARGET_TORQUE) || has_role(drive.tx_pdo, PdoRole::TORQUE_ACTUAL);
    if (uses_torque && !(drive.rated_torque_nm > 0.0))
    {
      errors.push_back(format(
          "drive '%s': rated_torque_nm must be > 0 when target_torque or torque_actual is mapped", name));
    }

    // ---- Interface names: <joint>/<interface> must be unique and valid ----
    std::set<std::string> interfaces;
    for (const auto * entries : {&drive.rx_pdo, &drive.tx_pdo})
    {
      for (const PdoEntry & entry : *entries)
      {
        if (entry.interface.empty()) continue;
        const bool valid_chars = std::all_of(
            entry.interface.begin(), entry.interface.end(),
            [](char c) { return std::isalnum(static_cast<unsigned char>(c)) || c == '_'; });
        if (!valid_chars)
        {
          errors.push_back(format(
              "drive '%s': interface name '%s' may only contain letters, digits and '_'", name,
              entry.interface.c_str()));
        }
        if (entry.interface == "position" || entry.interface == "velocity" || entry.interface == "effort")
        {
          errors.push_back(format(
              "drive '%s': interface name '%s' is reserved for the joint's standard interfaces", name,
              entry.interface.c_str()));
        }
        else if (!interfaces.insert(entry.interface).second)
        {
          errors.push_back(format(
              "drive '%s': interface name '%s' is used more than once", name, entry.interface.c_str()));
        }
      }
    }

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
