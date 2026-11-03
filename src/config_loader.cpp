// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

#include "ird_ethercat_hardware_interface/config_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <set>
#include <string>
#include <vector>

namespace ird_ethercat_hardware_interface
{
namespace
{

// Reads typed values from YAML nodes and collects readable errors such as
//   line 42: drives[1] (right): unknown key 'gear_ration' (valid keys: ...)
class Reader
{
public:
  explicit Reader(std::vector<std::string> & errors) : errors_(errors) {}

  void error(const YAML::Node & node, const std::string & where, const std::string & message)
  {
    std::string text;
    if (node.IsDefined() && node.Mark().line >= 0)
    {
      text = "line " + std::to_string(node.Mark().line + 1) + ": ";
    }
    errors_.push_back(text + where + ": " + message);
  }

  bool expect_map(const YAML::Node & node, const std::string & where)
  {
    if (node.IsMap()) return true;
    error(node, where, "expected a mapping (key: value pairs)");
    return false;
  }

  bool expect_sequence(const YAML::Node & node, const std::string & where)
  {
    if (node.IsSequence()) return true;
    error(node, where, "expected a list");
    return false;
  }

  void check_keys(const YAML::Node & map, const std::string & where, const std::vector<std::string> & allowed)
  {
    for (const auto & item : map)
    {
      const std::string key = item.first.as<std::string>();
      if (std::find(allowed.begin(), allowed.end(), key) != allowed.end()) continue;
      std::string list;
      for (const auto & a : allowed) list += (list.empty() ? "" : ", ") + a;
      error(item.first, where, "unknown key '" + key + "' (valid keys: " + list + ")");
    }
  }

  // Integers accept decimal, hexadecimal (0x...) and negative values.
  bool to_integer(const YAML::Node & node, const std::string & where, int64_t & out)
  {
    if (!node.IsScalar())
    {
      error(node, where, "expected an integer");
      return false;
    }
    const std::string text = node.Scalar();
    errno = 0;
    char * end = nullptr;
    const long long value = std::strtoll(text.c_str(), &end, 0);
    if (text.empty() || *end != '\0' || errno == ERANGE)
    {
      error(node, where, "'" + text + "' is not an integer");
      return false;
    }
    out = value;
    return true;
  }

  template<typename T>
  void integer(const YAML::Node & map, const char * key, const std::string & where, T & out)
  {
    const YAML::Node node = map[key];
    if (!node) return;
    int64_t value = 0;
    if (!to_integer(node, where + "." + key, value)) return;
    if (value < static_cast<int64_t>(std::numeric_limits<T>::min()) ||
        value > static_cast<int64_t>(std::numeric_limits<T>::max()))
    {
      error(node, where + "." + key, "value " + node.Scalar() + " is out of range");
      return;
    }
    out = static_cast<T>(value);
  }

  void real(const YAML::Node & map, const char * key, const std::string & where, double & out)
  {
    const YAML::Node node = map[key];
    if (!node) return;
    const std::string text = node.IsScalar() ? node.Scalar() : "";
    char * end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (text.empty() || *end != '\0')
    {
      error(node, where + "." + key, "'" + text + "' is not a number");
      return;
    }
    out = value;
  }

  void boolean(const YAML::Node & map, const char * key, const std::string & where, bool & out)
  {
    const YAML::Node node = map[key];
    if (!node) return;
    const std::string text = node.IsScalar() ? node.Scalar() : "";
    if (text == "true") out = true;
    else if (text == "false") out = false;
    else error(node, where + "." + key, "expected true or false, got '" + text + "'");
  }

  void string(const YAML::Node & map, const char * key, const std::string & where, std::string & out)
  {
    const YAML::Node node = map[key];
    if (!node) return;
    if (!node.IsScalar())
    {
      error(node, where + "." + key, "expected a text value");
      return;
    }
    out = node.Scalar();
  }

  // Object address in the notation of the drive manuals: "0x6040:00" (index
  // and subindex in hexadecimal). "0x6040" alone means subindex 0.
  bool object(const YAML::Node & map, const std::string & where, uint16_t & index, uint8_t & subindex)
  {
    const YAML::Node node = map["object"];
    if (!node || !node.IsScalar())
    {
      error(map, where, "missing 'object' (e.g. object: \"0x6040:00\")");
      return false;
    }
    const std::string text = node.Scalar();
    const size_t colon = text.find(':');
    const std::string index_text = text.substr(0, colon);
    const std::string sub_text = colon == std::string::npos ? "0" : text.substr(colon + 1);
    char * end_index = nullptr;
    char * end_sub = nullptr;
    const unsigned long i = std::strtoul(index_text.c_str(), &end_index, 16);
    const unsigned long s = std::strtoul(sub_text.c_str(), &end_sub, 16);
    if (index_text.empty() || sub_text.empty() || *end_index != '\0' || *end_sub != '\0' || i > 0xFFFF || s > 0xFF)
    {
      error(node, where + ".object", "'" + text + "' is not an object address like \"0x6040:00\"");
      return false;
    }
    index = static_cast<uint16_t>(i);
    subindex = static_cast<uint8_t>(s);
    return true;
  }

  bool data_type(const YAML::Node & map, const std::string & where, DataType & out)
  {
    const YAML::Node node = map["type"];
    if (!node || !node.IsScalar() || !parse_data_type(node.Scalar(), out))
    {
      error(node ? node : map, where + ".type", "expected one of int8, uint8, int16, uint16, int32, uint32");
      return false;
    }
    return true;
  }

private:
  std::vector<std::string> & errors_;
};

// ---------------------------------------------------------------------------

void read_bus(Reader & r, const YAML::Node & node, BusConfig & bus)
{
  const std::string where = "bus";
  if (!r.expect_map(node, where)) return;
  r.check_keys(
      node, where,
      {"interface", "cycle_time_us", "rt_priority", "rt_cpu", "distributed_clocks", "sync_mode", "sync0_shift_us",
       "dc_lock_tolerance_us", "dc_lock_ms", "dc_lock_timeout_ms", "startup_settle_ms", "dc_setup_tolerance_us",
       "dc_setup_attempts", "dc_sync_tolerance_ns", "dc_sync_timeout_ms"});
  r.string(node, "interface", where, bus.interface);
  r.integer(node, "cycle_time_us", where, bus.cycle_time_us);
  r.integer(node, "rt_priority", where, bus.rt_priority);
  r.integer(node, "rt_cpu", where, bus.rt_cpu);
  r.boolean(node, "distributed_clocks", where, bus.distributed_clocks);
  if (const YAML::Node mode = node["sync_mode"])
  {
    const std::string text = mode.IsScalar() ? mode.Scalar() : "";
    if (text == "hard_sync") bus.sync_mode = SyncMode::HARD_SYNC;
    else if (text == "soft_sync") bus.sync_mode = SyncMode::SOFT_SYNC;
    else r.error(mode, where + ".sync_mode", "expected hard_sync or soft_sync, got '" + text + "'");
  }
  r.integer(node, "sync0_shift_us", where, bus.sync0_shift_us);
  r.integer(node, "dc_lock_tolerance_us", where, bus.dc_lock_tolerance_us);
  r.integer(node, "dc_lock_ms", where, bus.dc_lock_ms);
  r.integer(node, "dc_lock_timeout_ms", where, bus.dc_lock_timeout_ms);
  r.integer(node, "startup_settle_ms", where, bus.startup_settle_ms);
  r.integer(node, "dc_setup_tolerance_us", where, bus.dc_setup_tolerance_us);
  r.integer(node, "dc_setup_attempts", where, bus.dc_setup_attempts);
  r.integer(node, "dc_sync_tolerance_ns", where, bus.dc_sync_tolerance_ns);
  r.integer(node, "dc_sync_timeout_ms", where, bus.dc_sync_timeout_ms);
}

void read_fault_handling(Reader & r, const YAML::Node & node, FaultHandlingConfig & faults)
{
  const std::string where = "fault_handling";
  if (!r.expect_map(node, where)) return;
  r.check_keys(
      node, where,
      {"enable_timeout_ms", "enable_stable_ms", "enable_fault_abort_ms", "max_recovery_attempts",
       "recovery_cooldown_ms", "reset_pulse_ms", "recovery_budget_refill_ms"});
  r.integer(node, "enable_timeout_ms", where, faults.enable_timeout_ms);
  r.integer(node, "enable_stable_ms", where, faults.enable_stable_ms);
  r.integer(node, "enable_fault_abort_ms", where, faults.enable_fault_abort_ms);
  r.integer(node, "max_recovery_attempts", where, faults.max_recovery_attempts);
  r.integer(node, "recovery_cooldown_ms", where, faults.recovery_cooldown_ms);
  r.integer(node, "reset_pulse_ms", where, faults.reset_pulse_ms);
  r.integer(node, "recovery_budget_refill_ms", where, faults.recovery_budget_refill_ms);
}

void read_pdo(Reader & r, const YAML::Node & node, const std::string & where, uint16_t & index, std::vector<PdoEntry> & entries)
{
  if (!r.expect_map(node, where)) return;
  r.check_keys(node, where, {"index", "entries"});
  r.integer(node, "index", where, index);
  const YAML::Node list = node["entries"];
  if (!list)
  {
    r.error(node, where, "missing 'entries'");
    return;
  }
  if (!r.expect_sequence(list, where + ".entries")) return;

  entries.clear();
  for (size_t i = 0; i < list.size(); ++i)
  {
    const YAML::Node item = list[i];
    const std::string at = where + ".entries[" + std::to_string(i) + "]";
    if (!r.expect_map(item, at)) continue;
    r.check_keys(item, at, {"object", "type", "role", "interface", "scale"});
    PdoEntry entry;
    bool ok = r.object(item, at, entry.index, entry.subindex);
    ok = r.data_type(item, at, entry.type) && ok;
    if (const YAML::Node role = item["role"])
    {
      if (!role.IsScalar() || !parse_pdo_role(role.Scalar(), entry.role))
      {
        r.error(
            role, at + ".role",
            "expected one of controlword, mode_of_operation, target_position, target_velocity, target_torque "
            "(RxPDO) or statusword, mode_display, position_actual, velocity_actual, torque_actual (TxPDO)");
        ok = false;
      }
    }
    r.string(item, "interface", at, entry.interface);
    r.real(item, "scale", at, entry.scale);
    if (ok) entries.push_back(entry);
  }
}

void read_pdo_limits(Reader & r, const YAML::Node & node, const std::string & where, PdoLimits & limits)
{
  if (!r.expect_map(node, where)) return;
  r.check_keys(node, where, {"max_bytes", "max_entries", "max_pdos"});
  r.integer(node, "max_bytes", where, limits.max_bytes);
  r.integer(node, "max_entries", where, limits.max_entries);
  r.integer(node, "max_pdos", where, limits.max_pdos);
}

void read_startup_sdos(Reader & r, const YAML::Node & list, const std::string & where, std::vector<SdoWrite> & out)
{
  if (!r.expect_sequence(list, where)) return;
  out.clear();
  for (size_t i = 0; i < list.size(); ++i)
  {
    const YAML::Node item = list[i];
    const std::string at = where + "[" + std::to_string(i) + "]";
    if (!r.expect_map(item, at)) continue;
    r.check_keys(item, at, {"object", "type", "value", "required"});
    SdoWrite sdo;
    bool ok = r.object(item, at, sdo.index, sdo.subindex);
    ok = r.data_type(item, at, sdo.type) && ok;
    if (const YAML::Node value = item["value"])
    {
      ok = r.to_integer(value, at + ".value", sdo.value) && ok;
    }
    else
    {
      r.error(item, at, "missing 'value'");
      ok = false;
    }
    r.boolean(item, "required", at, sdo.required);
    if (ok) out.push_back(sdo);
  }
}

void read_fault_snapshot(Reader & r, const YAML::Node & list, const std::string & where, std::vector<SdoRead> & out)
{
  if (!r.expect_sequence(list, where)) return;
  out.clear();
  for (size_t i = 0; i < list.size(); ++i)
  {
    const YAML::Node item = list[i];
    const std::string at = where + "[" + std::to_string(i) + "]";
    if (!r.expect_map(item, at)) continue;
    r.check_keys(item, at, {"object", "type", "name"});
    SdoRead read;
    bool ok = r.object(item, at, read.index, read.subindex);
    ok = r.data_type(item, at, read.type) && ok;
    r.string(item, "name", at, read.name);
    if (read.name.empty())
    {
      char buffer[16];
      std::snprintf(buffer, sizeof(buffer), "0x%04X:%02X", read.index, read.subindex);
      read.name = buffer;
    }
    if (ok) out.push_back(read);
  }
}

// Applies the keys of `node` to `drive`. `per_drive` allows the keys that
// identify one drive (not allowed in drive_defaults).
void read_drive(Reader & r, const YAML::Node & node, const std::string & where, bool per_drive, DriveConfig & drive)
{
  if (!r.expect_map(node, where)) return;
  std::vector<std::string> keys = {
      "mode", "encoder_ticks_per_rev", "gear_ratio", "rated_torque_nm", "max_motor_rpm", "enable_ramp_motor_rad_s2",
      "expected_vendor_id", "expected_product_code", "rx_pdo", "tx_pdo", "pdo_limits", "startup_sdos",
      "fault_snapshot"};
  if (per_drive)
  {
    keys.insert(keys.begin(), {"name", "joint", "slave_position"});
  }
  r.check_keys(node, where, keys);

  if (per_drive)
  {
    r.string(node, "name", where, drive.name);
    r.string(node, "joint", where, drive.joint);
    r.integer(node, "slave_position", where, drive.slave_position);
  }
  if (const YAML::Node mode = node["mode"])
  {
    const std::string text = mode.IsScalar() ? mode.Scalar() : "";
    if (text == "csv") drive.mode = OperationMode::CYCLIC_SYNC_VELOCITY;
    else if (text == "csp") drive.mode = OperationMode::CYCLIC_SYNC_POSITION;
    else if (text == "cst") drive.mode = OperationMode::CYCLIC_SYNC_TORQUE;
    else r.error(mode, where + ".mode", "expected csv, csp or cst, got '" + text + "'");
  }
  r.real(node, "encoder_ticks_per_rev", where, drive.encoder_ticks_per_rev);
  r.real(node, "gear_ratio", where, drive.gear_ratio);
  r.real(node, "rated_torque_nm", where, drive.rated_torque_nm);
  r.real(node, "max_motor_rpm", where, drive.max_motor_rpm);
  r.real(node, "enable_ramp_motor_rad_s2", where, drive.enable_ramp_motor_rad_s2);
  r.integer(node, "expected_vendor_id", where, drive.expected_vendor_id);
  r.integer(node, "expected_product_code", where, drive.expected_product_code);
  if (node["rx_pdo"]) read_pdo(r, node["rx_pdo"], where + ".rx_pdo", drive.rx_pdo_index, drive.rx_pdo);
  if (node["tx_pdo"]) read_pdo(r, node["tx_pdo"], where + ".tx_pdo", drive.tx_pdo_index, drive.tx_pdo);
  if (node["pdo_limits"]) read_pdo_limits(r, node["pdo_limits"], where + ".pdo_limits", drive.pdo_limits);
  if (node["startup_sdos"]) read_startup_sdos(r, node["startup_sdos"], where + ".startup_sdos", drive.startup_sdos);
  if (node["fault_snapshot"])
  {
    read_fault_snapshot(r, node["fault_snapshot"], where + ".fault_snapshot", drive.fault_snapshot);
  }
}

std::vector<std::string> load(const YAML::Node & root, SystemConfig & config)
{
  std::vector<std::string> errors;
  Reader r(errors);
  config = SystemConfig{};

  if (!root.IsMap())
  {
    errors.push_back("the configuration must be a mapping with the keys bus, fault_handling, drive_defaults, drives");
    return errors;
  }
  r.check_keys(root, "configuration", {"bus", "fault_handling", "drive_defaults", "drives"});

  if (root["bus"]) read_bus(r, root["bus"], config.bus);
  if (root["fault_handling"]) read_fault_handling(r, root["fault_handling"], config.faults);

  DriveConfig defaults = make_ird_csv_drive();
  if (root["drive_defaults"])
  {
    read_drive(r, root["drive_defaults"], "drive_defaults", /*per_drive=*/false, defaults);
  }

  const YAML::Node drives = root["drives"];
  if (!drives)
  {
    errors.push_back("configuration: missing 'drives' (the list of drives on the bus)");
    return errors;
  }
  if (!r.expect_sequence(drives, "drives")) return errors;
  for (size_t i = 0; i < drives.size(); ++i)
  {
    DriveConfig drive = defaults;
    std::string where = "drives[" + std::to_string(i) + "]";
    if (drives[i].IsMap() && drives[i]["name"] && drives[i]["name"].IsScalar())
    {
      where += " (" + drives[i]["name"].Scalar() + ")";
    }
    read_drive(r, drives[i], where, /*per_drive=*/true, drive);
    config.drives.push_back(drive);
  }
  return errors;
}

}  // namespace

std::vector<std::string> load_config_string(const std::string & yaml, SystemConfig & config)
{
  try
  {
    return load(YAML::Load(yaml), config);
  }
  catch (const YAML::Exception & e)
  {
    return {std::string("YAML syntax error: ") + e.what()};
  }
}

std::vector<std::string> load_config_file(const std::string & path, SystemConfig & config)
{
  try
  {
    return load(YAML::LoadFile(path), config);
  }
  catch (const YAML::BadFile &)
  {
    return {"cannot open configuration file '" + path + "'"};
  }
  catch (const YAML::Exception & e)
  {
    return {"'" + path + "': YAML syntax error: " + e.what()};
  }
}

}  // namespace ird_ethercat_hardware_interface
