// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "ird_ethercat_hardware_interface/config_loader.hpp"
#include "test_helpers.hpp"

using namespace ird_ethercat_hardware_interface;

namespace
{
const std::string CONFIG_DIR = IRD_CONFIG_DIR;

std::string joined(const std::vector<std::string> & errors)
{
  std::string text;
  for (const auto & e : errors) text += e + "\n";
  return text;
}

bool contains(const std::vector<std::string> & errors, const std::string & text)
{
  return joined(errors).find(text) != std::string::npos;
}

void expect_same_entries(const std::vector<PdoEntry> & a, const std::vector<PdoEntry> & b)
{
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i)
  {
    EXPECT_EQ(a[i].index, b[i].index);
    EXPECT_EQ(a[i].subindex, b[i].subindex);
    EXPECT_EQ(a[i].type, b[i].type);
    EXPECT_EQ(a[i].role, b[i].role);
  }
}

void expect_same(const SystemConfig & a, const SystemConfig & b)
{
  EXPECT_EQ(a.bus.interface, b.bus.interface);
  EXPECT_EQ(a.bus.cycle_time_us, b.bus.cycle_time_us);
  EXPECT_EQ(a.bus.rt_priority, b.bus.rt_priority);
  EXPECT_EQ(a.bus.distributed_clocks, b.bus.distributed_clocks);
  EXPECT_EQ(a.bus.sync_mode, b.bus.sync_mode);
  EXPECT_EQ(a.bus.sync0_shift_us, b.bus.sync0_shift_us);
  EXPECT_EQ(a.bus.dc_lock_tolerance_us, b.bus.dc_lock_tolerance_us);
  EXPECT_EQ(a.bus.dc_lock_ms, b.bus.dc_lock_ms);
  EXPECT_EQ(a.bus.dc_lock_timeout_ms, b.bus.dc_lock_timeout_ms);
  EXPECT_EQ(a.bus.startup_settle_ms, b.bus.startup_settle_ms);
  EXPECT_EQ(a.bus.rt_cpu, b.bus.rt_cpu);
  EXPECT_EQ(a.bus.dc_setup_tolerance_us, b.bus.dc_setup_tolerance_us);
  EXPECT_EQ(a.bus.dc_setup_attempts, b.bus.dc_setup_attempts);
  EXPECT_EQ(a.bus.dc_sync_tolerance_ns, b.bus.dc_sync_tolerance_ns);
  EXPECT_EQ(a.bus.dc_sync_timeout_ms, b.bus.dc_sync_timeout_ms);

  EXPECT_EQ(a.faults.enable_timeout_ms, b.faults.enable_timeout_ms);
  EXPECT_EQ(a.faults.enable_stable_ms, b.faults.enable_stable_ms);
  EXPECT_EQ(a.faults.enable_fault_abort_ms, b.faults.enable_fault_abort_ms);
  EXPECT_EQ(a.faults.max_recovery_attempts, b.faults.max_recovery_attempts);
  EXPECT_EQ(a.faults.recovery_cooldown_ms, b.faults.recovery_cooldown_ms);
  EXPECT_EQ(a.faults.reset_pulse_ms, b.faults.reset_pulse_ms);
  EXPECT_EQ(a.faults.recovery_budget_refill_ms, b.faults.recovery_budget_refill_ms);

  ASSERT_EQ(a.drives.size(), b.drives.size());
  for (size_t i = 0; i < a.drives.size(); ++i)
  {
    SCOPED_TRACE(a.drives[i].name);
    const DriveConfig & x = a.drives[i];
    const DriveConfig & y = b.drives[i];
    EXPECT_EQ(x.name, y.name);
    EXPECT_EQ(x.joint, y.joint);
    EXPECT_EQ(x.slave_position, y.slave_position);
    EXPECT_EQ(x.expected_vendor_id, y.expected_vendor_id);
    EXPECT_EQ(x.expected_product_code, y.expected_product_code);
    EXPECT_EQ(x.mode, y.mode);
    EXPECT_DOUBLE_EQ(x.encoder_ticks_per_rev, y.encoder_ticks_per_rev);
    EXPECT_DOUBLE_EQ(x.gear_ratio, y.gear_ratio);
    EXPECT_DOUBLE_EQ(x.max_motor_rpm, y.max_motor_rpm);
    EXPECT_DOUBLE_EQ(x.enable_ramp_motor_rad_s2, y.enable_ramp_motor_rad_s2);
    EXPECT_EQ(x.rx_pdo_index, y.rx_pdo_index);
    EXPECT_EQ(x.tx_pdo_index, y.tx_pdo_index);
    expect_same_entries(x.rx_pdo, y.rx_pdo);
    expect_same_entries(x.tx_pdo, y.tx_pdo);
    EXPECT_EQ(x.pdo_limits.max_bytes, y.pdo_limits.max_bytes);
    EXPECT_EQ(x.pdo_limits.max_entries, y.pdo_limits.max_entries);
    EXPECT_EQ(x.pdo_limits.max_pdos, y.pdo_limits.max_pdos);
    ASSERT_EQ(x.startup_sdos.size(), y.startup_sdos.size());
    ASSERT_EQ(x.fault_snapshot.size(), y.fault_snapshot.size());
    for (size_t k = 0; k < x.fault_snapshot.size(); ++k)
    {
      EXPECT_EQ(x.fault_snapshot[k].index, y.fault_snapshot[k].index);
      EXPECT_EQ(x.fault_snapshot[k].subindex, y.fault_snapshot[k].subindex);
      EXPECT_EQ(x.fault_snapshot[k].type, y.fault_snapshot[k].type);
      EXPECT_EQ(x.fault_snapshot[k].name, y.fault_snapshot[k].name);
    }
  }
}

const char * BASE = R"(
drive_defaults:
  encoder_ticks_per_rev: 2048
drives:
  - {name: a, joint: ja, slave_position: 1}
)";
}  // namespace

TEST(ConfigLoader, ShippedExampleIsTheValidatedNaviwallConfiguration)
{
  SystemConfig config;
  const auto errors = load_config_file(CONFIG_DIR + "/ird_csv_example.yaml", config);
  ASSERT_TRUE(errors.empty()) << joined(errors);
  EXPECT_TRUE(validate(config).empty()) << joined(validate(config));
  expect_same(config, naviwall_config());
}

TEST(ConfigLoader, MinimalExampleIsValid)
{
  SystemConfig config;
  const auto errors = load_config_file(CONFIG_DIR + "/minimal_example.yaml", config);
  ASSERT_TRUE(errors.empty()) << joined(errors);
  EXPECT_TRUE(validate(config).empty()) << joined(validate(config));
  ASSERT_EQ(config.drives.size(), 1u);
  // Everything not in the file comes from the built-in IRD defaults.
  expect_same_entries(config.drives[0].rx_pdo, make_ird_csv_drive().rx_pdo);
  EXPECT_EQ(config.bus.sync0_shift_us, 2000u);
}

TEST(ConfigLoader, OtherModesExampleIsValid)
{
  SystemConfig config;
  const auto errors = load_config_file(CONFIG_DIR + "/other_modes_example.yaml", config);
  ASSERT_TRUE(errors.empty()) << joined(errors);
  EXPECT_TRUE(validate(config).empty()) << joined(validate(config));
  ASSERT_EQ(config.drives.size(), 2u);
  EXPECT_EQ(config.drives[0].mode, OperationMode::CYCLIC_SYNC_POSITION);
  EXPECT_EQ(config.drives[1].mode, OperationMode::CYCLIC_SYNC_TORQUE);
}

TEST(ConfigLoader, PerDriveKeysOverrideDriveDefaults)
{
  SystemConfig config;
  const auto errors = load_config_string(R"(
drive_defaults:
  encoder_ticks_per_rev: 2048
  gear_ratio: 20
  max_motor_rpm: 3000
drives:
  - {name: a, joint: ja, slave_position: 1}
  - {name: b, joint: jb, slave_position: 2, gear_ratio: 51.67}
)", config);
  ASSERT_TRUE(errors.empty()) << joined(errors);
  ASSERT_EQ(config.drives.size(), 2u);
  EXPECT_DOUBLE_EQ(config.drives[0].gear_ratio, 20.0);
  EXPECT_DOUBLE_EQ(config.drives[1].gear_ratio, 51.67);
  EXPECT_DOUBLE_EQ(config.drives[1].max_motor_rpm, 3000.0);
}

TEST(ConfigLoader, ListsAreReplacedAsAWhole)
{
  SystemConfig config;
  const auto errors = load_config_string(R"(
drive_defaults:
  encoder_ticks_per_rev: 2048
drives:
  - name: a
    joint: ja
    slave_position: 1
    tx_pdo:
      index: 0x1A01
      entries:
        - {object: "0x6041:00", type: uint16, role: statusword}
        - {object: "0x606C:00", type: int32, role: velocity_actual}
        - {object: "0x6077:00", type: int16}
)", config);
  ASSERT_TRUE(errors.empty()) << joined(errors);
  const DriveConfig & drive = config.drives[0];
  EXPECT_EQ(drive.tx_pdo_index, 0x1A01);
  ASSERT_EQ(drive.tx_pdo.size(), 3u);
  EXPECT_EQ(drive.tx_pdo[2].index, 0x6077);
  EXPECT_EQ(drive.tx_pdo[2].role, PdoRole::NONE);
  EXPECT_EQ(drive.rx_pdo.size(), 3u);  // untouched default
}

TEST(ConfigLoader, ClockChecksCpuAndPdoLimits)
{
  SystemConfig config;
  const auto errors = load_config_string(R"(
bus:
  rt_cpu: -2
  dc_setup_tolerance_us: 500
  dc_setup_attempts: 5
  dc_sync_tolerance_ns: 2000
  dc_sync_timeout_ms: 3000
drive_defaults:
  encoder_ticks_per_rev: 2048
  pdo_limits: {max_bytes: 0, max_entries: 8, max_pdos: 1}
drives:
  - {name: a, joint: ja, slave_position: 1}
)", config);
  ASSERT_TRUE(errors.empty()) << joined(errors);
  EXPECT_EQ(config.bus.rt_cpu, -2);
  EXPECT_EQ(config.bus.dc_setup_tolerance_us, 500u);
  EXPECT_EQ(config.bus.dc_setup_attempts, 5u);
  EXPECT_EQ(config.bus.dc_sync_tolerance_ns, 2000u);
  EXPECT_EQ(config.bus.dc_sync_timeout_ms, 3000u);
  EXPECT_EQ(config.drives[0].pdo_limits.max_bytes, 0u);
  EXPECT_EQ(config.drives[0].pdo_limits.max_entries, 8u);
  EXPECT_EQ(config.drives[0].pdo_limits.max_pdos, 1u);

  const auto bad = load_config_string(R"(
drive_defaults:
  pdo_limits: {max_byte: 8}
drives:
  - {name: a, joint: ja, slave_position: 1}
)", config);
  EXPECT_TRUE(contains(bad, "drive_defaults.pdo_limits: unknown key 'max_byte'"));
}

TEST(ConfigLoader, StartupSdosAndHexValues)
{
  SystemConfig config;
  const auto errors = load_config_string(R"(
drive_defaults:
  encoder_ticks_per_rev: 2048
  expected_vendor_id: 0x000001DD
  startup_sdos:
    - {object: "0x6083:00", type: uint32, value: 100000}
    - {object: "0x2012:01", type: uint16, value: 0x4, required: false}
    - {object: "0x607C:00", type: int32, value: -1500}
drives:
  - {name: a, joint: ja, slave_position: 1}
)", config);
  ASSERT_TRUE(errors.empty()) << joined(errors);
  const auto & sdos = config.drives[0].startup_sdos;
  ASSERT_EQ(sdos.size(), 3u);
  EXPECT_EQ(sdos[0].value, 100000);
  EXPECT_TRUE(sdos[0].required);
  EXPECT_EQ(sdos[1].subindex, 0x01);
  EXPECT_EQ(sdos[1].value, 4);
  EXPECT_FALSE(sdos[1].required);
  EXPECT_EQ(sdos[2].value, -1500);
  EXPECT_EQ(config.drives[0].expected_vendor_id, 0x1DDu);
}

TEST(ConfigLoader, UnknownKeyIsReportedWithItsLine)
{
  SystemConfig config;
  const auto errors = load_config_string(R"(
drive_defaults:
  encoder_ticks_per_rev: 2048
drives:
  - name: a
    joint: ja
    slave_position: 1
    gear_ration: 20
)", config);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_TRUE(contains(errors, "line 8"));
  EXPECT_TRUE(contains(errors, "drives[0] (a)"));
  EXPECT_TRUE(contains(errors, "unknown key 'gear_ration'"));
}

TEST(ConfigLoader, DriveIdentityIsNotAllowedInDefaults)
{
  SystemConfig config;
  const auto errors = load_config_string(std::string("drive_defaults:\n  slave_position: 1\n") + R"(
drives:
  - {name: a, joint: ja, slave_position: 1}
)", config);
  EXPECT_TRUE(contains(errors, "drive_defaults: unknown key 'slave_position'"));
}

TEST(ConfigLoader, BadValuesAreReported)
{
  SystemConfig config;
  const auto errors = load_config_string(R"(
bus:
  cycle_time_us: -4
  sync_mode: hardsync
  distributed_clocks: yes please
drive_defaults:
  encoder_ticks_per_rev: lots
  mode: velocity
  rx_pdo:
    entries:
      - {object: "6040", type: uint17, role: controlword}
      - {object: "0x6040:zz", type: uint16}
      - {object: "0x60FF:00", type: int32, role: target_speed}
  startup_sdos:
    - {object: "0x6083:00", type: uint32}
drives:
  - {name: a, joint: ja, slave_position: 1}
)", config);
  EXPECT_TRUE(contains(errors, "bus.cycle_time_us: value -4 is out of range"));
  EXPECT_TRUE(contains(errors, "expected hard_sync or soft_sync"));
  EXPECT_TRUE(contains(errors, "expected true or false"));
  EXPECT_TRUE(contains(errors, "'lots' is not a number"));
  EXPECT_TRUE(contains(errors, "expected csv, csp or cst"));
  EXPECT_TRUE(contains(errors, "entries[0].type: expected one of int8"));
  EXPECT_TRUE(contains(errors, "'0x6040:zz' is not an object address"));
  EXPECT_TRUE(contains(errors, "entries[2].role: expected one of"));
  EXPECT_TRUE(contains(errors, "missing 'value'"));
}

TEST(ConfigLoader, ObjectWithoutSubindexMeansSubindexZero)
{
  SystemConfig config;
  const auto errors = load_config_string(R"(
drive_defaults:
  encoder_ticks_per_rev: 2048
  fault_snapshot:
    - {object: "0x603F", type: uint16}
drives:
  - {name: a, joint: ja, slave_position: 1}
)", config);
  ASSERT_TRUE(errors.empty()) << joined(errors);
  EXPECT_EQ(config.drives[0].fault_snapshot[0].subindex, 0);
  EXPECT_EQ(config.drives[0].fault_snapshot[0].name, "0x603F:00");  // default name
}

TEST(ConfigLoader, StructuralErrors)
{
  SystemConfig config;
  EXPECT_TRUE(contains(load_config_string("bus: {}\n", config), "missing 'drives'"));
  EXPECT_TRUE(contains(load_config_string("drives: {a: 1}\n", config), "drives: expected a list"));
  EXPECT_TRUE(contains(load_config_string("- 1\n- 2\n", config), "must be a mapping"));
  EXPECT_TRUE(contains(load_config_string("bus: [\n", config), "YAML syntax error"));
  EXPECT_TRUE(contains(load_config_string(std::string(BASE) + "extra: 1\n", config), "unknown key 'extra'"));
  EXPECT_TRUE(contains(load_config_file("/nonexistent/drives.yaml", config), "cannot open configuration file"));
}

TEST(ConfigLoader, LoadingResetsPreviousContent)
{
  SystemConfig config = naviwall_config();
  ASSERT_TRUE(load_config_string(BASE, config).empty());
  EXPECT_EQ(config.drives.size(), 1u);
}

TEST(ConfigLoader, ModesInterfacesAndScales)
{
  SystemConfig config;
  const auto errors = load_config_string(R"(
drive_defaults:
  encoder_ticks_per_rev: 2048
drives:
  - name: arm
    joint: arm_joint
    slave_position: 1
    mode: csp
    rated_torque_nm: 0.8
    rx_pdo:
      entries:
        - {object: "0x6040:00", type: uint16, role: controlword}
        - {object: "0x607A:00", type: int32, role: target_position}
        - {object: "0x2201:00", type: uint8, interface: digital_outputs}
    tx_pdo:
      entries:
        - {object: "0x6041:00", type: uint16, role: statusword, interface: statusword}
        - {object: "0x6064:00", type: int32, role: position_actual}
        - {object: "0x6077:00", type: int16, role: torque_actual}
        - {object: "0x6078:00", type: int16, interface: current_actual, scale: 0.001}
)", config);
  ASSERT_TRUE(errors.empty()) << joined(errors);
  EXPECT_TRUE(validate(config).empty()) << joined(validate(config));
  const DriveConfig & drive = config.drives[0];
  EXPECT_EQ(drive.mode, OperationMode::CYCLIC_SYNC_POSITION);
  EXPECT_DOUBLE_EQ(drive.rated_torque_nm, 0.8);
  EXPECT_EQ(drive.rx_pdo[2].interface, "digital_outputs");
  EXPECT_EQ(drive.tx_pdo[0].role, PdoRole::STATUSWORD);
  EXPECT_EQ(drive.tx_pdo[0].interface, "statusword");
  EXPECT_DOUBLE_EQ(drive.tx_pdo[3].scale, 0.001);
}
