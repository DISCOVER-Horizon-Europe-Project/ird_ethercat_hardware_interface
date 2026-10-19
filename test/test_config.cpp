// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "ird_ethercat_hardware_interface/config.hpp"
#include "test_helpers.hpp"

using namespace ird_ethercat_hardware_interface;

namespace
{
bool contains(const std::vector<std::string> & errors, const std::string & text)
{
  for (const auto & error : errors)
  {
    if (error.find(text) != std::string::npos) return true;
  }
  return false;
}
}  // namespace

TEST(Config, NaviwallConfigurationIsValid)
{
  EXPECT_TRUE(validate(naviwall_config()).empty());
}

TEST(Config, DefaultsConvertToTheValidatedCycleCounts)
{
  // v0.1.0 constants at 4 ms: 10 stable cycles, 25 abort cycles, 125 cooldown
  // cycles, 50 reset-pulse cycles, 7500 refill cycles, 50 DC lock cycles and
  // 250 settle cycles.
  const SystemConfig config = naviwall_config();
  const uint32_t cycle = config.bus.cycle_time_us;
  EXPECT_EQ(ms_to_cycles(config.faults.enable_stable_ms, cycle), 10u);
  EXPECT_EQ(ms_to_cycles(config.faults.enable_fault_abort_ms, cycle), 25u);
  EXPECT_EQ(ms_to_cycles(config.faults.recovery_cooldown_ms, cycle), 125u);
  EXPECT_EQ(ms_to_cycles(config.faults.reset_pulse_ms, cycle), 50u);
  EXPECT_EQ(ms_to_cycles(config.faults.recovery_budget_refill_ms, cycle), 7500u);
  EXPECT_EQ(ms_to_cycles(config.bus.dc_lock_ms, cycle), 50u);
  EXPECT_EQ(ms_to_cycles(config.bus.startup_settle_ms, cycle), 250u);
  EXPECT_EQ(config.faults.max_recovery_attempts, 5u);
  EXPECT_EQ(config.bus.sync0_shift_us, 2000u);
  EXPECT_EQ(config.bus.rt_priority, 80);
}

TEST(Config, MsToCyclesIsAtLeastOne)
{
  EXPECT_EQ(ms_to_cycles(0, 4000), 1u);
  EXPECT_EQ(ms_to_cycles(1, 4000), 1u);
  EXPECT_EQ(ms_to_cycles(10, 1000), 10u);
}

TEST(Config, DuplicatesAreRejected)
{
  SystemConfig config = naviwall_config();
  config.drives[1].name = config.drives[0].name;
  config.drives[2].slave_position = config.drives[0].slave_position;
  config.drives[2].joint = config.drives[1].joint;
  const auto errors = validate(config);
  EXPECT_TRUE(contains(errors, "used more than once"));
  EXPECT_TRUE(contains(errors, "is used by another drive"));
  EXPECT_TRUE(contains(errors, "driven by more than one drive"));
}

TEST(Config, MissingAndMistypedRolesAreRejected)
{
  SystemConfig config = naviwall_config();
  config.drives[0].tx_pdo.pop_back();                   // no velocity_actual
  config.drives[1].rx_pdo[2].type = DataType::UINT32;   // target_velocity must be int32
  config.drives[2].rx_pdo.push_back({0x6040, 1, DataType::UINT16, PdoRole::CONTROLWORD});  // twice
  const auto errors = validate(config);
  EXPECT_TRUE(contains(errors, "needs an entry with role velocity_actual"));
  EXPECT_TRUE(contains(errors, "must be of type int32"));
  EXPECT_TRUE(contains(errors, "role controlword appears 2 times"));
}

TEST(Config, RoleInTheWrongPdoIsRejected)
{
  SystemConfig config = naviwall_config();
  config.drives[0].rx_pdo.push_back({0x6041, 0, DataType::UINT16, PdoRole::STATUSWORD});
  EXPECT_TRUE(contains(validate(config), "role statusword is not allowed in the RxPDO"));
}

TEST(Config, CspNeedsPositionRoles)
{
  SystemConfig config = naviwall_config();
  config.drives[0].mode = OperationMode::CYCLIC_SYNC_POSITION;  // still the CSV mapping
  const auto errors = validate(config);
  EXPECT_TRUE(contains(errors, "needs an entry with role target_position"));
  EXPECT_TRUE(contains(errors, "needs an entry with role position_actual"));
  EXPECT_TRUE(contains(errors, "role target_velocity is not allowed in the RxPDO"));
}

TEST(Config, ValidCspAndCstDrives)
{
  SystemConfig config = naviwall_config();
  DriveConfig & csp = config.drives[0];
  csp.mode = OperationMode::CYCLIC_SYNC_POSITION;
  csp.rx_pdo = {{0x6040, 0, DataType::UINT16, PdoRole::CONTROLWORD},
                {0x6060, 0, DataType::INT8, PdoRole::MODE_OF_OPERATION},
                {0x607A, 0, DataType::INT32, PdoRole::TARGET_POSITION}};
  csp.tx_pdo = {{0x6041, 0, DataType::UINT16, PdoRole::STATUSWORD},
                {0x6064, 0, DataType::INT32, PdoRole::POSITION_ACTUAL},
                {0x606C, 0, DataType::INT32, PdoRole::VELOCITY_ACTUAL}};
  DriveConfig & cst = config.drives[1];
  cst.mode = OperationMode::CYCLIC_SYNC_TORQUE;
  cst.rated_torque_nm = 1.2;
  cst.rx_pdo = {{0x6040, 0, DataType::UINT16, PdoRole::CONTROLWORD},
                {0x6071, 0, DataType::INT16, PdoRole::TARGET_TORQUE}};
  cst.tx_pdo = {{0x6041, 0, DataType::UINT16, PdoRole::STATUSWORD},
                {0x606C, 0, DataType::INT32, PdoRole::VELOCITY_ACTUAL},
                {0x6077, 0, DataType::INT16, PdoRole::TORQUE_ACTUAL}};
  EXPECT_TRUE(validate(config).empty());

  cst.rated_torque_nm = 0.0;
  EXPECT_TRUE(contains(validate(config), "rated_torque_nm must be > 0"));
}

TEST(Config, TorqueModeStillNeedsAPositionOrVelocityFeedback)
{
  SystemConfig config = naviwall_config();
  DriveConfig & cst = config.drives[0];
  cst.mode = OperationMode::CYCLIC_SYNC_TORQUE;
  cst.rated_torque_nm = 1.0;
  cst.rx_pdo = {{0x6040, 0, DataType::UINT16, PdoRole::CONTROLWORD}, {0x6071, 0, DataType::INT16, PdoRole::TARGET_TORQUE}};
  cst.tx_pdo = {{0x6041, 0, DataType::UINT16, PdoRole::STATUSWORD}};
  EXPECT_TRUE(contains(validate(config), "needs position_actual or velocity_actual"));
}

TEST(Config, ExtraInterfaceRules)
{
  SystemConfig config = naviwall_config();
  DriveConfig & drive = config.drives[0];
  drive.tx_pdo.push_back({0x6078, 0, DataType::INT16, PdoRole::NONE, "current_actual", 0.001});
  drive.tx_pdo.push_back({0x6079, 0, DataType::UINT32, PdoRole::NONE, "dc_link_voltage", 0.001});
  drive.tx_pdo[0].interface = "statusword";  // a role entry can also be exposed (read-only)
  EXPECT_TRUE(validate(config).empty());

  drive.tx_pdo.push_back({0x2000, 0, DataType::UINT8});                                   // unused
  drive.tx_pdo.push_back({0x2001, 0, DataType::UINT8, PdoRole::NONE, "current_actual"});  // duplicate
  drive.tx_pdo.push_back({0x2002, 0, DataType::UINT8, PdoRole::NONE, "velocity"});        // reserved
  drive.tx_pdo.push_back({0x2003, 0, DataType::UINT8, PdoRole::NONE, "bad-name"});        // characters
  drive.tx_pdo.push_back({0x2004, 0, DataType::UINT8, PdoRole::NONE, "zero", 0.0});       // scale
  drive.rx_pdo[0].interface = "controlword";  // the driver writes it: cannot be a command interface
  const auto errors = validate(config);
  EXPECT_TRUE(contains(errors, "0x2000:00 has neither a role nor an interface name"));
  EXPECT_TRUE(contains(errors, "'current_actual' is used more than once"));
  EXPECT_TRUE(contains(errors, "'velocity' is reserved"));
  EXPECT_TRUE(contains(errors, "'bad-name' may only contain"));
  EXPECT_TRUE(contains(errors, "scale must be a non-zero number"));
  EXPECT_TRUE(contains(errors, "cannot also be a command interface"));
}

TEST(Config, CommandInterfaceNames)
{
  EXPECT_STREQ(command_interface_name(OperationMode::CYCLIC_SYNC_POSITION), "position");
  EXPECT_STREQ(command_interface_name(OperationMode::CYCLIC_SYNC_VELOCITY), "velocity");
  EXPECT_STREQ(command_interface_name(OperationMode::CYCLIC_SYNC_TORQUE), "effort");
}

TEST(Config, BusLimitsAreChecked)
{
  SystemConfig config = naviwall_config();
  config.bus.sync0_shift_us = config.bus.cycle_time_us;
  config.bus.rt_priority = 0;
  const auto errors = validate(config);
  EXPECT_TRUE(contains(errors, "sync0_shift_us"));
  EXPECT_TRUE(contains(errors, "rt_priority"));
}

TEST(Config, StartupSdoValueMustFitItsType)
{
  SystemConfig config = naviwall_config();
  config.drives[0].startup_sdos.push_back({0x6091, 0x01, DataType::UINT8, 300});
  EXPECT_TRUE(contains(validate(config), "does not fit in uint8"));
}

TEST(Config, UnitsMustBePositive)
{
  SystemConfig config = naviwall_config();
  config.drives[0].encoder_ticks_per_rev = 0;
  config.drives[1].gear_ratio = -1;
  const auto errors = validate(config);
  EXPECT_TRUE(contains(errors, "encoder_ticks_per_rev must be > 0"));
  EXPECT_TRUE(contains(errors, "gear_ratio must be > 0"));
}

TEST(Config, TypeAndRoleNamesRoundTrip)
{
  for (DataType type : {DataType::INT8, DataType::UINT8, DataType::INT16, DataType::UINT16, DataType::INT32,
                        DataType::UINT32})
  {
    DataType parsed{};
    ASSERT_TRUE(parse_data_type(to_string(type), parsed));
    EXPECT_EQ(parsed, type);
  }
  for (PdoRole role : {PdoRole::NONE, PdoRole::CONTROLWORD, PdoRole::MODE_OF_OPERATION, PdoRole::TARGET_POSITION,
                       PdoRole::TARGET_VELOCITY, PdoRole::TARGET_TORQUE, PdoRole::STATUSWORD, PdoRole::MODE_DISPLAY,
                       PdoRole::POSITION_ACTUAL, PdoRole::VELOCITY_ACTUAL, PdoRole::TORQUE_ACTUAL})
  {
    PdoRole parsed{};
    ASSERT_TRUE(parse_pdo_role(to_string(role), parsed));
    EXPECT_EQ(parsed, role);
  }
  DataType unused{};
  EXPECT_FALSE(parse_data_type("float", unused));
}
