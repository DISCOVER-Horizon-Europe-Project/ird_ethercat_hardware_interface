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
  config.drives[2].rx_pdo.push_back({0x6040, 0, DataType::UINT16, PdoRole::CONTROLWORD});  // twice
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

TEST(Config, OnlyCsvIsAcceptedForNow)
{
  SystemConfig config = naviwall_config();
  config.drives[0].mode = OperationMode::CYCLIC_SYNC_POSITION;
  EXPECT_TRUE(contains(validate(config), "only mode CSV"));
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
  for (PdoRole role : {PdoRole::NONE, PdoRole::CONTROLWORD, PdoRole::MODE_OF_OPERATION, PdoRole::TARGET_VELOCITY,
                       PdoRole::STATUSWORD, PdoRole::MODE_DISPLAY, PdoRole::VELOCITY_ACTUAL})
  {
    PdoRole parsed{};
    ASSERT_TRUE(parse_pdo_role(to_string(role), parsed));
    EXPECT_EQ(parsed, role);
  }
  DataType unused{};
  EXPECT_FALSE(parse_data_type("float", unused));
}
