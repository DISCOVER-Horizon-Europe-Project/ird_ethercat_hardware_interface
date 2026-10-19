// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "ird_ethercat_hardware_interface/startup_plan.hpp"
#include "test_helpers.hpp"

using namespace ird_ethercat_hardware_interface;

namespace
{
struct Expected
{
  uint16_t index;
  uint8_t subindex;
  DataType type;
  int64_t value;
  bool required;
};

void expect_plan(const std::vector<SdoWrite> & plan, const std::vector<Expected> & expected)
{
  ASSERT_EQ(plan.size(), expected.size());
  for (size_t i = 0; i < plan.size(); ++i)
  {
    SCOPED_TRACE(i);
    EXPECT_EQ(plan[i].index, expected[i].index);
    EXPECT_EQ(plan[i].subindex, expected[i].subindex);
    EXPECT_EQ(plan[i].type, expected[i].type);
    EXPECT_EQ(plan[i].value, expected[i].value);
    EXPECT_EQ(plan[i].required, expected[i].required);
  }
}

// PDO mapping writes of v0.1.0's configure_slave(), in order.
const std::vector<Expected> V010_MAPPING = {
    {0x1C12, 0x00, DataType::UINT8, 0, true},
    {0x1C13, 0x00, DataType::UINT8, 0, true},
    {0x1600, 0x00, DataType::UINT8, 0, true},
    {0x1A00, 0x00, DataType::UINT8, 0, true},
    {0x1600, 0x01, DataType::UINT32, 0x60400010, true},
    {0x1600, 0x02, DataType::UINT32, 0x60600008, true},
    {0x1600, 0x03, DataType::UINT32, 0x60FF0020, true},
    {0x1600, 0x00, DataType::UINT8, 3, true},
    {0x1A00, 0x01, DataType::UINT32, 0x60410010, true},
    {0x1A00, 0x02, DataType::UINT32, 0x60610008, true},
    {0x1A00, 0x03, DataType::UINT32, 0x606C0020, true},
    {0x1A00, 0x00, DataType::UINT8, 3, true},
    {0x1C12, 0x01, DataType::UINT16, 0x1600, true},
    {0x1C12, 0x00, DataType::UINT8, 1, true},
    {0x1C13, 0x01, DataType::UINT16, 0x1A00, true},
    {0x1C13, 0x00, DataType::UINT8, 1, true},
};

std::vector<Expected> with(std::vector<Expected> base, const std::vector<Expected> & tail)
{
  base.insert(base.end(), tail.begin(), tail.end());
  return base;
}
}  // namespace

TEST(StartupPlan, HardSyncMatchesV010)
{
  const SystemConfig config = naviwall_config();
  for (const DriveConfig & drive : config.drives)
  {
    expect_plan(
        build_startup_plan(drive, config.bus),
        with(V010_MAPPING, {
                               {0x1006, 0x00, DataType::UINT32, 4000, false},
                               {0x2012, 0x01, DataType::UINT16, 4, false},
                           }));
  }
}

TEST(StartupPlan, SoftSyncMatchesV010)
{
  SystemConfig config = naviwall_config();
  config.bus.sync_mode = SyncMode::SOFT_SYNC;
  expect_plan(
      build_startup_plan(config.drives[0], config.bus),
      with(V010_MAPPING, {{0x2012, 0x01, DataType::UINT16, 2, false}}));
}

TEST(StartupPlan, NoSyncObjectsWithoutDistributedClocks)
{
  SystemConfig config = naviwall_config();
  config.bus.distributed_clocks = false;
  expect_plan(build_startup_plan(config.drives[0], config.bus), V010_MAPPING);
}

TEST(StartupPlan, CycleTimeComesFromTheBus)
{
  SystemConfig config = naviwall_config();
  config.bus.cycle_time_us = 2000;
  const auto plan = build_startup_plan(config.drives[0], config.bus);
  EXPECT_EQ(plan[plan.size() - 2].index, 0x1006);
  EXPECT_EQ(plan[plan.size() - 2].value, 2000);
}

TEST(StartupPlan, UserSdosComeFirst)
{
  SystemConfig config = naviwall_config();
  config.drives[0].startup_sdos = {{0x6091, 0x01, DataType::UINT32, 1}, {0x6091, 0x02, DataType::UINT32, 1}};
  const auto plan = build_startup_plan(config.drives[0], config.bus);
  ASSERT_EQ(plan.size(), V010_MAPPING.size() + 4);
  EXPECT_EQ(plan[0].index, 0x6091);
  EXPECT_EQ(plan[1].subindex, 0x02);
  EXPECT_EQ(plan[2].index, 0x1C12);
}

TEST(StartupPlan, CustomMapping)
{
  SystemConfig config = naviwall_config();
  DriveConfig & drive = config.drives[0];
  drive.tx_pdo.push_back({0x6064, 0x00, DataType::INT32, PdoRole::NONE});
  drive.tx_pdo_index = 0x1A01;
  const auto plan = build_startup_plan(drive, config.bus);
  // 4 clears + 4 Rx + 5 Tx + 4 assigns + 2 sync
  ASSERT_EQ(plan.size(), 19u);
  EXPECT_EQ(plan[3].index, 0x1A01);
  EXPECT_EQ(plan[11].value, 0x60640020);
  EXPECT_EQ(plan[12].value, 4);  // entry count
  EXPECT_EQ(plan[15].value, 0x1A01);
}
