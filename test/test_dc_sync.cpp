// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>

#include "ird_ethercat_hardware_interface/dc_sync.hpp"

using ird_ethercat_hardware_interface::DcSync;

namespace
{
constexpr int64_t CYCLE = 4'000'000;
constexpr int64_t BASE = 1'000'000 * CYCLE;  // an arbitrary DC time on a cycle boundary

// v0.1.0's sync_linux_to_dc(), with its integral passed explicitly.
int64_t v010_sync(int64_t reftime, int64_t & integral)
{
  int64_t delta = reftime % CYCLE;
  if (delta > CYCLE / 2) delta -= CYCLE;
  const int64_t timeerror = -delta;
  integral += timeerror;
  return static_cast<int64_t>((timeerror * 0.01f) + (integral * 0.00002f));
}
}  // namespace

TEST(DcSync, PhaseWrapsToHalfCycle)
{
  const DcSync dc(CYCLE, 0);
  EXPECT_EQ(dc.phase_ns(BASE), 0);
  EXPECT_EQ(dc.phase_ns(BASE + 1500), 1500);
  EXPECT_EQ(dc.phase_ns(BASE - 1500), -1500);
  EXPECT_EQ(dc.phase_ns(BASE + CYCLE / 2), -CYCLE / 2);
  EXPECT_EQ(dc.phase_ns(BASE + CYCLE / 2 - 1), CYCLE / 2 - 1);
}

TEST(DcSync, LateFrameShortensTheNextPeriod)
{
  DcSync dc(CYCLE, 0);
  EXPECT_LT(dc.update(BASE + 100'000), 0);
  dc.reset();
  EXPECT_GT(dc.update(BASE - 100'000), 0);
}

TEST(DcSync, MatchesV010Controller)
{
  DcSync dc(CYCLE, 0);
  int64_t integral = 0;
  int64_t dc_time = BASE + 1'234'567;
  for (int i = 0; i < 5000; ++i)
  {
    const int64_t expected = v010_sync(dc_time, integral);
    ASSERT_EQ(dc.update(dc_time), expected) << "cycle " << i;
    EXPECT_EQ(dc.integral_term(), static_cast<int64_t>(integral * 0.00002f));
    // Next frame lands where the correction puts it, plus some jitter.
    dc_time += CYCLE + expected + ((i * 7919) % 41 - 20) * 1000;
  }
}

TEST(DcSync, ConvergesToTheTargetPhase)
{
  DcSync dc(CYCLE, 0);
  int64_t dc_time = BASE + 300'000;  // starts 300 us late
  for (int i = 0; i < 3000; ++i)
  {
    dc_time += CYCLE + dc.update(dc_time);
  }
  EXPECT_LT(std::abs(dc.phase_ns(dc_time)), 1'000);
}

TEST(DcSync, SyncDifferenceRegisterIsSignMagnitude)
{
  using ird_ethercat_hardware_interface::decode_dc_sync_difference;
  EXPECT_EQ(decode_dc_sync_difference(0x00000013u), 19);
  EXPECT_EQ(decode_dc_sync_difference(0x80000013u), -19);
  EXPECT_EQ(decode_dc_sync_difference(0x7FFFFFFFu), 0x7FFFFFFF);
  EXPECT_EQ(decode_dc_sync_difference(0x80000000u), 0);
}

TEST(DcSync, EthercatTimeCountsFrom2000)
{
  using ird_ethercat_hardware_interface::to_ethercat_time_ns;
  EXPECT_EQ(to_ethercat_time_ns(946684800, 5), 5);
  EXPECT_EQ(to_ethercat_time_ns(946684801, 0), 1'000'000'000);
  EXPECT_LT(to_ethercat_time_ns(0, 0), 0);  // a host clock still at 1970
}
