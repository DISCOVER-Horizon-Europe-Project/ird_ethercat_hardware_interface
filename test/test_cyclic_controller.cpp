// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <random>
#include <vector>

#include "ird_ethercat_hardware_interface/cia402.hpp"
#include "ird_ethercat_hardware_interface/cyclic_controller.hpp"
#include "test_helpers.hpp"

using namespace ird_ethercat_hardware_interface;

namespace
{
constexpr int32_t RAMP_STEP = 91;  // 70 rad/s^2 * 4 ms at 2048 ticks/rev, as in v0.1.0

// =========================================================================
// Reference: the per-cycle logic of v0.1.0's processdata_loop(), transcribed
// verbatim (three named drives turned into vectors, nothing else changed).
// The new CyclicController must produce exactly the same outputs.
// =========================================================================
class V010Loop
{
public:
  explicit V010Loop(size_t n) : fault_cycles_(n, 0), applied_(n, 0) {}

  void stage(
      SharedFlags & shared, const std::vector<int32_t> & targets, const std::vector<uint16_t> & sws,
      std::vector<uint16_t> & cws, std::vector<int32_t> & tvs)
  {
    enable_request_ = shared.enable_request.load();
    operation_enabled_ = shared.operation_enabled.load();
    force_zero_ = shared.force_zero.load();

    auto slew = [&](int32_t target, int32_t & applied)
    {
      if (!ramp_active_)
      {
        applied = target;
        return;
      }
      const bool reversing = (target > 0 && applied < 0) || (target < 0 && applied > 0);
      if (target == 0 || reversing)
      {
        applied = 0;
      }
      else if (std::abs(target) <= std::abs(applied))
      {
        applied = target;
      }
      else
      {
        applied += std::clamp(target - applied, -RAMP_STEP, RAMP_STEP);
      }
    };

    if (enable_request_ && operation_enabled_ && !force_zero_)
    {
      for (size_t i = 0; i < applied_.size(); ++i) slew(targets[i], applied_[i]);
      bool all_equal = true;
      for (size_t i = 0; i < applied_.size(); ++i) all_equal = all_equal && applied_[i] == targets[i];
      if (ramp_active_ && all_equal) ramp_active_ = false;
    }
    else
    {
      std::fill(applied_.begin(), applied_.end(), 0);
    }

    for (size_t i = 0; i < sws.size(); ++i)
    {
      const uint16_t sw = sws[i];
      uint16_t cw = 0x0000;
      if (enable_request_)
      {
        if (operation_enabled_)
        {
          cw = (sw & 0x0008) ? 0x0000 : 0x000F;
        }
        else if (recovery_pulse_ && (sw & 0x0008))
        {
          cw = 0x0080;
        }
        else
        {
          cw = next_cw_from_sw(sw);
        }
      }
      cws[i] = cw;
      tvs[i] = (enable_request_ && operation_enabled_ && !force_zero_ && !(sw & 0x0008)) ? applied_[i] : 0;
    }
  }

  void evaluate(SharedFlags & shared, const std::vector<uint16_t> & sws)
  {
    for (size_t i = 0; i < sws.size(); ++i)
    {
      if (sws[i] & 0x0008) ++fault_cycles_[i];
      else fault_cycles_[i] = 0;
    }
    const bool any_fault = std::any_of(fault_cycles_.begin(), fault_cycles_.end(), [](int c) { return c > 0; });

    if (operation_enabled_ && !any_fault)
    {
      if (recovery_attempts_ > 0 && ++healthy_cycles_ >= 7500)
      {
        recovery_attempts_ = 0;
        healthy_cycles_ = 0;
      }
    }
    else
    {
      healthy_cycles_ = 0;
    }

    if (operation_enabled_ && any_fault)
    {
      shared.operation_enabled.store(false);
      shared.force_zero.store(true);
      if (recovery_attempts_ < 5)
      {
        ++recovery_attempts_;
        in_recovery_ = true;
        recovery_pulse_ = false;
        fault_cooldown_ = 125;
        reset_pulse_cycles_ = 0;
      }
      else
      {
        shared.enable_request.store(false);
        in_recovery_ = false;
      }
    }

    if (enable_request_ && !operation_enabled_)
    {
      if (in_recovery_)
      {
        if (fault_cooldown_ > 0)
        {
          --fault_cooldown_;
          recovery_pulse_ = false;
        }
        else if (any_fault)
        {
          recovery_pulse_ = true;
          ++reset_pulse_cycles_;
          if (reset_pulse_cycles_ > 50)
          {
            recovery_pulse_ = false;
            reset_pulse_cycles_ = 0;
            if (recovery_attempts_ < 5)
            {
              ++recovery_attempts_;
              fault_cooldown_ = 125;
            }
            else
            {
              in_recovery_ = false;
              shared.enable_request.store(false);
            }
          }
        }
        else
        {
          recovery_pulse_ = false;
        }
      }

      const bool all_enabled = std::all_of(
          sws.begin(), sws.end(), [](uint16_t sw) { return (sw & 0x006F) == 0x0027; });
      if (all_enabled)
      {
        ++stable_enabled_cycles_;
        if (stable_enabled_cycles_ >= 10)
        {
          ramp_active_ = true;
          std::fill(applied_.begin(), applied_.end(), 0);
          shared.operation_enabled.store(true);
          shared.force_zero.store(false);
          if (in_recovery_)
          {
            in_recovery_ = false;
            recovery_pulse_ = false;
            reset_pulse_cycles_ = 0;
          }
        }
      }
      else
      {
        stable_enabled_cycles_ = 0;
      }

      if (!in_recovery_ && std::any_of(fault_cycles_.begin(), fault_cycles_.end(), [](int c) { return c >= 25; }))
      {
        shared.enable_fault_abort.store(true);
        shared.enable_request.store(false);
        shared.operation_enabled.store(false);
      }
    }
  }

private:
  static uint16_t next_cw_from_sw(uint16_t sw)
  {
    const uint16_t sw_m = (sw & 0x006F);
    if (sw_m == 0x0040) return 0x0006;
    if (sw_m == 0x0021) return 0x0007;
    if (sw_m == 0x0023) return 0x000F;
    if (sw_m == 0x0027) return 0x000F;
    if (sw & 0x0008) return 0x0000;
    return 0x0006;
  }

  bool enable_request_{false};
  bool operation_enabled_{false};
  bool force_zero_{false};
  std::vector<int> fault_cycles_;
  std::vector<int32_t> applied_;
  bool ramp_active_{false};
  int stable_enabled_cycles_{0};
  bool in_recovery_{false};
  bool recovery_pulse_{false};
  int recovery_attempts_{0};
  int fault_cooldown_{0};
  int reset_pulse_cycles_{0};
  int healthy_cycles_{0};
};

// =========================================================================
// Harness: a CyclicController driving simulated drives.
// =========================================================================
struct Bench
{
  explicit Bench(size_t n = 3) : Bench(std::vector<DriveMotion>(n, DriveMotion{OperationMode::CYCLIC_SYNC_VELOCITY, RAMP_STEP})) {}

  explicit Bench(const std::vector<DriveMotion> & motion)
  : controller(naviwall_config().faults, 4000, motion),
    drives(motion.size()), targets(motion.size(), 0), statuswords(motion.size(), 0),
    actual_positions(motion.size(), 0), controlwords(motion.size(), 0), references(motion.size(), 0)
  {
    for (size_t i = 0; i < motion.size(); ++i) statuswords[i] = drives[i].statusword();
  }

  CycleEvents cycle()
  {
    const CycleFlags f = CyclicController::begin_cycle(flags);
    controller.stage(f, targets, statuswords, actual_positions, controlwords, references);
    if (frame_ok)  // otherwise the drives get nothing and the statuswords stay stale
    {
      for (size_t i = 0; i < drives.size(); ++i)
      {
        drives[i].receive(controlwords[i]);
        statuswords[i] = drives[i].statusword();
      }
    }
    return controller.evaluate(f, statuswords, frame_ok, flags);
  }

  /// Runs until `done` or `limit` cycles; returns the number of cycles run.
  template<typename Pred>
  int run_until(Pred done, int limit)
  {
    for (int i = 1; i <= limit; ++i)
    {
      last = cycle();
      if (done(last)) return i;
    }
    return -1;
  }

  void enable_and_wait()
  {
    flags.enable_request = true;
    ASSERT_GT(run_until([this](const CycleEvents &) { return flags.operation_enabled.load(); }, 100), 0);
  }

  CyclicController controller;
  SharedFlags flags;
  std::vector<FakeDrive> drives;
  std::vector<int32_t> targets;
  std::vector<uint16_t> statuswords;
  std::vector<int32_t> actual_positions;
  std::vector<uint16_t> controlwords;
  std::vector<int32_t> references;
  bool frame_ok{true};
  CycleEvents last;
};

}  // namespace

TEST(CyclicController, EnableSequenceTakesThreeTransitionsPlusTheStablePeriod)
{
  Bench bench;
  bench.flags.enable_request = true;

  std::vector<uint16_t> sent;
  const int cycles = bench.run_until(
      [&](const CycleEvents &)
      {
        sent.push_back(bench.controlwords[0]);
        return bench.flags.operation_enabled.load();
      },
      100);

  // Shutdown, Switch On, Enable Operation, then 10 stable cycles (the third
  // transition is the first of them).
  EXPECT_EQ(cycles, 3 + 10 - 1);
  ASSERT_GE(sent.size(), 3u);
  EXPECT_EQ(sent[0], cia402::CW_SHUTDOWN);
  EXPECT_EQ(sent[1], cia402::CW_SWITCH_ON);
  EXPECT_EQ(sent[2], cia402::CW_ENABLE_OPERATION);
  EXPECT_FALSE(bench.flags.force_zero.load());
}

TEST(CyclicController, NoReferenceBeforeOperationEnabled)
{
  Bench bench;
  bench.flags.enable_request = true;
  std::fill(bench.targets.begin(), bench.targets.end(), 5000);
  for (int i = 0; i < 12; ++i)
  {
    bench.cycle();
    EXPECT_EQ(bench.references[0], 0) << "cycle " << i;
  }
}

TEST(CyclicController, ReferencesRampUpAfterEnableThenFollowImmediately)
{
  Bench bench;
  bench.enable_and_wait();

  std::fill(bench.targets.begin(), bench.targets.end(), 1000);
  std::vector<int32_t> refs;
  for (int i = 0; i < 15; ++i)
  {
    bench.cycle();
    refs.push_back(bench.references[0]);
  }
  EXPECT_EQ(refs[0], RAMP_STEP);
  EXPECT_EQ(refs[1], 2 * RAMP_STEP);
  EXPECT_EQ(refs[10], 1000);  // 11 steps of 91 reach 1001 -> capped at the target

  // Ramp finished: a new, larger target is applied at once.
  std::fill(bench.targets.begin(), bench.targets.end(), 8000);
  bench.cycle();
  EXPECT_EQ(bench.references[0], 8000);
}

TEST(CyclicController, StopIsNeverRamped)
{
  Bench bench;
  bench.enable_and_wait();
  std::fill(bench.targets.begin(), bench.targets.end(), 5000);
  bench.cycle();
  bench.cycle();
  std::fill(bench.targets.begin(), bench.targets.end(), 0);
  bench.cycle();
  EXPECT_EQ(bench.references[0], 0);
}

TEST(CyclicController, ForceZeroKeepsDrivesEnabledWithZeroReference)
{
  Bench bench;
  bench.enable_and_wait();
  std::fill(bench.targets.begin(), bench.targets.end(), 3000);
  bench.flags.force_zero = true;
  for (int i = 0; i < 5; ++i)
  {
    bench.cycle();
    EXPECT_EQ(bench.references[1], 0);
    EXPECT_EQ(bench.controlwords[1], cia402::CW_ENABLE_OPERATION);
  }
}

TEST(CyclicController, TransientFaultIsRecovered)
{
  Bench bench;
  bench.enable_and_wait();
  std::fill(bench.targets.begin(), bench.targets.end(), 2000);
  bench.run_until([](const CycleEvents &) { return false; }, 50);

  bench.drives[1].inject_fault();
  const CycleEvents detected = bench.cycle();
  EXPECT_TRUE(detected.fault_detected);
  EXPECT_TRUE(detected.recovery_scheduled);
  EXPECT_EQ(detected.fault_mask, 0b010u);
  EXPECT_EQ(detected.attempt, 1);
  EXPECT_FALSE(bench.flags.operation_enabled.load());
  EXPECT_TRUE(bench.flags.force_zero.load());

  // During the 125-cycle cooldown: no reset pulse, zero references.
  for (int i = 0; i < 125; ++i)
  {
    bench.cycle();
    EXPECT_NE(bench.controlwords[1], cia402::CW_FAULT_RESET) << "cooldown cycle " << i;
    EXPECT_EQ(bench.references[0], 0);
  }

  // Then a reset pulse, the enable sequence and the stable period.
  const int cycles = bench.run_until([](const CycleEvents & e) { return e.recovered; }, 100);
  EXPECT_EQ(cycles, 1 + 1 + 3 + 10 - 1);
  EXPECT_TRUE(bench.flags.operation_enabled.load());
  EXPECT_FALSE(bench.flags.force_zero.load());

  // References ramp up again.
  bench.cycle();
  EXPECT_EQ(bench.references[0], RAMP_STEP);
}

TEST(CyclicController, PersistentFaultExhaustsTheAttempts)
{
  Bench bench;
  bench.enable_and_wait();
  bench.drives[2].inject_fault(/*clears_on_reset=*/false);

  int retries = 0;
  const int cycles = bench.run_until(
      [&](const CycleEvents & e)
      {
        retries += e.reset_retry ? 1 : 0;
        return e.reset_persistent;
      },
      10'000);
  ASSERT_GT(cycles, 0);
  EXPECT_EQ(retries, 4);  // attempts 2..5 after the first one
  EXPECT_FALSE(bench.flags.enable_request.load());
  EXPECT_FALSE(bench.flags.operation_enabled.load());

  // Afterwards everything is disabled.
  bench.cycle();
  for (uint16_t cw : bench.controlwords) EXPECT_EQ(cw, cia402::CW_DISABLE_VOLTAGE);
}

TEST(CyclicController, BudgetRefillsAfterThirtySecondsWithoutFaults)
{
  Bench bench;
  bench.enable_and_wait();
  bench.drives[0].inject_fault();
  ASSERT_GT(bench.run_until([](const CycleEvents & e) { return e.recovered; }, 1000), 0);
  const int cycles = bench.run_until([](const CycleEvents & e) { return e.budget_refilled; }, 10'000);
  EXPECT_EQ(cycles, 7500);
}

TEST(CyclicController, DriveInFaultAbortsTheInitialEnable)
{
  Bench bench;
  bench.drives[0].inject_fault();
  bench.flags.enable_request = true;
  const int cycles = bench.run_until([](const CycleEvents & e) { return e.enable_aborted; }, 100);
  EXPECT_EQ(cycles, 25);
  EXPECT_TRUE(bench.flags.enable_fault_abort.load());
  EXPECT_FALSE(bench.flags.enable_request.load());
}

TEST(CyclicController, BusLossStopsTheDrivesAfterTheTimeout)
{
  Bench bench;
  bench.enable_and_wait();
  std::fill(bench.targets.begin(), bench.targets.end(), 2000);
  bench.run_until([](const CycleEvents &) { return false; }, 50);

  // E-stop: no frame comes back. The stale statuswords still read
  // Operation Enabled, so no fault is seen; 100 ms = 25 cycles.
  bench.frame_ok = false;
  const int cycles = bench.run_until([](const CycleEvents & e) { return e.bus_lost; }, 1000);
  EXPECT_EQ(cycles, 25);
  EXPECT_EQ(bench.controller.bus_loss_cycles(), 25u);
  EXPECT_TRUE(bench.flags.bus_lost.load());
  EXPECT_FALSE(bench.flags.enable_request.load());
  EXPECT_FALSE(bench.flags.operation_enabled.load());
  EXPECT_TRUE(bench.flags.force_zero.load());
  EXPECT_FALSE(bench.last.fault_detected);

  bench.cycle();
  for (uint16_t cw : bench.controlwords) EXPECT_EQ(cw, cia402::CW_DISABLE_VOLTAGE);
  for (int32_t ref : bench.references) EXPECT_EQ(ref, 0);
}

TEST(CyclicController, NoRecoveryAfterABusLoss)
{
  Bench bench;
  bench.enable_and_wait();
  bench.frame_ok = false;
  ASSERT_GT(bench.run_until([](const CycleEvents & e) { return e.bus_lost; }, 1000), 0);

  // The drives answer again, in FAULT (as after a fieldbus error): they stay
  // disabled, nothing is reset and no further event is reported.
  bench.frame_ok = true;
  for (auto & drive : bench.drives) drive.inject_fault();
  for (int i = 0; i < 10'000; ++i)
  {
    const CycleEvents e = bench.cycle();
    ASSERT_FALSE(e.bus_lost || e.fault_detected || e.recovered || e.recovery_scheduled) << "cycle " << i;
    for (uint16_t cw : bench.controlwords) ASSERT_EQ(cw, cia402::CW_DISABLE_VOLTAGE) << "cycle " << i;
  }
  EXPECT_TRUE(bench.flags.bus_lost.load());

  // Until the next activation.
  bench.controller.reset();
  bench.flags.bus_lost = false;
  for (auto & drive : bench.drives) drive = FakeDrive();
  bench.enable_and_wait();
}

TEST(CyclicController, ShortFrameLossesAreTolerated)
{
  Bench bench;
  bench.enable_and_wait();
  std::fill(bench.targets.begin(), bench.targets.end(), 2000);
  for (int burst = 0; burst < 20; ++burst)
  {
    bench.frame_ok = false;
    for (int i = 0; i < 24; ++i) ASSERT_FALSE(bench.cycle().bus_lost);
    bench.frame_ok = true;
    ASSERT_FALSE(bench.cycle().bus_lost);
  }
  EXPECT_TRUE(bench.flags.operation_enabled.load());
  EXPECT_FALSE(bench.flags.bus_lost.load());
  EXPECT_EQ(bench.references[0], 2000);
}

TEST(CyclicController, IncompleteFramesOutsideOperationAreIgnored)
{
  // Before the enable (SAFE-OP) and after a stop, the working counter is
  // legitimately short.
  Bench bench;
  bench.frame_ok = false;
  for (int i = 0; i < 1000; ++i) ASSERT_FALSE(bench.cycle().bus_lost);

  bench.frame_ok = true;
  bench.enable_and_wait();
  bench.flags.enable_request = false;
  bench.flags.operation_enabled = false;
  bench.frame_ok = false;
  for (int i = 0; i < 1000; ++i) ASSERT_FALSE(bench.cycle().bus_lost);
  EXPECT_FALSE(bench.flags.bus_lost.load());
}

TEST(CyclicController, BusLossDuringTheEnableIsDetected)
{
  Bench bench;
  bench.flags.enable_request = true;
  bench.cycle();
  bench.frame_ok = false;
  EXPECT_EQ(bench.run_until([](const CycleEvents & e) { return e.bus_lost; }, 1000), 25);
  EXPECT_FALSE(bench.flags.operation_enabled.load());
}

TEST(CyclicController, BusLossTimeoutZeroNeverDetects)
{
  FaultHandlingConfig faults = naviwall_config().faults;
  faults.bus_loss_timeout_ms = 0;
  CyclicController controller(faults, 4000, std::vector<DriveMotion>(3));
  SharedFlags flags;
  flags.enable_request = true;
  flags.operation_enabled = true;
  const std::vector<uint16_t> statuswords(3, 0x0637);
  for (int i = 0; i < 10'000; ++i)
  {
    ASSERT_FALSE(controller.evaluate(CyclicController::begin_cycle(flags), statuswords, false, flags).bus_lost);
  }
  EXPECT_EQ(controller.bus_loss_cycles(), 0u);
}

TEST(CyclicController, IdenticalToV010UnderRandomFaults)
{
  constexpr size_t N = 3;
  Bench bench(N);
  V010Loop reference(N);
  SharedFlags ref_flags;
  std::vector<FakeDrive> ref_drives(N);
  std::vector<uint16_t> ref_sw(N), ref_cw(N);
  std::vector<int32_t> ref_tv(N), ref_targets(N);
  for (size_t i = 0; i < N; ++i) ref_sw[i] = ref_drives[i].statusword();

  std::mt19937 rng(20261019);
  std::uniform_int_distribution<int> percent(0, 99);
  std::uniform_int_distribution<int> which(0, N - 1);
  std::uniform_int_distribution<int32_t> speed(-20000, 20000);

  bench.flags.enable_request = true;
  ref_flags.enable_request = true;
  int disabled_cycles = 0;
  int faults = 0;
  int recoveries = 0;

  for (int cycle = 0; cycle < 600'000; ++cycle)
  {
    // Commands as write() stages them (zero unless enabled and not forced).
    if (percent(rng) < 2)
    {
      const int32_t command = percent(rng) < 20 ? 0 : speed(rng);
      const size_t i = which(rng);
      bench.targets[i] = command;
      ref_targets[i] = command;
    }
    for (size_t i = 0; i < N; ++i)
    {
      const bool apply = bench.flags.operation_enabled && !bench.flags.force_zero;
      const bool ref_apply = ref_flags.operation_enabled && !ref_flags.force_zero;
      ASSERT_EQ(apply, ref_apply);
      if (!apply) bench.targets[i] = ref_targets[i] = 0;
    }

    // Random faults, about one every 6 s; 30 % of them do not clear on reset.
    if (rng() % 1500 == 0)
    {
      const size_t i = which(rng);
      const bool clears = percent(rng) >= 30;
      bench.drives[i].inject_fault(clears);
      ref_drives[i].inject_fault(clears);
      ++faults;
    }

    // After the drives gave up, re-activate a while later, as a relaunch
    // would: the process-data thread restarts with fresh state.
    if (!bench.flags.enable_request)
    {
      if (++disabled_cycles == 1000)
      {
        disabled_cycles = 0;
        bench.controller.reset();
        reference = V010Loop(N);
        for (size_t i = 0; i < N; ++i)
        {
          bench.drives[i].reset_clears = ref_drives[i].reset_clears = true;
          bench.drives[i].receive(0x0080);
          ref_drives[i].receive(0x0080);
          bench.statuswords[i] = bench.drives[i].statusword();
          ref_sw[i] = ref_drives[i].statusword();
        }
        for (SharedFlags * f : {&bench.flags, &ref_flags})
        {
          f->enable_fault_abort = false;
          f->operation_enabled = false;
          f->force_zero = false;
          f->enable_request = true;
        }
      }
    }

    // One cycle of each implementation.
    const CycleEvents events = bench.cycle();
    recoveries += events.recovered ? 1 : 0;

    reference.stage(ref_flags, ref_targets, ref_sw, ref_cw, ref_tv);
    for (size_t i = 0; i < N; ++i)
    {
      ref_drives[i].receive(ref_cw[i]);
      ref_sw[i] = ref_drives[i].statusword();
    }
    reference.evaluate(ref_flags, ref_sw);

    ASSERT_EQ(bench.controlwords, ref_cw) << "cycle " << cycle;
    ASSERT_EQ(bench.references, ref_tv) << "cycle " << cycle;
    ASSERT_EQ(bench.flags.enable_request.load(), ref_flags.enable_request.load()) << "cycle " << cycle;
    ASSERT_EQ(bench.flags.operation_enabled.load(), ref_flags.operation_enabled.load()) << "cycle " << cycle;
    ASSERT_EQ(bench.flags.force_zero.load(), ref_flags.force_zero.load()) << "cycle " << cycle;
    ASSERT_EQ(bench.flags.enable_fault_abort.load(), ref_flags.enable_fault_abort.load()) << "cycle " << cycle;
  }

  // The scenario must actually have exercised the recovery paths.
  EXPECT_GT(faults, 50);
  EXPECT_GT(recoveries, 20);
}

// =========================================================================
// CSP and CST
// =========================================================================
namespace
{
constexpr int32_t MAX_STEP = 500;  // ticks per cycle

Bench csp_bench()
{
  return Bench(std::vector<DriveMotion>{{OperationMode::CYCLIC_SYNC_POSITION, 0, MAX_STEP}});
}
}  // namespace

TEST(CyclicControllerCsp, HoldsTheActualPositionUntilEnabled)
{
  Bench bench = csp_bench();
  bench.actual_positions[0] = 123456;
  bench.targets[0] = 0;  // a stale command must not be applied
  bench.flags.enable_request = true;
  for (int i = 0; i < 12; ++i)
  {
    bench.cycle();
    EXPECT_EQ(bench.references[0], 123456) << "cycle " << i;
  }
  EXPECT_TRUE(bench.flags.operation_enabled.load());
}

TEST(CyclicControllerCsp, PositionStepIsLimited)
{
  Bench bench = csp_bench();
  bench.actual_positions[0] = 1000;
  bench.targets[0] = 1000;
  bench.enable_and_wait();

  bench.targets[0] = 1000 + 10 * MAX_STEP + 123;
  std::vector<int32_t> refs;
  for (int i = 0; i < 12; ++i)
  {
    bench.cycle();
    refs.push_back(bench.references[0]);
  }
  EXPECT_EQ(refs[0], 1000 + MAX_STEP);
  EXPECT_EQ(refs[9], 1000 + 10 * MAX_STEP);
  EXPECT_EQ(refs[10], 1000 + 10 * MAX_STEP + 123);
  EXPECT_EQ(refs[11], 1000 + 10 * MAX_STEP + 123);
}

TEST(CyclicControllerCsp, StartsFromTheActualPositionAfterEnable)
{
  Bench bench = csp_bench();
  bench.actual_positions[0] = -50000;
  bench.targets[0] = -49000;
  bench.enable_and_wait();
  bench.cycle();
  EXPECT_EQ(bench.references[0], -50000 + MAX_STEP);
}

TEST(CyclicControllerCsp, ForceZeroHoldsPosition)
{
  Bench bench = csp_bench();
  bench.actual_positions[0] = 777;
  bench.targets[0] = 777;
  bench.enable_and_wait();
  bench.flags.force_zero = true;
  bench.targets[0] = 999999;
  bench.cycle();
  EXPECT_EQ(bench.references[0], 777);
}

TEST(CyclicControllerCsp, MovesAcrossTheInt32Wrap)
{
  Bench bench = csp_bench();
  bench.actual_positions[0] = INT32_MAX - 100;
  bench.targets[0] = INT32_MAX - 100;
  bench.enable_and_wait();
  bench.targets[0] = INT32_MIN + 99;  // 200 ticks further, past the wrap
  bench.cycle();
  EXPECT_EQ(bench.references[0], INT32_MIN + 99);
}

TEST(CyclicControllerCst, TorqueFollowsTheCommandAndIsZeroWhenIdle)
{
  Bench bench(std::vector<DriveMotion>{{OperationMode::CYCLIC_SYNC_TORQUE}});
  bench.targets[0] = 300;
  bench.flags.enable_request = true;
  bench.cycle();
  EXPECT_EQ(bench.references[0], 0);
  bench.enable_and_wait();
  bench.cycle();
  EXPECT_EQ(bench.references[0], 300);
  bench.targets[0] = -250;  // reversing is applied at once
  bench.cycle();
  EXPECT_EQ(bench.references[0], -250);
  bench.flags.force_zero = true;
  bench.cycle();
  EXPECT_EQ(bench.references[0], 0);
}
