// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

#include "ird_ethercat_hardware_interface/cyclic_controller.hpp"

#include <algorithm>
#include <cstdlib>
#include <utility>

#include "ird_ethercat_hardware_interface/cia402.hpp"

namespace ird_ethercat_hardware_interface
{

CyclicController::CyclicController(
    const FaultHandlingConfig & faults, uint32_t cycle_time_us, std::vector<DriveMotion> drives)
: stable_cycles_(ms_to_cycles(faults.enable_stable_ms, cycle_time_us)),
  abort_cycles_(ms_to_cycles(faults.enable_fault_abort_ms, cycle_time_us)),
  max_attempts_(faults.max_recovery_attempts),
  cooldown_cycles_(ms_to_cycles(faults.recovery_cooldown_ms, cycle_time_us)),
  reset_pulse_limit_(ms_to_cycles(faults.reset_pulse_ms, cycle_time_us)),
  refill_cycles_(ms_to_cycles(faults.recovery_budget_refill_ms, cycle_time_us)),
  motion_(std::move(drives))
{
  reset();
}

void CyclicController::reset()
{
  fault_cycles_.assign(motion_.size(), 0);
  applied_.assign(motion_.size(), 0);
  last_actual_.assign(motion_.size(), 0);
  ramp_active_ = false;
  stable_enabled_cycles_ = 0;
  in_recovery_ = false;
  recovery_pulse_ = false;
  recovery_attempts_ = 0;
  cooldown_ = 0;
  reset_pulse_cycles_ = 0;
  healthy_cycles_ = 0;
}

CycleFlags CyclicController::begin_cycle(const SharedFlags & shared)
{
  return {shared.enable_request.load(), shared.operation_enabled.load(), shared.force_zero.load()};
}

// Post-enable ramp: the reference catches up with the target by at most
// motion_[i].ramp_step per cycle. Stopping, reversing and slowing down are applied
// at once (never delay a stop).
void CyclicController::slew(size_t i, int32_t target)
{
  int32_t & applied = applied_[i];
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
    applied += std::clamp(target - applied, -motion_[i].ramp_step, motion_[i].ramp_step);
  }
}

// CSP: the position reference moves towards the target by at most
// max_position_step per cycle (the speed limit). Positions wrap around at
// the int32 limits like the drive's own position, hence the modular difference.
void CyclicController::step_position(size_t i, int32_t target)
{
  const int32_t step = motion_[i].max_position_step;
  const int64_t diff = static_cast<int32_t>(static_cast<uint32_t>(target) - static_cast<uint32_t>(applied_[i]));
  const int64_t move = std::clamp<int64_t>(diff, -step, step);
  applied_[i] = static_cast<int32_t>(static_cast<uint32_t>(applied_[i]) + static_cast<uint32_t>(move));
}

void CyclicController::stage(
    const CycleFlags & flags, const std::vector<int32_t> & targets, const std::vector<uint16_t> & statuswords,
    const std::vector<int32_t> & actual_positions, std::vector<uint16_t> & controlwords,
    std::vector<int32_t> & references)
{
  const size_t n = applied_.size();
  std::copy(actual_positions.begin(), actual_positions.end(), last_actual_.begin());

  // ---- References (after the ramp / step limit) ----
  if (flags.enable_request && flags.operation_enabled && !flags.force_zero)
  {
    bool caught_up = true;  // all ramped (CSV/CST) references match their targets
    for (size_t i = 0; i < n; ++i)
    {
      if (motion_[i].mode == OperationMode::CYCLIC_SYNC_POSITION)
      {
        step_position(i, targets[i]);
        continue;
      }
      slew(i, targets[i]);
      caught_up = caught_up && applied_[i] == targets[i];
    }
    if (ramp_active_ && caught_up)
    {
      ramp_active_ = false;
    }
  }
  else
  {
    for (size_t i = 0; i < n; ++i)
    {
      applied_[i] = idle_reference(i);
    }
  }

  // ---- Controlwords ----
  for (size_t i = 0; i < n; ++i)
  {
    const uint16_t sw = statuswords[i];
    uint16_t cw = cia402::CW_DISABLE_VOLTAGE;
    if (flags.enable_request)
    {
      if (flags.operation_enabled)
      {
        // In operation: hold the drive enabled. A fault is not reset here
        // (that avoids brake/power-stage chatter); evaluate() handles it.
        cw = cia402::is_fault(sw) ? cia402::CW_DISABLE_VOLTAGE : cia402::CW_ENABLE_OPERATION;
      }
      else if (recovery_pulse_ && cia402::is_fault(sw))
      {
        // Time-limited fault-reset pulse while recovering from a runtime fault.
        cw = cia402::CW_FAULT_RESET;
      }
      else
      {
        cw = cia402::next_controlword(sw);
      }
    }
    controlwords[i] = cw;

    const bool apply = flags.enable_request && flags.operation_enabled && !flags.force_zero && !cia402::is_fault(sw);
    references[i] = apply ? applied_[i] : idle_reference(i);
  }
}

CycleEvents CyclicController::evaluate(
    const CycleFlags & flags, const std::vector<uint16_t> & statuswords, SharedFlags & shared)
{
  CycleEvents events;
  const size_t n = fault_cycles_.size();

  bool any_fault = false;
  uint64_t fault_mask = 0;
  for (size_t i = 0; i < n; ++i)
  {
    fault_cycles_[i] = cia402::is_fault(statuswords[i]) ? fault_cycles_[i] + 1 : 0;
    if (fault_cycles_[i] > 0)
    {
      any_fault = true;
      fault_mask |= uint64_t{1} << i;
    }
  }

  // ---- Recovery budget: refilled after a long enough fault-free period ----
  // The attempt limit is per incident, not per session, so occasional
  // transient fieldbus errors over a long session never strand the machine.
  if (flags.operation_enabled && !any_fault)
  {
    if (recovery_attempts_ > 0 && ++healthy_cycles_ >= refill_cycles_)
    {
      recovery_attempts_ = 0;
      healthy_cycles_ = 0;
      events.budget_refilled = true;
    }
  }
  else
  {
    healthy_cycles_ = 0;
  }

  // ---- Fault while in operation: stop everything, schedule a recovery ----
  // Error 49/50 (fieldbus timing) are transient: they clear as soon as the
  // frames are regular again, so a single blip must not disable the machine
  // until relaunch. Other faults do not clear with a reset and exhaust the
  // attempts.
  if (flags.operation_enabled && any_fault)
  {
    events.fault_detected = true;
    events.fault_mask = fault_mask;
    shared.operation_enabled.store(false);
    shared.force_zero.store(true);

    if (recovery_attempts_ < max_attempts_)
    {
      ++recovery_attempts_;
      in_recovery_ = true;
      recovery_pulse_ = false;
      cooldown_ = cooldown_cycles_;
      reset_pulse_cycles_ = 0;
      events.recovery_scheduled = true;
    }
    else
    {
      shared.enable_request.store(false);
      in_recovery_ = false;
      events.recovery_exhausted = true;
    }
    events.attempt = static_cast<int>(recovery_attempts_);
  }

  // ---- Enabling (initially or after a fault) ----
  if (flags.enable_request && !flags.operation_enabled)
  {
    if (in_recovery_)
    {
      if (cooldown_ > 0)
      {
        // Wait before the reset: spaces out reset pulses (brake chatter) if
        // the fault keeps coming back.
        --cooldown_;
        recovery_pulse_ = false;
      }
      else if (any_fault)
      {
        recovery_pulse_ = true;
        if (++reset_pulse_cycles_ > reset_pulse_limit_)
        {
          // The fault does not clear: not a transient fieldbus error.
          recovery_pulse_ = false;
          reset_pulse_cycles_ = 0;
          if (recovery_attempts_ < max_attempts_)
          {
            ++recovery_attempts_;
            cooldown_ = cooldown_cycles_;
            events.reset_retry = true;
          }
          else
          {
            in_recovery_ = false;
            shared.enable_request.store(false);
            events.reset_persistent = true;
          }
          events.attempt = static_cast<int>(recovery_attempts_);
        }
      }
      else
      {
        // Fault cleared: stage() now runs the enable sequence again.
        recovery_pulse_ = false;
      }
    }

    const bool all_enabled = std::all_of(statuswords.begin(), statuswords.end(), cia402::is_operation_enabled);
    if (all_enabled)
    {
      if (++stable_enabled_cycles_ >= stable_cycles_)
      {
        ramp_active_ = true;
        for (size_t i = 0; i < n; ++i)
        {
          applied_[i] = idle_reference(i);
        }
        shared.operation_enabled.store(true);
        shared.force_zero.store(false);
        if (in_recovery_)
        {
          in_recovery_ = false;
          recovery_pulse_ = false;
          reset_pulse_cycles_ = 0;
          events.recovered = true;
        }
      }
    }
    else
    {
      stable_enabled_cycles_ = 0;
    }

    // During the initial enable (no recovery running), a drive that stays
    // in FAULT aborts the activation.
    const bool stuck_in_fault = std::any_of(
        fault_cycles_.begin(), fault_cycles_.end(), [this](uint32_t cycles) { return cycles >= abort_cycles_; });
    if (!in_recovery_ && stuck_in_fault)
    {
      shared.enable_fault_abort.store(true);
      shared.enable_request.store(false);
      shared.operation_enabled.store(false);
      events.enable_aborted = true;
    }
  }

  return events;
}

}  // namespace ird_ethercat_hardware_interface
