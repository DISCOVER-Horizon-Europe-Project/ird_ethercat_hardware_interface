// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

// Synchronisation of the master's cycle to the drives' distributed clock.
//
// The drives latch their references on SYNC0, which their DC clock generates
// every cycle at `sync0_shift` after the cycle boundary. The master cannot
// change that clock; instead it steers its own wake-up time so that its frame
// passes the DC reference drive at a fixed phase (the cycle boundary, i.e.
// `sync0_shift` ahead of SYNC0). Every cycle the received DC time gives the
// current phase, and a PI controller turns the phase error into a small
// correction of the next sleep period.

#ifndef IRD_ETHERCAT_HARDWARE_INTERFACE__DC_SYNC_HPP_
#define IRD_ETHERCAT_HARDWARE_INTERFACE__DC_SYNC_HPP_

#include <cstdint>

namespace ird_ethercat_hardware_interface
{

class DcSync
{
public:
  /// cycle_ns: process-data cycle. target_phase_ns: where the frame should
  /// pass the reference drive, relative to the DC cycle boundary.
  DcSync(int64_t cycle_ns, int64_t target_phase_ns) : cycle_ns_(cycle_ns), target_phase_ns_(target_phase_ns) {}

  /// Forgets the integral term (call before a new start-up).
  void reset() { integral_ = 0; }

  /// Phase of a frame with DC time `dc_time_ns`, in [-cycle/2, cycle/2).
  int64_t phase_ns(int64_t dc_time_ns) const
  {
    int64_t delta = (dc_time_ns - target_phase_ns_) % cycle_ns_;
    if (delta >= cycle_ns_ / 2)
    {
      delta -= cycle_ns_;
    }
    else if (delta < -(cycle_ns_ / 2))
    {
      delta += cycle_ns_;
    }
    return delta;
  }

  /// PI step: returns the correction [ns] to add to the next cycle period.
  int64_t update(int64_t dc_time_ns)
  {
    int64_t delta = (dc_time_ns - target_phase_ns_) % cycle_ns_;
    if (delta > cycle_ns_ / 2)
    {
      delta -= cycle_ns_;
    }
    const int64_t error = -delta;
    integral_ += error;
    return static_cast<int64_t>((error * PGAIN) + (integral_ * IGAIN));
  }

  /// Integral part of the last correction [ns], for diagnostics.
  int64_t integral_term() const { return static_cast<int64_t>(integral_ * IGAIN); }

private:
  static constexpr float PGAIN = 0.01f;
  static constexpr float IGAIN = 0.00002f;

  int64_t cycle_ns_;
  int64_t target_phase_ns_;
  int64_t integral_{0};
};

}  // namespace ird_ethercat_hardware_interface

#endif  // IRD_ETHERCAT_HARDWARE_INTERFACE__DC_SYNC_HPP_
