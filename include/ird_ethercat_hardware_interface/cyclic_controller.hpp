// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

// Per-cycle logic of the process-data thread, independent of EtherCAT:
//  - CiA 402 enable sequence through the cyclic controlword,
//  - runtime fault detection and bounded automatic recovery,
//  - velocity reference ramp after an enable.
//
// The real-time thread calls, every cycle:
//   flags = begin_cycle(shared)            snapshot of the shared flags
//   stage(flags, targets, statuswords, controlwords, references)
//   ... exchange the EtherCAT frame ...
//   events = evaluate(flags, statuswords, shared)
//
// `stage` uses the statuswords of the previous frame, `evaluate` those of the
// frame just received. All drives are handled together: one drive faulting
// stops all of them, and they are re-enabled together.

#ifndef IRD_ETHERCAT_HARDWARE_INTERFACE__CYCLIC_CONTROLLER_HPP_
#define IRD_ETHERCAT_HARDWARE_INTERFACE__CYCLIC_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>
#include <vector>

#include "ird_ethercat_hardware_interface/config.hpp"

namespace ird_ethercat_hardware_interface
{

/// Flags shared between the lifecycle callbacks, write() and the RT thread.
struct SharedFlags
{
  std::atomic<bool> enable_request{false};      ///< lifecycle -> RT: enable the drives and keep them enabled
  std::atomic<bool> operation_enabled{false};   ///< RT -> all: every drive is Operation Enabled, references are applied
  std::atomic<bool> force_zero{false};          ///< any -> RT: send zero references
  std::atomic<bool> enable_fault_abort{false};  ///< RT -> lifecycle: the enable sequence gave up (drive in FAULT)
};

/// Values of the shared flags read once at the start of a cycle.
struct CycleFlags
{
  bool enable_request{false};
  bool operation_enabled{false};
  bool force_zero{false};
};

/// What happened in a cycle, for the caller to log.
struct CycleEvents
{
  bool fault_detected{false};       ///< a drive faulted while in operation
  uint64_t fault_mask{0};           ///< bit i = drive i in FAULT (when fault_detected)
  bool recovery_scheduled{false};   ///< ... and a recovery attempt was scheduled
  bool recovery_exhausted{false};   ///< ... or no attempts were left: drives stay disabled
  bool reset_retry{false};          ///< a fault reset did not clear; another attempt scheduled
  bool reset_persistent{false};     ///< a fault reset did not clear and no attempts are left
  bool recovered{false};            ///< drives back in Operation Enabled after a fault
  bool budget_refilled{false};      ///< fault-free long enough: attempts reset to zero
  bool enable_aborted{false};       ///< a drive stayed in FAULT during the initial enable
  int attempt{0};                   ///< recovery attempt number (with the fields above)
};

class CyclicController
{
public:
  /// ramp_steps[i]: largest increase of drive i's reference per cycle while
  /// ramping up after an enable (drive units).
  CyclicController(const FaultHandlingConfig & faults, uint32_t cycle_time_us, std::vector<int32_t> ramp_steps);

  /// Restores the state of a freshly started process-data thread.
  void reset();

  static CycleFlags begin_cycle(const SharedFlags & shared);

  /// Computes the controlword and reference of every drive for the next frame.
  /// targets: references requested by write(). statuswords: last received.
  void stage(
      const CycleFlags & flags, const std::vector<int32_t> & targets, const std::vector<uint16_t> & statuswords,
      std::vector<uint16_t> & controlwords, std::vector<int32_t> & references);

  /// Updates the fault and enable state from the statuswords just received.
  CycleEvents evaluate(const CycleFlags & flags, const std::vector<uint16_t> & statuswords, SharedFlags & shared);

  uint32_t max_recovery_attempts() const { return max_attempts_; }
  uint32_t reset_pulse_cycles() const { return reset_pulse_limit_; }
  uint32_t budget_refill_cycles() const { return refill_cycles_; }

private:
  void slew(size_t i, int32_t target);

  // Configuration, in cycles
  uint32_t stable_cycles_;
  uint32_t abort_cycles_;
  uint32_t max_attempts_;
  uint32_t cooldown_cycles_;
  uint32_t reset_pulse_limit_;
  uint32_t refill_cycles_;
  std::vector<int32_t> ramp_steps_;

  // State
  std::vector<uint32_t> fault_cycles_;  ///< consecutive cycles each drive has been in FAULT
  std::vector<int32_t> applied_;        ///< references after the ramp
  bool ramp_active_{false};
  uint32_t stable_enabled_cycles_{0};
  bool in_recovery_{false};
  bool recovery_pulse_{false};
  uint32_t recovery_attempts_{0};
  uint32_t cooldown_{0};
  uint32_t reset_pulse_cycles_{0};
  uint32_t healthy_cycles_{0};
};

}  // namespace ird_ethercat_hardware_interface

#endif  // IRD_ETHERCAT_HARDWARE_INTERFACE__CYCLIC_CONTROLLER_HPP_
