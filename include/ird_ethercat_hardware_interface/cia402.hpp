// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

// CiA 402 (CANopen drive profile) statusword / controlword helpers.
//
// Enable sequence driven by this interface:
//   Switch On Disabled --0x0006 Shutdown--> Ready To Switch On
//   --0x0007 Switch On--> Switched On --0x000F Enable Operation--> Operation Enabled
// A drive in FAULT is only reset by an explicit, time-limited 0x0080 pulse.

#ifndef IRD_ETHERCAT_HARDWARE_INTERFACE__CIA402_HPP_
#define IRD_ETHERCAT_HARDWARE_INTERFACE__CIA402_HPP_

#include <cstdint>

namespace ird_ethercat_hardware_interface
{
namespace cia402
{

// Statusword (0x6041): state bits and the values they take in each state.
constexpr uint16_t SW_STATE_MASK = 0x006F;
constexpr uint16_t SW_SWITCH_ON_DISABLED = 0x0040;
constexpr uint16_t SW_READY_TO_SWITCH_ON = 0x0021;
constexpr uint16_t SW_SWITCHED_ON = 0x0023;
constexpr uint16_t SW_OPERATION_ENABLED = 0x0027;
constexpr uint16_t SW_FAULT_BIT = 0x0008;

// Controlword (0x6040) commands.
constexpr uint16_t CW_DISABLE_VOLTAGE = 0x0000;
constexpr uint16_t CW_SHUTDOWN = 0x0006;
constexpr uint16_t CW_SWITCH_ON = 0x0007;
constexpr uint16_t CW_ENABLE_OPERATION = 0x000F;
constexpr uint16_t CW_FAULT_RESET = 0x0080;

inline bool is_fault(uint16_t statusword)
{
  return (statusword & SW_FAULT_BIT) != 0;
}

inline bool is_operation_enabled(uint16_t statusword)
{
  return (statusword & SW_STATE_MASK) == SW_OPERATION_ENABLED;
}

/// Controlword that moves a drive one step along the enable sequence.
/// Faults are never reset here (repeating 0x0080 causes brake chatter);
/// a faulted drive gets 0x0000.
inline uint16_t next_controlword(uint16_t statusword)
{
  const uint16_t state = statusword & SW_STATE_MASK;
  if (state == SW_SWITCH_ON_DISABLED) return CW_SHUTDOWN;
  if (state == SW_READY_TO_SWITCH_ON) return CW_SWITCH_ON;
  if (state == SW_SWITCHED_ON) return CW_ENABLE_OPERATION;
  if (state == SW_OPERATION_ENABLED) return CW_ENABLE_OPERATION;  // hold enabled
  if (is_fault(statusword)) return CW_DISABLE_VOLTAGE;
  return CW_SHUTDOWN;  // any other state: safe shutdown
}

}  // namespace cia402
}  // namespace ird_ethercat_hardware_interface

#endif  // IRD_ETHERCAT_HARDWARE_INTERFACE__CIA402_HPP_
