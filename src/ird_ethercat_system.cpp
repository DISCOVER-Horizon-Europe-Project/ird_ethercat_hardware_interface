// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only
//
// Portions (lifecycle skeleton, joint interface checks) derived from the
// ros2_control demos, Copyright 2021 ros2_control Development Team,
// licensed under the Apache License, Version 2.0.
//
// ros2_control hardware interface for Infranor IRD servo drives on EtherCAT.
//
// Life of the interface:
//   on_init      read the URDF parameters, check the joint interfaces.
//   on_activate  1. open the NIC, scan the bus, configure every drive by SDO
//                   in PRE-OP (mode CSV, PDO mapping, sync mode),
//                2. start distributed clocks (SYNC0) and go to SAFE-OP,
//                3. start the RT process-data thread and wait until the
//                   frames are phase-locked to the drives' DC clock,
//                4. go to OPERATIONAL and run the CiA 402 enable sequence
//                   through the cyclic controlword (no SDO at runtime).
//   read/write   exchange joint values with the RT thread (atomics only).
//   on_deactivate / on_shutdown
//                disable the drives through the cyclic PDO, drop to
//                SAFE-OP, then stop the RT thread and close the bus.

#include "ird_ethercat_hardware_interface/ird_ethercat_system.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <pthread.h>
#include <set>
#include <sys/mman.h>
#include <type_traits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ird_ethercat_hardware_interface
{
namespace
{
// =========================================================================
// Timing and synchronisation
// =========================================================================

constexpr int64_t PROCESS_CYCLE_NS = 4'000'000;  // EtherCAT cycle: 4 ms (250 Hz)
constexpr uint32_t PROCESS_CYCLE_US = 4'000;     // same, as written to 0x1006:00
constexpr int PROCESSDATA_THREAD_PRIORITY = 80;  // SCHED_FIFO priority of the RT thread
constexpr bool USE_DISTRIBUTED_CLOCKS = true;
// Request HardSync (0x2012:01 = 4) by pre-configuring 0x1006/0x2012 in
// PRE-OP before the PS transition. If false, the drives auto-configure
// SoftSync (= 2) from SYNC0/SM2 events, which tolerates far more master
// jitter. On the Naviwall robot (EtherCAT sharing the NIC with lidar
// traffic), SoftSync faulted 15-25x more often (error 49 on arrival jitter)
// than HardSync's ~2-3 error-50 faults per hour.
constexpr bool REQUEST_HARDSYNC = true;
// IRD manual (error 50 troubleshooting): the reference RPDO must precede the
// SYNC signal by at least 100 us. Infranor support recommends 200 us of
// master lead for a 4 ms cycle (TwinCAT default). On a NIC shared with
// sensor traffic the jitter tail exceeds 100 us: with a 1 ms SYNC0 shift
// (~900 us headroom) the drives still faulted every 10-15 min (frames
// reached the drives up to 1.6 ms late). SYNC0 now sits mid-cycle, 2 ms
// after the frame, giving ~1.9 ms of headroom; the most the 4 ms cycle
// allows before a frame could land ahead of the previous SYNC0 (overflow).
// Cost: references apply 2 ms later, negligible at these speeds.
constexpr int64_t DC_SYNC_SHIFT_NS = 2'000'000;
constexpr int64_t DC_MASTER_ADVANCE_NS = 2'000'000;
constexpr int64_t DC_SYNC_OFFSET_NS = DC_SYNC_SHIFT_NS - DC_MASTER_ADVANCE_NS;
// Gains of the PI loop that steers the master's cycle onto the DC clock.
constexpr float DC_PI_PGAIN = 0.01f;
constexpr float DC_PI_IGAIN = 0.00002f;
// Before requesting OPERATIONAL, on_activate() waits until the frame has
// stayed this close to its DC phase target for DC_LOCK_CYCLES in a row.
// Without it the drives were enabled while the PI loop was still pulling in
// from the RT thread's arbitrary start phase (first 10 s windows reached
// 1876 us and -1999 us against SYNC0 at 2000 us).
constexpr int32_t DC_LOCK_TOLERANCE_US = 200;
constexpr uint32_t DC_LOCK_CYCLES = 50;  // 200 ms at 250 Hz
constexpr int DC_LOCK_TIMEOUT_MS = 3000;
// Cycles exchanged in SAFE-OP (all outputs zero) to let the DC loop settle
// before the RT thread starts.
constexpr size_t STARTUP_SETTLE_CYCLES = 250;  // 1 s at 250 Hz

// =========================================================================
// Enable sequence and fault recovery
// =========================================================================

constexpr auto ENABLE_TIMEOUT = std::chrono::seconds(5);
// All drives must report Operation Enabled for this many consecutive cycles
// before the interface accepts velocity references.
constexpr int ENABLE_STABLE_CYCLES = 10;
// During the initial enable sequence, a drive stuck in FAULT this many
// cycles aborts activation.
constexpr int ENABLE_FAULT_ABORT_CYCLES = 25;
// Bounded automatic recovery from runtime fieldbus faults (error 49/50 are
// transient master-timing errors; a single blip must not disable the robot
// until relaunch). Non-communication faults exhaust the retries and latch.
constexpr int MAX_FAULT_RECOVERY_ATTEMPTS = 5;
// Error 49/50 clear as soon as the RPDO stream is regular again, i.e. one
// cycle later; the cooldown only has to space out reset pulses (brake
// chatter) if the fault keeps coming back, and every 100 ms of it is time
// the robot stands still before resuming.
constexpr int RECOVERY_COOLDOWN_CYCLES = 125;   // 0.5 s at 250 Hz
constexpr int RECOVERY_RESET_PULSE_LIMIT = 50;  // 200 ms of 0x0080
// After this long fault-free in Operation Enabled, the retry budget refills:
// the limit is per incident, not per activation, so occasional transient
// fieldbus errors over a long session never strand the robot.
constexpr int RECOVERY_BUDGET_REFILL_CYCLES = 7500;  // 30 s at 250 Hz

// After (re-)enabling, the controller's own acceleration limiter still sits
// at the pre-fault command, so without this the first write() would step the
// reference from 0 to full speed in one 4 ms cycle: a jerk for the machine
// and itself a known error-50 trigger. Ramp the motor-shaft reference up at
// this rate until it has caught up with the command; slowing down, stopping
// and reversing are never delayed. (On the Naviwall robot: 70 rad/s^2 at the
// motor is ~0.44 m/s^2 at the wheel, gentler than its controllers' limit.)
constexpr double ENABLE_RAMP_MOTOR_RAD_S2 = 70.0;

// Motor-shaft speed limit; commands above it are clamped.
constexpr double MAX_RPM = 4910.0;
constexpr double MAX_MOTOR_RAD_S = MAX_RPM * 2.0 * M_PI / 60.0;

// =========================================================================
// Diagnostics
// =========================================================================

// The diagnostics thread owns the SDO fault snapshots (each SDO read can
// block up to EC_TIMEOUTRXM = 700 ms) and the periodic ESC error-counter
// reads, so neither runs in read() where it would stall ros2_control.
constexpr int DIAG_POLL_MS = 20;
constexpr int ESC_COUNTERS_PERIOD_MS = 10'000;
constexpr int32_t TIMING_LATE_WAKE_US = 200;  // marks a cycle as anomalous in the timing dump

// =========================================================================
// CiA 402 and PDO layout
// =========================================================================

constexpr int8_t MODE_CSV = 9;  // Cyclic Synchronous Velocity

constexpr uint16_t STATUS_MASK = 0x006F;  // CiA 402 state bits of the statusword
constexpr uint16_t SW_SWITCH_ON_DISABLED = 0x0040;
constexpr uint16_t SW_READY_TO_SWITCH_ON = 0x0021;
constexpr uint16_t SW_SWITCHED_ON = 0x0023;
constexpr uint16_t SW_OPERATION_ENABLED = 0x0027;
constexpr uint16_t SW_FAULT_BIT = 0x0008;

constexpr uint16_t CW_DISABLE_VOLTAGE = 0x0000;
constexpr uint16_t CW_SHUTDOWN = 0x0006;
constexpr uint16_t CW_SWITCH_ON = 0x0007;
constexpr uint16_t CW_ENABLE_OPERATION = 0x000F;
constexpr uint16_t CW_FAULT_RESET = 0x0080;

// RxPDO 0x1600 (master -> drive): 6040 controlword (16) + 6060 mode (8) + 60FF target velocity (32)
constexpr int RX_PDO_SIZE = 7;
constexpr int RX_OFF_CW = 0;
constexpr int RX_OFF_MODE = 2;
constexpr int RX_OFF_TV = 3;
// TxPDO 0x1A00 (drive -> master): 6041 statusword (16) + 6061 mode display (8) + 606C actual velocity (32)
constexpr int TX_PDO_SIZE = 7;
constexpr int TX_OFF_SW = 0;
constexpr int TX_OFF_VEL = 3;

// =========================================================================
// Small helpers
// =========================================================================

void add_time_ns(ec_timet * ts, int64_t addtime)
{
  ec_timet addts;
  addts.tv_nsec = addtime % 1000000000LL;
  addts.tv_sec = (addtime - addts.tv_nsec) / 1000000000LL;
  osal_timespecadd(ts, &addts, ts);
}

int64_t ns_between(const ec_timet & from, const ec_timet & to)
{
  return (static_cast<int64_t>(to.tv_sec) - from.tv_sec) * 1'000'000'000LL +
         (static_cast<int64_t>(to.tv_nsec) - from.tv_nsec);
}

// Where the frame passed the DC reference slave relative to the cycle
// boundary (same phase as sync_linux_to_dc() regulates), in [-cycle/2, cycle/2).
int64_t dc_phase_ns(int64_t dctime, int64_t cycletime)
{
  int64_t delta = (dctime - DC_SYNC_OFFSET_NS) % cycletime;
  if (delta >= (cycletime / 2))
  {
    delta -= cycletime;
  }
  else if (delta < -(cycletime / 2))
  {
    delta += cycletime;
  }
  return delta;
}

// Little-endian access to the process image.
inline uint16_t rd_u16(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

inline int32_t rd_i32(const uint8_t * p)
{
  return static_cast<int32_t>(
      static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
      (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24));
}

inline void wr_u16(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

inline void wr_i32(uint8_t * p, int32_t v)
{
  const uint32_t u = static_cast<uint32_t>(v);
  p[0] = static_cast<uint8_t>(u & 0xFF);
  p[1] = static_cast<uint8_t>((u >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((u >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((u >> 24) & 0xFF);
}

inline void copy_from_process(const uint8_t * src, std::vector<uint8_t> & dst)
{
  if (!src || dst.empty()) return;
  std::memcpy(dst.data(), src, dst.size());
}

inline void copy_to_process(const std::vector<uint8_t> & src, uint8_t * dst)
{
  if (!dst || src.empty()) return;
  std::memcpy(dst, src.data(), src.size());
}

// Controlword that moves a drive one step along the CiA 402 enable sequence.
uint16_t next_cw_from_sw(uint16_t sw)
{
  const uint16_t state = sw & STATUS_MASK;

  if (state == SW_SWITCH_ON_DISABLED) return CW_SHUTDOWN;
  if (state == SW_READY_TO_SWITCH_ON) return CW_SWITCH_ON;
  if (state == SW_SWITCHED_ON) return CW_ENABLE_OPERATION;
  if (state == SW_OPERATION_ENABLED) return CW_ENABLE_OPERATION;  // hold enabled

  // Do not auto-reset faults here. Fault reset is handled explicitly before
  // OP and by the bounded recovery; repeating 0x0080 can cause brake chatter.
  if (sw & SW_FAULT_BIT) return CW_DISABLE_VOLTAGE;

  return CW_SHUTDOWN;  // any other state: safe shutdown
}

// Typed SDO read. CA (complete access) = FALSE.
template<typename T>
bool sdo_read(ecx_contextt * ctx, uint16_t slave, uint16_t idx, uint8_t sub, T & out)
{
  int size = sizeof(out);
  out = 0;
  const int wkc = ecx_SDOread(ctx, slave, idx, sub, FALSE, &size, &out, EC_TIMEOUTRXM);
  return wkc > 0 && size == static_cast<int>(sizeof(out));
}

const char * describe_drive_error_code(uint16_t code)
{
  switch (code)
  {
    case 0x0000: return "no active drive error";
    case 0x8181: return "extension error / fieldbus watchdog error";
    case 0x8780: return "fieldbus SYNC cycle error (error 49: master cycle jitter/period out of tolerance)";
    case 0x8782: return "fieldbus IP reference underflow/overflow (error 50: reference RPDO missing or <100us before SYNC)";
    case 0x8130: return "fieldbus guarding error (error 51)";
    default: return "see IRD user manual 0x603F table";
  }
}

const char * describe_al_status_code(uint16_t code)
{
  switch (code)
  {
    case 0x0000: return "no AL error";
    case 0x001B: return "RPDO watchdog issue";
    case 0x0026: return "inconsistent sync/DC settings";
    default: return "see EtherCAT interface manual";
  }
}

void log_soem_error_stack(ecx_contextt * ctx, const rclcpp::Logger & logger, const char * prefix)
{
  ec_errort err{};
  while (ecx_poperror(ctx, &err))
  {
    RCLCPP_ERROR(
        logger, "%s SOEM error: slave=%u type=%d idx=0x%04X sub=0x%02X abort=0x%08X",
        prefix, err.Slave, static_cast<int>(err.Etype), err.Index, err.SubIdx,
        static_cast<uint32_t>(err.AbortCode));
  }
}

void log_slave_mailbox_context(ecx_contextt * ctx, const rclcpp::Logger & logger, uint16_t slave, const char * prefix)
{
  const auto & sl = ctx->slavelist[slave];
  RCLCPP_ERROR(
      logger, "%s slave %u context: state=0x%02X AL=0x%04X configadr=0x%04X mbx_l=%u mbx_rl=%u hasdc=%d",
      prefix, slave, sl.state, sl.ALstatuscode, sl.configadr, sl.mbx_l, sl.mbx_rl, sl.hasdc);
}

// Reads the drive's error objects by SDO and logs them in one line.
// latched_before_activation: the fault was already set when activation
// started (typically error 50 left by a previous session that was killed
// before its clean shutdown) and is about to be reset, so it is logged as a
// warning that cannot be mistaken for a runtime fault.
void dump_drive_fault_snapshot(
    ecx_contextt * ctx, const rclcpp::Logger & logger, uint16_t slave, const char * name,
    uint16_t statusword, uint16_t al_statuscode, bool latched_before_activation = false)
{
  uint16_t error_code = 0;      // 0x603F:00 CiA 402 error code
  uint8_t error_register = 0;   // 0x1001:00 error register
  uint32_t error_bits_1 = 0;    // 0x3022:01 IRD error bits
  uint32_t error_bits_2 = 0;    // 0x3022:02 IRD error bits
  uint32_t warning_code = 0;    // 0x3024:00 IRD warning code

  const bool ok_603f = sdo_read(ctx, slave, 0x603F, 0x00, error_code);
  const bool ok_1001 = sdo_read(ctx, slave, 0x1001, 0x00, error_register);
  const bool ok_3022_1 = sdo_read(ctx, slave, 0x3022, 0x01, error_bits_1);
  const bool ok_3022_2 = sdo_read(ctx, slave, 0x3022, 0x02, error_bits_2);
  const bool ok_3024 = sdo_read(ctx, slave, 0x3024, 0x00, warning_code);

  char details[512];
  std::snprintf(
      details, sizeof(details),
      "SW=0x%04X  AL=0x%04X (%s)  0x603F=%s0x%04X%s (%s)  "
      "0x1001=%s0x%02X%s  0x3022:01=%s0x%08X%s  0x3022:02=%s0x%08X%s  0x3024:00=%s0x%08X%s",
      statusword,
      al_statuscode, describe_al_status_code(al_statuscode),
      ok_603f ? "" : "FAIL(", ok_603f ? error_code : 0, ok_603f ? "" : ")",
      ok_603f ? describe_drive_error_code(error_code) : "read failed",
      ok_1001 ? "" : "FAIL(", ok_1001 ? error_register : 0, ok_1001 ? "" : ")",
      ok_3022_1 ? "" : "FAIL(", ok_3022_1 ? error_bits_1 : 0, ok_3022_1 ? "" : ")",
      ok_3022_2 ? "" : "FAIL(", ok_3022_2 ? error_bits_2 : 0, ok_3022_2 ? "" : ")",
      ok_3024 ? "" : "FAIL(", ok_3024 ? warning_code : 0, ok_3024 ? "" : ")");

  if (latched_before_activation)
  {
    RCLCPP_WARN(
        logger,
        "%s (slave %u) fault latched before activation (left by a previous session that did not shut "
        "down cleanly; not a runtime fault, being cleared now): %s",
        name, slave, details);
  }
  else
  {
    RCLCPP_ERROR(logger, "%s (slave %u) fault snapshot: %s", name, slave, details);
  }
}

void log_dc_snapshot(ecx_contextt * ctx, const rclcpp::Logger & logger, uint16_t slave)
{
  uint8_t dccuc = 0;
  uint8_t dcsyncact = 0;
  uint64_t start0 = 0;
  uint32_t cycle0 = 0;

  const uint16_t configadr = ctx->slavelist[slave].configadr;
  const int wkc_dccuc = ecx_FPRD(&ctx->port, configadr, ECT_REG_DCCUC, sizeof(dccuc), &dccuc, EC_TIMEOUTRET);
  const int wkc_syncact = ecx_FPRD(&ctx->port, configadr, ECT_REG_DCSYNCACT, sizeof(dcsyncact), &dcsyncact, EC_TIMEOUTRET);
  const int wkc_start0 = ecx_FPRD(&ctx->port, configadr, ECT_REG_DCSTART0, sizeof(start0), &start0, EC_TIMEOUTRET);
  const int wkc_cycle0 = ecx_FPRD(&ctx->port, configadr, ECT_REG_DCCYCLE0, sizeof(cycle0), &cycle0, EC_TIMEOUTRET);

  RCLCPP_DEBUG(
      logger,
      "Slave %u DC snapshot: configadr=0x%04X hasdc=%d SOEM{active=%u cycle=%d shift=%d} "
      "ESC{DCCUC=%s0x%02X%s DCSYNCACT=%s0x%02X%s START0=%s0x%016llX%s CYCLE0=%s%u%s}",
      slave, configadr, ctx->slavelist[slave].hasdc, ctx->slavelist[slave].DCactive,
      ctx->slavelist[slave].DCcycle, ctx->slavelist[slave].DCshift,
      wkc_dccuc > 0 ? "" : "FAIL(", wkc_dccuc > 0 ? dccuc : 0, wkc_dccuc > 0 ? "" : ")",
      wkc_syncact > 0 ? "" : "FAIL(", wkc_syncact > 0 ? dcsyncact : 0, wkc_syncact > 0 ? "" : ")",
      wkc_start0 > 0 ? "" : "FAIL(", wkc_start0 > 0 ? static_cast<unsigned long long>(start0) : 0ULL,
      wkc_start0 > 0 ? "" : ")",
      wkc_cycle0 > 0 ? "" : "FAIL(", wkc_cycle0 > 0 ? cycle0 : 0U, wkc_cycle0 > 0 ? "" : ")");
}

}  // namespace

// =========================================================================
// ros2_control: initialisation and interfaces
// =========================================================================

hardware_interface::CallbackReturn IrdEthercatSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  rx_buffer_left_.assign(RX_PDO_SIZE, 0);
  tx_buffer_left_.assign(TX_PDO_SIZE, 0);
  rx_buffer_right_.assign(RX_PDO_SIZE, 0);
  tx_buffer_right_.assign(TX_PDO_SIZE, 0);
  rx_buffer_turret_.assign(RX_PDO_SIZE, 0);
  tx_buffer_turret_.assign(TX_PDO_SIZE, 0);

  // --- URDF <hardware> parameters, with explicit errors for missing/invalid values ---
  auto get_string = [&](const std::string & key, std::string & out) -> bool
  {
    const auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end() || it->second.empty())
    {
      RCLCPP_FATAL(logger_, "Missing required <param name=\"%s\"> in the <hardware> block of the URDF.", key.c_str());
      return false;
    }
    out = it->second;
    return true;
  };
  auto get_double = [&](const std::string & key, double & out) -> bool
  {
    std::string text;
    if (!get_string(key, text)) return false;
    try
    {
      size_t used = 0;
      out = std::stod(text, &used);
      if (used == text.size()) return true;
    }
    catch (const std::exception &)
    {
    }
    RCLCPP_FATAL(logger_, "URDF parameter '%s' = '%s' is not a valid number.", key.c_str(), text.c_str());
    return false;
  };
  auto get_int = [&](const std::string & key, int & out) -> bool
  {
    std::string text;
    if (!get_string(key, text)) return false;
    try
    {
      size_t used = 0;
      out = std::stoi(text, &used);
      if (used == text.size()) return true;
    }
    catch (const std::exception &)
    {
    }
    RCLCPP_FATAL(logger_, "URDF parameter '%s' = '%s' is not a valid integer.", key.c_str(), text.c_str());
    return false;
  };

  if (!get_string("left_wheel_name", cfg_.left_wheel_name) ||
      !get_string("right_wheel_name", cfg_.right_wheel_name) ||
      !get_string("turret_name", cfg_.turret_name) ||
      !get_double("transmission_ratio_wheels", cfg_.transmission_ratio_wheels) ||
      !get_double("transmission_ratio_turret", cfg_.transmission_ratio_turret) ||
      !get_int("encoder_ticks_per_rev", cfg_.encoder_ticks_per_rev) ||
      !get_int("slave_left_motor", cfg_.slave_left_motor) ||
      !get_int("slave_right_motor", cfg_.slave_right_motor) ||
      !get_int("slave_turret_motor", cfg_.slave_turret_motor))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Optional: network interface of the EtherCAT master (default eno1).
  const auto iface = info_.hardware_parameters.find("ethercat_interface");
  if (iface != info_.hardware_parameters.end() && !iface->second.empty())
  {
    ethercat_interface_ = iface->second;
  }

  if (cfg_.transmission_ratio_wheels <= 0.0 || cfg_.transmission_ratio_turret <= 0.0)
  {
    RCLCPP_FATAL(
        logger_, "transmission_ratio_wheels/turret must be > 0 (got %.3f / %.3f)",
        cfg_.transmission_ratio_wheels, cfg_.transmission_ratio_turret);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.encoder_ticks_per_rev <= 0)
  {
    RCLCPP_FATAL(logger_, "encoder_ticks_per_rev must be > 0 (got %d)", cfg_.encoder_ticks_per_rev);
    return hardware_interface::CallbackReturn::ERROR;
  }
  const std::set<int> slaves{cfg_.slave_left_motor, cfg_.slave_right_motor, cfg_.slave_turret_motor};
  if (slaves.size() != 3 || *slaves.begin() < 1)
  {
    RCLCPP_FATAL(
        logger_, "slave_left/right/turret_motor must be distinct EtherCAT positions >= 1 (got %d, %d, %d)",
        cfg_.slave_left_motor, cfg_.slave_right_motor, cfg_.slave_turret_motor);
    return hardware_interface::CallbackReturn::ERROR;
  }

  wheel_l_.name = cfg_.left_wheel_name;
  wheel_r_.name = cfg_.right_wheel_name;
  turret_.name = cfg_.turret_name;

  // --- Joint interfaces: one velocity command, position + velocity states ---
  for (const std::string & name : {cfg_.left_wheel_name, cfg_.right_wheel_name, cfg_.turret_name})
  {
    const bool declared = std::any_of(
        info_.joints.begin(), info_.joints.end(),
        [&](const hardware_interface::ComponentInfo & joint) { return joint.name == name; });
    if (!declared)
    {
      RCLCPP_FATAL(logger_, "Joint '%s' (from the <hardware> params) is not declared in this <ros2_control> block.", name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
          logger_, "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
          logger_, "Joint '%s' has '%s' command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
          logger_, "Joint '%s' has %zu state interfaces. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
          logger_, "Joint '%s' has '%s' as first state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
          logger_, "Joint '%s' has '%s' as second state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> IrdEthercatSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (Joint * joint : {&wheel_l_, &wheel_r_, &turret_})
  {
    state_interfaces.emplace_back(joint->name, hardware_interface::HW_IF_POSITION, &joint->pos);
    state_interfaces.emplace_back(joint->name, hardware_interface::HW_IF_VELOCITY, &joint->vel);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> IrdEthercatSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (Joint * joint : {&wheel_l_, &wheel_r_, &turret_})
  {
    command_interfaces.emplace_back(joint->name, hardware_interface::HW_IF_VELOCITY, &joint->cmd);
  }
  return command_interfaces;
}

// =========================================================================
// ros2_control: lifecycle
// =========================================================================

hardware_interface::CallbackReturn IrdEthercatSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // The bus is opened in on_activate(), so a deactivate/activate cycle fully
  // re-initialises the drives.
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IrdEthercatSystem::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IrdEthercatSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating IRD EtherCAT hardware interface...");

  // Lock all process memory: a single page fault in the RT thread can stall
  // the 4 ms cycle long enough to trip the drives' fieldbus SYNC monitor.
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
  {
    RCLCPP_WARN(
        logger_, "mlockall failed (%s). RT PDO thread may suffer page-fault jitter; check memlock limits (ulimit -l).",
        std::strerror(errno));
  }

  if (!init_ethercat())
  {
    RCLCPP_ERROR(logger_, "EtherCAT initialization failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // A drive can still hold a fault from a previous session (e.g. one killed
  // before its clean shutdown). Clear it by SDO now, while still in SAFE-OP.
  auto clear_latched_fault = [&](int slave, const char * name) -> bool
  {
    uint16_t statusword = 0;
    if (!sdo_read(&ec_context_, slave, 0x6041, 0x00, statusword))
    {
      RCLCPP_WARN(logger_, "%s (slave %d): Failed to read statusword before PDO enable.", name, slave);
      return true;
    }
    if (!(statusword & SW_FAULT_BIT))
    {
      return true;
    }

    RCLCPP_WARN(
        logger_, "%s (slave %d) starts activation in FAULT (SW=0x%04X). Clearing latched fault via SDO reset.",
        name, slave, statusword);
    dump_drive_fault_snapshot(
        &ec_context_, logger_, slave, name, statusword, ec_context_.slavelist[slave].ALstatuscode,
        /*latched_before_activation=*/true);

    write_sdo<uint16_t>(slave, 0x6040, 0x00, CW_FAULT_RESET);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    write_sdo<uint16_t>(slave, 0x6040, 0x00, CW_DISABLE_VOLTAGE);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!sdo_read(&ec_context_, slave, 0x6041, 0x00, statusword))
    {
      RCLCPP_ERROR(logger_, "%s (slave %d): Failed to read statusword after SDO fault reset.", name, slave);
      return false;
    }
    if (statusword & SW_FAULT_BIT)
    {
      RCLCPP_ERROR(logger_, "%s (slave %d): fault reset did not clear FAULT (SW=0x%04X).", name, slave, statusword);
      dump_drive_fault_snapshot(&ec_context_, logger_, slave, name, statusword, ec_context_.slavelist[slave].ALstatuscode);
      return false;
    }

    RCLCPP_INFO(logger_, "%s (slave %d): latched fault cleared, current SW=0x%04X", name, slave, statusword);
    return true;
  };

  if (!clear_latched_fault(cfg_.slave_left_motor, "LEFT") ||
      !clear_latched_fault(cfg_.slave_right_motor, "RIGHT") ||
      !clear_latched_fault(cfg_.slave_turret_motor, "TURRET"))
  {
    stage_zero_rpdo(CW_DISABLE_VOLTAGE);
    exchange_processdata();
    shutdown_ethercat();
    return hardware_interface::CallbackReturn::ERROR;
  }

  stage_zero_rpdo(CW_DISABLE_VOLTAGE);

  target_tps_left_.store(0);
  target_tps_right_.store(0);
  target_tps_turret_.store(0);
  actual_tps_left_.store(0);
  actual_tps_right_.store(0);
  actual_tps_turret_.store(0);
  statusword_left_.store(rd_u16(tx_buffer_left_.data() + TX_OFF_SW));
  statusword_right_.store(rd_u16(tx_buffer_right_.data() + TX_OFF_SW));
  statusword_turret_.store(rd_u16(tx_buffer_turret_.data() + TX_OFF_SW));
  force_zero_velocity_.store(false);
  fault_dump_pending_.store(0);
  low_wkc_reads_ = 0;

  RCLCPP_INFO(logger_, "Starting RT processdata loop in SAFE-OP before requesting OPERATIONAL...");
  start_processdata_thread();
  start_diagnostics_thread();
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  if (USE_DISTRIBUTED_CLOCKS && ec_context_.slavelist[0].hasdc)
  {
    const auto lock_start = std::chrono::steady_clock::now();
    int waited_ms = 0;
    while (dc_locked_cycles_.load() < DC_LOCK_CYCLES && waited_ms < DC_LOCK_TIMEOUT_MS)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      waited_ms = static_cast<int>(
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lock_start).count());
    }
    const int32_t sync0_us = static_cast<int32_t>(DC_SYNC_SHIFT_NS / 1000);
    if (dc_locked_cycles_.load() >= DC_LOCK_CYCLES)
    {
      RCLCPP_INFO(
          logger_, "DC phase locked (%d ms after the 250 ms settle): frame at %d us, %d us ahead of SYNC0.",
          waited_ms, dc_phase_us_.load(), sync0_us - dc_phase_us_.load());
    }
    else
    {
      RCLCPP_WARN(
          logger_,
          "DC phase not settled within %d ms (frame at %d us, SYNC0 at %d us); continuing, "
          "an error 50 right after enable is possible.",
          DC_LOCK_TIMEOUT_MS, dc_phase_us_.load(), sync0_us);
    }
  }

  RCLCPP_INFO(logger_, "Requesting OPERATIONAL state...");
  ec_context_.slavelist[0].state = EC_STATE_OPERATIONAL;
  {
    std::lock_guard<std::mutex> guard(ethercat_bus_mutex_);
    ecx_writestate(&ec_context_, 0);
  }

  if (!wait_for_state(EC_STATE_OPERATIONAL, static_cast<int64_t>(EC_TIMEOUTSTATE) * 4))
  {
    for (int slave : {cfg_.slave_left_motor, cfg_.slave_right_motor, cfg_.slave_turret_motor})
    {
      if (ec_context_.slavelist[slave].state != EC_STATE_OPERATIONAL)
      {
        RCLCPP_ERROR(
            logger_, "Slave %d failed to reach OPERATIONAL. State=0x%02X, AL=0x%04X", slave,
            ec_context_.slavelist[slave].state, ec_context_.slavelist[slave].ALstatuscode);
      }
    }
    disable_drives_and_leave_op();
    shutdown_ethercat();
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "All slaves reached OPERATIONAL state.");

  if (!enable_slaves_via_pdo())
  {
    // Stop through the cyclic PDO first: stopping the RT thread while the
    // drives are still in OP would latch a fieldbus fault (error 49/50).
    disable_drives_and_leave_op();
    dump_drive_fault_snapshot(
        &ec_context_, logger_, cfg_.slave_left_motor, "LEFT",
        statusword_left_.load(), ec_context_.slavelist[cfg_.slave_left_motor].ALstatuscode);
    dump_drive_fault_snapshot(
        &ec_context_, logger_, cfg_.slave_right_motor, "RIGHT",
        statusword_right_.load(), ec_context_.slavelist[cfg_.slave_right_motor].ALstatuscode);
    dump_drive_fault_snapshot(
        &ec_context_, logger_, cfg_.slave_turret_motor, "TURRET",
        statusword_turret_.load(), ec_context_.slavelist[cfg_.slave_turret_motor].ALstatuscode);
    RCLCPP_ERROR(logger_, "Failed to enable slaves via PDO state machine");
    shutdown_ethercat();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
      logger_, "All drives enabled successfully. Ready for PDO cyclic operation (DC phase %d us, SYNC0 at %d us).",
      dc_phase_us_.load(), static_cast<int32_t>(DC_SYNC_SHIFT_NS / 1000));
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IrdEthercatSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating hardware interface...");
  if (ec_initialized_)
  {
    disable_drives_and_leave_op();
    shutdown_ethercat();
    RCLCPP_INFO(logger_, "Hardware interface deactivated successfully.");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IrdEthercatSystem::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Shutting down hardware interface...");
  if (ec_initialized_)
  {
    RCLCPP_WARN(logger_, "EtherCAT still active during shutdown. Forcing cleanup...");
    disable_drives_and_leave_op();
    shutdown_ethercat();
  }
  RCLCPP_INFO(logger_, "Hardware interface shutdown complete.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// =========================================================================
// ros2_control: read / write (never touch the bus)
// =========================================================================

hardware_interface::return_type IrdEthercatSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!ec_initialized_) return hardware_interface::return_type::OK;

  const int recv_wkc = last_processdata_wkc_.load();
  if (recv_wkc > 0 && recv_wkc < expected_wkc_)
  {
    if (++low_wkc_reads_ % 250 == 0)
    {
      RCLCPP_WARN(logger_, "read(): WKC below expectation: got %d expected at least %d", recv_wkc, expected_wkc_);
    }
  }

  // Drive feedback is motor-shaft ticks/s; joints are on the gearbox output.
  wheel_l_.vel = ticks_s_to_rad_s(actual_tps_left_.load()) / cfg_.transmission_ratio_wheels;
  wheel_r_.vel = ticks_s_to_rad_s(actual_tps_right_.load()) / cfg_.transmission_ratio_wheels;
  turret_.vel = ticks_s_to_rad_s(actual_tps_turret_.load()) / cfg_.transmission_ratio_turret;

  wheel_l_.pos += wheel_l_.vel * period.seconds();
  wheel_r_.pos += wheel_r_.vel * period.seconds();
  turret_.pos += turret_.vel * period.seconds();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IrdEthercatSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!ec_initialized_) return hardware_interface::return_type::OK;

  const bool force_zero = force_zero_velocity_.load();
  if (force_zero && !first_write_warned_)
  {
    RCLCPP_WARN(logger_, "SAFETY: force_zero_velocity enabled. Target velocity forced to 0.");
    first_write_warned_ = true;
  }
  else if (!force_zero)
  {
    first_write_warned_ = false;
  }

  auto stage_target = [&](int slave, const Joint & joint, std::atomic<int32_t> & target_tps,
                          double transmission_ratio, const char * name)
  {
    int32_t tps = 0;
    if (!force_zero && operation_enabled_.load())
    {
      double joint_rad_s = joint.cmd;
      if (!std::isfinite(joint_rad_s))
      {
        RCLCPP_ERROR_THROTTLE(
            logger_, steady_clock_, 1000,
            "%s (slave %d) received non-finite velocity command (%f rad/s). Forcing target velocity to 0.",
            name, slave, joint_rad_s);
        joint_rad_s = 0.0;
      }

      // Joint command is gearbox-output rad/s; the drive expects motor-shaft
      // ticks/s, so scale by the transmission ratio. MAX_RPM is a motor-shaft
      // limit, hence clamped after scaling.
      double motor_rad_s = joint_rad_s * transmission_ratio;
      if (std::fabs(motor_rad_s) > MAX_MOTOR_RAD_S)
      {
        const double clamped = std::clamp(motor_rad_s, -MAX_MOTOR_RAD_S, MAX_MOTOR_RAD_S);
        RCLCPP_WARN_THROTTLE(
            logger_, steady_clock_, 1000,
            "%s (slave %d) motor velocity %.6f rad/s exceeds %.6f rad/s (%.1f rpm). Clamping to %.6f rad/s.",
            name, slave, motor_rad_s, MAX_MOTOR_RAD_S, MAX_RPM, clamped);
        motor_rad_s = clamped;
      }
      tps = rad_s_to_ticks_s(motor_rad_s);
    }
    target_tps.store(tps);
  };

  stage_target(cfg_.slave_left_motor, wheel_l_, target_tps_left_, cfg_.transmission_ratio_wheels, "LEFT");
  stage_target(cfg_.slave_right_motor, wheel_r_, target_tps_right_, cfg_.transmission_ratio_wheels, "RIGHT");
  stage_target(cfg_.slave_turret_motor, turret_, target_tps_turret_, cfg_.transmission_ratio_turret, "TURRET");

  return hardware_interface::return_type::OK;
}

// motor-shaft rad/s -> drive velocity units (encoder ticks/s)
int32_t IrdEthercatSystem::rad_s_to_ticks_s(double rad_s) const
{
  return static_cast<int32_t>(rad_s * cfg_.encoder_ticks_per_rev / (2.0 * M_PI));
}

// drive velocity units (encoder ticks/s) -> motor-shaft rad/s
double IrdEthercatSystem::ticks_s_to_rad_s(int32_t ticks_s) const
{
  return static_cast<double>(ticks_s) * 2.0 * M_PI / cfg_.encoder_ticks_per_rev;
}

// =========================================================================
// Bus start-up and shutdown
// =========================================================================

bool IrdEthercatSystem::init_ethercat()
{
  RCLCPP_INFO(logger_, "Initializing SOEM on interface %s...", ethercat_interface_.c_str());

  if (ecx_init(&ec_context_, ethercat_interface_.c_str()) <= 0)
  {
    RCLCPP_ERROR(
        logger_, "Failed to initialize SOEM on %s (does the interface exist, and does the process have "
        "CAP_NET_RAW? see README).",
        ethercat_interface_.c_str());
    return false;
  }

  RCLCPP_INFO(logger_, "SOEM initialized. Scanning EtherCAT network...");
  if (ecx_config_init(&ec_context_) <= 0)
  {
    RCLCPP_ERROR(logger_, "No EtherCAT slaves found.");
    ecx_close(&ec_context_);
    return false;
  }

  const int highest_slave = std::max({cfg_.slave_left_motor, cfg_.slave_right_motor, cfg_.slave_turret_motor});
  if (ec_context_.slavecount < highest_slave)
  {
    RCLCPP_ERROR(
        logger_, "Found %d slave(s) on the bus, but the configuration uses slave position %d.",
        ec_context_.slavecount, highest_slave);
    ecx_close(&ec_context_);
    return false;
  }
  RCLCPP_INFO(logger_, "Found %d slave(s).", ec_context_.slavecount);
  esc_counters_last_.assign(ec_context_.slavecount + 1, {});
  dc_integral_ = 0;

  // Suppress SOEM's automatic SAFE-OP request at the end of
  // ecx_config_map_group(). The IRD selects its sync mode at the
  // PRE-OP -> SAFE-OP (PS) transition, so SYNC0 (ecx_dcsync0) and the
  // 0x1006/0x2012 objects must be configured BEFORE that transition,
  // otherwise the drive falls back to SoftSync (0x2012:01=2) and ignores
  // later HardSync writes. SAFE-OP is requested explicitly further below.
  // (Set only after ecx_config_init(): the same flag would suppress the
  // INIT -> PRE-OP request too.)
  ec_context_.manualstatechange = 1;

  const struct { int slave; const char * name; } drives[] = {
      {cfg_.slave_left_motor, "left"}, {cfg_.slave_right_motor, "right"}, {cfg_.slave_turret_motor, "turret"}};
  for (const auto & d : drives)
  {
    if (!configure_slave(d.slave))
    {
      RCLCPP_ERROR(logger_, "Failed to configure %s motor slave", d.name);
      ecx_close(&ec_context_);
      return false;
    }
  }

  RCLCPP_INFO(logger_, "All slaves configured. Setting up process data mapping...");
  ecx_config_map_group(&ec_context_, io_map_, 0);

  if (USE_DISTRIBUTED_CLOCKS && !REQUEST_HARDSYNC)
  {
    // SoftSync: the drive picks HardSync at the PS transition whenever SYNC0
    // is already running, regardless of 0x2012:01, so complete PRE-OP ->
    // SAFE-OP before SYNC0 is enabled below.
    ec_context_.slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&ec_context_, 0);
    for (const auto & d : drives)
    {
      ecx_statecheck(&ec_context_, d.slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
      if (ec_context_.slavelist[d.slave].state != EC_STATE_SAFE_OP)
      {
        RCLCPP_ERROR(
            logger_, "Slave %d failed to reach SAFE-OP before SYNC0 (SoftSync). State=0x%02X",
            d.slave, ec_context_.slavelist[d.slave].state);
        ecx_close(&ec_context_);
        return false;
      }
    }
    RCLCPP_INFO(logger_, "SoftSync: all slaves in SAFE-OP before enabling SYNC0.");
  }

  if (USE_DISTRIBUTED_CLOCKS)
  {
    ecx_configdc(&ec_context_);
    for (const auto & d : drives)
    {
      ecx_dcsync0(&ec_context_, d.slave, TRUE, PROCESS_CYCLE_NS, DC_SYNC_SHIFT_NS);
    }
    RCLCPP_INFO(
        logger_, "Configured DC SYNC0: cycle=%ld ns shift=%ld ns master_phase=%ld ns",
        static_cast<long>(PROCESS_CYCLE_NS), static_cast<long>(DC_SYNC_SHIFT_NS),
        static_cast<long>(DC_SYNC_OFFSET_NS));
    for (const auto & d : drives)
    {
      log_dc_snapshot(&ec_context_, logger_, d.slave);
    }
  }
  else
  {
    RCLCPP_INFO(logger_, "Using SoftSync PDO mode (DC/SYNC0 disabled, cycle=%u us)", PROCESS_CYCLE_US);
  }

  auto bind = [&](int slave, uint8_t *& rx, uint8_t *& tx)
  {
    rx = reinterpret_cast<uint8_t *>(ec_context_.slavelist[slave].outputs);
    tx = reinterpret_cast<uint8_t *>(ec_context_.slavelist[slave].inputs);
  };
  bind(cfg_.slave_left_motor, rx_left_, tx_left_);
  bind(cfg_.slave_right_motor, rx_right_, tx_right_);
  bind(cfg_.slave_turret_motor, rx_turret_, tx_turret_);

  exchange_processdata();

  // The mapped sizes must match the PDO layout this driver encodes/decodes.
  for (const auto & d : drives)
  {
    const auto & sl = ec_context_.slavelist[d.slave];
    if (sl.Obytes != RX_PDO_SIZE || sl.Ibytes != TX_PDO_SIZE)
    {
      RCLCPP_ERROR(
          logger_, "Slave %d PDO size mismatch! RxPDO expected=%d actual=%u, TxPDO expected=%d actual=%u",
          d.slave, RX_PDO_SIZE, sl.Obytes, TX_PDO_SIZE, sl.Ibytes);
      ecx_close(&ec_context_);
      return false;
    }
  }

  ec_context_.grouplist[0].blockLRW = 0;  // use combined LRW datagrams
  RCLCPP_INFO(logger_, "Process data mapping validated for all slaves.");

  ec_context_.slavelist[0].state = EC_STATE_SAFE_OP;
  ecx_writestate(&ec_context_, 0);
  for (const auto & d : drives)
  {
    ecx_statecheck(&ec_context_, d.slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    if (ec_context_.slavelist[d.slave].state != EC_STATE_SAFE_OP)
    {
      RCLCPP_ERROR(
          logger_, "Slave %d failed to reach SAFE-OP. State=0x%02X", d.slave, ec_context_.slavelist[d.slave].state);
      ecx_close(&ec_context_);
      return false;
    }
  }
  RCLCPP_INFO(logger_, "All slaves reached SAFE-OP.");

  if (USE_DISTRIBUTED_CLOCKS)
  {
    RCLCPP_INFO(logger_, "Letting the EtherCAT network settle against DC clocks in SAFE-OP...");
    run_startup_processdata_cycles(STARTUP_SETTLE_CYCLES, CW_DISABLE_VOLTAGE);
  }

  // Report the sync mode each drive actually latched.
  for (const auto & d : drives)
  {
    uint32_t cycle_us = 0;
    uint16_t sync_type = 0;
    sdo_read(&ec_context_, d.slave, 0x1006, 0x00, cycle_us);
    sdo_read(&ec_context_, d.slave, 0x2012, 0x01, sync_type);
    RCLCPP_INFO(
        logger_, "Slave %d SAFE-OP sync settings: 0x1006:00=%u us, 0x2012:01=%u, hasdc=%d",
        d.slave, cycle_us, sync_type, ec_context_.slavelist[d.slave].hasdc);

    if (USE_DISTRIBUTED_CLOCKS)
    {
      if (!ec_context_.slavelist[d.slave].hasdc)
      {
        RCLCPP_ERROR(logger_, "Slave %d does not report distributed clocks capability.", d.slave);
        ecx_close(&ec_context_);
        return false;
      }
      if (!REQUEST_HARDSYNC)
      {
        if (sync_type == 2)
        {
          RCLCPP_INFO(logger_, "Slave %d: SoftSync established (0x2012:01=2, 0x1006:00=%u us).", d.slave, cycle_us);
        }
        else
        {
          RCLCPP_WARN(
              logger_, "Slave %d sync settings unexpected (observed 0x1006:00=%u us, 0x2012:01=%u; "
              "expected SoftSync, sync type 2).", d.slave, cycle_us, sync_type);
        }
      }
      else if (sync_type == 4 && cycle_us == PROCESS_CYCLE_US)
      {
        RCLCPP_INFO(logger_, "Slave %d: HardSync established (0x2012:01=4, %u us).", d.slave, cycle_us);
      }
      else if (sync_type == 2)
      {
        RCLCPP_WARN(
            logger_, "Slave %d fell back to SoftSync (0x2012:01=2, 0x1006:00=%u us). "
            "HardSync was requested in PRE-OP but did not latch at the PS transition.", d.slave, cycle_us);
      }
      else
      {
        RCLCPP_WARN(
            logger_, "Slave %d sync settings unexpected (observed 0x1006:00=%u us, 0x2012:01=%u; "
            "expected %u us and sync type 4).", d.slave, cycle_us, sync_type, PROCESS_CYCLE_US);
      }
    }
    else if (sync_type != 2 || cycle_us != PROCESS_CYCLE_US)
    {
      RCLCPP_WARN(
          logger_, "Slave %d SoftSync configuration mismatch: expected 0x2012:01=2 and 0x1006:00=%u us",
          d.slave, PROCESS_CYCLE_US);
    }
  }

  const uint16_t outputs_wkc = ec_context_.grouplist[0].outputsWKC;
  const uint16_t inputs_wkc = ec_context_.grouplist[0].inputsWKC;
  expected_wkc_ = ec_context_.grouplist[0].blockLRW ? static_cast<int>(outputs_wkc + inputs_wkc)
                                                    : static_cast<int>((outputs_wkc * 2) + inputs_wkc);

  ec_initialized_ = true;
  return true;
}

// Configures one drive by SDO while it is in PRE-OP: mode of operation, PDO
// mapping and (with distributed clocks) the synchronisation objects.
bool IrdEthercatSystem::configure_slave(int slave)
{
  RCLCPP_INFO(logger_, "Configuring slave %d...", slave);

  ecx_statecheck(&ec_context_, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
  if (ec_context_.slavelist[slave].state != EC_STATE_PRE_OP)
  {
    RCLCPP_ERROR(logger_, "Slave %d failed to reach PRE-OP. State=0x%02X", slave, ec_context_.slavelist[slave].state);
    return false;
  }

  // ---- Mode of operation: CSV ----
  // Written only if needed: some restart cases keep the drive in CSV already
  // even when a fresh write temporarily fails.
  int8_t current_mode = 0;
  const bool had_mode_read = sdo_read(&ec_context_, slave, 0x6060, 0x00, current_mode);
  if (!had_mode_read)
  {
    RCLCPP_WARN(logger_, "Slave %d: Failed to read 0x6060:00 before CSV request", slave);
  }
  if (!had_mode_read || current_mode != MODE_CSV)
  {
    if (!write_sdo<int8_t>(slave, 0x6060, 0x00, MODE_CSV))
    {
      int8_t readback_mode = 0;
      if (sdo_read(&ec_context_, slave, 0x6060, 0x00, readback_mode) && readback_mode == MODE_CSV)
      {
        RCLCPP_WARN(
            logger_, "Slave %d: CSV mode write reported failure, but readback confirms 0x6060:00=0x%02X. Continuing.",
            slave, static_cast<uint8_t>(readback_mode));
      }
      else
      {
        RCLCPP_ERROR(logger_, "Slave %d: Failed to set operation mode to CSV (9)", slave);
        return false;
      }
    }
  }

  // ---- PDO mapping ----
  // 1) Disable the assignments, 2) clear the mapping objects, 3-4) write the
  // entries (0xIIIISSLL = index, subindex, length in bits) and their count,
  // 5) assign the mapping objects again.
  if (!write_sdo<uint8_t>(slave, 0x1C12, 0x00, 0)) return false;  // RxPDO assign count = 0
  if (!write_sdo<uint8_t>(slave, 0x1C13, 0x00, 0)) return false;  // TxPDO assign count = 0
  if (!write_sdo<uint8_t>(slave, 0x1600, 0x00, 0)) return false;  // RxPDO map count = 0
  if (!write_sdo<uint8_t>(slave, 0x1A00, 0x00, 0)) return false;  // TxPDO map count = 0

  if (!write_sdo<uint32_t>(slave, 0x1600, 0x01, 0x60400010)) return false;  // Controlword
  if (!write_sdo<uint32_t>(slave, 0x1600, 0x02, 0x60600008)) return false;  // Modes of operation
  if (!write_sdo<uint32_t>(slave, 0x1600, 0x03, 0x60FF0020)) return false;  // Target velocity
  if (!write_sdo<uint8_t>(slave, 0x1600, 0x00, 3)) return false;

  if (!write_sdo<uint32_t>(slave, 0x1A00, 0x01, 0x60410010)) return false;  // Statusword
  if (!write_sdo<uint32_t>(slave, 0x1A00, 0x02, 0x60610008)) return false;  // Modes of operation display
  if (!write_sdo<uint32_t>(slave, 0x1A00, 0x03, 0x606C0020)) return false;  // Velocity actual value
  if (!write_sdo<uint8_t>(slave, 0x1A00, 0x00, 3)) return false;

  if (!write_sdo<uint16_t>(slave, 0x1C12, 0x01, 0x1600)) return false;
  if (!write_sdo<uint8_t>(slave, 0x1C12, 0x00, 1)) return false;
  if (!write_sdo<uint16_t>(slave, 0x1C13, 0x01, 0x1A00)) return false;
  if (!write_sdo<uint8_t>(slave, 0x1C13, 0x00, 1)) return false;

  // ---- Synchronisation ----
  if (USE_DISTRIBUTED_CLOCKS && REQUEST_HARDSYNC)
  {
    // HardSync pre-configuration. The drive latches its sync mode at the
    // PRE-OP -> SAFE-OP transition (EtherCAT manual ch. 5: 0x1006/0x2012
    // "need to be carefully setup before entering the Safe-Op state"), so
    // both objects must be written HERE, in PRE-OP, and SYNC0 must already
    // be running before the SAFE-OP request (see init_ethercat ordering).
    if (!write_sdo<uint32_t>(slave, 0x1006, 0x00, PROCESS_CYCLE_US))
    {
      RCLCPP_WARN(logger_, "Slave %d: failed to write 0x1006:00=%u us in PRE-OP", slave, PROCESS_CYCLE_US);
    }
    if (!write_sdo<uint16_t>(slave, 0x2012, 0x01, 4))
    {
      RCLCPP_WARN(logger_, "Slave %d: failed to write 0x2012:01=4 (HardSync) in PRE-OP", slave);
    }
  }
  else if (USE_DISTRIBUTED_CLOCKS)
  {
    // Clear a 4 left over from an earlier HardSync session. Not sufficient on
    // its own: see the SAFE-OP-before-SYNC0 ordering in init_ethercat().
    if (!write_sdo<uint16_t>(slave, 0x2012, 0x01, 2))
    {
      RCLCPP_WARN(logger_, "Slave %d: failed to write 0x2012:01=2 (SoftSync) in PRE-OP", slave);
    }
  }

  uint32_t cycle_us = 0;
  sdo_read(&ec_context_, slave, 0x1006, 0x00, cycle_us);
  RCLCPP_INFO(
      logger_, "Slave %d: Current PDO cycle period is %u us (%s, diagnostic only)",
      slave, cycle_us, USE_DISTRIBUTED_CLOCKS ? "DC SYNC0 enabled" : "no DC");
  if (!USE_DISTRIBUTED_CLOCKS && cycle_us != PROCESS_CYCLE_US)
  {
    RCLCPP_WARN(
        logger_, "Slave %d: SoftSync requested 0x1006:00=%u us but read back %u us", slave, PROCESS_CYCLE_US, cycle_us);
  }

  RCLCPP_DEBUG(logger_, "Slave %d: PDO mapping configured successfully.", slave);
  return true;
}

// Waits until all configured drives report `state`, polling without holding
// the bus lock between polls so the RT thread keeps exchanging frames.
bool IrdEthercatSystem::wait_for_state(uint16_t state, int64_t timeout_us)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(timeout_us);
  while (true)
  {
    bool all_reached = true;
    for (int slave : {cfg_.slave_left_motor, cfg_.slave_right_motor, cfg_.slave_turret_motor})
    {
      {
        std::lock_guard<std::mutex> guard(ethercat_bus_mutex_);
        ecx_statecheck(&ec_context_, slave, state, 0);
      }
      if (ec_context_.slavelist[slave].state != state)
      {
        all_reached = false;
        break;
      }
    }
    if (all_reached) return true;
    if (std::chrono::steady_clock::now() > deadline) return false;
    osal_usleep(PROCESS_CYCLE_US);
  }
}

// Brings the drives to a safe stop while the cyclic RPDO stream is still
// running, then stops the RT thread. If the PDO thread were stopped first,
// the drives would be left in OP without process data and immediately latch a
// fieldbus sync fault (0x8780/0x8782), seen as a latched FAULT on the next
// activation.
void IrdEthercatSystem::disable_drives_and_leave_op()
{
  RCLCPP_INFO(logger_, "Stopping all motors and disabling drives via PDO...");
  force_zero_velocity_.store(true);
  target_tps_left_.store(0);
  target_tps_right_.store(0);
  target_tps_turret_.store(0);
  enable_request_.store(false);  // RT thread now stages controlword 0x0000
  operation_enabled_.store(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Leave OPERATIONAL before stopping the cyclic exchange: the sync/RPDO
  // monitoring is only armed in OP, so no fault can latch in SAFE-OP.
  ec_context_.slavelist[0].state = EC_STATE_SAFE_OP;
  {
    std::lock_guard<std::mutex> guard(ethercat_bus_mutex_);
    ecx_writestate(&ec_context_, 0);
  }
  if (!wait_for_state(EC_STATE_SAFE_OP, EC_TIMEOUTSTATE))
  {
    RCLCPP_WARN(logger_, "Not all drives confirmed SAFE-OP before stopping the process data.");
  }

  stop_processdata_thread();
}

void IrdEthercatSystem::shutdown_ethercat()
{
  if (!ec_initialized_) return;

  RCLCPP_INFO(logger_, "Shutting down EtherCAT...");

  // No more bus traffic from the diagnostics thread once we start closing.
  stop_diagnostics_thread();

  ec_context_.slavelist[0].state = EC_STATE_SAFE_OP;
  ecx_writestate(&ec_context_, 0);
  for (int slave : {cfg_.slave_left_motor, cfg_.slave_right_motor, cfg_.slave_turret_motor})
  {
    ecx_statecheck(&ec_context_, slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
  }

  ec_context_.slavelist[0].state = EC_STATE_INIT;
  ecx_writestate(&ec_context_, 0);

  ecx_close(&ec_context_);
  ec_initialized_ = false;
  RCLCPP_INFO(logger_, "EtherCAT connection closed.");
}

// Asks the RT thread to run the CiA 402 enable sequence through the cyclic
// controlword and waits until all drives are Operation Enabled.
bool IrdEthercatSystem::enable_slaves_via_pdo()
{
  RCLCPP_INFO(logger_, "Enabling all slaves via cyclic PDO controlwords...");

  const auto start_time = std::chrono::steady_clock::now();
  enable_fault_abort_.store(false);
  operation_enabled_.store(false);
  enable_request_.store(true);

  for (int polls = 1; std::chrono::steady_clock::now() - start_time < ENABLE_TIMEOUT; ++polls)
  {
    const uint16_t sw_left = statusword_left_.load();
    const uint16_t sw_right = statusword_right_.load();
    const uint16_t sw_turret = statusword_turret_.load();

    if (operation_enabled_.load())
    {
      RCLCPP_INFO(logger_, "All slaves reached Operation Enabled through PDO.");
      return true;
    }
    if (enable_fault_abort_.load())
    {
      RCLCPP_ERROR(
          logger_,
          "Aborting PDO enable sequence after repeated drive faults. Last TPDO statuswords: left=0x%04X right=0x%04X turret=0x%04X",
          sw_left, sw_right, sw_turret);
      return false;
    }
    if (polls % 5 == 0)
    {
      RCLCPP_DEBUG(
          logger_, "PDO enable status: left=0x%04X right=0x%04X turret=0x%04X", sw_left, sw_right, sw_turret);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  enable_request_.store(false);
  RCLCPP_ERROR(
      logger_, "Timed out waiting for PDO enable sequence. Final TPDO statuswords: left=0x%04X right=0x%04X turret=0x%04X",
      statusword_left_.load(), statusword_right_.load(), statusword_turret_.load());
  return false;
}

// =========================================================================
// SDO access
// =========================================================================

// Writes one object by SDO, retrying up to 3 times (the mailbox occasionally
// misses a request during start-up).
template<typename T>
bool IrdEthercatSystem::write_sdo(uint16_t slave, uint16_t index, uint8_t subindex, T value)
{
  for (int attempt = 1; attempt <= 3; ++attempt)
  {
    const int wkc = ecx_SDOwrite(&ec_context_, slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    if (wkc > 0) return true;
    if (attempt < 3) osal_usleep(10'000);
  }

  RCLCPP_ERROR(
      logger_, "SDO write (%zu bytes) failed after retries: slave=%u idx=0x%04X sub=0x%02X value=0x%llX",
      sizeof(value), slave, index, subindex,
      static_cast<unsigned long long>(static_cast<std::make_unsigned_t<T>>(value)));
  log_slave_mailbox_context(&ec_context_, logger_, slave, "write_sdo");
  log_soem_error_stack(&ec_context_, logger_, "write_sdo");
  return false;
}

// =========================================================================
// Process data
// =========================================================================

void IrdEthercatSystem::copy_tx_processdata()
{
  copy_from_process(tx_left_, tx_buffer_left_);
  copy_from_process(tx_right_, tx_buffer_right_);
  copy_from_process(tx_turret_, tx_buffer_turret_);
}

// Sends the staged outputs and receives the inputs of all drives in one frame.
int IrdEthercatSystem::exchange_processdata()
{
  std::lock_guard<std::mutex> guard(ethercat_bus_mutex_);
  ecx_send_processdata_group(&ec_context_, 0);
  const int recv_wkc = ecx_receive_processdata_group(&ec_context_, 0, EC_TIMEOUTRET);
  copy_tx_processdata();
  return recv_wkc;
}

// Stages the same controlword, CSV mode and zero target velocity for every drive.
void IrdEthercatSystem::stage_zero_rpdo(uint16_t controlword)
{
  auto stage_one = [&](std::vector<uint8_t> & rx_buf, uint8_t * rx_ptr)
  {
    wr_u16(rx_buf.data() + RX_OFF_CW, controlword);
    rx_buf[RX_OFF_MODE] = static_cast<uint8_t>(MODE_CSV);
    wr_i32(rx_buf.data() + RX_OFF_TV, 0);
    copy_to_process(rx_buf, rx_ptr);
  };
  stage_one(rx_buffer_left_, rx_left_);
  stage_one(rx_buffer_right_, rx_right_);
  stage_one(rx_buffer_turret_, rx_turret_);
}

// PI controller that shifts the master's cycle so the frame reaches the DC
// reference slave at DC_SYNC_OFFSET_NS past the cycle boundary.
// integral_term (optional) receives the integral part of *offsettime, for the
// timing diagnostics.
void IrdEthercatSystem::sync_linux_to_dc(int64_t reftime, int64_t cycletime, int64_t * offsettime, int64_t * integral_term)
{
  int64_t delta = (reftime - DC_SYNC_OFFSET_NS) % cycletime;
  if (delta > (cycletime / 2))
  {
    delta -= cycletime;
  }

  const int64_t timeerror = -delta;
  dc_integral_ += timeerror;
  *offsettime = static_cast<int64_t>((timeerror * DC_PI_PGAIN) + (dc_integral_ * DC_PI_IGAIN));
  if (integral_term)
  {
    *integral_term = static_cast<int64_t>(dc_integral_ * DC_PI_IGAIN);
  }
}

// Exchanges `cycles` frames on the DC-locked schedule with all outputs zero,
// before the RT thread exists (used to settle the DC loop in SAFE-OP).
void IrdEthercatSystem::run_startup_processdata_cycles(size_t cycles, uint16_t controlword)
{
  if (cycles == 0) return;

  const bool use_dc_sync = USE_DISTRIBUTED_CLOCKS && ec_context_.slavelist[0].hasdc;
  int64_t toff = 0;
  ec_timet ts;

  stage_zero_rpdo(controlword);
  exchange_processdata();

  // Start on the next whole millisecond.
  osal_get_monotonic_time(&ts);
  const long next_ms = static_cast<long>(ts.tv_nsec / 1000000) + 1;
  ts.tv_sec += next_ms / 1000;
  ts.tv_nsec = (next_ms % 1000) * 1000000;

  // Same one-step alignment as processdata_loop(), so the shared PI
  // integral is not wound up by the pull-in from an arbitrary start phase.
  bool phase_aligned = !use_dc_sync;
  int64_t realign_ns = 0;

  for (size_t i = 0; i < cycles; ++i)
  {
    add_time_ns(&ts, PROCESS_CYCLE_NS + toff + realign_ns);
    realign_ns = 0;
    osal_monotonic_sleep(&ts);

    stage_zero_rpdo(controlword);
    const int recv_wkc = exchange_processdata();
    if (use_dc_sync && recv_wkc > 0)
    {
      if (!phase_aligned)
      {
        realign_ns = -dc_phase_ns(ec_context_.DCtime, PROCESS_CYCLE_NS);
        phase_aligned = true;
      }
      else
      {
        sync_linux_to_dc(ec_context_.DCtime, PROCESS_CYCLE_NS, &toff);
      }
    }
  }
}

void IrdEthercatSystem::start_processdata_thread()
{
  if (processdata_thread_running_.load()) return;
  processdata_thread_stop_.store(false);
  processdata_thread_running_.store(true);
  processdata_thread_ = std::thread(&IrdEthercatSystem::processdata_loop, this);
}

void IrdEthercatSystem::stop_processdata_thread()
{
  processdata_thread_stop_.store(true);
  if (processdata_thread_.joinable())
  {
    processdata_thread_.join();
  }
  processdata_thread_running_.store(false);
}

// The RT loop. Every cycle it:
//  1. sleeps until the next absolute deadline (corrected by the DC PI loop),
//  2. stages controlword + target velocity for each drive from the CiA 402
//     state and the latest write() targets (ramped after an enable),
//  3. exchanges one frame and updates the DC lock / timing history,
//  4. publishes statuswords and actual velocities for read(),
//  5. detects drive faults and runs the bounded recovery.
void IrdEthercatSystem::processdata_loop()
{
  const bool use_dc_sync = USE_DISTRIBUTED_CLOCKS && ec_context_.slavelist[0].hasdc;
  int64_t toff = 0;
  int stable_enabled_cycles = 0;
  int left_fault_cycles = 0;
  int right_fault_cycles = 0;
  int turret_fault_cycles = 0;
  // Runtime fault recovery state (PDO-only, no SDO from this thread)
  bool in_recovery = false;
  bool recovery_pulse = false;
  int recovery_attempts = 0;
  int fault_cooldown = 0;
  int reset_pulse_cycles = 0;
  int healthy_cycles = 0;
  ec_timet ts;
  int64_t toff_integral = 0;
  // The loop starts on an arbitrary phase relative to the DC cycle. The
  // first received frame's phase is removed in one step (folded into the
  // next period, which stays positive since |phase| <= cycle/2: SOEM's
  // timespec add does not normalise a negative increment) instead of
  // letting the PI loop pull in over seconds and wind up its integral.
  bool phase_aligned = !use_dc_sync;
  int64_t realign_ns = 0;
  dc_locked_cycles_.store(0);

  // Post-enable reference ramp (see ENABLE_RAMP_MOTOR_RAD_S2).
  const int32_t ramp_step_tps = std::max<int32_t>(
      1, rad_s_to_ticks_s(ENABLE_RAMP_MOTOR_RAD_S2 * PROCESS_CYCLE_NS / 1e9));
  bool ramp_active = false;
  int32_t applied_left = 0;
  int32_t applied_right = 0;
  int32_t applied_turret = 0;
  auto slew = [&](int32_t target, int32_t & applied)
  {
    if (!ramp_active)
    {
      applied = target;
      return;
    }
    const bool reversing = (target > 0 && applied < 0) || (target < 0 && applied > 0);
    if (target == 0 || reversing)
    {
      applied = 0;  // stop / reverse: immediate
    }
    else if (std::abs(target) <= std::abs(applied))
    {
      applied = target;  // slowing down: immediate
    }
    else
    {
      applied += std::clamp(target - applied, -ramp_step_tps, ramp_step_tps);
    }
  };

  sched_param sched{};
  sched.sched_priority = PROCESSDATA_THREAD_PRIORITY;
  const int sched_rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sched);
  if (sched_rc == 0)
  {
    RCLCPP_INFO(logger_, "Processdata thread running with SCHED_FIFO priority %d", sched.sched_priority);
  }
  else
  {
    RCLCPP_WARN(
        logger_, "Processdata thread could not enable SCHED_FIFO priority %d: %s (does the process have CAP_SYS_NICE?)",
        sched.sched_priority, std::strerror(sched_rc));
  }

  osal_get_monotonic_time(&ts);
  const long next_ms = static_cast<long>(ts.tv_nsec / 1000000) + 1;
  ts.tv_sec += next_ms / 1000;
  ts.tv_nsec = (next_ms % 1000) * 1000000;

  while (!processdata_thread_stop_.load())
  {
    // ---- 1. Wait for the next cycle ----
    add_time_ns(&ts, PROCESS_CYCLE_NS + toff + realign_ns);
    realign_ns = 0;
    osal_monotonic_sleep(&ts);
    ec_timet woke;
    osal_get_monotonic_time(&woke);

    const bool enable_request = enable_request_.load();
    const bool operation_enabled = operation_enabled_.load();
    const bool force_zero = force_zero_velocity_.load();

    // ---- 2. Stage outputs ----
    if (enable_request && operation_enabled && !force_zero)
    {
      const int32_t target_left = target_tps_left_.load();
      const int32_t target_right = target_tps_right_.load();
      const int32_t target_turret = target_tps_turret_.load();
      slew(target_left, applied_left);
      slew(target_right, applied_right);
      slew(target_turret, applied_turret);
      if (ramp_active && applied_left == target_left && applied_right == target_right && applied_turret == target_turret)
      {
        ramp_active = false;  // caught up with the command
      }
    }
    else
    {
      applied_left = applied_right = applied_turret = 0;
    }

    auto stage_slave = [&](std::vector<uint8_t> & rx_buf, uint8_t * rx_ptr, const std::vector<uint8_t> & tx_buf, int32_t target_tps)
    {
      const uint16_t sw = rd_u16(tx_buf.data() + TX_OFF_SW);
      uint16_t cw = CW_DISABLE_VOLTAGE;
      if (enable_request)
      {
        if (operation_enabled)
        {
          // Once cyclic operation is achieved, keep the drive enabled and stop
          // auto-resetting faults; that avoids brake/power-stage chatter.
          cw = (sw & SW_FAULT_BIT) ? CW_DISABLE_VOLTAGE : CW_ENABLE_OPERATION;
        }
        else if (recovery_pulse && (sw & SW_FAULT_BIT))
        {
          // Bounded fault-reset pulse while recovering from a runtime
          // fieldbus fault; never sent continuously (brake chatter).
          cw = CW_FAULT_RESET;
        }
        else
        {
          cw = next_cw_from_sw(sw);
        }
      }
      const int32_t tv = (enable_request && operation_enabled && !force_zero && !(sw & SW_FAULT_BIT)) ? target_tps : 0;

      wr_u16(rx_buf.data() + RX_OFF_CW, cw);
      rx_buf[RX_OFF_MODE] = static_cast<uint8_t>(MODE_CSV);
      wr_i32(rx_buf.data() + RX_OFF_TV, tv);
      copy_to_process(rx_buf, rx_ptr);
    };

    stage_slave(rx_buffer_left_, rx_left_, tx_buffer_left_, applied_left);
    stage_slave(rx_buffer_right_, rx_right_, tx_buffer_right_, applied_right);
    stage_slave(rx_buffer_turret_, rx_turret_, tx_buffer_turret_, applied_turret);

    // ---- 3. Exchange the frame, DC lock and timing history ----
    ec_timet exchange_start;
    osal_get_monotonic_time(&exchange_start);
    const int recv_wkc = exchange_processdata();
    ec_timet exchange_end;
    osal_get_monotonic_time(&exchange_end);
    last_processdata_wkc_.store(recv_wkc);

    if (use_dc_sync && recv_wkc > 0)
    {
      const int64_t phase_ns = dc_phase_ns(ec_context_.DCtime, PROCESS_CYCLE_NS);
      if (!phase_aligned)
      {
        realign_ns = -phase_ns;
        phase_aligned = true;
      }
      else
      {
        sync_linux_to_dc(ec_context_.DCtime, PROCESS_CYCLE_NS, &toff, &toff_integral);
      }

      const int32_t phase_us = static_cast<int32_t>(phase_ns / 1000);
      dc_phase_us_.store(phase_us);
      if (std::abs(phase_us) <= DC_LOCK_TOLERANCE_US)
      {
        dc_locked_cycles_.fetch_add(1);
      }
      else
      {
        dc_locked_cycles_.store(0);
      }
    }

    {
      CycleTiming & sample = timing_history_[timing_history_next_];
      timing_history_next_ = (timing_history_next_ + 1) % TIMING_HISTORY_CYCLES;
      sample.wake_late_us = static_cast<int32_t>(ns_between(ts, woke) / 1000);
      sample.exchange_us = static_cast<int32_t>(ns_between(exchange_start, exchange_end) / 1000);
      sample.dc_phase_us = (use_dc_sync && recv_wkc > 0)
          ? static_cast<int32_t>(dc_phase_ns(ec_context_.DCtime, PROCESS_CYCLE_NS) / 1000)
          : 0;
      sample.toff_us = static_cast<int32_t>(toff / 1000);
      sample.toff_i_us = static_cast<int32_t>(toff_integral / 1000);
      sample.wkc = static_cast<int16_t>(recv_wkc);
    }

    // ---- 4. Publish inputs ----
    const uint16_t sw_left = rd_u16(tx_buffer_left_.data() + TX_OFF_SW);
    const uint16_t sw_right = rd_u16(tx_buffer_right_.data() + TX_OFF_SW);
    const uint16_t sw_turret = rd_u16(tx_buffer_turret_.data() + TX_OFF_SW);
    statusword_left_.store(sw_left);
    statusword_right_.store(sw_right);
    statusword_turret_.store(sw_turret);
    actual_tps_left_.store(rd_i32(tx_buffer_left_.data() + TX_OFF_VEL));
    actual_tps_right_.store(rd_i32(tx_buffer_right_.data() + TX_OFF_VEL));
    actual_tps_turret_.store(rd_i32(tx_buffer_turret_.data() + TX_OFF_VEL));

    // ---- 5. Fault detection and recovery ----
    auto update_fault_cycles = [](uint16_t sw, int & cycles) { cycles = (sw & SW_FAULT_BIT) ? cycles + 1 : 0; };
    update_fault_cycles(sw_left, left_fault_cycles);
    update_fault_cycles(sw_right, right_fault_cycles);
    update_fault_cycles(sw_turret, turret_fault_cycles);
    const bool any_fault = left_fault_cycles > 0 || right_fault_cycles > 0 || turret_fault_cycles > 0;

    if (operation_enabled && !any_fault)
    {
      if (recovery_attempts > 0 && ++healthy_cycles >= RECOVERY_BUDGET_REFILL_CYCLES)
      {
        recovery_attempts = 0;
        healthy_cycles = 0;
        RCLCPP_INFO(logger_, "Fault-free for 30 s: runtime fault recovery budget refilled.");
      }
    }
    else
    {
      healthy_cycles = 0;
    }

    if (operation_enabled && any_fault)
    {
      // Diagnostics (SDO reads) are deferred to the diagnostics thread;
      // running them here stalls the cyclic RPDO for tens of ms and faults
      // the remaining drives with a fieldbus SYNC error.
      uint8_t dump_mask = 0;
      if (left_fault_cycles > 0) dump_mask |= 0x01;
      if (right_fault_cycles > 0) dump_mask |= 0x02;
      if (turret_fault_cycles > 0) dump_mask |= 0x04;
      if (!timing_dump_ready_.load(std::memory_order_acquire))
      {
        for (size_t i = 0; i < TIMING_HISTORY_CYCLES; ++i)
        {
          timing_dump_[i] = timing_history_[(timing_history_next_ + i) % TIMING_HISTORY_CYCLES];
        }
        timing_dump_ready_.store(true, std::memory_order_release);
      }
      fault_dump_pending_.store(dump_mask, std::memory_order_release);

      operation_enabled_.store(false);
      force_zero_velocity_.store(true);

      if (recovery_attempts < MAX_FAULT_RECOVERY_ATTEMPTS)
      {
        ++recovery_attempts;
        in_recovery = true;
        recovery_pulse = false;
        fault_cooldown = RECOVERY_COOLDOWN_CYCLES;
        reset_pulse_cycles = 0;
        RCLCPP_WARN(
            logger_, "Runtime drive fault detected (SW L=0x%04X R=0x%04X T=0x%04X). Auto-recovery attempt %d/%d after cooldown.",
            sw_left, sw_right, sw_turret, recovery_attempts, MAX_FAULT_RECOVERY_ATTEMPTS);
      }
      else
      {
        enable_request_.store(false);
        in_recovery = false;
        RCLCPP_ERROR(
            logger_, "Runtime drive fault: recovery attempts exhausted (%d). Drives stay disabled.",
            MAX_FAULT_RECOVERY_ATTEMPTS);
      }
    }

    if (enable_request && !operation_enabled)
    {
      if (in_recovery)
      {
        if (fault_cooldown > 0)
        {
          --fault_cooldown;
          recovery_pulse = false;
        }
        else if (any_fault)
        {
          recovery_pulse = true;
          if (++reset_pulse_cycles > RECOVERY_RESET_PULSE_LIMIT)
          {
            // Fault does not clear: not a transient fieldbus error.
            recovery_pulse = false;
            reset_pulse_cycles = 0;
            if (recovery_attempts < MAX_FAULT_RECOVERY_ATTEMPTS)
            {
              ++recovery_attempts;
              fault_cooldown = RECOVERY_COOLDOWN_CYCLES;
              RCLCPP_WARN(
                  logger_, "Fault reset did not clear within %d cycles. Retry %d/%d.",
                  RECOVERY_RESET_PULSE_LIMIT, recovery_attempts, MAX_FAULT_RECOVERY_ATTEMPTS);
            }
            else
            {
              in_recovery = false;
              enable_request_.store(false);
              RCLCPP_ERROR(
                  logger_, "Drive fault is persistent after %d reset attempts. Drives stay disabled.",
                  MAX_FAULT_RECOVERY_ATTEMPTS);
            }
          }
        }
        else
        {
          // Fault cleared; let next_cw_from_sw() re-run the CiA 402 enable
          // sequence on the following cycles.
          recovery_pulse = false;
        }
      }

      const bool all_enabled = (sw_left & STATUS_MASK) == SW_OPERATION_ENABLED &&
                               (sw_right & STATUS_MASK) == SW_OPERATION_ENABLED &&
                               (sw_turret & STATUS_MASK) == SW_OPERATION_ENABLED;
      if (all_enabled)
      {
        if (++stable_enabled_cycles >= ENABLE_STABLE_CYCLES)
        {
          ramp_active = true;
          applied_left = applied_right = applied_turret = 0;
          operation_enabled_.store(true);
          force_zero_velocity_.store(false);
          if (in_recovery)
          {
            in_recovery = false;
            recovery_pulse = false;
            reset_pulse_cycles = 0;
            RCLCPP_INFO(
                logger_,
                "Drives recovered from runtime fault: Operation Enabled restored; "
                "references ramp up at %.0f rad/s^2 (motor) until they match the command.",
                ENABLE_RAMP_MOTOR_RAD_S2);
          }
        }
      }
      else
      {
        stable_enabled_cycles = 0;
      }

      if (!in_recovery &&
          (left_fault_cycles >= ENABLE_FAULT_ABORT_CYCLES || right_fault_cycles >= ENABLE_FAULT_ABORT_CYCLES ||
           turret_fault_cycles >= ENABLE_FAULT_ABORT_CYCLES))
      {
        enable_fault_abort_.store(true);
        enable_request_.store(false);
        operation_enabled_.store(false);
      }
    }
  }

  stage_zero_rpdo(CW_DISABLE_VOLTAGE);
  exchange_processdata();
}

// =========================================================================
// Diagnostics thread
// =========================================================================

void IrdEthercatSystem::start_diagnostics_thread()
{
  if (diagnostics_thread_.joinable()) return;
  diagnostics_thread_stop_.store(false);
  std::fill(esc_counters_last_.begin(), esc_counters_last_.end(), std::array<uint8_t, 20>{});
  diagnostics_thread_ = std::thread(&IrdEthercatSystem::diagnostics_loop, this);
}

void IrdEthercatSystem::stop_diagnostics_thread()
{
  diagnostics_thread_stop_.store(true);
  if (diagnostics_thread_.joinable())
  {
    diagnostics_thread_.join();
  }
}

void IrdEthercatSystem::diagnostics_loop()
{
  // Plain time-sharing priority: this thread must never compete with the
  // RT PDO thread or the ros2_control update loop.
  sched_param sched{};
  pthread_setschedparam(pthread_self(), SCHED_OTHER, &sched);

  int since_counters_ms = 0;
  while (!diagnostics_thread_stop_.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(DIAG_POLL_MS));

    log_timing_diagnostics();

    const uint8_t dump_mask = fault_dump_pending_.exchange(0, std::memory_order_acquire);
    if (dump_mask)
    {
      if (dump_mask & 0x01)
      {
        dump_drive_fault_snapshot(
            &ec_context_, logger_, cfg_.slave_left_motor, "LEFT",
            statusword_left_.load(), ec_context_.slavelist[cfg_.slave_left_motor].ALstatuscode);
      }
      if (dump_mask & 0x02)
      {
        dump_drive_fault_snapshot(
            &ec_context_, logger_, cfg_.slave_right_motor, "RIGHT",
            statusword_right_.load(), ec_context_.slavelist[cfg_.slave_right_motor].ALstatuscode);
      }
      if (dump_mask & 0x04)
      {
        dump_drive_fault_snapshot(
            &ec_context_, logger_, cfg_.slave_turret_motor, "TURRET",
            statusword_turret_.load(), ec_context_.slavelist[cfg_.slave_turret_motor].ALstatuscode);
      }
      log_esc_error_counters(true);
      since_counters_ms = 0;
    }

    since_counters_ms += DIAG_POLL_MS;
    if (since_counters_ms >= ESC_COUNTERS_PERIOD_MS)
    {
      since_counters_ms = 0;
      log_esc_error_counters(false);
    }
  }
}

void IrdEthercatSystem::log_esc_error_counters(bool force)
{
  // ESC registers 0x0300..0x0313: per-port invalid-frame / RX-error
  // counters, forwarded-RX-error counters, processing-unit and PDI error
  // counters, lost-link counters. They count since drive power-up and are
  // the one place a corrupted frame on the wire (EMI, cabling) shows up
  // when the master-side NIC counters stay clean.
  const struct { int slave; const char * name; } slaves[] = {
      {cfg_.slave_left_motor, "LEFT"}, {cfg_.slave_right_motor, "RIGHT"}, {cfg_.slave_turret_motor, "TURRET"}};
  for (const auto & s : slaves)
  {
    if (s.slave < 1 || s.slave >= static_cast<int>(esc_counters_last_.size()))
    {
      continue;
    }
    std::array<uint8_t, 20> regs{};
    const uint16_t configadr = ec_context_.slavelist[s.slave].configadr;
    const int wkc = ecx_FPRD(&ec_context_.port, configadr, 0x0300, regs.size(), regs.data(), EC_TIMEOUTRET);
    if (wkc <= 0)
    {
      if (force)
      {
        RCLCPP_WARN(logger_, "%s (slave %d): ESC error counter read failed (wkc=%d)", s.name, s.slave, wkc);
      }
      continue;
    }
    auto & last = esc_counters_last_[s.slave];
    if (!force && regs == last)
    {
      continue;
    }
    last = regs;
    RCLCPP_WARN(
        logger_,
        "%s (slave %d) ESC error counters: port0 invalid-frame=%u rx-err=%u fwd-rx-err=%u lost-link=%u | "
        "port1 invalid-frame=%u rx-err=%u fwd-rx-err=%u lost-link=%u | PU-err=%u PDI-err=%u",
        s.name, s.slave,
        regs[0], regs[1], regs[8], regs[16],
        regs[2], regs[3], regs[9], regs[17],
        regs[12], regs[13]);
  }
}

void IrdEthercatSystem::log_timing_diagnostics()
{
  // HardSync needs each frame at the drives >= 100 us before SYNC0, i.e. a
  // DC phase below (DC_SYNC_SHIFT_NS/1000 - 100) us. The phase is reported
  // in [-cycle/2, cycle/2), so with SYNC0 mid-cycle a frame later than SYNC0
  // wraps to just above -cycle/2: flag both ends.
  const int32_t sync0_us = static_cast<int32_t>(DC_SYNC_SHIFT_NS / 1000);
  const int32_t phase_limit_us = sync0_us - 100;

  if (!timing_dump_ready_.load(std::memory_order_acquire)) return;

  // Print only cycles that could have faulted the drives, plus the last
  // few before detection for context.
  constexpr size_t TAIL = 8;
  RCLCPP_WARN(
      logger_,
      "EtherCAT timing before the fault (last %zu cycles, oldest first; expected WKC %d, SYNC0 at DC phase %d us):",
      TIMING_HISTORY_CYCLES, expected_wkc_, sync0_us);
  for (size_t i = 0; i < TIMING_HISTORY_CYCLES; ++i)
  {
    const CycleTiming & c = timing_dump_[i];
    const bool anomalous = c.wkc < expected_wkc_ || c.wake_late_us > TIMING_LATE_WAKE_US ||
                           std::abs(c.dc_phase_us) > phase_limit_us || c.exchange_us > 1000;
    if (!anomalous && i + TAIL < TIMING_HISTORY_CYCLES) continue;
    RCLCPP_WARN(
        logger_, "  cycle %4d: wake late %5d us, exchange %5d us, DC phase %5d us, toff %4d us (I %4d), WKC %d%s",
        static_cast<int>(i) - static_cast<int>(TIMING_HISTORY_CYCLES), c.wake_late_us, c.exchange_us,
        c.dc_phase_us, c.toff_us, c.toff_i_us, c.wkc, anomalous ? "  <--" : "");
  }
  timing_dump_ready_.store(false, std::memory_order_release);
}

}  // namespace ird_ethercat_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ird_ethercat_hardware_interface::IrdEthercatSystem, hardware_interface::SystemInterface)
