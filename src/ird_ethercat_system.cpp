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
//   on_init      read and validate the configuration, check the joints.
//   on_activate  1. open the network interface, scan the bus, configure
//                   every drive by SDO in PRE-OP (mode, PDO mapping, sync),
//                2. start distributed clocks (SYNC0) and go to SAFE-OP,
//                3. start the RT process-data thread and wait until its
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
#include <limits>
#include <pthread.h>
#include <set>
#include <sys/mman.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ird_ethercat_hardware_interface/cia402.hpp"
#include "ird_ethercat_hardware_interface/config_loader.hpp"
#include "ird_ethercat_hardware_interface/startup_plan.hpp"

namespace ird_ethercat_hardware_interface
{
namespace
{
// The master steers its frames to pass the DC reference drive at the cycle
// boundary; SYNC0 fires bus.sync0_shift_us later. Infranor recommends at
// least 100 us (IRD manual, error 50) and 200 us for a 4 ms cycle; on a NIC
// shared with sensor traffic the Naviwall robot needed 2 ms (mid-cycle) to
// absorb late frames (see README).
constexpr int64_t DC_TARGET_PHASE_NS = 0;

// Minimum lead of the frame over SYNC0 required by the IRD (error 50).
constexpr int32_t IRD_MIN_LEAD_US = 100;

constexpr int DIAG_POLL_MS = 20;
constexpr int ESC_COUNTERS_PERIOD_MS = 10'000;
constexpr int32_t TIMING_LATE_WAKE_US = 200;  // marks a cycle as anomalous in the timing dump
constexpr auto SAFE_STOP_DELAY = std::chrono::milliseconds(200);  // PDO disable before leaving OP
constexpr auto THREAD_START_SETTLE = std::chrono::milliseconds(250);

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

// First wake-up: the next whole millisecond.
ec_timet next_whole_ms()
{
  ec_timet ts;
  osal_get_monotonic_time(&ts);
  const long next_ms = static_cast<long>(ts.tv_nsec / 1000000) + 1;
  ts.tv_sec += next_ms / 1000;
  ts.tv_nsec = (next_ms % 1000) * 1000000;
  return ts;
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

}  // namespace

IrdEthercatSystem::IrdEthercatSystem() = default;

IrdEthercatSystem::~IrdEthercatSystem()
{
  // Normally done by on_deactivate/on_shutdown; this only covers the plugin
  // being destroyed while still active, so that no thread outlives it.
  if (ec_initialized_)
  {
    disable_drives_and_leave_op();
    shutdown_ethercat();
  }
  stop_processdata_thread();
  stop_diagnostics_thread();
}

// =========================================================================
// ros2_control: initialisation and interfaces
// =========================================================================

hardware_interface::CallbackReturn IrdEthercatSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!load_config())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const std::vector<std::string> errors = validate(config_);
  for (const std::string & error : errors)
  {
    RCLCPP_FATAL(logger_, "Configuration error: %s", error.c_str());
  }
  if (!errors.empty())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- Per-drive runtime state ----
  const double cycle_s = config_.bus.cycle_time_us * 1e-6;
  std::vector<int32_t> ramp_steps;
  drives_.clear();
  for (const DriveConfig & cfg : config_.drives)
  {
    auto drive = std::make_unique<Drive>();
    drive->cfg = &cfg;
    drive->rx_layout = compute_layout(cfg.rx_pdo);
    drive->tx_layout = compute_layout(cfg.tx_pdo);
    drive->pdo_controlword = find_role(cfg.rx_pdo, drive->rx_layout, PdoRole::CONTROLWORD);
    drive->pdo_mode_of_operation = find_role(cfg.rx_pdo, drive->rx_layout, PdoRole::MODE_OF_OPERATION);
    drive->pdo_target_velocity = find_role(cfg.rx_pdo, drive->rx_layout, PdoRole::TARGET_VELOCITY);
    drive->pdo_statusword = find_role(cfg.tx_pdo, drive->tx_layout, PdoRole::STATUSWORD);
    drive->pdo_velocity_actual = find_role(cfg.tx_pdo, drive->tx_layout, PdoRole::VELOCITY_ACTUAL);
    drive->ticks_per_rad = cfg.encoder_ticks_per_rev / (2.0 * M_PI);
    drive->max_motor_rad_s = cfg.max_motor_rpm * 2.0 * M_PI / 60.0;

    // Largest reference increase per cycle while ramping up after an enable.
    const double step = cfg.enable_ramp_motor_rad_s2 * cycle_s * drive->ticks_per_rad;
    ramp_steps.push_back(
        cfg.enable_ramp_motor_rad_s2 > 0.0 ? std::max<int32_t>(1, static_cast<int32_t>(step))
                                           : std::numeric_limits<int32_t>::max() / 2);
    drives_.push_back(std::move(drive));
  }

  if (!check_joints())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t n = drives_.size();
  rt_targets_.assign(n, 0);
  rt_references_.assign(n, 0);
  rt_statuswords_.assign(n, 0);
  rt_controlwords_.assign(n, 0);
  esc_counters_last_.assign(n, {});
  controller_.emplace(config_.faults, config_.bus.cycle_time_us, ramp_steps);
  dc_sync_.emplace(static_cast<int64_t>(config_.bus.cycle_time_us) * 1000, DC_TARGET_PHASE_NS);
  master_ = std::make_unique<EthercatMaster>(logger_);

  RCLCPP_INFO(
      logger_, "Configured %zu drive(s) on %s, cycle %u us.", n, config_.bus.interface.c_str(),
      config_.bus.cycle_time_us);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// The configuration comes from the YAML file named by the URDF parameter
// `config_file`. Without it, the legacy URDF parameters of the three-drive
// Naviwall layout are used (kept so older robot descriptions still work).
// The URDF parameter `ethercat_interface` overrides bus.interface either way,
// so the network interface can still be chosen from a launch argument.
bool IrdEthercatSystem::load_config()
{
  const auto & params = info_.hardware_parameters;
  const auto file = params.find("config_file");
  if (file != params.end() && !file->second.empty())
  {
    const std::vector<std::string> errors = load_config_file(file->second, config_);
    for (const std::string & error : errors)
    {
      RCLCPP_FATAL(logger_, "Configuration file: %s", error.c_str());
    }
    if (!errors.empty())
    {
      return false;
    }
    for (const char * legacy : {"left_wheel_name", "right_wheel_name", "turret_name", "slave_left_motor",
                                "slave_right_motor", "slave_turret_motor", "transmission_ratio_wheels",
                                "transmission_ratio_turret", "encoder_ticks_per_rev"})
    {
      if (params.count(legacy))
      {
        RCLCPP_WARN(
            logger_, "URDF parameter '%s' is ignored: the configuration comes from config_file.", legacy);
      }
    }
    RCLCPP_INFO(logger_, "Configuration loaded from %s", file->second.c_str());
  }
  else
  {
    RCLCPP_INFO(
        logger_, "No 'config_file' parameter: using the legacy URDF parameters (three drives: left, right, turret).");
    if (!load_config_from_urdf_params())
    {
      return false;
    }
  }

  const auto iface = params.find("ethercat_interface");
  if (iface != params.end() && !iface->second.empty())
  {
    config_.bus.interface = iface->second;
  }
  return true;
}

// Builds the configuration from the legacy URDF <hardware> parameters of the
// three-drive Naviwall layout (left wheel, right wheel, turret).
bool IrdEthercatSystem::load_config_from_urdf_params()
{
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

  config_ = SystemConfig{};
  double ratio_wheels = 0.0;
  double ratio_turret = 0.0;
  int ticks_per_rev = 0;
  if (!get_double("transmission_ratio_wheels", ratio_wheels) ||
      !get_double("transmission_ratio_turret", ratio_turret) ||
      !get_int("encoder_ticks_per_rev", ticks_per_rev))
  {
    return false;
  }

  const struct { const char * name; const char * joint_param; const char * slave_param; double ratio; } layout[] = {
      {"left", "left_wheel_name", "slave_left_motor", ratio_wheels},
      {"right", "right_wheel_name", "slave_right_motor", ratio_wheels},
      {"turret", "turret_name", "slave_turret_motor", ratio_turret},
  };
  for (const auto & d : layout)
  {
    DriveConfig drive = make_ird_csv_drive();
    drive.name = d.name;
    drive.gear_ratio = d.ratio;
    drive.encoder_ticks_per_rev = ticks_per_rev;
    if (!get_string(d.joint_param, drive.joint) || !get_int(d.slave_param, drive.slave_position))
    {
      return false;
    }
    config_.drives.push_back(drive);
  }
  return true;
}

// Every drive's joint must be declared in the <ros2_control> block with one
// velocity command and position + velocity states, and every declared joint
// must be driven by a drive.
bool IrdEthercatSystem::check_joints()
{
  std::set<std::string> driven;
  for (const auto & drive : drives_)
  {
    driven.insert(drive->cfg->joint);
    const bool declared = std::any_of(
        info_.joints.begin(), info_.joints.end(),
        [&](const hardware_interface::ComponentInfo & joint) { return joint.name == drive->cfg->joint; });
    if (!declared)
    {
      RCLCPP_FATAL(
          logger_, "Joint '%s' of drive '%s' is not declared in this <ros2_control> block.",
          drive->cfg->joint.c_str(), drive->cfg->name.c_str());
      return false;
    }
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (driven.count(joint.name) == 0)
    {
      RCLCPP_FATAL(logger_, "Joint '%s' is declared in the <ros2_control> block but no drive drives it.", joint.name.c_str());
      return false;
    }
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
          logger_, "Joint '%s' must have exactly one command interface, '%s'.", joint.name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return false;
    }
    if (joint.state_interfaces.size() != 2 || joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
          logger_, "Joint '%s' must have exactly two state interfaces, '%s' then '%s'.", joint.name.c_str(),
          hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
      return false;
    }
  }
  return true;
}

std::vector<hardware_interface::StateInterface> IrdEthercatSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const auto & drive : drives_)
  {
    state_interfaces.emplace_back(drive->cfg->joint, hardware_interface::HW_IF_POSITION, &drive->pos);
    state_interfaces.emplace_back(drive->cfg->joint, hardware_interface::HW_IF_VELOCITY, &drive->vel);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> IrdEthercatSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto & drive : drives_)
  {
    command_interfaces.emplace_back(drive->cfg->joint, hardware_interface::HW_IF_VELOCITY, &drive->cmd);
  }
  return command_interfaces;
}

std::vector<int> IrdEthercatSystem::slave_positions() const
{
  std::vector<int> positions;
  for (const auto & drive : drives_)
  {
    positions.push_back(drive->cfg->slave_position);
  }
  return positions;
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
  // the cycle long enough to trip the drives' fieldbus SYNC monitor.
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
  {
    RCLCPP_WARN(
        logger_, "mlockall failed (%s). RT PDO thread may suffer page-fault jitter; check memlock limits "
        "(ulimit -l) or CAP_IPC_LOCK.", std::strerror(errno));
  }

  if (!init_ethercat())
  {
    RCLCPP_ERROR(logger_, "EtherCAT initialization failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!clear_latched_faults())
  {
    stage_outputs(cia402::CW_DISABLE_VOLTAGE);
    master_->exchange();
    shutdown_ethercat();
    return hardware_interface::CallbackReturn::ERROR;
  }

  stage_outputs(cia402::CW_DISABLE_VOLTAGE);
  for (const auto & drive : drives_)
  {
    drive->target.store(0);
    drive->actual_velocity.store(0);
    drive->statusword.store(static_cast<uint16_t>(read_value(drive->inputs + drive->pdo_statusword.offset, DataType::UINT16)));
  }
  flags_.force_zero.store(false);
  fault_dump_pending_.store(0);
  low_wkc_reads_ = 0;

  RCLCPP_INFO(logger_, "Starting RT processdata loop in SAFE-OP before requesting OPERATIONAL...");
  start_processdata_thread();
  start_diagnostics_thread();
  std::this_thread::sleep_for(THREAD_START_SETTLE);

  if (config_.bus.distributed_clocks && master_->has_dc())
  {
    wait_for_dc_lock();
  }

  RCLCPP_INFO(logger_, "Requesting OPERATIONAL state...");
  master_->request_state(EC_STATE_OPERATIONAL);
  if (!master_->wait_for_state(slave_positions(), EC_STATE_OPERATIONAL, static_cast<int64_t>(EC_TIMEOUTSTATE) * 4))
  {
    for (const auto & drive : drives_)
    {
      const int slave = drive->cfg->slave_position;
      if (master_->state(slave) != EC_STATE_OPERATIONAL)
      {
        RCLCPP_ERROR(
            logger_, "Drive '%s' (slave %d) failed to reach OPERATIONAL. State=0x%02X, AL=0x%04X",
            drive->cfg->name.c_str(), slave, master_->state(slave), master_->al_status_code(slave));
      }
    }
    disable_drives_and_leave_op();
    shutdown_ethercat();
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "All slaves reached OPERATIONAL state.");

  if (!enable_drives())
  {
    // Stop through the cyclic PDO first: stopping the RT thread while the
    // drives are still in OP would latch a fieldbus fault (error 49/50).
    disable_drives_and_leave_op();
    for (const auto & drive : drives_)
    {
      dump_fault_snapshot(*drive);
    }
    RCLCPP_ERROR(logger_, "Failed to enable the drives via the PDO state machine.");
    shutdown_ethercat();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
      logger_, "All drives enabled successfully. Ready for PDO cyclic operation (DC phase %d us, SYNC0 at %u us).",
      dc_phase_us_.load(), config_.bus.sync0_shift_us);
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
  if (recv_wkc > 0 && recv_wkc < master_->expected_wkc())
  {
    if (++low_wkc_reads_ % 250 == 0)
    {
      RCLCPP_WARN(
          logger_, "read(): WKC below expectation: got %d expected at least %d", recv_wkc, master_->expected_wkc());
    }
  }

  // Drive feedback is motor-shaft ticks/s; joints are on the gearbox output.
  for (const auto & drive : drives_)
  {
    drive->vel = drive->actual_velocity.load() / drive->ticks_per_rad / drive->cfg->gear_ratio;
    drive->pos += drive->vel * period.seconds();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IrdEthercatSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!ec_initialized_) return hardware_interface::return_type::OK;

  const bool force_zero = flags_.force_zero.load();
  if (force_zero && !first_write_warned_)
  {
    RCLCPP_WARN(logger_, "SAFETY: force_zero_velocity enabled. Target velocity forced to 0.");
    first_write_warned_ = true;
  }
  else if (!force_zero)
  {
    first_write_warned_ = false;
  }

  const bool apply = !force_zero && flags_.operation_enabled.load();
  for (const auto & drive : drives_)
  {
    int32_t ticks_s = 0;
    if (apply)
    {
      double joint_rad_s = drive->cmd;
      if (!std::isfinite(joint_rad_s))
      {
        RCLCPP_ERROR_THROTTLE(
            logger_, steady_clock_, 1000,
            "Drive '%s' received non-finite velocity command (%f rad/s). Forcing target velocity to 0.",
            drive->cfg->name.c_str(), joint_rad_s);
        joint_rad_s = 0.0;
      }

      // Joint command is gearbox-output rad/s; the drive expects motor-shaft
      // ticks/s. The speed limit is a motor-shaft limit, hence applied after
      // the gear ratio.
      double motor_rad_s = joint_rad_s * drive->cfg->gear_ratio;
      if (std::fabs(motor_rad_s) > drive->max_motor_rad_s)
      {
        const double clamped = std::clamp(motor_rad_s, -drive->max_motor_rad_s, drive->max_motor_rad_s);
        RCLCPP_WARN_THROTTLE(
            logger_, steady_clock_, 1000,
            "Drive '%s' motor velocity %.6f rad/s exceeds %.6f rad/s (%.1f rpm). Clamping to %.6f rad/s.",
            drive->cfg->name.c_str(), motor_rad_s, drive->max_motor_rad_s, drive->cfg->max_motor_rpm, clamped);
        motor_rad_s = clamped;
      }
      ticks_s = static_cast<int32_t>(motor_rad_s * drive->ticks_per_rad);
    }
    drive->target.store(ticks_s);
  }
  return hardware_interface::return_type::OK;
}

// =========================================================================
// Bus start-up and shutdown
// =========================================================================

bool IrdEthercatSystem::init_ethercat()
{
  const BusConfig & bus = config_.bus;
  const std::vector<int> slaves = slave_positions();
  const bool dc = bus.distributed_clocks;
  const bool hard_sync = bus.sync_mode == SyncMode::HARD_SYNC;

  auto fail = [&]()
  {
    master_->close();
    return false;
  };

  if (!master_->open(bus.interface))
  {
    return false;
  }
  master_->log_slaves();

  const int highest = *std::max_element(slaves.begin(), slaves.end());
  if (master_->slave_count() < highest)
  {
    RCLCPP_ERROR(
        logger_, "Found %d slave(s) on the bus, but the configuration uses slave position %d.",
        master_->slave_count(), highest);
    return fail();
  }
  for (const auto & drive : drives_)
  {
    if (!master_->check_identity(*drive->cfg)) return fail();
  }
  dc_sync_->reset();

  // Suppress SOEM's automatic SAFE-OP request at the end of
  // map_process_data(). The IRD selects its sync mode at the PRE-OP ->
  // SAFE-OP (PS) transition, so SYNC0 and the 0x1006/0x2012 objects must be
  // configured BEFORE that transition, otherwise the drive falls back to
  // SoftSync and ignores later HardSync writes. SAFE-OP is requested
  // explicitly below. (Set only after the bus scan in open(): the same flag
  // would suppress the INIT -> PRE-OP request too.)
  master_->set_manual_state_change(true);

  for (const auto & drive : drives_)
  {
    if (!master_->configure_drive(*drive->cfg, bus))
    {
      RCLCPP_ERROR(logger_, "Failed to configure drive '%s'", drive->cfg->name.c_str());
      return fail();
    }
  }

  RCLCPP_INFO(logger_, "All slaves configured. Setting up process data mapping...");
  if (!master_->map_process_data()) return fail();

  if (dc && !hard_sync)
  {
    // SoftSync: the drive picks HardSync at the PS transition whenever SYNC0
    // is already running, regardless of 0x2012:01, so complete PRE-OP ->
    // SAFE-OP before SYNC0 is enabled below.
    master_->request_state(EC_STATE_SAFE_OP);
    if (!master_->wait_for_state(slaves, EC_STATE_SAFE_OP, static_cast<int64_t>(EC_TIMEOUTSTATE) * 4))
    {
      RCLCPP_ERROR(logger_, "Not all drives reached SAFE-OP before SYNC0 (SoftSync).");
      return fail();
    }
    RCLCPP_INFO(logger_, "SoftSync: all slaves in SAFE-OP before enabling SYNC0.");
  }

  if (dc)
  {
    const int64_t cycle_ns = static_cast<int64_t>(bus.cycle_time_us) * 1000;
    const int64_t shift_ns = static_cast<int64_t>(bus.sync0_shift_us) * 1000;
    master_->configure_dc(slaves, cycle_ns, shift_ns);
    RCLCPP_INFO(
        logger_, "Configured DC SYNC0: cycle=%ld ns shift=%ld ns master_phase=%ld ns", static_cast<long>(cycle_ns),
        static_cast<long>(shift_ns), static_cast<long>(DC_TARGET_PHASE_NS));
    for (int slave : slaves)
    {
      master_->log_dc_snapshot(slave);
    }
  }
  else
  {
    RCLCPP_INFO(logger_, "Using free-run PDO mode (DC/SYNC0 disabled, cycle=%u us)", bus.cycle_time_us);
  }

  for (const auto & drive : drives_)
  {
    drive->outputs = master_->outputs(drive->cfg->slave_position);
    drive->inputs = master_->inputs(drive->cfg->slave_position);
  }
  master_->exchange();

  // The mapped sizes must match the PDO layout this driver encodes/decodes.
  for (const auto & drive : drives_)
  {
    const int slave = drive->cfg->slave_position;
    if (master_->output_bytes(slave) != drive->rx_layout.size_bytes ||
        master_->input_bytes(slave) != drive->tx_layout.size_bytes)
    {
      RCLCPP_ERROR(
          logger_, "Drive '%s' (slave %d) PDO size mismatch! RxPDO expected=%zu actual=%zu, TxPDO expected=%zu actual=%zu",
          drive->cfg->name.c_str(), slave, drive->rx_layout.size_bytes, master_->output_bytes(slave),
          drive->tx_layout.size_bytes, master_->input_bytes(slave));
      return fail();
    }
  }
  RCLCPP_INFO(logger_, "Process data mapping validated for all slaves.");

  master_->request_state(EC_STATE_SAFE_OP);
  if (!master_->wait_for_state(slaves, EC_STATE_SAFE_OP, static_cast<int64_t>(EC_TIMEOUTSTATE) * 4))
  {
    for (int slave : slaves)
    {
      if (master_->state(slave) != EC_STATE_SAFE_OP)
      {
        RCLCPP_ERROR(logger_, "Slave %d failed to reach SAFE-OP. State=0x%02X", slave, master_->state(slave));
      }
    }
    return fail();
  }
  RCLCPP_INFO(logger_, "All slaves reached SAFE-OP.");

  if (dc)
  {
    RCLCPP_INFO(logger_, "Letting the EtherCAT network settle against DC clocks in SAFE-OP...");
    run_startup_cycles(ms_to_cycles(bus.startup_settle_ms, bus.cycle_time_us));
  }

  // Report the sync mode each drive actually latched.
  for (const auto & drive : drives_)
  {
    const int slave = drive->cfg->slave_position;
    int64_t cycle_us = 0;
    int64_t sync_type = 0;
    master_->sdo_read(slave, OBJ_CYCLE_PERIOD, 0x00, DataType::UINT32, cycle_us);
    master_->sdo_read(slave, OBJ_SYNC_MODE, 0x01, DataType::UINT16, sync_type);
    RCLCPP_INFO(
        logger_, "Slave %d SAFE-OP sync settings: 0x1006:00=%lld us, 0x2012:01=%lld, hasdc=%d", slave,
        static_cast<long long>(cycle_us), static_cast<long long>(sync_type), master_->slave_has_dc(slave));

    if (!dc)
    {
      continue;
    }
    if (!master_->slave_has_dc(slave))
    {
      RCLCPP_ERROR(logger_, "Slave %d does not report distributed clocks capability.", slave);
      return fail();
    }
    if (!hard_sync)
    {
      if (sync_type == IRD_SYNC_TYPE_SOFT)
      {
        RCLCPP_INFO(logger_, "Slave %d: SoftSync established (0x2012:01=2, 0x1006:00=%lld us).", slave,
                    static_cast<long long>(cycle_us));
      }
      else
      {
        RCLCPP_WARN(
            logger_, "Slave %d sync settings unexpected (observed 0x1006:00=%lld us, 0x2012:01=%lld; "
            "expected SoftSync, sync type 2).", slave, static_cast<long long>(cycle_us),
            static_cast<long long>(sync_type));
      }
    }
    else if (sync_type == IRD_SYNC_TYPE_HARD && cycle_us == bus.cycle_time_us)
    {
      RCLCPP_INFO(logger_, "Slave %d: HardSync established (0x2012:01=4, %lld us).", slave, static_cast<long long>(cycle_us));
    }
    else if (sync_type == IRD_SYNC_TYPE_SOFT)
    {
      RCLCPP_WARN(
          logger_, "Slave %d fell back to SoftSync (0x2012:01=2, 0x1006:00=%lld us). "
          "HardSync was requested in PRE-OP but did not latch at the PS transition.", slave,
          static_cast<long long>(cycle_us));
    }
    else
    {
      RCLCPP_WARN(
          logger_, "Slave %d sync settings unexpected (observed 0x1006:00=%lld us, 0x2012:01=%lld; "
          "expected %u us and sync type 4).", slave, static_cast<long long>(cycle_us),
          static_cast<long long>(sync_type), bus.cycle_time_us);
    }
  }

  ec_initialized_ = true;
  return true;
}

// A drive can still hold a fault from a previous session (e.g. one killed
// before its clean shutdown). Clear it by SDO now, while still in SAFE-OP.
bool IrdEthercatSystem::clear_latched_faults()
{
  constexpr uint16_t OBJ_CONTROLWORD = 0x6040;
  constexpr uint16_t OBJ_STATUSWORD = 0x6041;

  for (const auto & drive : drives_)
  {
    const int slave = drive->cfg->slave_position;
    const char * name = drive->cfg->name.c_str();

    int64_t statusword = 0;
    if (!master_->sdo_read(slave, OBJ_STATUSWORD, 0x00, DataType::UINT16, statusword))
    {
      RCLCPP_WARN(logger_, "Drive '%s' (slave %d): Failed to read statusword before PDO enable.", name, slave);
      continue;
    }
    if (!cia402::is_fault(static_cast<uint16_t>(statusword)))
    {
      continue;
    }

    RCLCPP_WARN(
        logger_, "Drive '%s' (slave %d) starts activation in FAULT (SW=0x%04X). Clearing latched fault via SDO reset.",
        name, slave, static_cast<unsigned>(statusword));
    drive->statusword.store(static_cast<uint16_t>(statusword));
    dump_fault_snapshot(*drive, /*latched_before_activation=*/true);

    master_->sdo_write(slave, {OBJ_CONTROLWORD, 0x00, DataType::UINT16, cia402::CW_FAULT_RESET});
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    master_->sdo_write(slave, {OBJ_CONTROLWORD, 0x00, DataType::UINT16, cia402::CW_DISABLE_VOLTAGE});
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!master_->sdo_read(slave, OBJ_STATUSWORD, 0x00, DataType::UINT16, statusword))
    {
      RCLCPP_ERROR(logger_, "Drive '%s' (slave %d): Failed to read statusword after SDO fault reset.", name, slave);
      return false;
    }
    if (cia402::is_fault(static_cast<uint16_t>(statusword)))
    {
      RCLCPP_ERROR(
          logger_, "Drive '%s' (slave %d): fault reset did not clear FAULT (SW=0x%04X).", name, slave,
          static_cast<unsigned>(statusword));
      drive->statusword.store(static_cast<uint16_t>(statusword));
      dump_fault_snapshot(*drive);
      return false;
    }
    RCLCPP_INFO(
        logger_, "Drive '%s' (slave %d): latched fault cleared, current SW=0x%04X", name, slave,
        static_cast<unsigned>(statusword));
  }
  return true;
}

// Waits until the RT thread's frames have stayed within the lock tolerance of
// their DC phase target long enough. Without it the drives were enabled while
// the PI loop was still pulling in from the thread's arbitrary start phase.
bool IrdEthercatSystem::wait_for_dc_lock()
{
  const BusConfig & bus = config_.bus;
  const uint32_t lock_cycles = ms_to_cycles(bus.dc_lock_ms, bus.cycle_time_us);
  const auto lock_start = std::chrono::steady_clock::now();
  int waited_ms = 0;
  while (dc_locked_cycles_.load() < lock_cycles && waited_ms < static_cast<int>(bus.dc_lock_timeout_ms))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    waited_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lock_start).count());
  }

  const int32_t sync0_us = static_cast<int32_t>(bus.sync0_shift_us);
  if (dc_locked_cycles_.load() >= lock_cycles)
  {
    RCLCPP_INFO(
        logger_, "DC phase locked (%d ms after the %lld ms settle): frame at %d us, %d us ahead of SYNC0.", waited_ms,
        static_cast<long long>(THREAD_START_SETTLE.count()), dc_phase_us_.load(), sync0_us - dc_phase_us_.load());
    return true;
  }
  RCLCPP_WARN(
      logger_,
      "DC phase not settled within %u ms (frame at %d us, SYNC0 at %d us); continuing, "
      "an error 50 right after enable is possible.",
      bus.dc_lock_timeout_ms, dc_phase_us_.load(), sync0_us);
  return false;
}

// Asks the RT thread to run the CiA 402 enable sequence through the cyclic
// controlword and waits until all drives are Operation Enabled.
bool IrdEthercatSystem::enable_drives()
{
  RCLCPP_INFO(logger_, "Enabling all slaves via cyclic PDO controlwords...");

  auto statuswords = [&]()
  {
    std::string text;
    for (const auto & drive : drives_)
    {
      char item[64];
      std::snprintf(item, sizeof(item), "%s%s=0x%04X", text.empty() ? "" : " ", drive->cfg->name.c_str(),
                    drive->statusword.load());
      text += item;
    }
    return text;
  };

  const auto timeout = std::chrono::milliseconds(config_.faults.enable_timeout_ms);
  const auto start_time = std::chrono::steady_clock::now();
  flags_.enable_fault_abort.store(false);
  flags_.operation_enabled.store(false);
  flags_.enable_request.store(true);

  for (int polls = 1; std::chrono::steady_clock::now() - start_time < timeout; ++polls)
  {
    if (flags_.operation_enabled.load())
    {
      RCLCPP_INFO(logger_, "All slaves reached Operation Enabled through PDO.");
      return true;
    }
    if (flags_.enable_fault_abort.load())
    {
      RCLCPP_ERROR(
          logger_, "Aborting PDO enable sequence after repeated drive faults. Last TPDO statuswords: %s",
          statuswords().c_str());
      return false;
    }
    if (polls % 5 == 0)
    {
      RCLCPP_DEBUG(logger_, "PDO enable status: %s", statuswords().c_str());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  flags_.enable_request.store(false);
  RCLCPP_ERROR(
      logger_, "Timed out waiting for PDO enable sequence. Final TPDO statuswords: %s", statuswords().c_str());
  return false;
}

// Brings the drives to a safe stop while the cyclic RPDO stream is still
// running, then stops the RT thread. If the PDO thread were stopped first,
// the drives would be left in OP without process data and immediately latch a
// fieldbus sync fault (0x8780/0x8782), seen as a latched FAULT on the next
// activation.
void IrdEthercatSystem::disable_drives_and_leave_op()
{
  RCLCPP_INFO(logger_, "Stopping all motors and disabling drives via PDO...");
  flags_.force_zero.store(true);
  for (const auto & drive : drives_)
  {
    drive->target.store(0);
  }
  flags_.enable_request.store(false);  // RT thread now stages controlword 0x0000
  flags_.operation_enabled.store(false);
  std::this_thread::sleep_for(SAFE_STOP_DELAY);

  // Leave OPERATIONAL before stopping the cyclic exchange: the sync/RPDO
  // monitoring is only armed in OP, so no fault can latch in SAFE-OP.
  master_->request_state(EC_STATE_SAFE_OP);
  if (!master_->wait_for_state(slave_positions(), EC_STATE_SAFE_OP, EC_TIMEOUTSTATE))
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

  master_->request_state(EC_STATE_SAFE_OP);
  master_->wait_for_state(slave_positions(), EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
  master_->request_state(EC_STATE_INIT);
  master_->close();
  ec_initialized_ = false;
  RCLCPP_INFO(logger_, "EtherCAT connection closed.");
}

// =========================================================================
// Process data
// =========================================================================

// Stages the same controlword and a zero reference for every drive.
void IrdEthercatSystem::stage_outputs(uint16_t controlword_all)
{
  for (const auto & drive : drives_)
  {
    if (!drive->outputs) continue;
    write_value(drive->outputs + drive->pdo_controlword.offset, DataType::UINT16, controlword_all);
    if (drive->pdo_mode_of_operation.mapped)
    {
      write_value(drive->outputs + drive->pdo_mode_of_operation.offset, DataType::INT8, static_cast<int8_t>(drive->cfg->mode));
    }
    write_value(drive->outputs + drive->pdo_target_velocity.offset, DataType::INT32, 0);
  }
}

// Exchanges `cycles` frames on the DC-locked schedule with all outputs zero,
// before the RT thread exists (settles the DC loop in SAFE-OP).
void IrdEthercatSystem::run_startup_cycles(uint32_t cycles)
{
  if (cycles == 0) return;

  const bool use_dc_sync = config_.bus.distributed_clocks && master_->has_dc();
  const int64_t cycle_ns = static_cast<int64_t>(config_.bus.cycle_time_us) * 1000;
  int64_t toff = 0;

  stage_outputs(cia402::CW_DISABLE_VOLTAGE);
  master_->exchange();

  ec_timet ts = next_whole_ms();
  // Same one-step alignment as processdata_loop(), so the shared PI
  // integral is not wound up by the pull-in from an arbitrary start phase.
  bool phase_aligned = !use_dc_sync;
  int64_t realign_ns = 0;

  for (uint32_t i = 0; i < cycles; ++i)
  {
    add_time_ns(&ts, cycle_ns + toff + realign_ns);
    realign_ns = 0;
    osal_monotonic_sleep(&ts);

    stage_outputs(cia402::CW_DISABLE_VOLTAGE);
    const int recv_wkc = master_->exchange();
    if (use_dc_sync && recv_wkc > 0)
    {
      if (!phase_aligned)
      {
        realign_ns = -dc_sync_->phase_ns(master_->dc_time());
        phase_aligned = true;
      }
      else
      {
        toff = dc_sync_->update(master_->dc_time());
      }
    }
  }
}

void IrdEthercatSystem::start_processdata_thread()
{
  if (processdata_thread_.joinable()) return;
  processdata_thread_stop_.store(false);
  processdata_thread_ = std::thread(&IrdEthercatSystem::processdata_loop, this);
}

void IrdEthercatSystem::stop_processdata_thread()
{
  processdata_thread_stop_.store(true);
  if (processdata_thread_.joinable())
  {
    processdata_thread_.join();
  }
}

// The RT loop. Every cycle it:
//  1. sleeps until the next absolute deadline (corrected by the DC PI loop),
//  2. stages controlword + reference of every drive (CyclicController::stage),
//  3. exchanges one frame and updates the DC lock and the timing history,
//  4. publishes statuswords and measured velocities for read(),
//  5. detects drive faults and runs the recovery (CyclicController::evaluate).
void IrdEthercatSystem::processdata_loop()
{
  const BusConfig & bus = config_.bus;
  const bool use_dc_sync = bus.distributed_clocks && master_->has_dc();
  const int64_t cycle_ns = static_cast<int64_t>(bus.cycle_time_us) * 1000;
  const size_t n = drives_.size();
  int64_t toff = 0;
  int64_t toff_integral = 0;
  // The loop starts on an arbitrary phase relative to the DC cycle. The
  // first received frame's phase is removed in one step (folded into the
  // next period, which stays positive since |phase| <= cycle/2: SOEM's
  // timespec add does not normalise a negative increment) instead of
  // letting the PI loop pull in over seconds and wind up its integral.
  bool phase_aligned = !use_dc_sync;
  int64_t realign_ns = 0;
  dc_locked_cycles_.store(0);
  controller_->reset();

  sched_param sched{};
  sched.sched_priority = bus.rt_priority;
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

  ec_timet ts = next_whole_ms();
  while (!processdata_thread_stop_.load())
  {
    // ---- 1. Wait for the next cycle ----
    add_time_ns(&ts, cycle_ns + toff + realign_ns);
    realign_ns = 0;
    osal_monotonic_sleep(&ts);
    ec_timet woke;
    osal_get_monotonic_time(&woke);

    // ---- 2. Stage outputs ----
    const CycleFlags flags = CyclicController::begin_cycle(flags_);
    for (size_t i = 0; i < n; ++i)
    {
      const Drive & drive = *drives_[i];
      rt_targets_[i] = drive.target.load();
      rt_statuswords_[i] = static_cast<uint16_t>(read_value(drive.inputs + drive.pdo_statusword.offset, DataType::UINT16));
    }
    controller_->stage(flags, rt_targets_, rt_statuswords_, rt_controlwords_, rt_references_);
    for (size_t i = 0; i < n; ++i)
    {
      Drive & drive = *drives_[i];
      write_value(drive.outputs + drive.pdo_controlword.offset, DataType::UINT16, rt_controlwords_[i]);
      if (drive.pdo_mode_of_operation.mapped)
      {
        write_value(drive.outputs + drive.pdo_mode_of_operation.offset, DataType::INT8, static_cast<int8_t>(drive.cfg->mode));
      }
      write_value(drive.outputs + drive.pdo_target_velocity.offset, DataType::INT32, rt_references_[i]);
    }

    // ---- 3. Exchange the frame, DC lock and timing history ----
    ec_timet exchange_start;
    osal_get_monotonic_time(&exchange_start);
    const int recv_wkc = master_->exchange();
    ec_timet exchange_end;
    osal_get_monotonic_time(&exchange_end);
    last_processdata_wkc_.store(recv_wkc);

    int32_t phase_us = 0;
    if (use_dc_sync && recv_wkc > 0)
    {
      const int64_t phase_ns = dc_sync_->phase_ns(master_->dc_time());
      if (!phase_aligned)
      {
        realign_ns = -phase_ns;
        phase_aligned = true;
      }
      else
      {
        toff = dc_sync_->update(master_->dc_time());
        toff_integral = dc_sync_->integral_term();
      }

      phase_us = static_cast<int32_t>(phase_ns / 1000);
      dc_phase_us_.store(phase_us);
      if (std::abs(phase_us) <= static_cast<int32_t>(bus.dc_lock_tolerance_us))
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
      sample.dc_phase_us = phase_us;
      sample.toff_us = static_cast<int32_t>(toff / 1000);
      sample.toff_i_us = static_cast<int32_t>(toff_integral / 1000);
      sample.wkc = static_cast<int16_t>(recv_wkc);
    }

    // ---- 4. Publish inputs ----
    for (size_t i = 0; i < n; ++i)
    {
      Drive & drive = *drives_[i];
      rt_statuswords_[i] = static_cast<uint16_t>(read_value(drive.inputs + drive.pdo_statusword.offset, DataType::UINT16));
      drive.statusword.store(rt_statuswords_[i]);
      drive.actual_velocity.store(
          static_cast<int32_t>(read_value(drive.inputs + drive.pdo_velocity_actual.offset, DataType::INT32)));
    }

    // ---- 5. Fault detection and recovery ----
    const CycleEvents events = controller_->evaluate(flags, rt_statuswords_, flags_);
    if (events.fault_detected)
    {
      // Diagnostics (SDO reads) are deferred to the diagnostics thread;
      // running them here would stall the cyclic RPDO for tens of ms and
      // fault the remaining drives with a fieldbus SYNC error.
      if (!timing_dump_ready_.load(std::memory_order_acquire))
      {
        for (size_t i = 0; i < TIMING_HISTORY_CYCLES; ++i)
        {
          timing_dump_[i] = timing_history_[(timing_history_next_ + i) % TIMING_HISTORY_CYCLES];
        }
        timing_dump_ready_.store(true, std::memory_order_release);
      }
      fault_dump_pending_.store(events.fault_mask, std::memory_order_release);
    }
    log_cycle_events(events);
  }

  stage_outputs(cia402::CW_DISABLE_VOLTAGE);
  master_->exchange();
}

void IrdEthercatSystem::log_cycle_events(const CycleEvents & events)
{
  const unsigned max_attempts = controller_->max_recovery_attempts();
  if (events.fault_detected)
  {
    char statuswords[512] = "";
    size_t used = 0;
    for (size_t i = 0; i < drives_.size() && used < sizeof(statuswords); ++i)
    {
      used += std::snprintf(
          statuswords + used, sizeof(statuswords) - used, "%s%s=0x%04X", i ? " " : "",
          drives_[i]->cfg->name.c_str(), drives_[i]->statusword.load());
    }
    if (events.recovery_scheduled)
    {
      RCLCPP_WARN(
          logger_, "Runtime drive fault detected (SW %s). Auto-recovery attempt %d/%u after cooldown.", statuswords,
          events.attempt, max_attempts);
    }
    else
    {
      RCLCPP_ERROR(
          logger_, "Runtime drive fault (SW %s): recovery attempts exhausted (%u). Drives stay disabled.", statuswords,
          max_attempts);
    }
  }
  if (events.reset_retry)
  {
    RCLCPP_WARN(
        logger_, "Fault reset did not clear within %u cycles. Retry %d/%u.", controller_->reset_pulse_cycles(),
        events.attempt, max_attempts);
  }
  if (events.reset_persistent)
  {
    RCLCPP_ERROR(logger_, "Drive fault is persistent after %u reset attempts. Drives stay disabled.", max_attempts);
  }
  if (events.recovered)
  {
    RCLCPP_INFO(
        logger_, "Drives recovered from runtime fault: Operation Enabled restored; references ramp up "
        "(enable_ramp_motor_rad_s2) until they match the command.");
  }
  if (events.budget_refilled)
  {
    RCLCPP_INFO(
        logger_, "Fault-free for %u s: runtime fault recovery budget refilled.",
        config_.faults.recovery_budget_refill_ms / 1000);
  }
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

    const uint64_t dump_mask = fault_dump_pending_.exchange(0, std::memory_order_acquire);
    if (dump_mask)
    {
      for (size_t i = 0; i < drives_.size(); ++i)
      {
        if (dump_mask & (uint64_t{1} << i))
        {
          dump_fault_snapshot(*drives_[i]);
        }
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

// Reads the drive's fault_snapshot objects by SDO and logs them in one line.
// latched_before_activation: the fault was already set when activation
// started (typically error 50 left by a previous session that was killed
// before its clean shutdown) and is about to be reset, so it is logged as a
// warning that cannot be mistaken for a runtime fault.
void IrdEthercatSystem::dump_fault_snapshot(const Drive & drive, bool latched_before_activation)
{
  const int slave = drive.cfg->slave_position;
  const uint16_t al_status = master_->al_status_code(slave);

  char details[1024];
  size_t used = std::snprintf(
      details, sizeof(details), "SW=0x%04X  AL=0x%04X (%s)", drive.statusword.load(), al_status,
      describe_al_status_code(al_status));
  for (const SdoRead & object : drive.cfg->fault_snapshot)
  {
    if (used >= sizeof(details)) break;
    int64_t value = 0;
    const int width = static_cast<int>(size_of(object.type) * 2);
    if (master_->sdo_read(slave, object.index, object.subindex, object.type, value))
    {
      const unsigned long long raw = static_cast<unsigned long long>(value) & ((1ULL << (width * 4)) - 1);
      used += std::snprintf(
          details + used, sizeof(details) - used, "  %s(0x%04X:%02X)=0x%0*llX", object.name.c_str(), object.index,
          object.subindex, width, raw);
      if (object.index == 0x603F && used < sizeof(details))
      {
        used += std::snprintf(
            details + used, sizeof(details) - used, " (%s)", describe_drive_error_code(static_cast<uint16_t>(value)));
      }
    }
    else
    {
      used += std::snprintf(
          details + used, sizeof(details) - used, "  %s(0x%04X:%02X)=FAIL", object.name.c_str(), object.index,
          object.subindex);
    }
  }

  if (latched_before_activation)
  {
    RCLCPP_WARN(
        logger_,
        "Drive '%s' (slave %d) fault latched before activation (left by a previous session that did not shut "
        "down cleanly; not a runtime fault, being cleared now): %s",
        drive.cfg->name.c_str(), slave, details);
  }
  else
  {
    RCLCPP_ERROR(logger_, "Drive '%s' (slave %d) fault snapshot: %s", drive.cfg->name.c_str(), slave, details);
  }
}

void IrdEthercatSystem::log_esc_error_counters(bool force)
{
  // ESC registers 0x0300..0x0313: per-port invalid-frame / RX-error
  // counters, forwarded-RX-error counters, processing-unit and PDI error
  // counters, lost-link counters. They count since drive power-up and are
  // the one place a corrupted frame on the wire (EMI, cabling) shows up
  // when the master-side NIC counters stay clean.
  for (size_t i = 0; i < drives_.size(); ++i)
  {
    const Drive & drive = *drives_[i];
    const int slave = drive.cfg->slave_position;
    std::array<uint8_t, 20> regs{};
    if (!master_->read_esc_error_counters(slave, regs))
    {
      if (force)
      {
        RCLCPP_WARN(logger_, "Drive '%s' (slave %d): ESC error counter read failed", drive.cfg->name.c_str(), slave);
      }
      continue;
    }
    if (!force && regs == esc_counters_last_[i])
    {
      continue;
    }
    esc_counters_last_[i] = regs;
    RCLCPP_WARN(
        logger_,
        "Drive '%s' (slave %d) ESC error counters: port0 invalid-frame=%u rx-err=%u fwd-rx-err=%u lost-link=%u | "
        "port1 invalid-frame=%u rx-err=%u fwd-rx-err=%u lost-link=%u | PU-err=%u PDI-err=%u",
        drive.cfg->name.c_str(), slave,
        regs[0], regs[1], regs[8], regs[16],
        regs[2], regs[3], regs[9], regs[17],
        regs[12], regs[13]);
  }
}

void IrdEthercatSystem::log_timing_diagnostics()
{
  if (!timing_dump_ready_.load(std::memory_order_acquire)) return;

  // The IRD needs each frame at the drives >= 100 us before SYNC0, i.e. a
  // DC phase below (sync0_shift_us - 100) us. The phase is reported in
  // [-cycle/2, cycle/2), so with SYNC0 mid-cycle a frame later than SYNC0
  // wraps to just above -cycle/2: flag both ends.
  const int32_t sync0_us = static_cast<int32_t>(config_.bus.sync0_shift_us);
  const int32_t phase_limit_us = sync0_us - IRD_MIN_LEAD_US;
  const int expected_wkc = master_->expected_wkc();

  // Print only cycles that could have faulted the drives, plus the last
  // few before detection for context.
  constexpr size_t TAIL = 8;
  RCLCPP_WARN(
      logger_,
      "EtherCAT timing before the fault (last %zu cycles, oldest first; expected WKC %d, SYNC0 at DC phase %d us):",
      TIMING_HISTORY_CYCLES, expected_wkc, sync0_us);
  for (size_t i = 0; i < TIMING_HISTORY_CYCLES; ++i)
  {
    const CycleTiming & c = timing_dump_[i];
    const bool anomalous = c.wkc < expected_wkc || c.wake_late_us > TIMING_LATE_WAKE_US ||
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
