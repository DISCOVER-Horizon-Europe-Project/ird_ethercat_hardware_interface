// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only
//
// Portions (lifecycle skeleton, joint interface checks) derived from the
// ros2_control demos, Copyright 2021 ros2_control Development Team,
// licensed under the Apache License, Version 2.0.

#ifndef IRD_ETHERCAT_HARDWARE_INTERFACE__IRD_ETHERCAT_SYSTEM_HPP_
#define IRD_ETHERCAT_HARDWARE_INTERFACE__IRD_ETHERCAT_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ird_ethercat_hardware_interface/config.hpp"
#include "ird_ethercat_hardware_interface/cyclic_controller.hpp"
#include "ird_ethercat_hardware_interface/dc_sync.hpp"
#include "ird_ethercat_hardware_interface/ethercat_master.hpp"
#include "ird_ethercat_hardware_interface/pdo.hpp"
#include "ird_ethercat_hardware_interface/visibility_control.h"

namespace ird_ethercat_hardware_interface
{

/// ros2_control system interface for Infranor IRD servo drives on EtherCAT.
///
/// Threads involved at runtime:
///  - ros2_control update loop: calls read()/write(). Never touches the bus;
///    it only exchanges values with the process-data thread through atomics.
///  - Process-data thread (SCHED_FIFO): one EtherCAT frame every cycle,
///    distributed-clock synchronisation, CiA 402 enable sequence, fault
///    detection and recovery (CyclicController).
///  - Diagnostics thread (normal priority): slow mailbox (SDO) and register
///    reads plus the log output requested by the process-data thread, so
///    neither of the other two loops is ever stalled by them.
class IrdEthercatSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IrdEthercatSystem)

  IrdEthercatSystem();
  ~IrdEthercatSystem() override;

  IRD_ETHERCAT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  IRD_ETHERCAT_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  IRD_ETHERCAT_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  IRD_ETHERCAT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  IRD_ETHERCAT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  IRD_ETHERCAT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  IRD_ETHERCAT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  IRD_ETHERCAT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  IRD_ETHERCAT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  IRD_ETHERCAT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// Runtime state of one drive.
  struct Drive
  {
    const DriveConfig * cfg{nullptr};  ///< points into config_.drives

    // Process image
    PdoLayout rx_layout;
    PdoLayout tx_layout;
    RoleLocation pdo_controlword;
    RoleLocation pdo_mode_of_operation;
    RoleLocation pdo_target_velocity;
    RoleLocation pdo_statusword;
    RoleLocation pdo_velocity_actual;
    uint8_t * outputs{nullptr};       ///< this drive's RxPDO in the process image
    const uint8_t * inputs{nullptr};  ///< this drive's TxPDO in the process image

    // Unit conversion (drive units: encoder ticks/s at the motor shaft)
    double ticks_per_rad{0.0};    ///< motor-shaft ticks per radian
    double max_motor_rad_s{0.0};  ///< command clamp at the motor shaft

    // ros2_control values (joint side, after the gearbox)
    double cmd{0.0};  ///< commanded velocity [rad/s]
    double pos{0.0};  ///< position [rad], integrated from the measured velocity
    double vel{0.0};  ///< measured velocity [rad/s]

    // Shared between write()/read() and the process-data thread
    std::atomic<int32_t> target{0};           ///< requested reference [ticks/s]
    std::atomic<int32_t> actual_velocity{0};  ///< measured velocity [ticks/s]
    std::atomic<uint16_t> statusword{0};
  };

  /// One process-data cycle as recorded by the RT thread, used to tell a late
  /// master (host) from a late or lost frame (network) when a drive faults.
  struct CycleTiming
  {
    int32_t wake_late_us{0};  ///< wake-up after the absolute cycle deadline
    int32_t exchange_us{0};   ///< send + receive of the process-data frame
    int32_t dc_phase_us{0};   ///< frame's DC time past the cycle boundary (SYNC0 is sync0_shift_us later)
    int32_t toff_us{0};       ///< DC PI correction applied to the next period
    int32_t toff_i_us{0};     ///< integral part of toff_us
    int16_t wkc{0};           ///< working counter of the frame
  };
  static constexpr size_t TIMING_HISTORY_CYCLES = 128;

  // --- Configuration ---
  bool load_config_from_urdf_params();
  bool check_joints();

  // --- Bus start-up and shutdown ---
  bool init_ethercat();
  bool clear_latched_faults();
  bool wait_for_dc_lock();
  bool enable_drives();
  void disable_drives_and_leave_op();
  void shutdown_ethercat();
  std::vector<int> slave_positions() const;

  // --- Process data ---
  void stage_outputs(uint16_t controlword_all);
  void run_startup_cycles(uint32_t cycles);
  void start_processdata_thread();
  void stop_processdata_thread();
  void processdata_loop();
  void log_cycle_events(const CycleEvents & events);

  // --- Diagnostics ---
  void start_diagnostics_thread();
  void stop_diagnostics_thread();
  void diagnostics_loop();
  void dump_fault_snapshot(const Drive & drive, bool latched_before_activation = false);
  void log_esc_error_counters(bool force);
  void log_timing_diagnostics();

  SystemConfig config_;
  std::vector<std::unique_ptr<Drive>> drives_;

  rclcpp::Logger logger_{rclcpp::get_logger("IrdEthercatSystem")};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};  // for throttled log messages

  std::unique_ptr<EthercatMaster> master_;
  bool ec_initialized_{false};
  std::optional<DcSync> dc_sync_;              ///< shared by the start-up cycles and the RT loop
  std::optional<CyclicController> controller_;
  SharedFlags flags_;
  bool first_write_warned_{false};
  uint32_t low_wkc_reads_{0};  ///< read() calls that saw a short working counter

  // --- Process-data thread ---
  std::thread processdata_thread_;
  std::atomic<bool> processdata_thread_stop_{false};
  std::atomic<int> last_processdata_wkc_{0};
  // DC phase of the last received frame and how many consecutive cycles it
  // has stayed within the lock tolerance; on_activate() waits on these
  // before requesting OPERATIONAL.
  std::atomic<int32_t> dc_phase_us_{0};
  std::atomic<uint32_t> dc_locked_cycles_{0};
  // Buffers used by the RT loop, allocated once in on_init()
  std::vector<int32_t> rt_targets_;
  std::vector<int32_t> rt_references_;
  std::vector<uint16_t> rt_statuswords_;
  std::vector<uint16_t> rt_controlwords_;

  // --- Diagnostics thread ---
  // Fault snapshots are read by SDO from the diagnostics thread, never from
  // the RT loop (the mailbox reads take tens of ms and would stall the cyclic
  // RPDO, cascading a single-drive fault into a bus-wide fieldbus SYNC error
  // on the remaining drives) nor from read() (they would stall the whole
  // ros2_control update, and with it other hardware such as a robot arm).
  std::atomic<uint64_t> fault_dump_pending_{0};  ///< bit i = drive i
  std::thread diagnostics_thread_;
  std::atomic<bool> diagnostics_thread_stop_{false};
  /// ESC error counters as last logged, per drive, to print only on change.
  std::vector<std::array<uint8_t, 20>> esc_counters_last_;

  // Cycle timing ring. The RT loop records every cycle and, on a fault, hands
  // a copy to the diagnostics thread through timing_dump_ready_; only that
  // thread logs it. Nothing is logged while the bus is healthy.
  std::array<CycleTiming, TIMING_HISTORY_CYCLES> timing_history_{};  ///< RT thread only
  size_t timing_history_next_{0};
  std::array<CycleTiming, TIMING_HISTORY_CYCLES> timing_dump_{};     ///< oldest first
  std::atomic<bool> timing_dump_ready_{false};
};

}  // namespace ird_ethercat_hardware_interface

#endif  // IRD_ETHERCAT_HARDWARE_INTERFACE__IRD_ETHERCAT_SYSTEM_HPP_
