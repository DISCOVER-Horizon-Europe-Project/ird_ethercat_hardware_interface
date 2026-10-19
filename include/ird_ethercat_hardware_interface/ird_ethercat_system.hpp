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
#include <mutex>
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
#include "ird_ethercat_hardware_interface/visibility_control.h"

// SOEM EtherCAT master library (C API)
extern "C"
{
#include "soem/soem.h"
}

namespace ird_ethercat_hardware_interface
{

/// ros2_control system interface for Infranor IRD servo drives on EtherCAT.
///
/// Threads involved at runtime:
///  - ros2_control update loop: calls read()/write(). Never touches the bus;
///    it only exchanges values with the process-data thread through atomics.
///  - Process-data thread (SCHED_FIFO): one EtherCAT frame every cycle,
///    distributed-clock synchronisation, CiA 402 state machine, fault
///    detection and bounded automatic fault recovery.
///  - Diagnostics thread (normal priority): slow mailbox (SDO) reads and log
///    output requested by the process-data thread, so neither of the other
///    two loops is ever stalled by them.
///
/// This first version still drives exactly three IRD drives (left wheel,
/// right wheel, turret), configured through URDF <param> tags.
class IrdEthercatSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IrdEthercatSystem)

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
  /// Values read from the URDF <hardware> <param> tags in on_init().
  struct Config
  {
    std::string left_wheel_name;
    std::string right_wheel_name;
    std::string turret_name;
    double transmission_ratio_wheels = 0.0;  // motor revolutions per joint revolution
    double transmission_ratio_turret = 0.0;
    int encoder_ticks_per_rev = 0;           // drive velocity unit: ticks/s at the motor shaft
    int slave_left_motor = 0;                // EtherCAT slave positions (1 = first on the bus)
    int slave_right_motor = 0;
    int slave_turret_motor = 0;
  };

  /// ros2_control joint values (joint side, i.e. after the gearbox).
  struct Joint
  {
    std::string name;
    double cmd = 0.0;  // commanded velocity [rad/s]
    double pos = 0.0;  // position [rad], integrated from the measured velocity
    double vel = 0.0;  // measured velocity [rad/s]
  };

  /// One process-data cycle as recorded by the RT thread, used to tell a late
  /// master (host) from a late or lost frame (network) when a drive faults.
  struct CycleTiming
  {
    int32_t wake_late_us{0};  // wake-up after the absolute cycle deadline
    int32_t exchange_us{0};   // send + receive of the process-data frame
    int32_t dc_phase_us{0};   // frame's DC time past the cycle boundary; SYNC0 fires DC_SYNC_SHIFT_NS after it
    int32_t toff_us{0};       // DC PI correction applied to the next period
    int32_t toff_i_us{0};     // integral part of toff_us
    int16_t wkc{0};           // working counter of the frame
  };
  static constexpr size_t TIMING_HISTORY_CYCLES = 128;  // 512 ms at 250 Hz

  // --- EtherCAT bring-up / tear-down (src: "Bus start-up and shutdown") ---
  bool init_ethercat();
  bool configure_slave(int slave);
  bool wait_for_state(uint16_t state, int64_t timeout_us);
  void disable_drives_and_leave_op();
  void shutdown_ethercat();
  bool enable_slaves_via_pdo();

  // --- Process data (src: "Process data") ---
  int exchange_processdata();
  void copy_tx_processdata();
  void stage_zero_rpdo(uint16_t controlword);
  void run_startup_processdata_cycles(size_t cycles, uint16_t controlword);
  void start_processdata_thread();
  void stop_processdata_thread();
  void processdata_loop();
  void sync_linux_to_dc(int64_t reftime, int64_t cycletime, int64_t * offsettime, int64_t * integral_term = nullptr);

  // --- Diagnostics (src: "Diagnostics thread") ---
  void start_diagnostics_thread();
  void stop_diagnostics_thread();
  void diagnostics_loop();
  void log_esc_error_counters(bool force);
  void log_timing_diagnostics();

  // --- SDO access with retries ---
  template<typename T>
  bool write_sdo(uint16_t slave, uint16_t index, uint8_t subindex, T value);

  // --- Unit conversions (motor shaft) ---
  int32_t rad_s_to_ticks_s(double rad_s) const;
  double ticks_s_to_rad_s(int32_t ticks_s) const;

  Config cfg_;
  Joint wheel_l_;
  Joint wheel_r_;
  Joint turret_;

  rclcpp::Logger logger_{rclcpp::get_logger("IrdEthercatSystem")};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};  // for throttled log messages

  // --- SOEM state ---
  ecx_contextt ec_context_{};       // SOEM master context
  std::mutex ethercat_bus_mutex_;   // serialises process-data frames and state changes
  std::string ethercat_interface_{"eno1"};
  char io_map_[4096]{};             // process image, mapped by SOEM
  int expected_wkc_{0};
  bool ec_initialized_{false};

  // Local copies of each drive's process data (RxPDO = outputs to the drive,
  // TxPDO = inputs from the drive).
  std::vector<uint8_t> rx_buffer_left_;
  std::vector<uint8_t> tx_buffer_left_;
  std::vector<uint8_t> rx_buffer_right_;
  std::vector<uint8_t> tx_buffer_right_;
  std::vector<uint8_t> rx_buffer_turret_;
  std::vector<uint8_t> tx_buffer_turret_;

  // Each drive's slice of io_map_, as mapped by SOEM.
  uint8_t * rx_left_{nullptr};
  uint8_t * tx_left_{nullptr};
  uint8_t * rx_right_{nullptr};
  uint8_t * tx_right_{nullptr};
  uint8_t * rx_turret_{nullptr};
  uint8_t * tx_turret_{nullptr};

  // --- Shared between write()/read(), lifecycle callbacks and the RT thread ---
  std::atomic<bool> force_zero_velocity_{false};
  std::atomic<bool> enable_request_{false};       // lifecycle -> RT: run the CiA 402 enable sequence
  std::atomic<bool> operation_enabled_{false};    // RT -> lifecycle: all drives Operation Enabled
  std::atomic<bool> enable_fault_abort_{false};   // RT -> lifecycle: enable sequence gave up
  std::atomic<int> last_processdata_wkc_{0};
  std::atomic<int32_t> target_tps_left_{0};       // motor-shaft velocity targets [ticks/s]
  std::atomic<int32_t> target_tps_right_{0};
  std::atomic<int32_t> target_tps_turret_{0};
  std::atomic<int32_t> actual_tps_left_{0};       // motor-shaft measured velocity [ticks/s]
  std::atomic<int32_t> actual_tps_right_{0};
  std::atomic<int32_t> actual_tps_turret_{0};
  std::atomic<uint16_t> statusword_left_{0};
  std::atomic<uint16_t> statusword_right_{0};
  std::atomic<uint16_t> statusword_turret_{0};
  bool first_write_warned_{false};
  uint32_t low_wkc_reads_{0};                     // read() calls that saw a short working counter

  // --- Process-data thread ---
  std::thread processdata_thread_;
  std::atomic<bool> processdata_thread_stop_{false};
  std::atomic<bool> processdata_thread_running_{false};
  // DC phase of the last received frame and how many consecutive cycles it
  // has stayed within DC_LOCK_TOLERANCE_US; on_activate() waits on these
  // before requesting OPERATIONAL.
  std::atomic<int32_t> dc_phase_us_{0};
  std::atomic<uint32_t> dc_locked_cycles_{0};
  // Integral term of the DC PI loop. Shared by the start-up cycles and the RT
  // loop (so the start-up pull-in carries over), reset in init_ethercat().
  int64_t dc_integral_{0};

  // --- Diagnostics thread ---
  // Fault diagnostics are dumped from the diagnostics thread, never from
  // the RT loop (the SDO mailbox reads take tens of ms and would stall the
  // cyclic RPDO, cascading a single-drive fault into a bus-wide fieldbus
  // SYNC error on the remaining drives) and not from read() either (the
  // three snapshots block ~55 ms, which stalls the whole ros2_control update
  // and, in a full robot, other hardware such as a robot arm).
  // Bit mask: bit0 = left, bit1 = right, bit2 = turret.
  std::atomic<uint8_t> fault_dump_pending_{0};
  std::thread diagnostics_thread_;
  std::atomic<bool> diagnostics_thread_stop_{false};
  // ESC error counters (registers 0x0300..0x0313) as last logged, indexed by
  // slave position, to print only on change. Sized in init_ethercat().
  std::vector<std::array<uint8_t, 20>> esc_counters_last_;

  // Cycle timing ring. The RT loop records every cycle and, on a fault, hands
  // a copy to the diagnostics thread through timing_dump_ready_; only that
  // thread logs it. Nothing is logged while the bus is healthy.
  std::array<CycleTiming, TIMING_HISTORY_CYCLES> timing_history_{};  // RT thread only
  size_t timing_history_next_{0};
  std::array<CycleTiming, TIMING_HISTORY_CYCLES> timing_dump_{};     // oldest first
  std::atomic<bool> timing_dump_ready_{false};
};

}  // namespace ird_ethercat_hardware_interface

#endif  // IRD_ETHERCAT_HARDWARE_INTERFACE__IRD_ETHERCAT_SYSTEM_HPP_
