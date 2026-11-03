// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

// Thin wrapper around the SOEM EtherCAT master: opening the network
// interface, configuring the drives by SDO, mapping the process data,
// distributed clocks, EtherCAT state changes and the cyclic frame exchange.
//
// Thread safety: exchange() and the state-change requests take an internal
// mutex, so the lifecycle thread can change states while the process-data
// thread is running. SDO and register reads from the diagnostics thread run
// without it (SOEM serialises access to the socket itself), so they never
// delay a process-data frame.

#ifndef IRD_ETHERCAT_HARDWARE_INTERFACE__ETHERCAT_MASTER_HPP_
#define IRD_ETHERCAT_HARDWARE_INTERFACE__ETHERCAT_MASTER_HPP_

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/logger.hpp"
#include "ird_ethercat_hardware_interface/config.hpp"

extern "C"
{
#include "soem/soem.h"
}

namespace ird_ethercat_hardware_interface
{

class EthercatMaster
{
public:
  explicit EthercatMaster(const rclcpp::Logger & logger) : logger_(logger) {}
  ~EthercatMaster() { close(); }
  EthercatMaster(const EthercatMaster &) = delete;
  EthercatMaster & operator=(const EthercatMaster &) = delete;

  // ---- Bus ----
  /// Opens the interface and scans the bus; slaves end up in PRE-OP.
  bool open(const std::string & interface);
  void close();
  bool is_open() const { return open_; }
  int slave_count() const { return ctx_.slavecount; }
  /// Logs name, vendor id and product code of every slave found.
  void log_slaves() const;
  /// Checks the expected vendor id / product code of a drive (if configured).
  bool check_identity(const DriveConfig & drive) const;

  // ---- Configuration in PRE-OP ----
  /// Keep SOEM from requesting SAFE-OP by itself at the end of map_process_data().
  void set_manual_state_change(bool manual) { ctx_.manualstatechange = manual ? 1 : 0; }
  /// Mode of operation, startup SDOs, PDO mapping and sync objects.
  bool configure_drive(const DriveConfig & drive, const BusConfig & bus);
  /// Maps the process image of all slaves.
  bool map_process_data();
  /// Sets up the distributed clocks of all slaves (offsets, propagation
  /// delays, drift compensation). SOEM ignores failed reads and writes here,
  /// so check the result (dc_time_minus_host) and repeat if needed.
  void setup_dc_clocks();
  /// Starts SYNC0 on the given slaves.
  void start_sync0(const std::vector<int> & slaves, int64_t cycle_ns, int64_t shift_ns);

  // ---- Process data ----
  uint8_t * outputs(int slave) { return ctx_.slavelist[slave].outputs; }
  const uint8_t * inputs(int slave) const { return ctx_.slavelist[slave].inputs; }
  size_t output_bytes(int slave) const { return ctx_.slavelist[slave].Obytes; }
  size_t input_bytes(int slave) const { return ctx_.slavelist[slave].Ibytes; }
  /// Sends the outputs and receives the inputs of all slaves in one frame.
  /// Returns the working counter. If `sent_at` is given, it receives the
  /// time the frame was handed to the kernel.
  int exchange(ec_timet * sent_at = nullptr);
  int expected_wkc() const { return expected_wkc_; }

  // ---- Distributed clocks ----
  bool has_dc() const { return ctx_.slavelist[0].hasdc; }
  bool slave_has_dc(int slave) const { return ctx_.slavelist[slave].hasdc; }
  /// DC time of the last received frame [ns].
  int64_t dc_time() const { return ctx_.DCtime; }

  // ---- EtherCAT states ----
  /// Requests `state` for all slaves.
  void request_state(uint16_t state);
  /// Waits until every slave in `slaves` reports `state`. Safe to call while
  /// the process-data thread runs: the bus lock is only held for each poll.
  /// With repeat_request_us > 0, `state` is requested again from every slave
  /// not there yet at that interval (a drive can miss the broadcast request).
  bool wait_for_state(
      const std::vector<int> & slaves, uint16_t state, int64_t timeout_us, int64_t repeat_request_us = 0);
  /// Reads the AL state and AL status code of all slaves (state() and
  /// al_status_code() otherwise only reflect the last state check).
  void read_states();
  uint16_t state(int slave) const { return ctx_.slavelist[slave].state; }
  uint16_t al_status_code(int slave) const { return ctx_.slavelist[slave].ALstatuscode; }

  // ---- Mailbox (SDO) ----
  bool sdo_write(int slave, const SdoWrite & sdo);  ///< with retries; logs failures
  bool sdo_read(int slave, uint16_t index, uint8_t subindex, DataType type, int64_t & value);

  // ---- Registers ----
  /// ESC error counters (registers 0x0300..0x0313).
  bool read_esc_error_counters(int slave, std::array<uint8_t, 20> & counters);
  /// Logs SOEM's and the ESC's view of a slave's DC configuration (debug level).
  void log_dc_snapshot(int slave);
  /// The slave's DC system time (register 0x0910) minus the host clock read
  /// just before, in the EtherCAT epoch the clock setup used [ns]. Equal for
  /// all slaves, within a frame round trip, when the setup worked.
  bool dc_time_minus_host(int slave, int64_t & delta_ns);
  /// System time offset written by the clock setup (register 0x0920) [ns].
  bool dc_system_time_offset(int slave, int64_t & offset_ns);
  /// System time difference to the reference clock (register 0x092C) [ns].
  bool dc_sync_difference(int slave, int64_t & difference_ns);
  int propagation_delay_ns(int slave) const { return ctx_.slavelist[slave].pdelay; }
  int entry_port(int slave) const { return ctx_.slavelist[slave].entryport; }

private:
  void log_mailbox_failure(int slave, const char * what);

  rclcpp::Logger logger_;
  ecx_contextt ctx_{};
  std::mutex bus_mutex_;
  char io_map_[4096]{};
  bool open_{false};
  int expected_wkc_{0};
};

}  // namespace ird_ethercat_hardware_interface

#endif  // IRD_ETHERCAT_HARDWARE_INTERFACE__ETHERCAT_MASTER_HPP_
