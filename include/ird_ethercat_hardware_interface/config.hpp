// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

// Configuration of the hardware interface: the EtherCAT bus, the fault
// handling and each drive (PDO mapping, startup SDOs, units and limits).
//
// Every default below is the value validated on the Naviwall robot (three
// IRD-eb-48/60 drives, CSV, HardSync, 4 ms cycle). This header has no ROS or
// SOEM dependency so that it can be unit-tested on its own.

#ifndef IRD_ETHERCAT_HARDWARE_INTERFACE__CONFIG_HPP_
#define IRD_ETHERCAT_HARDWARE_INTERFACE__CONFIG_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ird_ethercat_hardware_interface
{

/// Data types allowed for SDO values and PDO entries (all byte-aligned).
enum class DataType { INT8, UINT8, INT16, UINT16, INT32, UINT32 };

size_t size_of(DataType type);         ///< size in bytes
bool is_signed(DataType type);
const char * to_string(DataType type);  ///< "int8", "uint16", ...
bool parse_data_type(const std::string & text, DataType & out);

/// What the driver uses a PDO entry for. Entries without a role are only
/// transferred (they will be exposed as extra ros2_control interfaces).
enum class PdoRole
{
  NONE,
  // RxPDO (master -> drive)
  CONTROLWORD,        ///< 0x6040, uint16
  MODE_OF_OPERATION,  ///< 0x6060, int8 (optional: written every cycle if mapped)
  TARGET_VELOCITY,    ///< 0x60FF, int32
  // TxPDO (drive -> master)
  STATUSWORD,         ///< 0x6041, uint16
  MODE_DISPLAY,       ///< 0x6061, int8 (optional, informative)
  VELOCITY_ACTUAL,    ///< 0x606C, int32
};

const char * to_string(PdoRole role);
bool parse_pdo_role(const std::string & text, PdoRole & out);

/// One object mapped into a PDO.
struct PdoEntry
{
  uint16_t index{0};
  uint8_t subindex{0};
  DataType type{DataType::UINT8};
  PdoRole role{PdoRole::NONE};
};

/// One object written by SDO while the drive is in PRE-OP.
struct SdoWrite
{
  uint16_t index{0};
  uint8_t subindex{0};
  DataType type{DataType::UINT8};
  int64_t value{0};
  /// If false, a failed write is logged as a warning and start-up continues.
  bool required{true};
};

/// One object read by SDO for the fault snapshot logged when a drive faults.
struct SdoRead
{
  uint16_t index{0};
  uint8_t subindex{0};
  DataType type{DataType::UINT8};
  std::string name;
};

/// CiA 402 modes of operation (object 0x6060).
enum class OperationMode : int8_t
{
  CYCLIC_SYNC_POSITION = 8,  ///< CSP
  CYCLIC_SYNC_VELOCITY = 9,  ///< CSV
  CYCLIC_SYNC_TORQUE = 10,   ///< CST
};

/// IRD synchronisation mode (object 0x2012:01), used with distributed clocks.
enum class SyncMode
{
  HARD_SYNC,  ///< 0x2012:01 = 4: drive locked to SYNC0, strict (needs a low-jitter master)
  SOFT_SYNC,  ///< 0x2012:01 = 2: drive follows SYNC0/SM2 events loosely, tolerates more jitter
};

struct BusConfig
{
  std::string interface{"eno1"};    ///< network interface connected to the EtherCAT bus
  uint32_t cycle_time_us{4000};      ///< process-data cycle (written to 0x1006:00)
  int rt_priority{80};               ///< SCHED_FIFO priority of the process-data thread
  bool distributed_clocks{true};     ///< synchronise the drives with DC SYNC0
  SyncMode sync_mode{SyncMode::HARD_SYNC};
  /// SYNC0 fires this long after the cycle boundary at which the master's
  /// frame passes the drives: the frame's headroom before the drives latch it.
  uint32_t sync0_shift_us{2000};
  /// Before OPERATIONAL, wait until the frame has stayed within
  /// dc_lock_tolerance_us of its DC target for dc_lock_ms (or dc_lock_timeout_ms).
  uint32_t dc_lock_tolerance_us{200};
  uint32_t dc_lock_ms{200};
  uint32_t dc_lock_timeout_ms{3000};
  /// Frames exchanged in SAFE-OP (outputs zero) to settle the DC loop at start-up.
  uint32_t startup_settle_ms{1000};
};

struct FaultHandlingConfig
{
  uint32_t enable_timeout_ms{5000};        ///< activation fails if not all drives are enabled by then
  uint32_t enable_stable_ms{40};           ///< all drives Operation Enabled this long before accepting commands
  uint32_t enable_fault_abort_ms{100};     ///< during activation, a drive in FAULT this long aborts it
  uint32_t max_recovery_attempts{5};       ///< runtime fault recoveries per incident (0 = never recover)
  uint32_t recovery_cooldown_ms{500};      ///< wait after a fault before sending the reset pulse
  uint32_t reset_pulse_ms{200};            ///< max duration of the fault-reset controlword (0x0080)
  uint32_t recovery_budget_refill_ms{30000};  ///< fault-free time in operation that refills the attempts
};

struct DriveConfig
{
  std::string name;          ///< used in log messages
  std::string joint;         ///< ros2_control joint driven by this drive
  int slave_position{0};     ///< EtherCAT position: 1 = first slave after the master
  uint32_t expected_vendor_id{0};     ///< checked at start-up if non-zero
  uint32_t expected_product_code{0};  ///< checked at start-up if non-zero

  OperationMode mode{OperationMode::CYCLIC_SYNC_VELOCITY};

  // Units: the drive's position unit is one encoder tick and its velocity
  // unit one tick per second, at the motor shaft. The joint is after a
  // gearbox of gear_ratio motor revolutions per joint revolution.
  double encoder_ticks_per_rev{0.0};
  double gear_ratio{1.0};

  double max_motor_rpm{4910.0};  ///< commands above this motor speed are clamped
  /// After (re-)enabling, ramp the motor-shaft reference up at this rate
  /// until it matches the command (0 = no ramp). Slowing down is immediate.
  double enable_ramp_motor_rad_s2{70.0};

  uint16_t rx_pdo_index{0x1600};  ///< RxPDO mapping object (master -> drive)
  uint16_t tx_pdo_index{0x1A00};  ///< TxPDO mapping object (drive -> master)
  std::vector<PdoEntry> rx_pdo;
  std::vector<PdoEntry> tx_pdo;

  std::vector<SdoWrite> startup_sdos;    ///< extra PRE-OP writes, before the PDO mapping
  std::vector<SdoRead> fault_snapshot;   ///< objects read and logged when the drive faults
};

struct SystemConfig
{
  BusConfig bus;
  FaultHandlingConfig faults;
  std::vector<DriveConfig> drives;
};

/// A drive with the IRD CSV defaults: the PDO mapping and fault snapshot
/// validated on the Naviwall robot. Name, joint, position and units are empty.
DriveConfig make_ird_csv_drive();

/// Checks the whole configuration. Returns one message per problem found
/// (empty if the configuration is usable).
std::vector<std::string> validate(const SystemConfig & config);

/// Converts a duration to a number of cycles (at least 1).
uint32_t ms_to_cycles(uint32_t ms, uint32_t cycle_time_us);

/// Largest number of drives supported (fault masks are 64-bit).
constexpr size_t MAX_DRIVES = 64;

}  // namespace ird_ethercat_hardware_interface

#endif  // IRD_ETHERCAT_HARDWARE_INTERFACE__CONFIG_HPP_
