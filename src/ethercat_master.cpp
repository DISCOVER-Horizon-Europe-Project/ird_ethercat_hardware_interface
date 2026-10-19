// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

#include "ird_ethercat_hardware_interface/ethercat_master.hpp"

#include <chrono>

#include "rclcpp/logging.hpp"
#include "ird_ethercat_hardware_interface/pdo.hpp"
#include "ird_ethercat_hardware_interface/startup_plan.hpp"

namespace ird_ethercat_hardware_interface
{

namespace
{
constexpr uint16_t OBJ_MODE_OF_OPERATION = 0x6060;
constexpr int SDO_WRITE_ATTEMPTS = 3;
constexpr int SDO_RETRY_DELAY_US = 10'000;
constexpr int STATE_POLL_US = 1'000;
}  // namespace

// =========================================================================
// Bus
// =========================================================================

bool EthercatMaster::open(const std::string & interface)
{
  RCLCPP_INFO(logger_, "Initializing SOEM on interface %s...", interface.c_str());
  if (ecx_init(&ctx_, interface.c_str()) <= 0)
  {
    RCLCPP_ERROR(
        logger_, "Failed to initialize SOEM on %s (does the interface exist, and does the process have "
        "CAP_NET_RAW? see README).", interface.c_str());
    return false;
  }
  open_ = true;

  RCLCPP_INFO(logger_, "SOEM initialized. Scanning EtherCAT network...");
  if (ecx_config_init(&ctx_) <= 0)
  {
    RCLCPP_ERROR(logger_, "No EtherCAT slaves found on %s.", interface.c_str());
    close();
    return false;
  }
  RCLCPP_INFO(logger_, "Found %d slave(s).", ctx_.slavecount);
  return true;
}

void EthercatMaster::close()
{
  if (open_)
  {
    ecx_close(&ctx_);
    open_ = false;
  }
}

void EthercatMaster::log_slaves() const
{
  for (int slave = 1; slave <= ctx_.slavecount; ++slave)
  {
    const auto & sl = ctx_.slavelist[slave];
    RCLCPP_INFO(
        logger_, "Slave %d: '%s' vendor_id=0x%08X product_code=0x%08X revision=0x%08X", slave, sl.name,
        sl.eep_man, sl.eep_id, sl.eep_rev);
  }
}

bool EthercatMaster::check_identity(const DriveConfig & drive) const
{
  const auto & sl = ctx_.slavelist[drive.slave_position];
  bool ok = true;
  if (drive.expected_vendor_id != 0 && sl.eep_man != drive.expected_vendor_id)
  {
    RCLCPP_ERROR(
        logger_, "Drive '%s' (slave %d): vendor_id is 0x%08X, configuration expects 0x%08X.",
        drive.name.c_str(), drive.slave_position, sl.eep_man, drive.expected_vendor_id);
    ok = false;
  }
  if (drive.expected_product_code != 0 && sl.eep_id != drive.expected_product_code)
  {
    RCLCPP_ERROR(
        logger_, "Drive '%s' (slave %d): product_code is 0x%08X, configuration expects 0x%08X.",
        drive.name.c_str(), drive.slave_position, sl.eep_id, drive.expected_product_code);
    ok = false;
  }
  return ok;
}

// =========================================================================
// Configuration in PRE-OP
// =========================================================================

bool EthercatMaster::configure_drive(const DriveConfig & drive, const BusConfig & bus)
{
  const int slave = drive.slave_position;
  const char * name = drive.name.c_str();
  RCLCPP_INFO(logger_, "Configuring drive '%s' (slave %d)...", name, slave);

  ecx_statecheck(&ctx_, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
  if (ctx_.slavelist[slave].state != EC_STATE_PRE_OP)
  {
    RCLCPP_ERROR(logger_, "Slave %d failed to reach PRE-OP. State=0x%02X", slave, ctx_.slavelist[slave].state);
    return false;
  }

  // ---- Mode of operation ----
  // Written only if needed: some restart cases keep the drive in the right
  // mode already even when a fresh write temporarily fails.
  const int64_t mode = static_cast<int64_t>(drive.mode);
  int64_t current_mode = 0;
  const bool had_mode_read = sdo_read(slave, OBJ_MODE_OF_OPERATION, 0x00, DataType::INT8, current_mode);
  if (!had_mode_read)
  {
    RCLCPP_WARN(logger_, "Slave %d: Failed to read 0x6060:00 before setting the mode of operation", slave);
  }
  if (!had_mode_read || current_mode != mode)
  {
    if (!sdo_write(slave, {OBJ_MODE_OF_OPERATION, 0x00, DataType::INT8, mode}))
    {
      int64_t readback = 0;
      if (sdo_read(slave, OBJ_MODE_OF_OPERATION, 0x00, DataType::INT8, readback) && readback == mode)
      {
        RCLCPP_WARN(
            logger_, "Slave %d: mode write reported failure, but readback confirms 0x6060:00=%lld. Continuing.",
            slave, static_cast<long long>(readback));
      }
      else
      {
        RCLCPP_ERROR(logger_, "Slave %d: Failed to set the mode of operation to %lld", slave, static_cast<long long>(mode));
        return false;
      }
    }
  }

  // ---- Startup SDOs, PDO mapping, synchronisation objects ----
  for (const SdoWrite & sdo : build_startup_plan(drive, bus))
  {
    if (sdo_write(slave, sdo)) continue;
    if (sdo.required)
    {
      RCLCPP_ERROR(logger_, "Drive '%s': required SDO 0x%04X:%02X could not be written.", name, sdo.index, sdo.subindex);
      return false;
    }
    RCLCPP_WARN(
        logger_, "Slave %d: failed to write 0x%04X:%02X=%lld in PRE-OP (not required, continuing)", slave, sdo.index,
        sdo.subindex, static_cast<long long>(sdo.value));
  }

  int64_t cycle_us = 0;
  sdo_read(slave, OBJ_CYCLE_PERIOD, 0x00, DataType::UINT32, cycle_us);
  RCLCPP_INFO(
      logger_, "Slave %d: Current PDO cycle period is %lld us (%s, diagnostic only)", slave,
      static_cast<long long>(cycle_us), bus.distributed_clocks ? "DC SYNC0 enabled" : "no DC");
  if (!bus.distributed_clocks && cycle_us != bus.cycle_time_us)
  {
    RCLCPP_WARN(
        logger_, "Slave %d: 0x1006:00 reads %lld us, cycle_time_us is %u us", slave, static_cast<long long>(cycle_us),
        bus.cycle_time_us);
  }

  RCLCPP_DEBUG(logger_, "Slave %d: PDO mapping configured successfully.", slave);
  return true;
}

bool EthercatMaster::map_process_data()
{
  const int used = ecx_config_map_group(&ctx_, io_map_, 0);
  if (used > static_cast<int>(sizeof(io_map_)))
  {
    RCLCPP_ERROR(logger_, "Process image needs %d bytes, more than the %zu reserved.", used, sizeof(io_map_));
    return false;
  }
  ctx_.grouplist[0].blockLRW = 0;  // use combined LRW datagrams

  const int outputs_wkc = ctx_.grouplist[0].outputsWKC;
  const int inputs_wkc = ctx_.grouplist[0].inputsWKC;
  expected_wkc_ = ctx_.grouplist[0].blockLRW ? outputs_wkc + inputs_wkc : outputs_wkc * 2 + inputs_wkc;
  return true;
}

void EthercatMaster::configure_dc(const std::vector<int> & slaves, int64_t cycle_ns, int64_t shift_ns)
{
  ecx_configdc(&ctx_);
  for (int slave : slaves)
  {
    ecx_dcsync0(&ctx_, slave, TRUE, cycle_ns, shift_ns);
  }
}

// =========================================================================
// Process data and states
// =========================================================================

int EthercatMaster::exchange()
{
  std::lock_guard<std::mutex> guard(bus_mutex_);
  ecx_send_processdata_group(&ctx_, 0);
  return ecx_receive_processdata_group(&ctx_, 0, EC_TIMEOUTRET);
}

void EthercatMaster::request_state(uint16_t state)
{
  std::lock_guard<std::mutex> guard(bus_mutex_);
  ctx_.slavelist[0].state = state;
  ecx_writestate(&ctx_, 0);
}

bool EthercatMaster::wait_for_state(const std::vector<int> & slaves, uint16_t state, int64_t timeout_us)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(timeout_us);
  while (true)
  {
    bool all_reached = true;
    for (int slave : slaves)
    {
      {
        std::lock_guard<std::mutex> guard(bus_mutex_);
        ecx_statecheck(&ctx_, slave, state, 0);
      }
      if (ctx_.slavelist[slave].state != state)
      {
        all_reached = false;
        break;
      }
    }
    if (all_reached) return true;
    if (std::chrono::steady_clock::now() > deadline) return false;
    osal_usleep(STATE_POLL_US);
  }
}

// =========================================================================
// Mailbox (SDO)
// =========================================================================

bool EthercatMaster::sdo_write(int slave, const SdoWrite & sdo)
{
  uint8_t data[4] = {};
  write_value(data, sdo.type, sdo.value);
  const int size = static_cast<int>(size_of(sdo.type));

  // The mailbox occasionally misses a request during start-up: retry.
  for (int attempt = 1; attempt <= SDO_WRITE_ATTEMPTS; ++attempt)
  {
    if (ecx_SDOwrite(&ctx_, slave, sdo.index, sdo.subindex, FALSE, size, data, EC_TIMEOUTRXM) > 0) return true;
    if (attempt < SDO_WRITE_ATTEMPTS) osal_usleep(SDO_RETRY_DELAY_US);
  }

  RCLCPP_ERROR(
      logger_, "SDO write (%s) failed after retries: slave=%d idx=0x%04X sub=0x%02X value=%lld", to_string(sdo.type),
      slave, sdo.index, sdo.subindex, static_cast<long long>(sdo.value));
  log_mailbox_failure(slave, "sdo_write");
  return false;
}

bool EthercatMaster::sdo_read(int slave, uint16_t index, uint8_t subindex, DataType type, int64_t & value)
{
  uint8_t data[4] = {};
  int size = static_cast<int>(size_of(type));
  value = 0;
  const int wkc = ecx_SDOread(&ctx_, slave, index, subindex, FALSE, &size, data, EC_TIMEOUTRXM);
  if (wkc <= 0 || size != static_cast<int>(size_of(type))) return false;
  value = read_value(data, type);
  return true;
}

void EthercatMaster::log_mailbox_failure(int slave, const char * what)
{
  const auto & sl = ctx_.slavelist[slave];
  RCLCPP_ERROR(
      logger_, "%s slave %d context: state=0x%02X AL=0x%04X configadr=0x%04X mbx_l=%u mbx_rl=%u hasdc=%d", what, slave,
      sl.state, sl.ALstatuscode, sl.configadr, sl.mbx_l, sl.mbx_rl, sl.hasdc);

  ec_errort err{};
  while (ecx_poperror(&ctx_, &err))
  {
    RCLCPP_ERROR(
        logger_, "%s SOEM error: slave=%u type=%d idx=0x%04X sub=0x%02X abort=0x%08X", what, err.Slave,
        static_cast<int>(err.Etype), err.Index, err.SubIdx, static_cast<uint32_t>(err.AbortCode));
  }
}

// =========================================================================
// Registers
// =========================================================================

bool EthercatMaster::read_esc_error_counters(int slave, std::array<uint8_t, 20> & counters)
{
  const uint16_t configadr = ctx_.slavelist[slave].configadr;
  return ecx_FPRD(&ctx_.port, configadr, 0x0300, counters.size(), counters.data(), EC_TIMEOUTRET) > 0;
}

void EthercatMaster::log_dc_snapshot(int slave)
{
  uint8_t dccuc = 0;
  uint8_t dcsyncact = 0;
  uint64_t start0 = 0;
  uint32_t cycle0 = 0;

  const auto & sl = ctx_.slavelist[slave];
  const uint16_t configadr = sl.configadr;
  const int wkc_dccuc = ecx_FPRD(&ctx_.port, configadr, ECT_REG_DCCUC, sizeof(dccuc), &dccuc, EC_TIMEOUTRET);
  const int wkc_syncact = ecx_FPRD(&ctx_.port, configadr, ECT_REG_DCSYNCACT, sizeof(dcsyncact), &dcsyncact, EC_TIMEOUTRET);
  const int wkc_start0 = ecx_FPRD(&ctx_.port, configadr, ECT_REG_DCSTART0, sizeof(start0), &start0, EC_TIMEOUTRET);
  const int wkc_cycle0 = ecx_FPRD(&ctx_.port, configadr, ECT_REG_DCCYCLE0, sizeof(cycle0), &cycle0, EC_TIMEOUTRET);

  RCLCPP_DEBUG(
      logger_,
      "Slave %d DC snapshot: configadr=0x%04X hasdc=%d SOEM{active=%u cycle=%d shift=%d} "
      "ESC{DCCUC=%s0x%02X%s DCSYNCACT=%s0x%02X%s START0=%s0x%016llX%s CYCLE0=%s%u%s}",
      slave, configadr, sl.hasdc, sl.DCactive, sl.DCcycle, sl.DCshift,
      wkc_dccuc > 0 ? "" : "FAIL(", wkc_dccuc > 0 ? dccuc : 0, wkc_dccuc > 0 ? "" : ")",
      wkc_syncact > 0 ? "" : "FAIL(", wkc_syncact > 0 ? dcsyncact : 0, wkc_syncact > 0 ? "" : ")",
      wkc_start0 > 0 ? "" : "FAIL(", wkc_start0 > 0 ? static_cast<unsigned long long>(start0) : 0ULL,
      wkc_start0 > 0 ? "" : ")",
      wkc_cycle0 > 0 ? "" : "FAIL(", wkc_cycle0 > 0 ? cycle0 : 0U, wkc_cycle0 > 0 ? "" : ")");
}

}  // namespace ird_ethercat_hardware_interface
