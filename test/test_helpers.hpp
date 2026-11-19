// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

#ifndef IRD_ETHERCAT_HARDWARE_INTERFACE__TEST_HELPERS_HPP_
#define IRD_ETHERCAT_HARDWARE_INTERFACE__TEST_HELPERS_HPP_

#include <cstdint>

#include "ird_ethercat_hardware_interface/config.hpp"

namespace ird_ethercat_hardware_interface
{

/// The configuration validated on the Naviwall robot (v0.1.0).
inline SystemConfig naviwall_config()
{
  SystemConfig config;
  const struct { const char * name; const char * joint; int slave; double ratio; } drives[] = {
      {"left", "drivewhl_l_joint", 1, 20.0},
      {"right", "drivewhl_r_joint", 2, 20.0},
      {"turret", "turret_joint", 3, 51.67},
  };
  for (const auto & d : drives)
  {
    DriveConfig drive = make_ird_csv_drive();
    drive.name = d.name;
    drive.joint = d.joint;
    drive.slave_position = d.slave;
    drive.gear_ratio = d.ratio;
    drive.encoder_ticks_per_rev = 2048;
    config.drives.push_back(drive);
  }
  return config;
}

/// Minimal CiA 402 drive: follows the controlword one transition per frame,
/// can be put in FAULT, and clears it on the rising edge of 0x0080 unless
/// told the fault is persistent.
struct FakeDrive
{
  static constexpr uint16_t REMOTE = 0x0200;  // bit 9, set by the IRD in every state
  static constexpr uint16_t FAULT_SW = 0x0218;

  uint16_t state{0x0040};  // Switch On Disabled
  bool fault{false};
  bool reset_clears{true};
  uint16_t last_cw{0};

  uint16_t statusword() const { return fault ? FAULT_SW : static_cast<uint16_t>(REMOTE | state); }

  void inject_fault(bool clears_on_reset = true)
  {
    fault = true;
    reset_clears = clears_on_reset;
  }

  void receive(uint16_t cw)
  {
    const bool reset_edge = (cw & 0x0080) && !(last_cw & 0x0080);
    last_cw = cw;
    if (fault)
    {
      if (reset_edge && reset_clears)
      {
        fault = false;
        state = 0x0040;
      }
      return;
    }
    switch (cw & 0x008F)
    {
      case 0x0006: state = 0x0021; break;                       // Shutdown -> Ready To Switch On
      case 0x0007: if (state == 0x0021 || state == 0x0027) state = 0x0023; break;  // Switch On
      case 0x000F: if (state == 0x0023) state = 0x0027; break;  // Enable Operation
      case 0x0000: state = 0x0040; break;                       // Disable Voltage
      default: break;
    }
  }
};

}  // namespace ird_ethercat_hardware_interface

#endif  // IRD_ETHERCAT_HARDWARE_INTERFACE__TEST_HELPERS_HPP_
