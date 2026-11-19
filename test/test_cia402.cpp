// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include "ird_ethercat_hardware_interface/cia402.hpp"

using namespace ird_ethercat_hardware_interface::cia402;

TEST(Cia402, EnableSequence)
{
  EXPECT_EQ(next_controlword(0x0240), CW_SHUTDOWN);          // Switch On Disabled
  EXPECT_EQ(next_controlword(0x0260), CW_SHUTDOWN);          // ... with quick-stop bit set
  EXPECT_EQ(next_controlword(0x0221), CW_SWITCH_ON);         // Ready To Switch On
  EXPECT_EQ(next_controlword(0x0223), CW_ENABLE_OPERATION);  // Switched On
  EXPECT_EQ(next_controlword(0x0227), CW_ENABLE_OPERATION);  // Operation Enabled: hold
}

TEST(Cia402, FaultIsNeverResetAutomatically)
{
  EXPECT_EQ(next_controlword(0x0218), CW_DISABLE_VOLTAGE);
  EXPECT_EQ(next_controlword(0x0008), CW_DISABLE_VOLTAGE);
}

TEST(Cia402, OtherStatesShutDown)
{
  EXPECT_EQ(next_controlword(0x0207), CW_SHUTDOWN);  // Quick Stop Active
  EXPECT_EQ(next_controlword(0x0000), CW_SHUTDOWN);  // Not Ready To Switch On
}

TEST(Cia402, Predicates)
{
  EXPECT_TRUE(is_fault(0x0218));
  EXPECT_FALSE(is_fault(0x0227));
  EXPECT_TRUE(is_operation_enabled(0x0237));  // bit 4 (voltage enabled) is outside the mask
  EXPECT_FALSE(is_operation_enabled(0x0223));
}
