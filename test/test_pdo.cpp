// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include <cstdint>

#include "ird_ethercat_hardware_interface/pdo.hpp"

using namespace ird_ethercat_hardware_interface;

TEST(Pdo, IrdCsvLayoutMatchesV010)
{
  // v0.1.0: RxPDO 7 bytes (CW @0, mode @2, target velocity @3),
  //         TxPDO 7 bytes (SW @0, mode display @2, actual velocity @3).
  const DriveConfig drive = make_ird_csv_drive();
  const PdoLayout rx = compute_layout(drive.rx_pdo);
  const PdoLayout tx = compute_layout(drive.tx_pdo);
  EXPECT_EQ(rx.size_bytes, 7u);
  EXPECT_EQ(tx.size_bytes, 7u);

  EXPECT_EQ(find_role(drive.rx_pdo, rx, PdoRole::CONTROLWORD).offset, 0u);
  EXPECT_EQ(find_role(drive.rx_pdo, rx, PdoRole::MODE_OF_OPERATION).offset, 2u);
  EXPECT_EQ(find_role(drive.rx_pdo, rx, PdoRole::TARGET_VELOCITY).offset, 3u);
  EXPECT_EQ(find_role(drive.tx_pdo, tx, PdoRole::STATUSWORD).offset, 0u);
  EXPECT_EQ(find_role(drive.tx_pdo, tx, PdoRole::VELOCITY_ACTUAL).offset, 3u);
  EXPECT_FALSE(find_role(drive.tx_pdo, tx, PdoRole::CONTROLWORD).mapped);
}

TEST(Pdo, MappingValues)
{
  const DriveConfig drive = make_ird_csv_drive();
  EXPECT_EQ(mapping_value(drive.rx_pdo[0]), 0x60400010u);
  EXPECT_EQ(mapping_value(drive.rx_pdo[1]), 0x60600008u);
  EXPECT_EQ(mapping_value(drive.rx_pdo[2]), 0x60FF0020u);
  EXPECT_EQ(mapping_value(drive.tx_pdo[0]), 0x60410010u);
  EXPECT_EQ(mapping_value(drive.tx_pdo[1]), 0x60610008u);
  EXPECT_EQ(mapping_value(drive.tx_pdo[2]), 0x606C0020u);
  EXPECT_EQ(mapping_value({0x3022, 0x02, DataType::UINT32, PdoRole::NONE}), 0x30220220u);
}

TEST(Pdo, LittleEndianEncoding)
{
  uint8_t buffer[4] = {};
  write_value(buffer, DataType::INT32, -2);
  EXPECT_EQ(buffer[0], 0xFE);
  EXPECT_EQ(buffer[1], 0xFF);
  EXPECT_EQ(buffer[2], 0xFF);
  EXPECT_EQ(buffer[3], 0xFF);
  EXPECT_EQ(read_value(buffer, DataType::INT32), -2);
  EXPECT_EQ(read_value(buffer, DataType::UINT32), 0xFFFFFFFE);

  write_value(buffer, DataType::UINT16, 0x1234);
  EXPECT_EQ(buffer[0], 0x34);
  EXPECT_EQ(buffer[1], 0x12);
  EXPECT_EQ(read_value(buffer, DataType::UINT16), 0x1234);
}

TEST(Pdo, SignExtension)
{
  uint8_t buffer[4] = {0x80, 0x80, 0x00, 0x00};
  EXPECT_EQ(read_value(buffer, DataType::INT8), -128);
  EXPECT_EQ(read_value(buffer, DataType::UINT8), 128);
  EXPECT_EQ(read_value(buffer, DataType::INT16), -32640);
  EXPECT_EQ(read_value(buffer, DataType::UINT16), 32896);
}

TEST(Pdo, WriteOnlyTouchesItsBytes)
{
  uint8_t buffer[7] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
  write_value(buffer + 2, DataType::INT8, 9);
  EXPECT_EQ(buffer[1], 0xAA);
  EXPECT_EQ(buffer[2], 9);
  EXPECT_EQ(buffer[3], 0xAA);
}
