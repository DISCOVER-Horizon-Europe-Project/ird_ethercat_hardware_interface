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

TEST(Pdo, Saturate)
{
  EXPECT_EQ(saturate(DataType::INT16, 40000), 32767);
  EXPECT_EQ(saturate(DataType::INT16, -40000), -32768);
  EXPECT_EQ(saturate(DataType::UINT8, -5), 0);
  EXPECT_EQ(saturate(DataType::UINT8, 300), 255);
  EXPECT_EQ(saturate(DataType::INT32, 5'000'000'000LL), INT32_MAX);
  EXPECT_EQ(saturate(DataType::UINT32, -1), 0);
  EXPECT_EQ(saturate(DataType::INT8, -7), -7);
}

TEST(Pdo, ExtendedLayoutForPowerMonitoring)
{
  // The CSV mapping plus position, current and DC-link voltage: 17 bytes.
  DriveConfig drive = make_ird_csv_drive();
  drive.tx_pdo.push_back({0x6064, 0, DataType::INT32, PdoRole::POSITION_ACTUAL});
  drive.tx_pdo.push_back({0x6078, 0, DataType::INT16, PdoRole::NONE, "current_actual", 0.001});
  drive.tx_pdo.push_back({0x6079, 0, DataType::UINT32, PdoRole::NONE, "dc_link_voltage", 0.001});
  const PdoLayout layout = compute_layout(drive.tx_pdo);
  EXPECT_EQ(layout.size_bytes, 17u);
  EXPECT_EQ(layout.offsets[3], 7u);
  EXPECT_EQ(layout.offsets[4], 11u);
  EXPECT_EQ(layout.offsets[5], 13u);
  EXPECT_EQ(find_role(drive.tx_pdo, layout, PdoRole::POSITION_ACTUAL).offset, 7u);
}

TEST(Pdo, DefaultMappingFitsInOnePdoEach)
{
  const DriveConfig drive = make_ird_csv_drive();
  const auto rx = split_into_pdos(drive.rx_pdo, drive.rx_pdo_index, drive.pdo_limits);
  const auto tx = split_into_pdos(drive.tx_pdo, drive.tx_pdo_index, drive.pdo_limits);
  ASSERT_EQ(rx.size(), 1u);
  ASSERT_EQ(tx.size(), 1u);
  EXPECT_EQ(rx[0].index, 0x1600);
  EXPECT_EQ(tx[0].index, 0x1A00);
  EXPECT_EQ(tx[0].entry_count, 3u);
  EXPECT_EQ(tx[0].size_bytes, 7u);
}

TEST(Pdo, SplitKeepsTheEntryOrderAndTheLayout)
{
  // 7 + 4 bytes do not fit in 8: position starts 0x1A01. Splitting does not
  // change where entries sit in the process image.
  DriveConfig drive = make_ird_csv_drive();
  drive.tx_pdo.push_back({0x6064, 0, DataType::INT32, PdoRole::POSITION_ACTUAL});
  drive.tx_pdo.push_back({0x6078, 0, DataType::INT16, PdoRole::NONE, "current_actual", 0.001});
  drive.tx_pdo.push_back({0x6079, 0, DataType::UINT32, PdoRole::NONE, "dc_link_voltage", 0.001});
  const auto objects = split_into_pdos(drive.tx_pdo, 0x1A00, drive.pdo_limits);
  ASSERT_EQ(objects.size(), 3u);
  EXPECT_EQ(objects[1].index, 0x1A01);
  EXPECT_EQ(objects[1].first_entry, 3u);
  EXPECT_EQ(objects[1].entry_count, 2u);
  EXPECT_EQ(objects[1].size_bytes, 6u);
  EXPECT_EQ(objects[2].index, 0x1A02);
  EXPECT_EQ(objects[2].first_entry, 5u);
  EXPECT_EQ(compute_layout(drive.tx_pdo).offsets[3], 7u);
}

TEST(Pdo, SplitByEntryCount)
{
  const std::vector<PdoEntry> entries(5, PdoEntry{0x2000, 0, DataType::UINT8});
  const auto objects = split_into_pdos(entries, 0x1A00, {0, 4, 0});
  ASSERT_EQ(objects.size(), 2u);
  EXPECT_EQ(objects[0].entry_count, 4u);
  EXPECT_EQ(objects[1].entry_count, 1u);
}

TEST(Pdo, NoLimitsMeansOnePdo)
{
  const std::vector<PdoEntry> entries(9, PdoEntry{0x2000, 0, DataType::INT32});
  const auto objects = split_into_pdos(entries, 0x1A00, {0, 0, 0});
  ASSERT_EQ(objects.size(), 1u);
  EXPECT_EQ(objects[0].size_bytes, 36u);
  EXPECT_TRUE(split_into_pdos({}, 0x1A00, {}).empty());
}
