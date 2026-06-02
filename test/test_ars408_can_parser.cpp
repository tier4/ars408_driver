// Copyright 2021 Perception Engine, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ars408_ros/ars408_can_encoder.hpp"
#include "ars408_ros/ars408_can_parser.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace
{
using ars408::can_parser::HasMinimumDlc;
using ars408::can_parser::ParseObjectGeneral;
using ars408::can_parser::ParseObjectListStatus;
using ars408::can_parser::ParseRadarState;
using ars408::can_parser::SensorIdFromCanId;
}  // namespace

TEST(Ars408CanParser, HasMinimumDlc)
{
  EXPECT_TRUE(HasMinimumDlc(8, 4));
  EXPECT_TRUE(HasMinimumDlc(4, 4));
  EXPECT_FALSE(HasMinimumDlc(3, 4));
}

TEST(Ars408CanParser, SensorIdFromCanId)
{
  EXPECT_EQ(SensorIdFromCanId(0x201), 0u);
  EXPECT_EQ(SensorIdFromCanId(0x211), 1u);
  EXPECT_EQ(SensorIdFromCanId(0x60B), 0u);
  EXPECT_EQ(SensorIdFromCanId(0x61B), 1u);
}

TEST(Ars408CanParser, ParseRadarStateMaxDistance)
{
  ars408::can_encoder::RadarCfgParams params;
  params.update_max_distance = true;
  params.max_distance_m = 260;

  const auto encoded = ars408::can_encoder::EncodeRadarCfg(params);

  ars408::RadarState state;
  ParseRadarState(encoded, state);

  EXPECT_EQ(state.MaxDistance, 260u);
}

TEST(Ars408CanParser, ParseObjectListStatus)
{
  // MeasurementCounter: LSB=16 (byte 2), MSB=15 (byte 1) — bytes go large→small.
  // byte 1 = upper byte, byte 2 = lower byte.
  // InterfaceVersion: bits 28-31 (byte 3 high nibble).
  std::array<uint8_t, 8> data{};
  data[0] = 5;
  data[1] = 0x12;  // MeasurementCounter upper byte
  data[2] = 0x34;  // MeasurementCounter lower byte
  data[3] = 0x10;  // InterfaceVersion = 1 (bits 28-31)

  ars408::Obj_0_Status status;
  ParseObjectListStatus(data, status);

  EXPECT_EQ(status.NumberOfObjects, 5u);
  EXPECT_EQ(status.MeasurementCounter, 0x1234u);
  EXPECT_EQ(status.InterfaceVersion, 1u);
}

TEST(Ars408CanParser, ParseObjectListStatusAcceptsDlc8)
{
  std::array<uint8_t, 8> data{};
  data[0] = 2;
  data[2] = 0x01;
  data[3] = 0x00;

  ars408::Obj_0_Status status;
  ASSERT_TRUE(HasMinimumDlc(8, ars408::OBJ_STATUS_BYTES));
  ParseObjectListStatus(data, status);
  EXPECT_EQ(status.NumberOfObjects, 2u);
  EXPECT_EQ(status.MeasurementCounter, 1u);
}

TEST(Ars408CanParser, ParseRadarStateRcsThreshold)
{
  std::array<uint8_t, 8> data{};
  data[7] = static_cast<uint8_t>(0x08);  // RCS threshold value 2 at bits 58-60

  ars408::RadarState state;
  ParseRadarState(data, state);

  EXPECT_EQ(state.Rcs_Threshold, ars408::RadarState::Rcs_ThresholdConfig::RCS_ERROR);
  data[7] = static_cast<uint8_t>(0x04);  // value 1: high sensitivity
  ParseRadarState(data, state);
  EXPECT_EQ(state.Rcs_Threshold, ars408::RadarState::Rcs_ThresholdConfig::HIGH_SENSITIVITY);
}

TEST(Ars408CanParser, ParseRadarStateOutputTypeIndependentFromRcs)
{
  std::array<uint8_t, 8> data{};
  data[5] = static_cast<uint8_t>(0x04);  // OutputType = 1 (objects)
  data[7] = static_cast<uint8_t>(0x00);  // RCS threshold = 0 (standard)

  ars408::RadarState state;
  ParseRadarState(data, state);

  EXPECT_EQ(state.OutputType, ars408::RadarState::OutputTypeConfig::OBJECTS);
  EXPECT_EQ(state.Rcs_Threshold, ars408::RadarState::Rcs_ThresholdConfig::NORMAL);
}

TEST(Ars408CanParser, ParseObjectGeneral)
{
  std::array<uint8_t, 8> data{};
  data[0] = 7;
  data[1] = 0x4F;
  data[2] = 0xB0;

  const ars408::RadarObject object = ParseObjectGeneral(data, 0x00AB);
  EXPECT_EQ(object.id, 7u);
  EXPECT_EQ(object.sequence_id, 0x00ABu);
  EXPECT_NEAR(object.distance_long_x, 10.0f, 0.01f);
}

TEST(Ars408CanParser, ParseObjectQualityExistenceProbability)
{
  std::array<uint8_t, 8> data{};
  data[0] = 3;
  // Obj_ProbOfExist at bit 53 (Intel): index 2 -> 50%
  data[6] = static_cast<uint8_t>(0x40);

  const ars408::Obj_2_Quality quality = ars408::can_parser::ParseObjectQuality(data);
  EXPECT_EQ(quality.Id, 3u);
  EXPECT_FLOAT_EQ(quality.ExistenceProbability, 0.5f);
}
