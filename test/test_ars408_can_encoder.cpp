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

#include "ars408_ros/detail/ars408_can_signal.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

using ars408::can_signal::UnpackSignalIntel;

TEST(Ars408CanEncoder, CanIdForSensor)
{
  EXPECT_EQ(ars408::can_encoder::CanIdForSensor(0x300, 0), 0x300u);
  EXPECT_EQ(ars408::can_encoder::CanIdForSensor(0x300, 1), 0x310u);
  EXPECT_EQ(ars408::can_encoder::CanIdForSensor(0x301, 2), 0x321u);
}

TEST(Ars408CanEncoder, EncodeSpeedInformation)
{
  // 10.0 m/s = 36.0 km/h; ARS408 factor = 0.02 km/h/bit → raw = 1800
  // LSB=8, MSB=4: bit sequence 8,9,…,15,0,1,2,3,4
  const auto data = ars408::can_encoder::EncodeSpeedInformation(
    10.0f, ars408::can_encoder::SpeedDirection::FORWARD);

  // SpeedDirection FORWARD=1: LSB=6, MSB=7 (byte 0 bits 6-7)
  EXPECT_EQ(UnpackSignalIntel(data, 6, 2), 1u);
  // Decode: raw * 0.02 [km/h] / 3.6 → [m/s]; tolerance = 1 LSB = 0.02 km/h ≈ 0.006 m/s
  EXPECT_NEAR(
    static_cast<float>(UnpackSignalIntel(data, 8, 13)) * 0.02f / 3.6f, 10.0f, 0.01f);
}

TEST(Ars408CanEncoder, EncodeSpeedStandstill)
{
  const auto data = ars408::can_encoder::EncodeSpeedInformation(
    0.0f, ars408::can_encoder::SpeedDirection::STANDSTILL);

  EXPECT_EQ(UnpackSignalIntel(data, 6, 2), 0u);
  EXPECT_EQ(UnpackSignalIntel(data, 8, 13), 0u);
}

TEST(Ars408CanEncoder, EncodeYawRateInformation)
{
  const auto data = ars408::can_encoder::EncodeYawRateInformation(0.0f);
  const float decoded = static_cast<float>(UnpackSignalIntel(data, 8, 16)) * 0.01f - 327.68f;
  EXPECT_NEAR(decoded, 0.0f, 0.02f);

  const auto left_turn = ars408::can_encoder::EncodeYawRateInformation(10.0f);
  const float decoded_turn =
    static_cast<float>(UnpackSignalIntel(left_turn, 8, 16)) * 0.01f - 327.68f;
  EXPECT_NEAR(decoded_turn, 10.0f, 0.05f);
}

TEST(Ars408CanEncoder, EncodeRadarCfgSensorIdOnly)
{
  ars408::can_encoder::RadarCfgParams params;
  params.update_sensor_id = true;
  params.sensor_id = 0;
  const auto data = ars408::can_encoder::EncodeRadarCfg(params);

  EXPECT_EQ(data[0], 0x02u);
  EXPECT_EQ(data[4] & 0x07u, 0u);
}

TEST(Ars408CanEncoder, EncodeRadarCfgObjectOutputDefaults)
{
  ars408::can_encoder::RadarCfgParams params;
  params.update_max_distance = true;
  params.update_radar_power = true;
  params.update_output_type = true;
  params.update_send_quality = true;
  params.update_send_ext_info = true;
  params.update_sort_index = true;
  params.update_store_in_nvm = true;
  params.update_ctrl_relay = true;
  params.update_rcs_threshold = true;
  params.max_distance_m = 260;
  params.output_type = ars408::can_encoder::OutputType::OBJECTS;
  params.radar_power = ars408::can_encoder::ParseRadarPowerSetting("minus_3db");
  params.send_quality = true;
  params.send_ext_info = true;
  params.sort_index = ars408::can_encoder::SortIndex::BY_RANGE;

  const auto data = ars408::can_encoder::EncodeRadarCfg(params);

  EXPECT_EQ(UnpackSignalIntel(data, 35, 2), 1u);
  EXPECT_EQ(UnpackSignalIntel(data, 42, 1), 1u);
  EXPECT_EQ(UnpackSignalIntel(data, 43, 1), 1u);
  EXPECT_EQ(UnpackSignalIntel(data, 22, 10), 130u);
  EXPECT_EQ(UnpackSignalIntel(data, 37, 3), 1u);  // minus_3db
}

TEST(Ars408CanEncoder, ParseRadarPowerRejectsStandard)
{
  EXPECT_THROW(
    ars408::can_encoder::ParseRadarPowerSetting("standard"), std::invalid_argument);
}

TEST(Ars408CanEncoder, ParseRadarPowerMinus6And9)
{
  EXPECT_EQ(
    ars408::can_encoder::ParseRadarPowerSetting("minus_6db"),
    ars408::can_encoder::RadarPower::MINUS_6DB);
  EXPECT_EQ(
    ars408::can_encoder::ParseRadarPowerSetting("minus_9db"),
    ars408::can_encoder::RadarPower::MINUS_9DB);
}
