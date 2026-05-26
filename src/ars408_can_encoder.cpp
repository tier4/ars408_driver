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

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ars408
{
namespace can_encoder
{
namespace
{

void packSignalIntel(
  std::array<uint8_t, 8> & data, const uint16_t start_bit, const uint8_t length,
  const uint32_t raw)
{
  for (uint8_t i = 0; i < length; ++i) {
    if ((raw >> i) & 0x01u) {
      const uint16_t bit_index = start_bit + i;
      data[bit_index / 8] |= static_cast<uint8_t>(1u << (bit_index % 8));
    }
  }
}

}  // namespace

uint32_t CanIdForSensor(const uint32_t base_id, const uint8_t sensor_id)
{
  return base_id + (static_cast<uint32_t>(sensor_id) * 0x10u);
}

RadarPower ParseRadarPowerSetting(const std::string & value)
{
  if (value == "standard") {
    throw std::invalid_argument(
      "radar_cfg.radar_power must not be \"standard\" (Japan radio regulations). "
      "Use minus_3db, minus_6db, or minus_9db");
  }
  if (value == "minus_3db") {
    return RadarPower::MINUS_3DB;
  }
  if (value == "minus_6db") {
    return RadarPower::MINUS_6DB;
  }
  if (value == "minus_9db") {
    return RadarPower::MINUS_9DB;
  }
  throw std::invalid_argument(
    "radar_cfg.radar_power must be minus_3db, minus_6db, or minus_9db");
}


std::array<uint8_t, 8> EncodeSpeedInformation(
  const float speed_mps, const SpeedDirection direction)
{
  std::array<uint8_t, 8> data{};
  const float clamped_speed = std::max(0.f, std::min(speed_mps, 163.8f));
  const uint32_t raw_speed = static_cast<uint32_t>(std::lround(clamped_speed / 0.02f)) & 0x1FFFu;
  const uint32_t raw_direction = static_cast<uint32_t>(direction) & 0x03u;

  packSignalIntel(data, 6, 2, raw_direction);
  packSignalIntel(data, 8, 13, raw_speed);
  return data;
}

std::array<uint8_t, 8> EncodeYawRateInformation(const float yaw_rate_deg_s)
{
  std::array<uint8_t, 8> data{};
  const float clamped_yaw = std::max(-327.68f, std::min(yaw_rate_deg_s, 327.68f));
  const uint32_t raw_yaw = static_cast<uint32_t>(std::lround((clamped_yaw + 327.68f) / 0.01f)) &
                         0xFFFFu;

  packSignalIntel(data, 8, 16, raw_yaw);
  return data;
}

std::array<uint8_t, 8> EncodeRadarCfg(const RadarCfgParams & params)
{
  std::array<uint8_t, 8> data{};

  if (params.update_max_distance) {
    packSignalIntel(data, 0, 1, 1);
  }
  if (params.update_sensor_id) {
    packSignalIntel(data, 1, 1, 1);
  }
  if (params.update_radar_power) {
    packSignalIntel(data, 2, 1, 1);
  }
  if (params.update_output_type) {
    packSignalIntel(data, 3, 1, 1);
  }
  if (params.update_send_quality) {
    packSignalIntel(data, 4, 1, 1);
  }
  if (params.update_send_ext_info) {
    packSignalIntel(data, 5, 1, 1);
  }
  if (params.update_sort_index) {
    packSignalIntel(data, 6, 1, 1);
  }
  if (params.update_store_in_nvm) {
    packSignalIntel(data, 7, 1, 1);
  }
  if (params.update_ctrl_relay) {
    packSignalIntel(data, 40, 1, 1);
  }
  if (params.update_rcs_threshold) {
    packSignalIntel(data, 48, 1, 1);
  }

  if (params.update_max_distance) {
    const uint32_t raw_distance =
      std::min(1023u, static_cast<uint32_t>(params.max_distance_m / 2u));
    packSignalIntel(data, 22, 10, raw_distance);
  }
  if (params.update_sensor_id) {
    packSignalIntel(data, 32, 3, params.sensor_id & 0x07u);
  }
  if (params.update_output_type) {
    packSignalIntel(data, 35, 2, static_cast<uint32_t>(params.output_type));
  }
  if (params.update_radar_power) {
    packSignalIntel(data, 37, 3, static_cast<uint32_t>(params.radar_power));
  }
  if (params.update_ctrl_relay) {
    packSignalIntel(data, 41, 1, params.ctrl_relay ? 1u : 0u);
  }
  if (params.update_send_quality) {
    packSignalIntel(data, 42, 1, params.send_quality ? 1u : 0u);
  }
  if (params.update_send_ext_info) {
    packSignalIntel(data, 43, 1, params.send_ext_info ? 1u : 0u);
  }
  if (params.update_sort_index) {
    packSignalIntel(data, 44, 3, static_cast<uint32_t>(params.sort_index));
  }
  if (params.update_store_in_nvm) {
    packSignalIntel(data, 47, 1, params.store_in_nvm ? 1u : 0u);
  }
  if (params.update_rcs_threshold) {
    packSignalIntel(data, 49, 3, static_cast<uint32_t>(params.rcs_threshold));
  }

  return data;
}

}  // namespace can_encoder
}  // namespace ars408
