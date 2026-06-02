// Copyright 2026 TIER IV, Inc. All rights reserved.
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

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ars408
{
namespace can_encoder
{
namespace
{

using ars408::can_signal::PackSignalIntel;

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
  // ARS408 ICD: RadarDevice_Speed factor = 0.02 km/h per bit; convert m/s → km/h first.
  const float speed_kmh = speed_mps * 3.6f;
  const float clamped_speed = std::max(0.f, std::min(speed_kmh, 163.8f));
  const uint32_t raw_speed = static_cast<uint32_t>(std::lround(clamped_speed / 0.02f)) & 0x1FFFu;
  const uint32_t raw_direction = static_cast<uint32_t>(direction) & 0x03u;

  // SpeedDirection: LSB=6, MSB=7 (byte 0 bits 6-7)
  PackSignalIntel(data, 6, 2, raw_direction);
  // Speed: LSB=8, MSB=4 — bit sequence 8,9,…,15,0,1,2,3,4
  PackSignalIntel(data, 8, 13, raw_speed);
  return data;
}

std::array<uint8_t, 8> EncodeYawRateInformation(const float yaw_rate_deg_s)
{
  std::array<uint8_t, 8> data{};
  const float clamped_yaw = std::max(-327.68f, std::min(yaw_rate_deg_s, 327.68f));
  const uint32_t raw_yaw = static_cast<uint32_t>(std::lround((clamped_yaw + 327.68f) / 0.01f)) &
                         0xFFFFu;

  PackSignalIntel(data, 8, 16, raw_yaw);
  return data;
}

std::array<uint8_t, 8> EncodeRadarCfg(const RadarCfgParams & params)
{
  std::array<uint8_t, 8> data{};

  if (params.update_max_distance) {
    PackSignalIntel(data, 0, 1, 1);
  }
  if (params.update_sensor_id) {
    PackSignalIntel(data, 1, 1, 1);
  }
  if (params.update_radar_power) {
    PackSignalIntel(data, 2, 1, 1);
  }
  if (params.update_output_type) {
    PackSignalIntel(data, 3, 1, 1);
  }
  if (params.update_send_quality) {
    PackSignalIntel(data, 4, 1, 1);
  }
  if (params.update_send_ext_info) {
    PackSignalIntel(data, 5, 1, 1);
  }
  if (params.update_sort_index) {
    PackSignalIntel(data, 6, 1, 1);
  }
  if (params.update_store_in_nvm) {
    PackSignalIntel(data, 7, 1, 1);
  }
  if (params.update_ctrl_relay) {
    PackSignalIntel(data, 40, 1, 1);
  }
  if (params.update_rcs_threshold) {
    PackSignalIntel(data, 48, 1, 1);
  }

  if (params.update_max_distance) {
    const uint32_t raw_distance =
      std::min(1023u, static_cast<uint32_t>(params.max_distance_m / 2u));
    PackSignalIntel(data, 22, 10, raw_distance);
  }
  if (params.update_sensor_id) {
    PackSignalIntel(data, 32, 3, params.sensor_id & 0x07u);
  }
  if (params.update_output_type) {
    PackSignalIntel(data, 35, 2, static_cast<uint32_t>(params.output_type));
  }
  if (params.update_radar_power) {
    PackSignalIntel(data, 37, 3, static_cast<uint32_t>(params.radar_power));
  }
  if (params.update_ctrl_relay) {
    PackSignalIntel(data, 41, 1, params.ctrl_relay ? 1u : 0u);
  }
  if (params.update_send_quality) {
    PackSignalIntel(data, 42, 1, params.send_quality ? 1u : 0u);
  }
  if (params.update_send_ext_info) {
    PackSignalIntel(data, 43, 1, params.send_ext_info ? 1u : 0u);
  }
  if (params.update_sort_index) {
    PackSignalIntel(data, 44, 3, static_cast<uint32_t>(params.sort_index));
  }
  if (params.update_store_in_nvm) {
    PackSignalIntel(data, 47, 1, params.store_in_nvm ? 1u : 0u);
  }
  if (params.update_rcs_threshold) {
    PackSignalIntel(data, 49, 3, static_cast<uint32_t>(params.rcs_threshold));
  }

  return data;
}

std::array<uint8_t, 8> EncodeFilterCfg(const filter_signals::FilterCfgEntry & entry)
{
  std::array<uint8_t, 8> data{};

  PackSignalIntel(data, 1, 1, 1u);
  PackSignalIntel(data, 2, 1, entry.active ? 1u : 0u);
  PackSignalIntel(data, 3, 4, filter_signals::FilterIndexToRaw(entry.index));
  PackSignalIntel(data, 7, 1, entry.for_objects ? 1u : 0u);

  const uint8_t value_bits = filter_signals::FilterIndexUses13BitRange(entry.index) ? 13u : 12u;
  if (!filter_signals::FilterIndexIgnoresMin(entry.index)) {
    PackSignalIntel(data, 16, value_bits, filter_signals::EncodeRawMin(entry));
  }
  PackSignalIntel(data, 32, value_bits, filter_signals::EncodeRawMax(entry));

  return data;
}

}  // namespace can_encoder
}  // namespace ars408
