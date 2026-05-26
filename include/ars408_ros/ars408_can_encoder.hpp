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

#ifndef ARS408_ROS__ARS408_CAN_ENCODER_HPP_
#define ARS408_ROS__ARS408_CAN_ENCODER_HPP_

#include <array>
#include <cstdint>
#include <string>

namespace ars408
{
namespace can_encoder
{

enum class SpeedDirection : uint8_t {
  STANDSTILL = 0,
  FORWARD = 1,
  BACKWARD = 2,
};

enum class OutputType : uint8_t {
  NONE = 0,
  OBJECTS = 1,
  CLUSTERS = 2,
};

enum class RadarPower : uint8_t {
  STANDARD = 0,
  MINUS_3DB = 1,
  MINUS_6DB = 2,
  MINUS_9DB = 3,
};

/// Default RadarCfg transmit power when YAML omits radar_power (Japan radio regulations).
constexpr RadarPower kDefaultRadarPowerJapan = RadarPower::MINUS_3DB;

/// Parses radar_cfg.radar_power. Standard (0 dB) is not allowed (Japan radio regulations).
RadarPower ParseRadarPowerSetting(const std::string & value);

enum class SortIndex : uint8_t {
  NO_SORT = 0,
  BY_RANGE = 1,
  BY_RCS = 2,
};

enum class RcsThreshold : uint8_t {
  NORMAL = 0,
  HIGH_SENSITIVITY = 1,
};

/// Parameters for RadarCfg (0x200). Set each `update_*` flag to apply that field.
struct RadarCfgParams
{
  bool update_max_distance{false};
  bool update_sensor_id{false};
  bool update_radar_power{false};
  bool update_output_type{false};
  bool update_send_quality{false};
  bool update_send_ext_info{false};
  bool update_sort_index{false};
  bool update_store_in_nvm{false};
  bool update_ctrl_relay{false};
  bool update_rcs_threshold{false};

  uint16_t max_distance_m{260};
  uint8_t sensor_id{0};
  OutputType output_type{OutputType::OBJECTS};
  RadarPower radar_power{kDefaultRadarPowerJapan};
  bool ctrl_relay{false};
  bool send_quality{true};
  bool send_ext_info{true};
  SortIndex sort_index{SortIndex::BY_RANGE};
  bool store_in_nvm{false};
  RcsThreshold rcs_threshold{RcsThreshold::NORMAL};
};

/// CAN arbitration ID for a sensor-specific message (Standard Radar Interface §4.1).
uint32_t CanIdForSensor(uint32_t base_id, uint8_t sensor_id);

/// RadarCfg (0x200).
std::array<uint8_t, 8> EncodeRadarCfg(const RadarCfgParams & params);

/// SpeedInformation (0x300): speed magnitude [m/s] and direction.
std::array<uint8_t, 8> EncodeSpeedInformation(float speed_mps, SpeedDirection direction);

/// YawRateInformation (0x301): yaw rate [deg/s].
std::array<uint8_t, 8> EncodeYawRateInformation(float yaw_rate_deg_s);

}  // namespace can_encoder
}  // namespace ars408

#endif  // ARS408_ROS__ARS408_CAN_ENCODER_HPP_
