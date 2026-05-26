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

namespace ars408
{
namespace can_encoder
{

enum class SpeedDirection : uint8_t {
  STANDSTILL = 0,
  FORWARD = 1,
  BACKWARD = 2,
};

/// CAN arbitration ID for a sensor-specific message (Standard Radar Interface §4.1).
uint32_t CanIdForSensor(uint32_t base_id, uint8_t sensor_id);

/// SpeedInformation (0x300): speed magnitude [m/s] and direction.
std::array<uint8_t, 8> EncodeSpeedInformation(float speed_mps, SpeedDirection direction);

/// YawRateInformation (0x301): yaw rate [deg/s].
std::array<uint8_t, 8> EncodeYawRateInformation(float yaw_rate_deg_s);

}  // namespace can_encoder
}  // namespace ars408

#endif  // ARS408_ROS__ARS408_CAN_ENCODER_HPP_
