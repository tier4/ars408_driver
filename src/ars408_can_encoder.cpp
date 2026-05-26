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

}  // namespace can_encoder
}  // namespace ars408
