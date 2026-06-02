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

#include "ars408_ros/ars408_radar_cfg_verify.hpp"

namespace ars408
{
namespace radar_cfg_verify
{
namespace
{

bool Check(
  const bool ok, const char * field, std::string * detail, const std::string & expected,
  const std::string & actual)
{
  if (ok) {
    return true;
  }
  if (detail != nullptr) {
    *detail = std::string(field) + ": expected " + expected + ", got " + actual;
  }
  return false;
}

RadarState::OutputTypeConfig ExpectedOutputType(const can_encoder::OutputType type)
{
  switch (type) {
    case can_encoder::OutputType::NONE:
      return RadarState::NONE;
    case can_encoder::OutputType::OBJECTS:
      return RadarState::OBJECTS;
    case can_encoder::OutputType::CLUSTERS:
      return RadarState::CLUSTERS;
  }
  return RadarState::OUTPUT_ERROR;
}

RadarState::PowerConfig ExpectedPowerMode(const can_encoder::RadarPower power)
{
  switch (power) {
    case can_encoder::RadarPower::STANDARD:
      return RadarState::STANDARD;
    case can_encoder::RadarPower::MINUS_3DB:
      return RadarState::MINUS_3dB_GAIN;
    case can_encoder::RadarPower::MINUS_6DB:
      return RadarState::MINUS_6dB_GAIN;
    case can_encoder::RadarPower::MINUS_9DB:
      return RadarState::MINUS_9dB_GAIN;
  }
  return RadarState::POWER_ERROR;
}

RadarState::SortingConfig ExpectedSorting(const can_encoder::SortIndex sort)
{
  switch (sort) {
    case can_encoder::SortIndex::NO_SORT:
      return RadarState::NO_SORT;
    case can_encoder::SortIndex::BY_RANGE:
      return RadarState::BY_RANGE;
    case can_encoder::SortIndex::BY_RCS:
      return RadarState::BY_RCS;
  }
  return RadarState::SORT_ERROR;
}

RadarState::Rcs_ThresholdConfig ExpectedRcs(const can_encoder::RcsThreshold rcs)
{
  return rcs == can_encoder::RcsThreshold::HIGH_SENSITIVITY ?
    RadarState::HIGH_SENSITIVITY :
    RadarState::NORMAL;
}

}  // namespace

bool RadarStateMatchesConfig(
  const RadarState & state, const can_encoder::RadarCfgParams & expected, const uint8_t node_radar_id,
  std::string * mismatch_detail)
{
  const uint8_t expected_sensor_id = node_radar_id;
  if (!Check(
      state.SensorID == expected_sensor_id, "sensor_id", mismatch_detail,
      std::to_string(expected_sensor_id), std::to_string(state.SensorID)))
  {
    return false;
  }

  if (!Check(
      state.MaxDistance == expected.max_distance_m, "max_distance_m", mismatch_detail,
      std::to_string(expected.max_distance_m), std::to_string(state.MaxDistance)))
  {
    return false;
  }

  const auto expected_output = ExpectedOutputType(expected.output_type);
  if (!Check(
      state.OutputType == expected_output, "output_type", mismatch_detail,
      std::to_string(static_cast<int>(expected_output)),
      std::to_string(static_cast<int>(state.OutputType))))
  {
    return false;
  }

  const auto expected_power = ExpectedPowerMode(expected.radar_power);
  if (!Check(
      state.PowerMode == expected_power, "radar_power", mismatch_detail,
      std::to_string(static_cast<int>(expected_power)),
      std::to_string(static_cast<int>(state.PowerMode))))
  {
    return false;
  }

  const RadarState::Config expected_quality =
    expected.send_quality ? RadarState::ACTIVE : RadarState::INACTIVE;
  if (!Check(
      state.SendQuality == expected_quality, "send_quality", mismatch_detail,
      expected.send_quality ? "active" : "inactive",
      state.SendQuality == RadarState::ACTIVE ? "active" : "inactive"))
  {
    return false;
  }

  if (expected.output_type == can_encoder::OutputType::OBJECTS) {
    const RadarState::Config expected_ext =
      expected.send_ext_info ? RadarState::ACTIVE : RadarState::INACTIVE;
    if (!Check(
        state.SendExtInfo == expected_ext, "send_ext_info", mismatch_detail,
        expected.send_ext_info ? "active" : "inactive",
        state.SendExtInfo == RadarState::ACTIVE ? "active" : "inactive"))
    {
      return false;
    }
  }

  const auto expected_sort = ExpectedSorting(expected.sort_index);
  if (!Check(
      state.SortingMode == expected_sort, "sort_index", mismatch_detail,
      std::to_string(static_cast<int>(expected_sort)),
      std::to_string(static_cast<int>(state.SortingMode))))
  {
    return false;
  }

  const RadarState::Config expected_relay =
    expected.ctrl_relay ? RadarState::ACTIVE : RadarState::INACTIVE;
  if (!Check(
      state.CtrlRelay == expected_relay, "ctrl_relay", mismatch_detail,
      expected.ctrl_relay ? "active" : "inactive",
      state.CtrlRelay == RadarState::ACTIVE ? "active" : "inactive"))
  {
    return false;
  }

  const auto expected_rcs = ExpectedRcs(expected.rcs_threshold);
  if (!Check(
      state.Rcs_Threshold == expected_rcs, "rcs_threshold", mismatch_detail,
      std::to_string(static_cast<int>(expected_rcs)),
      std::to_string(static_cast<int>(state.Rcs_Threshold))))
  {
    return false;
  }

  if (mismatch_detail != nullptr) {
    mismatch_detail->clear();
  }
  return true;
}

}  // namespace radar_cfg_verify
}  // namespace ars408
