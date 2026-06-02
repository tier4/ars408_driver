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

#include <gtest/gtest.h>

namespace
{
ars408::can_encoder::RadarCfgParams DefaultExpectedParams()
{
  ars408::can_encoder::RadarCfgParams params;
  params.max_distance_m = 260;
  params.output_type = ars408::can_encoder::OutputType::OBJECTS;
  params.radar_power = ars408::can_encoder::RadarPower::MINUS_3DB;
  params.send_quality = true;
  params.send_ext_info = true;
  params.sort_index = ars408::can_encoder::SortIndex::BY_RANGE;
  params.rcs_threshold = ars408::can_encoder::RcsThreshold::NORMAL;
  params.ctrl_relay = false;
  return params;
}

ars408::RadarState StateMatchingDefaults()
{
  ars408::RadarState state;
  state.SensorID = 0;
  state.MaxDistance = 260;
  state.OutputType = ars408::RadarState::OBJECTS;
  state.PowerMode = ars408::RadarState::MINUS_3dB_GAIN;
  state.SendQuality = ars408::RadarState::ACTIVE;
  state.SendExtInfo = ars408::RadarState::ACTIVE;
  state.SortingMode = ars408::RadarState::BY_RANGE;
  state.CtrlRelay = ars408::RadarState::INACTIVE;
  state.Rcs_Threshold = ars408::RadarState::NORMAL;
  return state;
}
}  // namespace

TEST(Ars408RadarCfgVerify, MatchesDefaultConfig)
{
  const auto params = DefaultExpectedParams();
  const auto state = StateMatchingDefaults();
  EXPECT_TRUE(ars408::radar_cfg_verify::RadarStateMatchesConfig(state, params, 0));
}

TEST(Ars408RadarCfgVerify, ReportsMaxDistanceMismatch)
{
  const auto params = DefaultExpectedParams();
  auto state = StateMatchingDefaults();
  state.MaxDistance = 200;
  std::string detail;
  EXPECT_FALSE(ars408::radar_cfg_verify::RadarStateMatchesConfig(state, params, 0, &detail));
  EXPECT_NE(detail.find("max_distance_m"), std::string::npos);
}
