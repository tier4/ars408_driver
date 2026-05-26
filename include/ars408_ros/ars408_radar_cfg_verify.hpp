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

#ifndef ARS408_ROS__ARS408_RADAR_CFG_VERIFY_HPP_
#define ARS408_ROS__ARS408_RADAR_CFG_VERIFY_HPP_

#include "ars408_ros/ars408_can_encoder.hpp"
#include "ars408_ros/ars408_commands.hpp"

#include <cstdint>
#include <string>

namespace ars408
{
namespace radar_cfg_verify
{

/// Returns true when RadarState (0x201) reflects the YAML RadarCfg parameters.
bool RadarStateMatchesConfig(
  const RadarState & state, const can_encoder::RadarCfgParams & expected, uint8_t node_radar_id,
  std::string * mismatch_detail = nullptr);

}  // namespace radar_cfg_verify
}  // namespace ars408

#endif  // ARS408_ROS__ARS408_RADAR_CFG_VERIFY_HPP_
