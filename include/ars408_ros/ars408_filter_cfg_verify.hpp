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

#ifndef ARS408_ROS__ARS408_FILTER_CFG_VERIFY_HPP_
#define ARS408_ROS__ARS408_FILTER_CFG_VERIFY_HPP_

#include "ars408_ros/ars408_filter_signals.hpp"

#include <string>
#include <vector>

namespace ars408
{
namespace filter_cfg_verify
{

constexpr double kFilterValueTolerance = 0.15;

bool FilterStateMatchesEntry(
  const filter_signals::FilterStateCfg & state,
  const filter_signals::FilterCfgEntry & expected, std::string * out_detail = nullptr);

/// Returns true when every expected entry has a matching FilterState_Cfg (0x204) report.
bool AllFilterEntriesMatch(
  const std::vector<filter_signals::FilterCfgEntry> & expected,
  const std::vector<filter_signals::FilterStateCfg> & reported_states,
  std::string * out_detail = nullptr);

}  // namespace filter_cfg_verify
}  // namespace ars408

#endif  // ARS408_ROS__ARS408_FILTER_CFG_VERIFY_HPP_
