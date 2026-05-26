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

#ifndef ARS408_ROS__ARS408_FILTER_SIGNALS_HPP_
#define ARS408_ROS__ARS408_FILTER_SIGNALS_HPP_

#include <cstdint>
#include <string>

namespace ars408
{
namespace filter_signals
{

/// FilterCfg_Index / FilterState_Index (Standard Radar Interface Table 4).
enum class FilterIndex : uint8_t {
  NOF_OBJ = 0x0,
  DISTANCE = 0x1,
  AZIMUTH = 0x2,
  VREL_ONCOME = 0x3,
  VREL_DEPART = 0x4,
  RCS = 0x5,
  LIFETIME = 0x6,
  SIZE = 0x7,
  PROB_EXISTS = 0x8,
  POS_Y = 0x9,
  POS_X = 0xA,
  VY_RIGHT_LEFT = 0xB,
  VX_ONCOME = 0xC,
  VY_LEFT_RIGHT = 0xD,
  VX_DEPART = 0xE,
  CLASS = 0xF,
};

/// One FilterCfg (0x202) entry loaded from YAML.
struct FilterCfgEntry
{
  FilterIndex index{FilterIndex::DISTANCE};
  bool active{true};
  bool for_objects{true};
  double min_value{0.0};
  double max_value{0.0};
};

/// Parsed FilterState_Cfg (0x204) for one criterion.
struct FilterStateCfg
{
  FilterIndex index{FilterIndex::DISTANCE};
  bool active{false};
  bool for_objects{false};
  double min_value{0.0};
  double max_value{0.0};
};

struct FilterStateHeader
{
  uint8_t cluster_filter_count{0};
  uint8_t object_filter_count{0};
};

uint8_t FilterIndexToRaw(const FilterIndex index);

FilterIndex FilterIndexFromRaw(const uint8_t raw);

/// Parses filter_cfg index names (e.g. distance, nof_obj, pos_x, 10, 0xA).
FilterIndex ParseFilterIndexSetting(const std::string & value);

const char * FilterIndexToString(const FilterIndex index);

bool FilterIndexIgnoresMin(const FilterIndex index);

bool FilterIndexUses13BitRange(const FilterIndex index);

uint32_t EncodeRawMin(const FilterCfgEntry & entry);

uint32_t EncodeRawMax(const FilterCfgEntry & entry);

double DecodeRawMin(const FilterIndex index, const uint32_t raw);

double DecodeRawMax(const FilterIndex index, const uint32_t raw);

}  // namespace filter_signals
}  // namespace ars408

#endif  // ARS408_ROS__ARS408_FILTER_SIGNALS_HPP_
