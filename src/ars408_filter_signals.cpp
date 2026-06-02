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

#include "ars408_ros/ars408_filter_signals.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace ars408
{
namespace filter_signals
{
namespace
{

uint32_t clampRaw(const uint32_t raw, const uint8_t bit_length)
{
  const uint32_t mask = (bit_length >= 32u) ? 0xFFFFFFFFu : ((1u << bit_length) - 1u);
  return raw & mask;
}

uint32_t encodeScaled(
  const double value, const double offset, const double resolution, const uint8_t bit_length)
{
  const double raw_f = (value - offset) / resolution;
  const auto raw = static_cast<int64_t>(std::lround(raw_f));
  const int64_t max_raw = (1ll << bit_length) - 1;
  return clampRaw(
    static_cast<uint32_t>(std::max<int64_t>(0, std::min<int64_t>(raw, max_raw))), bit_length);
}

double decodeScaled(
  const uint32_t raw, const double offset, const double resolution)
{
  return static_cast<double>(raw) * resolution + offset;
}

uint8_t probExistsToDiscrete(const double value)
{
  if (value <= 0.0) {
    return 0u;
  }
  if (value <= 0.25) {
    return 1u;
  }
  if (value <= 0.5) {
    return 2u;
  }
  if (value <= 0.75) {
    return 3u;
  }
  if (value <= 0.9) {
    return 4u;
  }
  if (value <= 0.99) {
    return 5u;
  }
  if (value <= 0.999) {
    return 6u;
  }
  return 7u;
}

double discreteToProbExists(const uint8_t raw)
{
  switch (raw & 0x0Fu) {
    case 0u: return 0.0;
    case 1u: return 0.25;
    case 2u: return 0.5;
    case 3u: return 0.75;
    case 4u: return 0.9;
    case 5u: return 0.99;
    case 6u: return 0.999;
    case 7u: return 1.0;
    default: return 0.0;
  }
}

uint8_t bitLengthForIndex(const FilterIndex index)
{
  return FilterIndexUses13BitRange(index) ? 13u : 12u;
}

}  // namespace

uint8_t FilterIndexToRaw(const FilterIndex index)
{
  return static_cast<uint8_t>(index);
}

FilterIndex FilterIndexFromRaw(const uint8_t raw)
{
  if (raw > 0x0Fu) {
    throw std::invalid_argument("filter index out of range [0, 15]");
  }
  return static_cast<FilterIndex>(raw);
}

FilterIndex ParseFilterIndexSetting(const std::string & value)
{
  if (value == "nof_obj" || value == "num_objects" || value == "0") {
    return FilterIndex::NOF_OBJ;
  }
  if (value == "distance" || value == "1") {
    return FilterIndex::DISTANCE;
  }
  if (value == "azimuth" || value == "2") {
    return FilterIndex::AZIMUTH;
  }
  if (value == "vrel_oncome" || value == "3") {
    return FilterIndex::VREL_ONCOME;
  }
  if (value == "vrel_depart" || value == "4") {
    return FilterIndex::VREL_DEPART;
  }
  if (value == "rcs" || value == "5") {
    return FilterIndex::RCS;
  }
  if (value == "lifetime" || value == "6") {
    return FilterIndex::LIFETIME;
  }
  if (value == "size" || value == "area" || value == "7") {
    return FilterIndex::SIZE;
  }
  if (value == "prob_exists" || value == "existence_probability" || value == "8") {
    return FilterIndex::PROB_EXISTS;
  }
  if (value == "y" || value == "pos_y" || value == "9") {
    return FilterIndex::POS_Y;
  }
  if (value == "x" || value == "pos_x" || value == "10" || value == "0xA" || value == "0xa") {
    return FilterIndex::POS_X;
  }
  if (value == "vy_right_left" || value == "11") {
    return FilterIndex::VY_RIGHT_LEFT;
  }
  if (value == "vx_oncome" || value == "12") {
    return FilterIndex::VX_ONCOME;
  }
  if (value == "vy_left_right" || value == "13") {
    return FilterIndex::VY_LEFT_RIGHT;
  }
  if (value == "vx_depart" || value == "14") {
    return FilterIndex::VX_DEPART;
  }
  if (value == "class" || value == "15") {
    return FilterIndex::CLASS;
  }

  char * end = nullptr;
  const unsigned long parsed = std::strtoul(value.c_str(), &end, 0);
  if (end != value.c_str() && *end == '\0' && parsed <= 0x0Fu) {
    return FilterIndexFromRaw(static_cast<uint8_t>(parsed));
  }

  throw std::invalid_argument(
    "filter index must be a known name (distance, nof_obj, pos_x, ...) or 0-15");
}

const char * FilterIndexToString(const FilterIndex index)
{
  switch (index) {
    case FilterIndex::NOF_OBJ: return "nof_obj";
    case FilterIndex::DISTANCE: return "distance";
    case FilterIndex::AZIMUTH: return "azimuth";
    case FilterIndex::VREL_ONCOME: return "vrel_oncome";
    case FilterIndex::VREL_DEPART: return "vrel_depart";
    case FilterIndex::RCS: return "rcs";
    case FilterIndex::LIFETIME: return "lifetime";
    case FilterIndex::SIZE: return "size";
    case FilterIndex::PROB_EXISTS: return "prob_exists";
    case FilterIndex::POS_Y: return "pos_y";
    case FilterIndex::POS_X: return "pos_x";
    case FilterIndex::VY_RIGHT_LEFT: return "vy_right_left";
    case FilterIndex::VX_ONCOME: return "vx_oncome";
    case FilterIndex::VY_LEFT_RIGHT: return "vy_left_right";
    case FilterIndex::VX_DEPART: return "vx_depart";
    case FilterIndex::CLASS: return "class";
    default: return "unknown";
  }
}

bool FilterIndexIgnoresMin(const FilterIndex index)
{
  return index == FilterIndex::NOF_OBJ || index == FilterIndex::CLASS;
}

bool FilterIndexUses13BitRange(const FilterIndex index)
{
  return index == FilterIndex::POS_X;
}

uint32_t EncodeRawMin(const FilterCfgEntry & entry)
{
  const uint8_t bits = bitLengthForIndex(entry.index);
  switch (entry.index) {
    case FilterIndex::NOF_OBJ:
    case FilterIndex::CLASS:
      return 0u;
    case FilterIndex::DISTANCE:
    case FilterIndex::LIFETIME:
      return encodeScaled(entry.min_value, 0.0, 0.1, bits);
    case FilterIndex::AZIMUTH:
    case FilterIndex::RCS:
      return encodeScaled(entry.min_value, -50.0, 0.025, bits);
    case FilterIndex::VREL_ONCOME:
    case FilterIndex::VREL_DEPART:
    case FilterIndex::VY_RIGHT_LEFT:
    case FilterIndex::VX_ONCOME:
    case FilterIndex::VY_LEFT_RIGHT:
    case FilterIndex::VX_DEPART:
      return encodeScaled(entry.min_value, 0.0, 0.0315, bits);
    case FilterIndex::SIZE:
      return encodeScaled(entry.min_value, 0.0, 0.025, bits);
    case FilterIndex::PROB_EXISTS:
      return static_cast<uint32_t>(probExistsToDiscrete(entry.min_value));
    case FilterIndex::POS_Y:
      return encodeScaled(entry.min_value, -409.5, 0.2, bits);
    case FilterIndex::POS_X:
      return encodeScaled(entry.min_value, -500.0, 0.2, bits);
    default:
      return 0u;
  }
}

uint32_t EncodeRawMax(const FilterCfgEntry & entry)
{
  const uint8_t bits = bitLengthForIndex(entry.index);
  switch (entry.index) {
    case FilterIndex::NOF_OBJ:
    case FilterIndex::CLASS:
      return encodeScaled(entry.max_value, 0.0, 1.0, bits);
    case FilterIndex::DISTANCE:
    case FilterIndex::LIFETIME:
      return encodeScaled(entry.max_value, 0.0, 0.1, bits);
    case FilterIndex::AZIMUTH:
    case FilterIndex::RCS:
      return encodeScaled(entry.max_value, -50.0, 0.025, bits);
    case FilterIndex::VREL_ONCOME:
    case FilterIndex::VREL_DEPART:
    case FilterIndex::VY_RIGHT_LEFT:
    case FilterIndex::VX_ONCOME:
    case FilterIndex::VY_LEFT_RIGHT:
    case FilterIndex::VX_DEPART:
      return encodeScaled(entry.max_value, 0.0, 0.0315, bits);
    case FilterIndex::SIZE:
      return encodeScaled(entry.max_value, 0.0, 0.025, bits);
    case FilterIndex::PROB_EXISTS:
      return static_cast<uint32_t>(probExistsToDiscrete(entry.max_value));
    case FilterIndex::POS_Y:
      return encodeScaled(entry.max_value, -409.5, 0.2, bits);
    case FilterIndex::POS_X:
      return encodeScaled(entry.max_value, -500.0, 0.2, bits);
    default:
      return 0u;
  }
}

double DecodeRawMin(const FilterIndex index, const uint32_t raw)
{
  const uint8_t bits = bitLengthForIndex(index);
  const uint32_t masked = clampRaw(raw, bits);
  switch (index) {
    case FilterIndex::NOF_OBJ:
    case FilterIndex::CLASS:
      return 0.0;
    case FilterIndex::DISTANCE:
    case FilterIndex::LIFETIME:
      return decodeScaled(masked, 0.0, 0.1);
    case FilterIndex::AZIMUTH:
    case FilterIndex::RCS:
      return decodeScaled(masked, -50.0, 0.025);
    case FilterIndex::VREL_ONCOME:
    case FilterIndex::VREL_DEPART:
    case FilterIndex::VY_RIGHT_LEFT:
    case FilterIndex::VX_ONCOME:
    case FilterIndex::VY_LEFT_RIGHT:
    case FilterIndex::VX_DEPART:
      return decodeScaled(masked, 0.0, 0.0315);
    case FilterIndex::SIZE:
      return decodeScaled(masked, 0.0, 0.025);
    case FilterIndex::PROB_EXISTS:
      return discreteToProbExists(static_cast<uint8_t>(masked));
    case FilterIndex::POS_Y:
      return decodeScaled(masked, -409.5, 0.2);
    case FilterIndex::POS_X:
      return decodeScaled(masked, -500.0, 0.2);
    default:
      return 0.0;
  }
}

double DecodeRawMax(const FilterIndex index, const uint32_t raw)
{
  const uint8_t bits = bitLengthForIndex(index);
  const uint32_t masked = clampRaw(raw, bits);
  switch (index) {
    case FilterIndex::NOF_OBJ:
    case FilterIndex::CLASS:
      return static_cast<double>(masked);
    case FilterIndex::DISTANCE:
    case FilterIndex::LIFETIME:
      return decodeScaled(masked, 0.0, 0.1);
    case FilterIndex::AZIMUTH:
    case FilterIndex::RCS:
      return decodeScaled(masked, -50.0, 0.025);
    case FilterIndex::VREL_ONCOME:
    case FilterIndex::VREL_DEPART:
    case FilterIndex::VY_RIGHT_LEFT:
    case FilterIndex::VX_ONCOME:
    case FilterIndex::VY_LEFT_RIGHT:
    case FilterIndex::VX_DEPART:
      return decodeScaled(masked, 0.0, 0.0315);
    case FilterIndex::SIZE:
      return decodeScaled(masked, 0.0, 0.025);
    case FilterIndex::PROB_EXISTS:
      return discreteToProbExists(static_cast<uint8_t>(masked));
    case FilterIndex::POS_Y:
      return decodeScaled(masked, -409.5, 0.2);
    case FilterIndex::POS_X:
      return decodeScaled(masked, -500.0, 0.2);
    default:
      return 0.0;
  }
}

}  // namespace filter_signals
}  // namespace ars408
