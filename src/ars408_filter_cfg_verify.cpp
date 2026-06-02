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

#include "ars408_ros/ars408_filter_cfg_verify.hpp"

#include <cmath>
#include <sstream>

namespace ars408
{
namespace filter_cfg_verify
{
namespace
{

bool valuesClose(const double a, const double b)
{
  return std::abs(a - b) <= kFilterValueTolerance;
}

const filter_signals::FilterStateCfg * findState(
  const std::vector<filter_signals::FilterStateCfg> & states,
  const filter_signals::FilterCfgEntry & expected)
{
  for (const auto & state : states) {
    if (
      state.index == expected.index && state.for_objects == expected.for_objects)
    {
      return &state;
    }
  }
  return nullptr;
}

}  // namespace

bool FilterStateMatchesEntry(
  const filter_signals::FilterStateCfg & state,
  const filter_signals::FilterCfgEntry & expected, std::string * out_detail)
{
  if (state.index != expected.index) {
    if (out_detail) {
      std::ostringstream oss;
      oss << "index mismatch for " << filter_signals::FilterIndexToString(expected.index);
      *out_detail = oss.str();
    }
    return false;
  }

  if (state.for_objects != expected.for_objects) {
    if (out_detail) {
      *out_detail = std::string("for_objects mismatch on ") +
        filter_signals::FilterIndexToString(expected.index);
    }
    return false;
  }

  if (state.active != expected.active) {
    if (out_detail) {
      *out_detail = std::string("active mismatch on ") +
        filter_signals::FilterIndexToString(expected.index);
    }
    return false;
  }

  if (!expected.active) {
    return true;
  }

  if (!filter_signals::FilterIndexIgnoresMin(expected.index)) {
    if (!valuesClose(state.min_value, expected.min_value)) {
      if (out_detail) {
        std::ostringstream oss;
        oss << filter_signals::FilterIndexToString(expected.index) << " min expected "
            << expected.min_value << " got " << state.min_value;
        *out_detail = oss.str();
      }
      return false;
    }
  }

  if (!valuesClose(state.max_value, expected.max_value)) {
    if (out_detail) {
      std::ostringstream oss;
      oss << filter_signals::FilterIndexToString(expected.index) << " max expected "
          << expected.max_value << " got " << state.max_value;
      *out_detail = oss.str();
    }
    return false;
  }

  return true;
}

bool AllFilterEntriesMatch(
  const std::vector<filter_signals::FilterCfgEntry> & expected,
  const std::vector<filter_signals::FilterStateCfg> & reported_states,
  std::string * out_detail)
{
  if (expected.empty()) {
    return true;
  }

  for (const auto & entry : expected) {
    const filter_signals::FilterStateCfg * state = findState(reported_states, entry);
    if (!state) {
      if (out_detail) {
        *out_detail = std::string("no FilterState_Cfg (0x204) for ") +
          filter_signals::FilterIndexToString(entry.index);
      }
      return false;
    }

    std::string detail;
    if (!FilterStateMatchesEntry(*state, entry, out_detail ? &detail : nullptr)) {
      if (out_detail) {
        *out_detail = detail;
      }
      return false;
    }
  }

  return true;
}

}  // namespace filter_cfg_verify
}  // namespace ars408
