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

#ifndef ARS408_ROS__ARS408_CAN_PARSER_HPP_
#define ARS408_ROS__ARS408_CAN_PARSER_HPP_

#include "ars408_ros/ars408_commands.hpp"
#include "ars408_ros/ars408_filter_signals.hpp"
#include "ars408_ros/ars408_object.hpp"

#include <array>
#include <cstdint>

namespace ars408
{
namespace can_parser
{

/// Returns true when the frame carries at least @p min_length payload bytes.
bool HasMinimumDlc(uint8_t dlc, uint8_t min_length);

/// Sensor ID encoded in CAN ID bits 4–6 (Standard Radar Interface §4.1).
uint8_t SensorIdFromCanId(uint32_t can_id);

void ParseRadarState(const std::array<uint8_t, 8> & in_can_data, RadarState & out_state);

void ParseObjectListStatus(const std::array<uint8_t, 8> & in_can_data, Obj_0_Status & out_status);

RadarObject ParseObjectGeneral(
  const std::array<uint8_t, 8> & in_can_data, uint16_t measurement_counter);

Obj_2_Quality ParseObjectQuality(const std::array<uint8_t, 8> & in_can_data);

Obj_3_Extended ParseObjectExtended(const std::array<uint8_t, 8> & in_can_data);

struct VersionId
{
  uint8_t major{0};
  uint8_t minor{0};
  uint8_t patch{0};
  bool extended_range{false};
  bool country_code_restricted{false};
};

VersionId ParseVersionId(const std::array<uint8_t, 8> & in_can_data);

filter_signals::FilterStateHeader ParseFilterStateHeader(
  const std::array<uint8_t, 8> & in_can_data);

filter_signals::FilterStateCfg ParseFilterStateCfg(const std::array<uint8_t, 8> & in_can_data);

struct Cluster0Status
{
  uint8_t nof_clusters_near{0};
  uint8_t nof_clusters_far{0};
  uint16_t meas_counter{0};
  uint8_t interface_version{0};
};

void ParseClusterStatus(const std::array<uint8_t, 8> & in_can_data, Cluster0Status & out_status);

RadarCluster ParseClusterGeneral(
  const std::array<uint8_t, 8> & in_can_data, uint16_t measurement_counter);

void ParseClusterQuality(const std::array<uint8_t, 8> & in_can_data, RadarCluster & out_cluster);

}  // namespace can_parser
}  // namespace ars408

#endif  // ARS408_ROS__ARS408_CAN_PARSER_HPP_
