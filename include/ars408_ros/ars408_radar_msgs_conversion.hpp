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

#ifndef ARS408_ROS__ARS408_RADAR_MSGS_CONVERSION_HPP_
#define ARS408_ROS__ARS408_RADAR_MSGS_CONVERSION_HPP_

#include "ars408_ros/ars408_commands.hpp"
#include "ars408_ros/ars408_object.hpp"
#include "radar_msgs/msg/radar_return.hpp"
#include "radar_msgs/msg/radar_track.hpp"

#include <cstdint>
#include <functional>

namespace ars408
{
namespace radar_msgs_conversion
{

struct TrackConversionOptions
{
  double fallback_size_x{1.8};
  double fallback_size_y{1.8};
  bool use_radar_reported_dimensions{true};
  bool inflate_covariance_by_existence_probability{true};
};

radar_msgs::msg::RadarTrack ToRadarTrack(
  const RadarObject & object, const TrackConversionOptions & options,
  const std::function<uint32_t(const Obj_3_Extended::ObjectClassProperty &)> & classification_fn);

radar_msgs::msg::RadarReturn ToRadarReturn(const RadarObject & object);

radar_msgs::msg::RadarReturn ToRadarReturn(const RadarCluster & cluster);

}  // namespace radar_msgs_conversion
}  // namespace ars408

#endif  // ARS408_ROS__ARS408_RADAR_MSGS_CONVERSION_HPP_
