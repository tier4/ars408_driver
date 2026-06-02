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

#include "ars408_ros/ars408_radar_msgs_conversion.hpp"

#include <gtest/gtest.h>

#include <functional>

TEST(Ars408RadarMsgsConversion, UsesReportedDimensions)
{
  ars408::RadarObject object;
  object.length = 4.2f;
  object.width = 1.6f;
  object.distance_long_x = 10.f;
  object.distance_lat_y = 1.f;
  object.speed_long_x = 5.f;
  object.speed_lat_y = 0.f;
  object.rcs = -10.f;

  ars408::radar_msgs_conversion::TrackConversionOptions options;
  options.use_radar_reported_dimensions = true;

  const std::function<uint32_t(const ars408::Obj_3_Extended::ObjectClassProperty &)> no_class;
  const auto track = ars408::radar_msgs_conversion::ToRadarTrack(object, options, no_class);
  EXPECT_NEAR(track.size.x, 4.2f, 1e-3f);
  EXPECT_NEAR(track.size.y, 1.6f, 1e-3f);
}

TEST(Ars408RadarMsgsConversion, RadarReturnAmplitudeAndDoppler)
{
  ars408::RadarObject object;
  object.distance_long_x = 10.f;
  object.distance_lat_y = 0.f;
  object.speed_long_x = 5.f;
  object.speed_lat_y = 0.f;
  object.rcs = 12.5f;

  const auto ret = ars408::radar_msgs_conversion::ToRadarReturn(object);
  EXPECT_NEAR(ret.amplitude, 12.5f, 1e-3f);
  EXPECT_NEAR(ret.doppler_velocity, 5.f, 1e-3f);
}

TEST(Ars408RadarMsgsConversion, QualityMapsToCovariance)
{
  ars408::RadarObject object;
  object.has_quality = true;
  object.dist_long_rms_m = 0.1f;
  object.dist_lat_rms_m = 0.2f;
  object.vrel_long_rms_mps = 0.3f;
  object.vrel_lat_rms_mps = 0.4f;
  object.probability_existence = 1.f;
  object.length = 2.f;
  object.width = 1.f;

  ars408::radar_msgs_conversion::TrackConversionOptions options;
  options.inflate_covariance_by_existence_probability = false;

  const std::function<uint32_t(const ars408::Obj_3_Extended::ObjectClassProperty &)> no_class;
  const auto track = ars408::radar_msgs_conversion::ToRadarTrack(object, options, no_class);
  EXPECT_NEAR(track.position_covariance[0], 0.01f, 1e-4f);
  EXPECT_NEAR(track.position_covariance[3], 0.04f, 1e-4f);
  EXPECT_NEAR(track.velocity_covariance[0], 0.09f, 1e-4f);
}
