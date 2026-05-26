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

#include "ars408_ros/ars408_radar_msgs_conversion.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>

namespace ars408
{
namespace radar_msgs_conversion
{
namespace
{
constexpr double kDefaultVariance = 1.0;

void SetDiagonalCovariance6(std::array<float, 6> & cov, const float var_x, const float var_y)
{
  cov.fill(0.f);
  cov.at(0) = var_x;
  cov.at(3) = var_y;
  cov.at(5) = var_y;
}

float VarianceFromRms(const float rms)
{
  return rms > 0.f ? rms * rms : static_cast<float>(kDefaultVariance);
}

float ExistenceInflationScale(
  const RadarObject & object, const TrackConversionOptions & options)
{
  if (!options.inflate_covariance_by_existence_probability || !object.has_quality) {
    return 1.f;
  }
  if (object.probability_existence <= 0.f) {
    return 100.f;
  }
  const float inv_p = 1.f / object.probability_existence;
  return inv_p * inv_p;
}

}  // namespace

radar_msgs::msg::RadarTrack ToRadarTrack(
  const RadarObject & object, const TrackConversionOptions & options,
  const std::function<uint32_t(const Obj_3_Extended::ObjectClassProperty &)> & classification_fn)
{
  radar_msgs::msg::RadarTrack track;

  track.position.x = object.distance_long_x;
  track.position.y = object.distance_lat_y;
  track.position.z = 0.0;

  track.velocity.x = object.speed_long_x;
  track.velocity.y = object.speed_lat_y;
  track.velocity.z = 0.0;

  track.acceleration.x = object.rel_acceleration_long_x;
  track.acceleration.y = object.rel_acceleration_lat_y;
  track.acceleration.z = 0.0;

  const bool use_reported_size = options.use_radar_reported_dimensions &&
    object.length > 0.f && object.width > 0.f;
  track.size.x = use_reported_size ? object.length : static_cast<float>(options.fallback_size_x);
  track.size.y = use_reported_size ? object.width : static_cast<float>(options.fallback_size_y);
  track.size.z = 1.0f;

  if (classification_fn) {
    track.classification = classification_fn(object.object_class);
  }

  const float inflation = ExistenceInflationScale(object, options);

  if (object.has_quality) {
    const float pos_var_x = VarianceFromRms(object.dist_long_rms_m) * inflation;
    const float pos_var_y = VarianceFromRms(object.dist_lat_rms_m) * inflation;
    SetDiagonalCovariance6(track.position_covariance, pos_var_x, pos_var_y);

    const float vel_var_x = VarianceFromRms(object.vrel_long_rms_mps) * inflation;
    const float vel_var_y = VarianceFromRms(object.vrel_lat_rms_mps) * inflation;
    SetDiagonalCovariance6(track.velocity_covariance, vel_var_x, vel_var_y);

    const float acc_var_x = VarianceFromRms(object.arel_long_rms_mps2) * inflation;
    const float acc_var_y = VarianceFromRms(object.arel_lat_rms_mps2) * inflation;
    SetDiagonalCovariance6(track.acceleration_covariance, acc_var_x, acc_var_y);

    const float orient_rad = object.orientation_rms_deg * static_cast<float>(M_PI) / 180.f;
    const float orient_var = orient_rad * orient_rad;
    const float size_var_x = track.size.x * track.size.x * orient_var;
    const float size_var_y = track.size.y * track.size.y * orient_var;
    SetDiagonalCovariance6(track.size_covariance, size_var_x, size_var_y);
  } else {
    SetDiagonalCovariance6(
      track.position_covariance, static_cast<float>(kDefaultVariance),
      static_cast<float>(kDefaultVariance));
    SetDiagonalCovariance6(
      track.velocity_covariance, 0.1f * static_cast<float>(inflation),
      0.1f * static_cast<float>(inflation));
    SetDiagonalCovariance6(track.acceleration_covariance, static_cast<float>(kDefaultVariance),
      static_cast<float>(kDefaultVariance));
    SetDiagonalCovariance6(track.size_covariance, 0.04f, 0.04f);
  }

  return track;
}

radar_msgs::msg::RadarReturn ToRadarReturn(const RadarObject & object)
{
  radar_msgs::msg::RadarReturn radar_return;
  const float range_sq = object.distance_long_x * object.distance_long_x +
    object.distance_lat_y * object.distance_lat_y;
  radar_return.range = std::sqrt(range_sq);
  radar_return.azimuth = std::atan2(object.distance_lat_y, object.distance_long_x);
  radar_return.elevation = 0.0f;
  radar_return.amplitude = static_cast<float>(object.rcs);

  if (radar_return.range > 1e-3f) {
    radar_return.doppler_velocity =
      (object.speed_long_x * object.distance_long_x +
      object.speed_lat_y * object.distance_lat_y) / radar_return.range;
  } else {
    radar_return.doppler_velocity = object.speed_long_x;
  }

  return radar_return;
}

}  // namespace radar_msgs_conversion
}  // namespace ars408
