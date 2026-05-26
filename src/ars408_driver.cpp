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

#include "ars408_ros/ars408_driver.hpp"

#include "ars408_ros/ars408_can_parser.hpp"
#include "ars408_ros/ars408_constants.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <unordered_map>
#include <utility>

namespace ars408
{

void Ars408Driver::AddDetectedObject(ars408::RadarObject in_object)
{
  if (in_object.sequence_id == current_objects_status_.MeasurementCounter) {
    radar_objects_.insert(std::pair<uint8_t, ars408::RadarObject>(in_object.id, in_object));
    updated_objects_general_++;
  }
}

void Ars408Driver::ClearRadarObjects()
{
  radar_objects_.clear();
  updated_objects_ext_ = 0;
  updated_objects_general_ = 0;
  updated_objects_quality_ = 0;
}

void Ars408Driver::CallDetectedObjectsCallback(
  std::unordered_map<uint8_t, ars408::RadarObject> & in_detected_objects,
  const rclcpp::Time & stamp)
{
  if (detected_objects_callback_) {
    detected_objects_callback_(in_detected_objects, stamp);
  }
}

void Ars408Driver::UpdateObjectQuality(
  uint8_t in_object_id, const ars408::Obj_2_Quality & in_object_quality)
{
  auto object_iterator = radar_objects_.find(in_object_id);
  if (object_iterator != radar_objects_.end()) {
    ars408::RadarObject object_found = object_iterator->second;
    object_found.probability_existence = in_object_quality.ExistenceProbability;
    radar_objects_.at(object_iterator->first) = object_found;
    updated_objects_quality_++;
  }
}

void Ars408Driver::UpdateObjectExtInfo(
  uint8_t in_object_id, const ars408::Obj_3_Extended & in_object_ext_info)
{
  auto object_iterator = radar_objects_.find(in_object_id);
  if (object_iterator != radar_objects_.end()) {
    ars408::RadarObject object_found = object_iterator->second;
    object_found.object_class = in_object_ext_info.ObjectClass;
    object_found.length = in_object_ext_info.Length;
    object_found.width = in_object_ext_info.Width;
    object_found.orientation_angle = in_object_ext_info.OrientationAngle;
    object_found.rel_acceleration_long_x = in_object_ext_info.RelativeLongitudinalAccelerationX;
    object_found.rel_acceleration_lat_y = in_object_ext_info.RelativeLateralAccelerationY;
    radar_objects_.at(object_iterator->first) = object_found;
    updated_objects_ext_++;
  }
}

bool Ars408Driver::DetectedObjectsReady()
{
  if (!valid_radar_state_) {
    return false;
  }
  if (updated_objects_general_ != current_objects_status_.NumberOfObjects) {
    return false;
  }
  if (
    current_radar_state_.SendQuality &&
    updated_objects_quality_ != current_objects_status_.NumberOfObjects)
  {
    return false;
  }
  if (
    current_radar_state_.SendExtInfo &&
    updated_objects_ext_ != current_objects_status_.NumberOfObjects)
  {
    return false;
  }
  return true;
}

bool Ars408Driver::GetCurrentRadarState(ars408::RadarState & out_current_state)
{
  if (valid_radar_state_) {
    out_current_state = current_radar_state_;
    return true;
  }
  return false;
}

void Ars408Driver::RegisterDetectedObjectsCallback(
  std::function<void(
    const std::unordered_map<uint8_t, ars408::RadarObject> &,
    const rclcpp::Time &
  )> objects_callback,
  bool sequential_publish)
{
  detected_objects_callback_ = objects_callback;
  sequential_publish_ = sequential_publish;
}

void Ars408Driver::ParseRadarState(const std::array<uint8_t, 8> & in_can_data)
{
  can_parser::ParseRadarState(in_can_data, current_radar_state_);
  valid_radar_state_ = true;
}

void Ars408Driver::ParseObject0_Status(const std::array<uint8_t, 8> & in_can_data)
{
  can_parser::ParseObjectListStatus(in_can_data, current_objects_status_);
}

ars408::RadarObject Ars408Driver::ParseObject1_General(const std::array<uint8_t, 8> & in_can_data)
{
  return can_parser::ParseObjectGeneral(in_can_data, current_objects_status_.MeasurementCounter);
}

ars408::Obj_2_Quality Ars408Driver::ParseObject2_Quality(const std::array<uint8_t, 8> & in_can_data)
{
  return can_parser::ParseObjectQuality(in_can_data);
}

ars408::Obj_3_Extended Ars408Driver::ParseObject3_Extended(
  const std::array<uint8_t, 8> & in_can_data)
{
  return can_parser::ParseObjectExtended(in_can_data);
}

std::string Ars408Driver::Parse(
  const uint32_t & can_id, const std::array<uint8_t, 8> & in_can_data,
  const uint8_t & in_data_length, const rclcpp::Time & stamp)
{
  if (can_parser::SensorIdFromCanId(can_id) != radar_id_) {
    return "";
  }

  switch (can_id) {
    case ars408::RADAR_STATE_00:
    case ars408::RADAR_STATE_01:
    case ars408::RADAR_STATE_02:
    case ars408::RADAR_STATE_03:
    case ars408::RADAR_STATE_04:
    case ars408::RADAR_STATE_05:
    case ars408::RADAR_STATE_06:
    case ars408::RADAR_STATE_07:
      if (can_parser::HasMinimumDlc(in_data_length, ars408::RADAR_STATE_BYTES)) {
        ParseRadarState(in_can_data);
      }
      break;
    case ars408::OBJ_STATUS_00:
    case ars408::OBJ_STATUS_01:
    case ars408::OBJ_STATUS_02:
    case ars408::OBJ_STATUS_03:
    case ars408::OBJ_STATUS_04:
    case ars408::OBJ_STATUS_05:
    case ars408::OBJ_STATUS_06:
    case ars408::OBJ_STATUS_07:
      if (can_parser::HasMinimumDlc(in_data_length, ars408::OBJ_STATUS_BYTES)) {
        if (!sequential_publish_ && DetectedObjectsReady()) {
          CallDetectedObjectsCallback(radar_objects_, stamp);
        }
        ParseObject0_Status(in_can_data);
        ClearRadarObjects();
      }
      break;
    case ars408::OBJ_GENERAL_00:
    case ars408::OBJ_GENERAL_01:
    case ars408::OBJ_GENERAL_02:
    case ars408::OBJ_GENERAL_03:
    case ars408::OBJ_GENERAL_04:
    case ars408::OBJ_GENERAL_05:
    case ars408::OBJ_GENERAL_06:
    case ars408::OBJ_GENERAL_07:
      if (can_parser::HasMinimumDlc(in_data_length, ars408::OBJ_GENERAL_BYTES)) {
        AddDetectedObject(ParseObject1_General(in_can_data));
      }
      break;
    case ars408::OBJ_QUALITY_00:
    case ars408::OBJ_QUALITY_01:
    case ars408::OBJ_QUALITY_02:
    case ars408::OBJ_QUALITY_03:
    case ars408::OBJ_QUALITY_04:
    case ars408::OBJ_QUALITY_05:
    case ars408::OBJ_QUALITY_06:
    case ars408::OBJ_QUALITY_07:
      if (can_parser::HasMinimumDlc(in_data_length, ars408::OBJ_QUALITY_BYTES)) {
        const ars408::Obj_2_Quality object_quality = ParseObject2_Quality(in_can_data);
        UpdateObjectQuality(object_quality.Id, object_quality);
      }
      break;
    case ars408::OBJ_EXTENDED_00:
    case ars408::OBJ_EXTENDED_01:
    case ars408::OBJ_EXTENDED_02:
    case ars408::OBJ_EXTENDED_03:
    case ars408::OBJ_EXTENDED_04:
    case ars408::OBJ_EXTENDED_05:
    case ars408::OBJ_EXTENDED_06:
    case ars408::OBJ_EXTENDED_07:
      if (can_parser::HasMinimumDlc(in_data_length, ars408::OBJ_EXTENDED_BYTES)) {
        const ars408::Obj_3_Extended object_ext_info = ParseObject3_Extended(in_can_data);
        UpdateObjectExtInfo(object_ext_info.Id, object_ext_info);
      }
      break;
    default:
      break;
  }

  if (sequential_publish_ && DetectedObjectsReady()) {
    CallDetectedObjectsCallback(radar_objects_, stamp);
  }

  return "";
}
}  // namespace ars408
