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

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <unordered_map>
#include <utility>

namespace ars408
{

void Ars408Driver::AddDetectedObject(ars408::RadarObject in_object, uint32_t radar_id)
{
  if (radar_id >= RADAR_CONNECTIONS_MAX) {
    throw std::runtime_error("radar_id out of range");
  }
  // check if this object belongs to the current registered sequence before registering it.
  if (in_object.sequence_id == current_objects_status_[radar_id].MeasurementCounter) {
    radar_objects_[radar_id].insert(std::pair<uint8_t, ars408::RadarObject>(in_object.id, in_object));
    updated_objects_general_[radar_id]++;
  }
}
void Ars408Driver::ClearRadarObjects(uint32_t radar_id)
{
  if (radar_id >= RADAR_CONNECTIONS_MAX) {
    throw std::runtime_error("radar_id out of range");
  }

  radar_objects_[radar_id].clear();
  updated_objects_ext_[radar_id] = 0;
  updated_objects_general_[radar_id] = 0;
  updated_objects_quality_[radar_id] = 0;
}

void Ars408Driver::CallDetectedObjectsCallback(
  std::unordered_map<uint8_t, ars408::RadarObject> & in_detected_objects,
  uint32_t radar_id, const rclcpp::Time & stamp)
{
  if (radar_id >= RADAR_CONNECTIONS_MAX) {
    throw std::runtime_error("radar_id out of range");
  }

  if (detected_objects_callback_) {  // check if callback was registered
    detected_objects_callback_(in_detected_objects, radar_id, stamp);
  }
}

void Ars408Driver::UpdateObjectQuality(
  uint8_t in_object_id, const ars408::Obj_2_Quality & in_object_quality, uint32_t radar_id)
{
  if (radar_id >= RADAR_CONNECTIONS_MAX) {
    throw std::runtime_error("radar_id out of range");
  }

  std::unordered_map<uint8_t, ars408::RadarObject>::const_iterator object_iterator;
  object_iterator = radar_objects_[radar_id].find(in_object_id);

  if (object_iterator != radar_objects_[radar_id].end()) {
    ars408::RadarObject object_found = object_iterator->second;
    object_found.probability_existence = in_object_quality.ExistenceProbability;

    radar_objects_[radar_id].at(object_iterator->first) = object_found;
    updated_objects_quality_[radar_id]++;
  }
}

void Ars408Driver::UpdateObjectExtInfo(
  uint8_t in_object_id, const ars408::Obj_3_Extended & in_object_ext_info, uint32_t radar_id)
{
  if (radar_id >= RADAR_CONNECTIONS_MAX) {
    throw std::runtime_error("radar_id out of range");
  }

  std::unordered_map<uint8_t, ars408::RadarObject>::const_iterator object_iterator;
  object_iterator = radar_objects_[radar_id].find(in_object_id);

  if (object_iterator != radar_objects_[radar_id].end()) {
    ars408::RadarObject object_found = object_iterator->second;
    object_found.object_class = in_object_ext_info.ObjectClass;
    object_found.length = in_object_ext_info.Length;
    object_found.width = in_object_ext_info.Width;
    object_found.orientation_angle = in_object_ext_info.OrientationAngle;
    object_found.rel_acceleration_long_x = in_object_ext_info.RelativeLongitudinalAccelerationX;
    object_found.rel_acceleration_lat_y = in_object_ext_info.RelativeLateralAccelerationY;

    radar_objects_[radar_id].at(object_iterator->first) = object_found;
    updated_objects_ext_[radar_id]++;
  }
}

bool Ars408Driver::DetectedObjectsReady(uint32_t radar_id)
{
  if (radar_id >= RADAR_CONNECTIONS_MAX) {
    throw std::runtime_error("radar_id out of range");
  }

  bool ready = true;
  if (valid_radar_state_[radar_id]) {
    if (updated_objects_general_[radar_id] == current_objects_status_[radar_id].NumberOfObjects) {
      if (
        current_radar_state_[radar_id].SendQuality &&
        (updated_objects_quality_[radar_id] != current_objects_status_[radar_id].NumberOfObjects)) {
        ready = false;
      }
      if (
        current_radar_state_[radar_id].SendExtInfo &&
        (updated_objects_ext_[radar_id] != current_objects_status_[radar_id].NumberOfObjects)) {
        ready = false;
      }
    } else {
      ready = false;
    }
  } else {
    ready = false;
  }
  return ready;
}

bool Ars408Driver::GetCurrentRadarState(ars408::RadarState & out_current_state, uint32_t radar_id)
{
  if (radar_id >= RADAR_CONNECTIONS_MAX) {
    throw std::runtime_error("radar_id out of range");
  }

  if (valid_radar_state_[radar_id]) {
    out_current_state = current_radar_state_[radar_id];
    return true;
  }
  return false;
}

void Ars408Driver::RegisterDetectedObjectsCallback(
  std::function<void(
    const std::unordered_map<uint8_t, ars408::RadarObject> &,
    uint8_t,
    const rclcpp::Time &
    )> objects_callback,
  const std::array<bool, 8> & sequential_publish)
{
  detected_objects_callback_ = objects_callback;
  std::copy(sequential_publish.begin(),
            sequential_publish.end(),
            sequential_publish_.begin());
}

void Ars408Driver::ParseRadarState(const std::array<uint8_t, 8> & in_can_data, uint32_t radar_id)
{
  if (radar_id >= RADAR_CONNECTIONS_MAX) {
    throw std::runtime_error("radar_id out of range");
  }

  current_radar_state_[radar_id].NvmWriteStatus = ((in_can_data[0] & 0x80u) >> 7u);
  current_radar_state_[radar_id].NvmReadStatus = ((in_can_data[0] & 0x40u) >> 6u);

  uint16_t distance =
    ((((in_can_data[1] & 0xFFu) << 2u) & 0xFFFFu) + ((in_can_data[2] & 0xC0u) >> 6u)) << 1u;
  current_radar_state_[radar_id].MaxDistance = distance;
  current_radar_state_[radar_id].PersistentError = (in_can_data[2] & 0x20u) >> 5u;
  current_radar_state_[radar_id].Interference = (in_can_data[2] & 0x10u) >> 4u;
  current_radar_state_[radar_id].TemperatureError = (in_can_data[2] & 0x08u) >> 3u;
  current_radar_state_[radar_id].TemporaryError = (in_can_data[2] & 0x04u) >> 2u;
  current_radar_state_[radar_id].VoltageError = (in_can_data[2] & 0x02u) >> 1u;
  current_radar_state_[radar_id].SensorID = (in_can_data[4] & 0x07u);
  current_radar_state_[radar_id].SortingMode =
    ars408::RadarState::SortingConfig((in_can_data[4] & 0x70u) >> 4u);
  current_radar_state_[radar_id].PowerMode =
    ars408::RadarState::PowerConfig((in_can_data[3] << 1u) + ((in_can_data[4] & 0x80u) >> 7u));
  current_radar_state_[radar_id].EgoMotionRxStatus =
    ars408::RadarState::MotionRx((in_can_data[5] & 0xC0u) >> 6u);
  current_radar_state_[radar_id].SendExtInfo = ars408::RadarState::Config((in_can_data[5] & 0x20u) >> 5u);
  current_radar_state_[radar_id].SendQuality = ars408::RadarState::Config((in_can_data[5] & 0x10u) >> 4u);
  current_radar_state_[radar_id].OutputType =
    ars408::RadarState::OutputTypeConfig((in_can_data[5] & 0x0Cu) >> 2u);
  current_radar_state_[radar_id].CtrlRelay = ars408::RadarState::Config((in_can_data[5] & 0x02u) >> 1u);
  current_radar_state_[radar_id].Rcs_Threshold =
    ars408::RadarState::Rcs_ThresholdConfig((in_can_data[5] & 0x1Cu) >> 2u);
  valid_radar_state_[radar_id] = true;

}

void Ars408Driver::ParseObject0_Status(const std::array<uint8_t, 8> & in_can_data, uint32_t radar_id)
{
  if (radar_id >= RADAR_CONNECTIONS_MAX) {
    throw std::runtime_error("radar_id out of range");
  }

  current_objects_status_[radar_id].NumberOfObjects = in_can_data[0] & 0xFFu;
  current_objects_status_[radar_id].MeasurementCounter = (in_can_data[1] << 8u) + (in_can_data[0]);
  current_objects_status_[radar_id].InterfaceVersion = (in_can_data[3] & 0xF0u) >> 4u;

}

ars408::RadarObject Ars408Driver::ParseObject1_General(const std::array<uint8_t, 8> & in_can_data, uint32_t radar_id)
{
  if (radar_id >= RADAR_CONNECTIONS_MAX) {
    throw std::runtime_error("radar_id out of range");
  }

  ars408::RadarObject current_object;
  current_object.sequence_id = current_objects_status_[radar_id].MeasurementCounter;
  current_object.id = in_can_data[0];
  current_object.dynamic_property = ars408::Obj_1_General::DynamicProperty(in_can_data[6] & 0x07u);
  current_object.rcs = (in_can_data[7] * 0.5) - 64.0;

  uint16_t dist_x_tmp = (in_can_data[1] << 5u) + ((in_can_data[2] & 0xF8u) >> 3u);
  current_object.distance_long_x = dist_x_tmp * 0.2f - 500.0f;

  uint16_t dist_y_tmp = ((in_can_data[2] & 0x07u) << 8u) + (in_can_data[3]);
  current_object.distance_lat_y = dist_y_tmp * 0.2f - 204.6f;

  uint16_t speed_x_tmp = (in_can_data[4] << 2u) + ((in_can_data[5] & 0xC0u) >> 6u);
  current_object.speed_long_x = (speed_x_tmp * 0.25f) - 128.0f;

  uint16_t speed_y_ymp = ((in_can_data[5] & 0x3Fu) << 3u) + ((in_can_data[6] & 0xE0u) >> 5u);
  current_object.speed_lat_y = (speed_y_ymp * 0.25f - 64.0f);
  return current_object;
}

ars408::Obj_2_Quality Ars408Driver::ParseObject2_Quality(const std::array<uint8_t, 8> & in_can_data)
{
  ars408::Obj_2_Quality obj_quality;
  obj_quality.Id = in_can_data[0];
  uint8_t prob_tmp = (in_can_data[6] & 0x1Cu) >> 2u;
  switch (prob_tmp) {
    case 0x00u:
      obj_quality.ExistenceProbability = 0;
      break;
    case 0x01u:
      obj_quality.ExistenceProbability = 0.25;
      break;
    case 0x02u:
      obj_quality.ExistenceProbability = 0.5;
      break;
    case 0x03u:
      obj_quality.ExistenceProbability = 0.75;
      break;
    case 0x04u:
      obj_quality.ExistenceProbability = 0.9;
      break;
    case 0x05u:
      obj_quality.ExistenceProbability = 0.99;
      break;
    case 0x06u:
      obj_quality.ExistenceProbability = 0.999;
      break;
    case 0x07u:
      obj_quality.ExistenceProbability = 1;
      break;
  }
  return obj_quality;
}

ars408::Obj_3_Extended Ars408Driver::ParseObject3_Extended(
  const std::array<uint8_t, 8> & in_can_data)
{
  ars408::Obj_3_Extended obj_extended;
  obj_extended.Id = in_can_data[0];
  uint16_t tmp_rel_acc_x = (in_can_data[1] << 3u) + ((in_can_data[2] & 0xE0u) >> 5u);
  obj_extended.RelativeLongitudinalAccelerationX = tmp_rel_acc_x * 0.01 - 10.f;

  uint16_t tmp_rel_acc_y = ((in_can_data[2] & 0x1Fu) << 4u) + ((in_can_data[3] & 0xF0u) >> 4u);
  obj_extended.RelativeLateralAccelerationY = tmp_rel_acc_y * 0.01 - 2.5f;

  uint8_t tmp_class = in_can_data[3] & 0x07u;
  switch (tmp_class) {
    case 0x00u:
      obj_extended.ObjectClass = ars408::Obj_3_Extended::ObjectClassProperty::POINT;
      break;
    case 0x01u:
      obj_extended.ObjectClass = ars408::Obj_3_Extended::ObjectClassProperty::CAR;
      break;
    case 0x02u:
      obj_extended.ObjectClass = ars408::Obj_3_Extended::ObjectClassProperty::TRUCK;
      break;
    case 0x04u:
      obj_extended.ObjectClass = ars408::Obj_3_Extended::ObjectClassProperty::MOTORCYCLE;
      break;
    case 0x05u:
      obj_extended.ObjectClass = ars408::Obj_3_Extended::ObjectClassProperty::BICYCLE;
      break;
    case 0x06u:
      obj_extended.ObjectClass = ars408::Obj_3_Extended::ObjectClassProperty::WIDE;
      break;
    default:
    case 0x07u:
    case 0x03u:
      obj_extended.ObjectClass = ars408::Obj_3_Extended::ObjectClassProperty::RESERVED_01;
      break;
  }

  uint16_t tmp_angle = (in_can_data[4] << 2u) + ((in_can_data[5] & 0xC0u) >> 6u);
  obj_extended.OrientationAngle = tmp_angle * 0.4f - 180.f;

  obj_extended.Length = in_can_data[6] * 0.2f;
  obj_extended.Width = in_can_data[7] * 0.2f;
  return obj_extended;
}

std::string Ars408Driver::Parse(
  const uint32_t & can_id, const std::array<uint8_t, 8> & in_can_data,
  const uint8_t & in_data_length, const rclcpp::Time & stamp)
{
  /* Derive Radar ID from CAN ID */
  uint32_t radar_id = (can_id & 0x00000070u) / 0x10u;

  switch (can_id) {
    case ars408::RADAR_STATE_00:  /// 0x201 the current configuration and sensor state in message
    case ars408::RADAR_STATE_01:  /// 0x211 the current configuration and sensor state in message
    case ars408::RADAR_STATE_02:  /// 0x221 the current configuration and sensor state in message
    case ars408::RADAR_STATE_03:  /// 0x231 the current configuration and sensor state in message
    case ars408::RADAR_STATE_04:  /// 0x241 the current configuration and sensor state in message
    case ars408::RADAR_STATE_05:  /// 0x251 the current configuration and sensor state in message
    case ars408::RADAR_STATE_06:  /// 0x261 the current configuration and sensor state in message
    case ars408::RADAR_STATE_07:  /// 0x271 the current configuration and sensor state in message
      if (ars408::RADAR_STATE_BYTES == in_data_length) {
        ParseRadarState(in_can_data, radar_id);
      }
      break;
    case ars408::OBJ_STATUS_00:  /// 0x60A contains list header information,
    case ars408::OBJ_STATUS_01:  /// 0x61A contains list header information,
    case ars408::OBJ_STATUS_02:  /// 0x62A contains list header information,
    case ars408::OBJ_STATUS_03:  /// 0x63A contains list header information,
    case ars408::OBJ_STATUS_04:  /// 0x64A contains list header information,
    case ars408::OBJ_STATUS_05:  /// 0x65A contains list header information,
    case ars408::OBJ_STATUS_06:  /// 0x66A contains list header information,
    case ars408::OBJ_STATUS_07:  /// 0x67A contains list header information,
                              /// i.e. the number of objects that are sent afterwards
      if (ars408::OBJ_STATUS_BYTES == in_data_length) {
        if (!sequential_publish_[radar_id] && DetectedObjectsReady(radar_id)) {
          CallDetectedObjectsCallback(radar_objects_[radar_id], radar_id, stamp);
        }
        ParseObject0_Status(in_can_data, radar_id);
        ClearRadarObjects(radar_id);
      }
      break;
    case ars408::OBJ_GENERAL_00:  /// 0x60B contains the position and velocity of the objects
    case ars408::OBJ_GENERAL_01:  /// 0x61B contains the position and velocity of the objects
    case ars408::OBJ_GENERAL_02:  /// 0x62B contains the position and velocity of the objects
    case ars408::OBJ_GENERAL_03:  /// 0x63B contains the position and velocity of the objects
    case ars408::OBJ_GENERAL_04:  /// 0x64B contains the position and velocity of the objects
    case ars408::OBJ_GENERAL_05:  /// 0x65B contains the position and velocity of the objects
    case ars408::OBJ_GENERAL_06:  /// 0x66B contains the position and velocity of the objects
    case ars408::OBJ_GENERAL_07:  /// 0x67B contains the position and velocity of the objects
      if (ars408::OBJ_GENERAL_BYTES == in_data_length) {
        ars408::RadarObject object = ParseObject1_General(in_can_data, radar_id);
        AddDetectedObject(object, radar_id);
      }
      break;
    case ars408::OBJ_QUALITY_00:  /// 0x60C contains the quality information of the objects
    case ars408::OBJ_QUALITY_01:  /// 0x61C contains the quality information of the objects
    case ars408::OBJ_QUALITY_02:  /// 0x62C contains the quality information of the objects
    case ars408::OBJ_QUALITY_03:  /// 0x63C contains the quality information of the objects
    case ars408::OBJ_QUALITY_04:  /// 0x64C contains the quality information of the objects
    case ars408::OBJ_QUALITY_05:  /// 0x65C contains the quality information of the objects
    case ars408::OBJ_QUALITY_06:  /// 0x66C contains the quality information of the objects
    case ars408::OBJ_QUALITY_07:  /// 0x67C contains the quality information of the objects
      if (ars408::OBJ_QUALITY_BYTES == in_data_length) {
        ars408::Obj_2_Quality object_quality = ParseObject2_Quality(in_can_data);
        UpdateObjectQuality(object_quality.Id, object_quality, radar_id);
      }
      break;
    case ars408::OBJ_EXTENDED_00:  /// 0x60D contains the quality information of the objects
    case ars408::OBJ_EXTENDED_01:  /// 0x61D contains the quality information of the objects
    case ars408::OBJ_EXTENDED_02:  /// 0x62D contains the quality information of the objects
    case ars408::OBJ_EXTENDED_03:  /// 0x63D contains the quality information of the objects
    case ars408::OBJ_EXTENDED_04:  /// 0x64D contains the quality information of the objects
    case ars408::OBJ_EXTENDED_05:  /// 0x65D contains the quality information of the objects
    case ars408::OBJ_EXTENDED_06:  /// 0x66D contains the quality information of the objects
    case ars408::OBJ_EXTENDED_07:  /// 0x67D contains the quality information of the objects
      if (ars408::OBJ_EXTENDED_BYTES == in_data_length) {
        ars408::Obj_3_Extended object_ext_info = ParseObject3_Extended(in_can_data);
        UpdateObjectExtInfo(object_ext_info.Id, object_ext_info, radar_id);
      }
      break;
  }

  if (sequential_publish_[radar_id] && DetectedObjectsReady(radar_id)) {
    CallDetectedObjectsCallback(radar_objects_[radar_id], radar_id, stamp);
  }

  return "";
}
}  // namespace ars408
