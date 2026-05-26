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

#ifndef ARS408_ROS__ARS408_DRIVER_HPP_
#define ARS408_ROS__ARS408_DRIVER_HPP_

#include "ars408_ros/ars408_can_parser.hpp"
#include "ars408_ros/ars408_commands.hpp"
#include "ars408_ros/ars408_constants.hpp"
#include "ars408_ros/ars408_object.hpp"

#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace ars408
{
class Ars408Driver
{
private:
  uint8_t radar_id_{0};

  bool valid_radar_state_{false};
  bool valid_version_id_{false};
  bool sequential_publish_{false};
  ars408::RadarState current_radar_state_{};
  ars408::can_parser::VersionId current_version_id_{};
  ars408::Obj_0_Status current_objects_status_{};
  ars408::Obj_1_General objects_general_{};
  ars408::Obj_2_Quality objects_quality_{};
  ars408::Obj_3_Extended objects_extended_{};

  std::unordered_map<uint8_t, ars408::RadarObject> radar_objects_{};
  uint8_t updated_objects_general_{0xFFu};
  uint8_t updated_objects_quality_{0xFFu};
  uint8_t updated_objects_ext_{0xFFu};

  std::function<void(const std::unordered_map<uint8_t, ars408::RadarObject> &,
    const rclcpp::Time &)>
    detected_objects_callback_;

  /**
   * Adds a RadarObject to the pool of objects
   */
  void AddDetectedObject(ars408::RadarObject in_object);

  /**
   * Calls the registered callback to send back the detected objects from a measurement.
   */
  void CallDetectedObjectsCallback(
    std::unordered_map<uint8_t, ars408::RadarObject> & in_detected_objects,
    const rclcpp::Time & stamp);

  /**
   * Checks whether all the detected objects have been received and parsed.
   */
  bool DetectedObjectsReady();

  /**
   * Resets Detected objects and its counters
   */
  void ClearRadarObjects();

  /**
   * Updates existing Object with its quality information
   */
  void UpdateObjectQuality(uint8_t in_object_id, const ars408::Obj_2_Quality & in_object_quality);

  /**
   * Updates existing Object with its extended information
   */
  void UpdateObjectExtInfo(
    uint8_t in_object_id, const ars408::Obj_3_Extended & in_object_ext_info);

  /**
   * Parses RadarState CAN 0x201
   */
  void ParseRadarState(const std::array<uint8_t, 8> & in_can_data);

  /**
   * Parses Object0_Status CAN 0x60A
   */
  void ParseObject0_Status(const std::array<uint8_t, 8> & in_can_data);

  /**
   * Parses Object1_General CAN 0x60B
   */
  ars408::RadarObject ParseObject1_General(const std::array<uint8_t, 8> & in_can_data);

  /**
   * Parses Object2_Quality CAN 0x60C
   */
  ars408::Obj_2_Quality ParseObject2_Quality(const std::array<uint8_t, 8> & in_can_data);

  /**
   * Parses Object3_Extended CAN 0x60D
   */
  ars408::Obj_3_Extended ParseObject3_Extended(const std::array<uint8_t, 8> & in_can_data);

  void ParseVersionIdFrame(const std::array<uint8_t, 8> & in_can_data);

public:
  /**
   * Sets the hardware Sensor ID (0–7) that this driver instance handles.
   * Frames whose CAN ID encodes a different Sensor ID are silently ignored.
   */
  void SetRadarId(uint8_t radar_id) { radar_id_ = radar_id; }

  /**
   * Parses incoming can_id and its byte array can_data
   */
  std::string Parse(
    const uint32_t & can_id, const std::array<uint8_t, 8> & in_can_data,
    const uint8_t & in_data_length, const rclcpp::Time & stamp);

  /**
   * Returns true if the RadarState has been received.
   */
  bool GetCurrentRadarState(ars408::RadarState & out_current_state);

  bool GetVersionId(ars408::can_parser::VersionId & out_version_id);

  /**
   * Register the function to be called once all the Radar objects are ready.
   */
  void RegisterDetectedObjectsCallback(
    std::function<void(
      const std::unordered_map<uint8_t, ars408::RadarObject> &,
      const rclcpp::Time &
    )> objects_callback,
    bool sequential_publish);
};
}  // namespace ars408

#endif  // ARS408_ROS__ARS408_DRIVER_HPP_
