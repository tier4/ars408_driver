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
  std::array<bool, RADAR_CONNECTIONS_MAX> valid_radar_state_;
  std::array<bool, RADAR_CONNECTIONS_MAX> sequential_publish_;
  std::array<ars408::RadarState, RADAR_CONNECTIONS_MAX> current_radar_state_;
  std::array<ars408::Obj_0_Status, RADAR_CONNECTIONS_MAX> current_objects_status_;
  std::array<ars408::Obj_1_General, RADAR_CONNECTIONS_MAX> objects_general_;
  std::array<ars408::Obj_2_Quality, RADAR_CONNECTIONS_MAX> objects_quality_;
  std::array<ars408::Obj_3_Extended, RADAR_CONNECTIONS_MAX> objects_extended_;

  std::array<std::unordered_map<uint8_t, ars408::RadarObject>, RADAR_CONNECTIONS_MAX> radar_objects_;
  std::array<uint8_t, RADAR_CONNECTIONS_MAX> updated_objects_general_{0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};
  std::array<uint8_t, RADAR_CONNECTIONS_MAX> updated_objects_quality_{0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};
  std::array<uint8_t, RADAR_CONNECTIONS_MAX> updated_objects_ext_{0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};

  std::function<void(const std::unordered_map<uint8_t, ars408::RadarObject> &,
    uint8_t, const rclcpp::Time &)>
    detected_objects_callback_;

  /**
   * Adds a RadarObject to the pool of objects
   * @param in_object RadarObject to be added
   */
  void AddDetectedObject(ars408::RadarObject in_object, uint32_t radar_id);

  /**
   * Calls the registered callback to send back the detected objects from a measurement.
   * @param in_detected_objects
   */
  void CallDetectedObjectsCallback(
    std::unordered_map<uint8_t, ars408::RadarObject> & in_detected_objects,
    uint32_t radar_id, const rclcpp::Time & stamp);

  /**
   * Checks whether all the detected objects have been received and parsed.
   * @return Returns true if all the objects were received
   */
  bool DetectedObjectsReady(uint32_t radar_id);

  /**
   * Resets Detected objects and its counters
   */
  void ClearRadarObjects(uint32_t radar_id);

  /**
   * Updates existing Object with its quality information
   * @param in_object_id Id of the object to update
   * @param in_object_quality quality information object to apply to the object
   */
  void UpdateObjectQuality(uint8_t in_object_id, const ars408::Obj_2_Quality & in_object_quality, uint32_t radar_id);

  /**
   * Updates existing Object with its extended information
   * @param in_object_id Id of the object to update
   * @param in_object_ext_info extended information object to apply to the object
   */
  void UpdateObjectExtInfo(uint8_t in_object_id, const ars408::Obj_3_Extended & in_object_ext_info, uint32_t radar_id);

  /**
   * Parses RadarState CAN 0x201 and stores internally current status (current_state_)
   * @param in_can_data std::array<uint8_t, 8> containing the CAN message
   */
  void ParseRadarState(const std::array<uint8_t, 8> & in_can_data, uint32_t radar_id);

  /**
   * Parses Object0_Status CAN 0x60A
   * @param in_can_data std::array<uint8_t, 8>
   */
  void ParseObject0_Status(const std::array<uint8_t, 8> & in_can_data, uint32_t radar_id);

  /**
   * Parses Object1_General CAN 0x60B
   * @param in_can_data std::array<uint8_t, 8> containing the CAN message
   * @return object filled with the details about one object
   */
  ars408::RadarObject ParseObject1_General(const std::array<uint8_t, 8> & in_can_data, uint32_t radar_id);

  /**
   * Parses Object2_Quality CAN 0x60C
   * @param in_can_data std::array<uint8_t, 8> containing the CAN message
   * @return quality object
   */
  ars408::Obj_2_Quality ParseObject2_Quality(const std::array<uint8_t, 8> & in_can_data);

  /**
   * Parses Object3_Extended CAN 0x60D
   * @param in_can_data std::array<uint8_t, 8> containing the CAN message
   * @return extended info object
   */
  ars408::Obj_3_Extended ParseObject3_Extended(const std::array<uint8_t, 8> & in_can_data);

public:
  /**
   * Parses incoming can_id and its byte array can_data
   * @param can_id CAN message Id
   * @param in_can_data Eight byte Array containing message content
   * @param in_data_length Number of bytes with valid data contained in can_data
   * @return String Message
   */
  std::string Parse(
    const uint32_t & can_id, const std::array<uint8_t, 8> & in_can_data,
    const uint8_t & in_data_length, const rclcpp::Time & stamp);

  /**
   * Returns true if the RadarState has been received, if true it fills current_state with the valid
   * state. If invalid, current_state is empty.
   * @param out_current_state
   * @return True if the RadarState has been received.
   */
  bool GetCurrentRadarState(ars408::RadarState & out_current_state, uint32_t radar_id);

  /**
   * Register the function to be called once all the Radar objects are ready.
   * @param objects_callback pointer to callback function
   * @param sequential_publish parameter of publish mode
   */

  void RegisterDetectedObjectsCallback(
    std::function<void(
      const std::unordered_map<uint8_t, ars408::RadarObject> &,
      uint8_t,
      const rclcpp::Time &
    )> objects_callback,
    const std::array<bool, RADAR_CONNECTIONS_MAX> & sequential_publish);
};
}  // namespace ars408

#endif  // ARS408_ROS__ARS408_DRIVER_HPP_
