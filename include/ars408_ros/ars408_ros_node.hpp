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

#ifndef ARS408_ROS__ARS408_ROS_NODE_HPP_
#define ARS408_ROS__ARS408_ROS_NODE_HPP_

#include "ars408_ros/ars408_driver.hpp"
#include "can_msgs/msg/frame.hpp"
#include "radar_msgs/msg/radar_scan.hpp"
#include "radar_msgs/msg/radar_tracks.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <random>
#include <string>
#include <unordered_map>
#include <vector>

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using DiagnosticArrayPublisher = rclcpp::Publisher<DiagnosticArray>;

class PeContinentalArs408Node : public rclcpp::Node
{
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
  rclcpp::Publisher<radar_msgs::msg::RadarTracks>::SharedPtr publisher_radar_tracks_;
  rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr publisher_radar_scan_;
  DiagnosticArrayPublisher::SharedPtr diagnostics_pub_;

  std::string output_frame_;
  bool publish_radar_track_;
  bool publish_radar_scan_;
  bool sequential_publish_;
  double size_x_;
  double size_y_;
  uint8_t radar_id_;
  std::string publish_objects_name_;
  std::string publish_scan_name_;
  double can_receive_check_rate_hz_;
  double can_receive_check_timeout_sec_;

  const uint8_t max_radar_id = 255;
  std::vector<unique_identifier_msgs::msg::UUID> UUID_table_;
  rclcpp::TimerBase::SharedPtr can_receive_check_timer_;
  std::optional<rclcpp::Time> can_receive_last_time_;
  rclcpp::Time last_warn_time_;

  ars408::Ars408Driver ars408_driver_{};

  void CanFrameCallback(const can_msgs::msg::Frame::SharedPtr can_msg);
  void OnCanReceiveCheck();
  void GenerateUUIDTable();
  void SetParameter();

  radar_msgs::msg::RadarTrack ConvertRadarObjectToRadarTrack(const ars408::RadarObject & in_object);
  radar_msgs::msg::RadarReturn ConvertRadarObjectToRadarReturn(
    const ars408::RadarObject & in_object);

  unique_identifier_msgs::msg::UUID GenerateRandomUUID();
  uint32_t ConvertRadarClassToAwSemanticClass(
    const ars408::Obj_3_Extended::ObjectClassProperty & in_radar_class);

public:
  explicit PeContinentalArs408Node(const rclcpp::NodeOptions & node_options);
  void RadarDetectedObjectsCallback(
    const std::unordered_map<uint8_t, ars408::RadarObject> & detected_objects,
    const rclcpp::Time & stamp);
  void Run();
};

#endif  // ARS408_ROS__ARS408_ROS_NODE_HPP_
