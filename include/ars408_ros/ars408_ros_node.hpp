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
#include "rclcpp/rclcpp.hpp"

#include "can_msgs/msg/frame.hpp"
#include "radar_msgs/msg/radar_scan.hpp"
#include "radar_msgs/msg/radar_tracks.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

#include <random>
#include <string>
#include <unordered_map>
#include <vector>

class PeContinentalArs408Node : public rclcpp::Node
{
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscriber_can_raw_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
  rclcpp::Publisher<radar_msgs::msg::RadarTracks>::SharedPtr publisher_radar_tracks_;
  rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr publisher_radar_scan_;

  std::string output_frame_;
  can_msgs::msg::Frame::ConstSharedPtr can_data_;
  bool publish_radar_track_;
  bool publish_radar_scan_;

  const uint8_t max_radar_id = 255;
  std::vector<unique_identifier_msgs::msg::UUID> UUID_table_;
  std::string topic_type_;

  ars408::Ars408Driver ars408_driver_{};

  void CanFrameCallback(const can_msgs::msg::Frame::SharedPtr can_msg);
  void GenerateUUIDTable();

  radar_msgs::msg::RadarTrack ConvertRadarObjectToRadarTrack(const ars408::RadarObject & in_object);
  radar_msgs::msg::RadarReturn ConvertRadarObjectToRadarReturn(
    const ars408::RadarObject & in_object);

  static uint32_t ConvertRadarClassToAwSemanticClass(
    const ars408::Obj_3_Extended::ObjectClassProperty & in_radar_class);
  static unique_identifier_msgs::msg::UUID GenerateRandomUUID();

public:
  explicit PeContinentalArs408Node(const rclcpp::NodeOptions & node_options);
  void RadarDetectedObjectsCallback(
    const std::unordered_map<uint8_t, ars408::RadarObject> & detected_objects);
  void Run();
};

#endif  // ARS408_ROS__ARS408_ROS_NODE_HPP_
