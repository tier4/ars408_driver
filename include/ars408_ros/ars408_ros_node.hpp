/*
* Copyright 2021. Perception Engine Inc. All rights reserved.
*
*/
#ifndef ARS408_ROS_H
#define ARS408_ROS_H

#include <string>
#include <vector>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <unique_identifier_msgs/msg/uuid.h>

#include "ars408_ros/ars408_driver.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"

class PeContinentalArs408Node : public rclcpp::Node {
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscriber_can_raw_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr publisher_dynamic_object_array_;

  std::string output_frame_;

  const uint8_t max_radar_id = 255;
  std::vector<unique_identifier_msgs::msg::UUID> UUID_table_;

  ars408::Ars408Driver ars408_driver_;

  void CanFrameCallback(const can_msgs::msg::Frame::SharedPtr can_msg);

  void GenerateUUIDTable();

  autoware_perception_msgs::msg::DynamicObject
  ConvertRadarObjectToAwDynamicObject(const ars408::RadarObject& in_object);

  static uint32_t
  ConvertRadarClassToAwSemanticClass(const ars408::Obj_3_Extended::ObjectClassProperty& in_radar_class);

  static unique_identifier_msgs::msg::UUID
  GenerateRandomUUID();



public:
  PeContinentalArs408Node();
  void RadarDetectedObjectsCallback(const std::unordered_map<uint8_t, ars408::RadarObject>& detected_objects);
  void Run();
};

#endif //ARS408_ROS_H
