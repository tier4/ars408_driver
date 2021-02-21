/*
* Copyright 2021. Perception Engine Inc. All rights reserved.
*
*/
#ifndef ARS408_ROS_H
#define ARS408_ROS_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <unique_id/unique_id.h>

#include "ars408_ros/ars408_driver.h"
#include "autoware_perception_msgs/DynamicObjectArray.h"

class PeContinentalArs408Node
{
  ros::NodeHandle global_node_handle_,
                private_node_handle_;
  ros::Subscriber subscriber_can_raw_;
  ros::Publisher publisher_dynamic_object_array_;

  std::string output_frame_;
  uint32_t sequence_id_;

  ars408::Ars408Driver ars408_driver_;

  void CanFrameCallback(const can_msgs::Frame::ConstPtr& can_msg);

  static autoware_perception_msgs::DynamicObject
  ConvertRadarObjectToAwDynamicObject(const ars408::RadarObject& in_object);

  static uint32_t
  ConvertRadarClassToAwSemanticClass(const ars408::Obj_3_Extended::ObjectClassProperty& in_radar_class);

public:
  PeContinentalArs408Node();
  void RadarDetectedObjectsCallback(const std::unordered_map<uint8_t , ars408::RadarObject>& detected_objects);
  void Run();
};

#endif //ARS408_ROS_H