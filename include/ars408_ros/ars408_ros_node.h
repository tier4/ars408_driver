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

#include "ars408_ros/ars408_parser.h"

class PeContinentalArs408Node
{
  ros::NodeHandle global_node_handle_,
                private_node_handle_;
  ros::Subscriber subscriber_can_raw_;

  void CanFrameCallback(const can_msgs::Frame::ConstPtr& can_msg);

  ars408::CanParser ars408_parser_;

public:
  PeContinentalArs408Node();
  void Run();
};

#endif //ARS408_ROS_H