/*
 * Copyright 2021. Perception Engine Inc. All rights reserved.
 */
#include "ars408_ros/ars408_ros_node.h"

PeContinentalArs408Node::PeContinentalArs408Node() :
    global_node_handle_(), private_node_handle_("~"), ars408_driver_()
{

}

void PeContinentalArs408Node::CanFrameCallback(const can_msgs::Frame::ConstPtr& can_msg)
{
  if (!can_msg->data.empty())
  {
    ars408_driver_.Parse(can_msg->id, can_msg->data, can_msg->dlc);
  }
}

void PeContinentalArs408Node::RadarDetectedObjectsCallback(const std::unordered_map<uint8_t , ars408::RadarObject>& detected_objects)
{

}

void PeContinentalArs408Node::Run()
{
  std::string can_input_topic = "can_raw";

  ars408_driver_.RegisterDetectedObjectsCallback(
    boost::bind(&PeContinentalArs408Node::RadarDetectedObjectsCallback, this, _1));
  subscriber_can_raw_ = global_node_handle_.subscribe(can_input_topic, 10, &PeContinentalArs408Node::CanFrameCallback, this);
  ROS_INFO_STREAM("Subscribed to " << can_input_topic);
  ros::spin();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ars408_ros_node");

  PeContinentalArs408Node app;

  app.Run();

  return 0;
}
