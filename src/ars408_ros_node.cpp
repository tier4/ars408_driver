/*
 * Copyright 2021. Perception Engine Inc. All rights reserved.
 */
#include "ars408_ros/ars408_ros_node.h"

PeContinentalArs408Node::PeContinentalArs408Node() :
    global_node_handle_(), private_node_handle_("~"), ars408_driver_(), sequence_id_(0)
{

}

void PeContinentalArs408Node::CanFrameCallback(const can_msgs::Frame::ConstPtr& can_msg)
{
  if (!can_msg->data.empty())
  {
    ars408_driver_.Parse(can_msg->id, can_msg->data, can_msg->dlc);
  }
}

uint32_t
PeContinentalArs408Node::ConvertRadarClassToAwSemanticClass(const ars408::Obj_3_Extended::ObjectClassProperty& in_radar_class)
{
  switch (in_radar_class)
  {
    case ars408::Obj_3_Extended::BICYCLE:
      return autoware_perception_msgs::Semantic::BICYCLE;
      break;
    case ars408::Obj_3_Extended::CAR:
      return autoware_perception_msgs::Semantic::CAR;
      break;
    case ars408::Obj_3_Extended::TRUCK:
      return autoware_perception_msgs::Semantic::TRUCK;
      break;
    case ars408::Obj_3_Extended::MOTORCYCLE:
      return autoware_perception_msgs::Semantic::MOTORBIKE;
      break;
    case ars408::Obj_3_Extended::POINT:
    case ars408::Obj_3_Extended::RESERVED_01:
    case ars408::Obj_3_Extended::WIDE:
    case ars408::Obj_3_Extended::RESERVED_02:
    default:
      return autoware_perception_msgs::Semantic::UNKNOWN;
      break;
  }
}

autoware_perception_msgs::DynamicObject
PeContinentalArs408Node::ConvertRadarObjectToAwDynamicObject(const ars408::RadarObject& in_object)
{
  autoware_perception_msgs::DynamicObject out_object;

  out_object.id = unique_id::toMsg(unique_id::fromRandom());
  out_object.semantic.type = ConvertRadarClassToAwSemanticClass(in_object.object_class);
  out_object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  out_object.shape.dimensions.x = 1.0;
  out_object.shape.dimensions.y = 1.0;
  out_object.shape.dimensions.z = 2.0;

  out_object.semantic.confidence = in_object.rcs;
  out_object.state.pose_covariance.pose.position.x = in_object.distance_long_x;
  out_object.state.pose_covariance.pose.position.y = in_object.distance_lat_y;

  out_object.state.twist_reliable = true;
  out_object.state.twist_covariance.twist.linear.x = in_object.speed_long_x;
  out_object.state.twist_covariance.twist.linear.y = in_object.speed_lat_y;
  out_object.state.twist_covariance.twist.angular.x = in_object.speed_long_x;
  out_object.state.twist_covariance.twist.angular.y = in_object.speed_lat_y;

  out_object.state.acceleration_reliable = true;
  out_object.state.acceleration_covariance.accel.angular.x = in_object.rel_acceleration_long_x;
  out_object.state.acceleration_covariance.accel.angular.y = in_object.rel_acceleration_lat_y;

  return out_object;
}

void PeContinentalArs408Node::RadarDetectedObjectsCallback(const std::unordered_map<uint8_t ,
                                                           ars408::RadarObject>& detected_objects)
{
  autoware_perception_msgs::DynamicObjectArray aw_output_objects;

  aw_output_objects.header.frame_id = output_frame_;
  ros::Time current_time = ros::Time::now();
  aw_output_objects.header.stamp = current_time;
  aw_output_objects.header.seq = sequence_id_++;

  for(auto object: detected_objects)
  {
    //ROS_INFO_STREAM(object.second.ToString());
    autoware_perception_msgs::DynamicObject aw_object = ConvertRadarObjectToAwDynamicObject(object.second);
    aw_output_objects.objects.emplace_back(aw_object);
  }
  publisher_dynamic_object_array_.publish(aw_output_objects);
}

void PeContinentalArs408Node::Run()
{
  std::string can_input_topic, object_output_topic;

  private_node_handle_.param<std::string>("can_input_topic",
                                          can_input_topic,
                                          "can_raw");
  private_node_handle_.param<std::string>("object_output_topic",
                                          object_output_topic,
                                          "/detection/radar/objects");
  private_node_handle_.param<std::string>("output_frame",
                                          output_frame_,
                                          "ars408");

  ars408_driver_.RegisterDetectedObjectsCallback(
    boost::bind(&PeContinentalArs408Node::RadarDetectedObjectsCallback,
                this,
                _1));

  subscriber_can_raw_ = global_node_handle_.subscribe(can_input_topic,
                                                      10,
                                                      &PeContinentalArs408Node::CanFrameCallback,
                                                      this);
  ROS_INFO_STREAM("Subscribed to " << can_input_topic);

  publisher_dynamic_object_array_ =
    global_node_handle_.advertise<autoware_perception_msgs::DynamicObjectArray>(object_output_topic,
                                                                                10,
                                                                                true);

  ros::spin();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pe_ars408_ros_node");

  PeContinentalArs408Node app;

  app.Run();

  return 0;
}
