/*
 * Copyright 2021. Perception Engine Inc. All rights reserved.
 */
#include "ars408_ros/ars408_ros_node.h"

PeContinentalArs408Node::PeContinentalArs408Node() : Node("ars408_node"),
    ars408_driver_()
{
    GenerateUUIDTable();
    Run();
}

void PeContinentalArs408Node::CanFrameCallback(const can_msgs::msg::Frame::SharedPtr can_msg)
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
      return autoware_perception_msgs::msg::Semantic::BICYCLE;
      break;
    case ars408::Obj_3_Extended::CAR:
      return autoware_perception_msgs::msg::Semantic::CAR;
      break;
    case ars408::Obj_3_Extended::TRUCK:
      return autoware_perception_msgs::msg::Semantic::TRUCK;
      break;
    case ars408::Obj_3_Extended::MOTORCYCLE:
      return autoware_perception_msgs::msg::Semantic::MOTORBIKE;
      break;
    case ars408::Obj_3_Extended::POINT:
    case ars408::Obj_3_Extended::RESERVED_01:
    case ars408::Obj_3_Extended::WIDE:
    case ars408::Obj_3_Extended::RESERVED_02:
    default:
      return autoware_perception_msgs::msg::Semantic::UNKNOWN;
      break;
  }
}

autoware_perception_msgs::msg::DynamicObject
PeContinentalArs408Node::ConvertRadarObjectToAwDynamicObject(const ars408::RadarObject& in_object)
{
  autoware_perception_msgs::msg::DynamicObject out_object;

//  out_object.id = unique_identifier_msgs::toMsg(unique_identifier_msgs::fromRandom());
  out_object.id = UUID_table_[in_object.id];
  out_object.semantic.type = ConvertRadarClassToAwSemanticClass(in_object.object_class);
  out_object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
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
  autoware_perception_msgs::msg::DynamicObjectArray aw_output_objects;

  aw_output_objects.header.frame_id = output_frame_;
  rclcpp::Time current_time = this->get_clock()->now();
  aw_output_objects.header.stamp = current_time;

  for(auto object: detected_objects)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), object.second.ToString());
    autoware_perception_msgs::msg::DynamicObject aw_object = ConvertRadarObjectToAwDynamicObject(object.second);
    aw_output_objects.objects.emplace_back(aw_object);
  }
  publisher_dynamic_object_array_->publish(aw_output_objects);
}

unique_identifier_msgs::msg::UUID PeContinentalArs408Node::GenerateRandomUUID()
{
    unique_identifier_msgs::msg::UUID uuid;
    std::mt19937 gen(std::random_device{} ());
    std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
    std::generate(uuid.uuid.begin(), uuid.uuid.end(), bit_eng);
    return uuid;
}

void PeContinentalArs408Node::GenerateUUIDTable()
{
  for (size_t i=0; i<=max_radar_id; i++) {
    UUID_table_.emplace_back(PeContinentalArs408Node::GenerateRandomUUID());
  }
}


void PeContinentalArs408Node::Run()
{
  std::string can_input_topic, object_output_topic;

  can_input_topic = this->declare_parameter<std::string>("can_input_topic", "/can_raw");
  object_output_topic = this->declare_parameter<std::string>("object_output_topic", "/detection/radar/objects");
  output_frame_ = this->declare_parameter<std::string>("output_frame", "ars408");

  ars408_driver_.RegisterDetectedObjectsCallback(
    std::bind(&PeContinentalArs408Node::RadarDetectedObjectsCallback,
                this,
                 std::placeholders::_1));

  subscription_ = this->create_subscription<can_msgs::msg::Frame>(can_input_topic, 10,
                                                                  std::bind(&PeContinentalArs408Node::CanFrameCallback,
                                                                            this, std::placeholders::_1));

  RCLCPP_INFO_STREAM(this->get_logger(), "Subscribed to " << can_input_topic);

  publisher_dynamic_object_array_ =
    this->create_publisher<autoware_perception_msgs::msg::DynamicObjectArray>(object_output_topic,
                                                                                10);

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PeContinentalArs408Node>());
  rclcpp::shutdown();
  return 0;
}
