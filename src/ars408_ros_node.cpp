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

#include "ars408_ros/ars408_ros_node.hpp"

#include <string>
#include <unordered_map>

PeContinentalArs408Node::PeContinentalArs408Node(const rclcpp::NodeOptions & node_options)
: Node("ars408_node", node_options)
{
  publish_tracked_object_ = declare_parameter("use_tracked_object", false);
  GenerateUUIDTable();
  Run();
}

void PeContinentalArs408Node::CanFrameCallback(const can_msgs::msg::Frame::SharedPtr can_msg)
{
  if (!can_msg->data.empty()) {
    ars408_driver_.Parse(can_msg->id, can_msg->data, can_msg->dlc);
  }
}

uint32_t PeContinentalArs408Node::ConvertRadarClassToAwSemanticClass(
  const ars408::Obj_3_Extended::ObjectClassProperty & in_radar_class)
{
  switch (in_radar_class) {
    case ars408::Obj_3_Extended::BICYCLE:
      return autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE;
      break;
    case ars408::Obj_3_Extended::CAR:
      return autoware_auto_perception_msgs::msg::ObjectClassification::CAR;
      break;
    case ars408::Obj_3_Extended::TRUCK:
      return autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK;
      break;
    case ars408::Obj_3_Extended::MOTORCYCLE:
      return autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE;
      break;
    case ars408::Obj_3_Extended::POINT:
    case ars408::Obj_3_Extended::RESERVED_01:
    case ars408::Obj_3_Extended::WIDE:
    case ars408::Obj_3_Extended::RESERVED_02:
    default:
      return autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
      break;
  }
}

autoware_auto_perception_msgs::msg::DetectedObject
PeContinentalArs408Node::ConvertRadarObjectToAwDetectedObject(const ars408::RadarObject & in_object)
{
  autoware_auto_perception_msgs::msg::ObjectClassification classification;
  autoware_auto_perception_msgs::msg::DetectedObject out_object;

  classification.label = ConvertRadarClassToAwSemanticClass(in_object.object_class);
  classification.probability = in_object.rcs;
  out_object.classification.emplace_back(classification);
  out_object.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
  out_object.shape.dimensions.x = 1.0;
  out_object.shape.dimensions.y = 1.0;
  out_object.shape.dimensions.z = 2.0;

  out_object.kinematics.pose_with_covariance.pose.position.x = in_object.distance_long_x;
  out_object.kinematics.pose_with_covariance.pose.position.y = in_object.distance_lat_y;

  out_object.kinematics.has_twist = true;
  out_object.kinematics.has_twist_covariance = false;
  out_object.kinematics.twist_with_covariance.twist.linear.x = in_object.speed_long_x;
  out_object.kinematics.twist_with_covariance.twist.linear.y = in_object.speed_lat_y;
  out_object.kinematics.twist_with_covariance.twist.angular.x = in_object.speed_long_x;
  out_object.kinematics.twist_with_covariance.twist.angular.y = in_object.speed_lat_y;

  return out_object;
}

autoware_auto_perception_msgs::msg::TrackedObject
PeContinentalArs408Node::ConvertRadarObjectToAwTrackedObject(const ars408::RadarObject & in_object)
{
  autoware_auto_perception_msgs::msg::ObjectClassification classification;
  autoware_auto_perception_msgs::msg::TrackedObject out_object;

  out_object.object_id = UUID_table_[in_object.id];
  classification.label = ConvertRadarClassToAwSemanticClass(in_object.object_class);
  classification.probability = in_object.rcs;
  out_object.classification.emplace_back(classification);
  out_object.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
  out_object.shape.dimensions.x = 1.0;
  out_object.shape.dimensions.y = 1.0;
  out_object.shape.dimensions.z = 2.0;

  out_object.kinematics.pose_with_covariance.pose.position.x = in_object.distance_long_x;
  out_object.kinematics.pose_with_covariance.pose.position.y = in_object.distance_lat_y;

  out_object.kinematics.twist_with_covariance.twist.linear.x = in_object.speed_long_x;
  out_object.kinematics.twist_with_covariance.twist.linear.y = in_object.speed_lat_y;
  out_object.kinematics.twist_with_covariance.twist.angular.x = in_object.speed_long_x;
  out_object.kinematics.twist_with_covariance.twist.angular.y = in_object.speed_lat_y;

  out_object.kinematics.acceleration_with_covariance.accel.angular.x =
    in_object.rel_acceleration_long_x;
  out_object.kinematics.acceleration_with_covariance.accel.angular.y =
    in_object.rel_acceleration_lat_y;

  return out_object;
}

void PeContinentalArs408Node::RadarDetectedObjectsCallback(
  const std::unordered_map<uint8_t, ars408::RadarObject> & detected_objects)
{
  if (publish_tracked_object_) {
    autoware_auto_perception_msgs::msg::TrackedObjects aw_output_objects;

    aw_output_objects.header.frame_id = output_frame_;
    rclcpp::Time current_time = this->get_clock()->now();
    aw_output_objects.header.stamp = current_time;

    for (const auto & object : detected_objects) {
      autoware_auto_perception_msgs::msg::TrackedObject aw_object =
        ConvertRadarObjectToAwTrackedObject(object.second);
      aw_output_objects.objects.emplace_back(aw_object);
      publisher_tracked_objects_->publish(aw_output_objects);
    }
  } else {
    autoware_auto_perception_msgs::msg::DetectedObjects aw_output_objects;

    aw_output_objects.header.frame_id = output_frame_;
    rclcpp::Time current_time = this->get_clock()->now();
    aw_output_objects.header.stamp = current_time;

    for (const auto & object : detected_objects) {
      autoware_auto_perception_msgs::msg::DetectedObject aw_object =
        ConvertRadarObjectToAwDetectedObject(object.second);
      aw_output_objects.objects.emplace_back(aw_object);
      publisher_detected_objects_->publish(aw_output_objects);
    }
  }
}

unique_identifier_msgs::msg::UUID PeContinentalArs408Node::GenerateRandomUUID()
{
  unique_identifier_msgs::msg::UUID uuid;
  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(uuid.uuid.begin(), uuid.uuid.end(), bit_eng);
  return uuid;
}

void PeContinentalArs408Node::GenerateUUIDTable()
{
  for (size_t i = 0; i <= max_radar_id; i++) {
    UUID_table_.emplace_back(PeContinentalArs408Node::GenerateRandomUUID());
  }
}

void PeContinentalArs408Node::Run()
{
  output_frame_ = this->declare_parameter<std::string>("output_frame", "ars408");

  ars408_driver_.RegisterDetectedObjectsCallback(
    std::bind(&PeContinentalArs408Node::RadarDetectedObjectsCallback, this, std::placeholders::_1));

  subscription_ = this->create_subscription<can_msgs::msg::Frame>(
    "~/input/frame", 10,
    std::bind(&PeContinentalArs408Node::CanFrameCallback, this, std::placeholders::_1));
  if (publish_tracked_object_) {
    publisher_tracked_objects_ =
      this->create_publisher<autoware_auto_perception_msgs::msg::TrackedObjects>(
        "~/output/objects", 10);
  } else {
    publisher_detected_objects_ =
      this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
        "~/output/objects", 10);
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PeContinentalArs408Node)
