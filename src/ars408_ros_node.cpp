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
  topic_type_ = declare_parameter("topic_type", "RadarTrack");
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
      return 32006;
      break;
    case ars408::Obj_3_Extended::CAR:
      return 32001;
      break;
    case ars408::Obj_3_Extended::TRUCK:
      return 32002;
      break;
    case ars408::Obj_3_Extended::MOTORCYCLE:
      return 32005;
      break;
    case ars408::Obj_3_Extended::POINT:
    case ars408::Obj_3_Extended::RESERVED_01:
    case ars408::Obj_3_Extended::WIDE:
    case ars408::Obj_3_Extended::RESERVED_02:
    default:
      return 32000;
      break;
  }
}

radar_msgs::msg::RadarTrack PeContinentalArs408Node::ConvertRadarObjectToRadarTrack(
  const ars408::RadarObject & in_object)
{
  radar_msgs::msg::RadarTrack out_object;
  out_object.uuid = UUID_table_[in_object.id];

  out_object.position.x = in_object.distance_long_x;
  out_object.position.y = in_object.distance_lat_y;

  out_object.velocity.x = in_object.speed_long_x;
  out_object.velocity.y = in_object.speed_lat_y;

  out_object.acceleration.x = in_object.rel_acceleration_long_x;
  out_object.acceleration.y = in_object.rel_acceleration_lat_y;

  out_object.size.x = 1.0;
  out_object.size.y = 1.0;
  out_object.size.z = 1.0;

  out_object.classification = ConvertRadarClassToAwSemanticClass(in_object.object_class);

  return out_object;
}

void PeContinentalArs408Node::RadarDetectedObjectsCallback(
  const std::unordered_map<uint8_t, ars408::RadarObject> & detected_objects)
{
  radar_msgs::msg::RadarTracks output_objects;
  output_objects.header.frame_id = output_frame_;

  rclcpp::Time current_time = this->get_clock()->now();
  output_objects.header.stamp = current_time;

  for (const auto & object : detected_objects) {
    radar_msgs::msg::RadarTrack object_ = ConvertRadarObjectToRadarTrack(object.second);
    output_objects.tracks.emplace_back(object_);
    publisher_radar_tracks_->publish(output_objects);
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

  publisher_radar_tracks_ =
    this->create_publisher<radar_msgs::msg::RadarTracks>("~/output/objects", 10);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PeContinentalArs408Node)
