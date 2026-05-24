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

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <unordered_map>

PeContinentalArs408Node::PeContinentalArs408Node(const rclcpp::NodeOptions & node_options)
: Node("ars408_node", node_options)
{
  SetParameter();
  GenerateUUIDTable();
  Run();
}

void PeContinentalArs408Node::CanFrameCallback(const can_msgs::msg::Frame::SharedPtr can_msg)
{
  if (!can_msg->data.empty()) {
    ars408_driver_.Parse(can_msg->id, can_msg->data, can_msg->dlc, can_msg->header.stamp);
  }
}

void PeContinentalArs408Node::OnCanReceiveCheck()
{
  const rclcpp::Time current_time = this->now();

  if (!can_receive_last_time_) {
    can_receive_last_time_ = current_time;
    return;
  }

  const double elapsed_sec = (current_time - can_receive_last_time_.value()).seconds();
  if (elapsed_sec <= can_receive_check_timeout_sec_) {
    return;
  }

  if ((current_time - last_warn_time_).seconds() > can_receive_check_timeout_sec_) {
    RCLCPP_ERROR(
      get_logger(), "CAN topic received timeout (%.3f sec). Radar ID=%d",
      elapsed_sec, radar_id_);
    last_warn_time_ = current_time;
  }

  DiagnosticArray diag_array;
  diag_array.header.stamp = current_time;

  DiagnosticStatus diag;
  diag.level = DiagnosticStatus::ERROR;
  diag.name = "ars408_driver";
  diag.message = "CAN topic received timeout - Radar ID(" + std::to_string(radar_id_) + ")";
  diag.hardware_id = output_frame_;

  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "error_type";
  kv.value = "CAN_MSGS_RECEIVE_TIMEOUT";
  diag.values.push_back(kv);
  diag_array.status.push_back(diag);

  diagnostics_pub_->publish(diag_array);
}

uint32_t PeContinentalArs408Node::ConvertRadarClassToAwSemanticClass(
  const ars408::Obj_3_Extended::ObjectClassProperty & in_radar_class)
{
  switch (in_radar_class) {
    case ars408::Obj_3_Extended::BICYCLE:    return 32006;
    case ars408::Obj_3_Extended::CAR:        return 32001;
    case ars408::Obj_3_Extended::TRUCK:      return 32002;
    case ars408::Obj_3_Extended::MOTORCYCLE: return 32005;
    case ars408::Obj_3_Extended::POINT:
    case ars408::Obj_3_Extended::RESERVED_01:
    case ars408::Obj_3_Extended::WIDE:
    case ars408::Obj_3_Extended::RESERVED_02:
    default:                                 return 32000;
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
  out_object.velocity_covariance.at(0) = 0.1;

  out_object.acceleration.x = in_object.rel_acceleration_long_x;
  out_object.acceleration.y = in_object.rel_acceleration_lat_y;

  out_object.size.x = size_x_;
  out_object.size.y = size_y_;
  out_object.size.z = 1.0;

  out_object.classification = ConvertRadarClassToAwSemanticClass(in_object.object_class);

  return out_object;
}

radar_msgs::msg::RadarReturn PeContinentalArs408Node::ConvertRadarObjectToRadarReturn(
  const ars408::RadarObject & in_object)
{
  radar_msgs::msg::RadarReturn radar_return;
  radar_return.range = std::sqrt(
    in_object.distance_long_x * in_object.distance_long_x +
    in_object.distance_lat_y * in_object.distance_lat_y);
  radar_return.azimuth = std::atan2(in_object.distance_lat_y, in_object.distance_long_x);
  radar_return.doppler_velocity = in_object.speed_long_x / std::cos(radar_return.azimuth);
  radar_return.elevation = 0.0;
  radar_return.amplitude = 0.0;
  return radar_return;
}

void PeContinentalArs408Node::RadarDetectedObjectsCallback(
  const std::unordered_map<uint8_t, ars408::RadarObject> & detected_objects,
  const rclcpp::Time & stamp)
{
  can_receive_last_time_ = this->now();

  radar_msgs::msg::RadarTracks output_objects;
  output_objects.header.frame_id = output_frame_;
  output_objects.header.stamp = stamp;

  radar_msgs::msg::RadarScan output_scan;
  output_scan.header.frame_id = output_frame_;
  output_scan.header.stamp = stamp;

  for (const auto & object : detected_objects) {
    if (publish_radar_track_) {
      output_objects.tracks.emplace_back(ConvertRadarObjectToRadarTrack(object.second));
    }
    if (publish_radar_scan_) {
      output_scan.returns.emplace_back(ConvertRadarObjectToRadarReturn(object.second));
    }
  }

  if (publish_radar_track_) {
    publisher_radar_tracks_->publish(output_objects);
  }
  if (publish_radar_scan_) {
    publisher_radar_scan_->publish(output_scan);
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
    UUID_table_.emplace_back(GenerateRandomUUID());
  }
}

void PeContinentalArs408Node::SetParameter()
{
  auto radar_id_int = this->declare_parameter<int>("radar_id");
  if (radar_id_int < 0 || radar_id_int >= static_cast<int>(ars408::RADAR_CONNECTIONS_MAX)) {
    throw std::invalid_argument(
      "'radar_id' out of range [expected 0–7]: " + std::to_string(radar_id_int));
  }
  radar_id_ = static_cast<uint8_t>(radar_id_int);

  output_frame_ = this->declare_parameter<std::string>("output_frame");
  publish_radar_track_ = this->declare_parameter<bool>("publish_radar_track");
  publish_radar_scan_ = this->declare_parameter<bool>("publish_radar_scan");
  sequential_publish_ = this->declare_parameter<bool>("sequential_publish");
  size_x_ = this->declare_parameter<double>("size_x");
  size_y_ = this->declare_parameter<double>("size_y");
  publish_objects_name_ = this->declare_parameter<std::string>("publish_objects_name");
  publish_scan_name_ = this->declare_parameter<std::string>("publish_scan_name");
  can_receive_check_rate_hz_ = this->declare_parameter<double>("can_receive_check_rate_hz");
  can_receive_check_timeout_sec_ = this->declare_parameter<double>("can_receive_check_timeout_sec");
}

void PeContinentalArs408Node::Run()
{
  can_receive_last_time_ = std::nullopt;
  last_warn_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

  ars408_driver_.SetRadarId(radar_id_);
  ars408_driver_.RegisterDetectedObjectsCallback(
    std::bind(
      &PeContinentalArs408Node::RadarDetectedObjectsCallback,
      this, std::placeholders::_1, std::placeholders::_2),
    sequential_publish_);

  subscription_ = this->create_subscription<can_msgs::msg::Frame>(
    "~/input/frame", 10,
    std::bind(&PeContinentalArs408Node::CanFrameCallback, this, std::placeholders::_1));

  publisher_radar_tracks_ =
    this->create_publisher<radar_msgs::msg::RadarTracks>(publish_objects_name_, 10);
  publisher_radar_scan_ =
    this->create_publisher<radar_msgs::msg::RadarScan>(publish_scan_name_, 10);
  diagnostics_pub_ =
    this->create_publisher<DiagnosticArray>("/diagnostics", rclcpp::QoS{1});

  can_receive_check_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(1000.0 / can_receive_check_rate_hz_)),
    std::bind(&PeContinentalArs408Node::OnCanReceiveCheck, this));
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PeContinentalArs408Node)
