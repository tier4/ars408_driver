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
  diagnostic_msgs::msg::DiagnosticArray diag_array;

  // Timeout check
  for (size_t i = 0; i < connection_count_; i++) {
    rclcpp::Time current_time = this->now();
    if (!can_receive_last_time_[i]) {
      can_receive_last_time_[i] = current_time;
    } else {
      const double elapsed_sec = (current_time - can_receive_last_time_[i].value()).seconds();
      if (elapsed_sec > can_receive_check_timeout_sec_) {
        // Timeout occurred
        if ((current_time - last_warn_times_[i]).seconds() > can_receive_check_timeout_sec_) {
          RCLCPP_ERROR(get_logger(), "can msos topic received timeout (%.3f sec)."
            "Radar ID=%d",
            elapsed_sec, radar_id_[i]);
          last_warn_times_[i] = current_time;
        }

        // Create MRM diagnostic message
        if (!diagnostic_published_[i]) {
          diagnostic_published_[i] = true;
          DiagnosticStatus diag;
          diag.level = DiagnosticStatus::ERROR;
          diag.name = "ars408_driver";
          diag.message = "can msos topic received timeout - Radar ID(" + std::to_string(radar_id_[i]) + ")";
          diag.hardware_id = "sensing";

          diagnostic_msgs::msg::KeyValue kv;
          kv.key = "error_type";
          kv.value = "CAN_MSGS_RECEIVE_TIMEOUT";
          diag.values.push_back(kv);
          diag_array.status.push_back(diag);
        }
      } else {
        diagnostic_published_[i] = false;
      }
    }
  }

  // Publish diagnostic message if there are any timeout errors
  if (!diag_array.status.empty()) {
    diag_array.header.stamp = this->now();
    diagnostics_pub_->publish(diag_array);
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
  const ars408::RadarObject & in_object, uint32_t tbl_idx)
{
  radar_msgs::msg::RadarTrack out_object;
  out_object.uuid = UUID_table_[tbl_idx][in_object.id];

  out_object.position.x = in_object.distance_long_x;
  out_object.position.y = in_object.distance_lat_y;

  out_object.velocity.x = in_object.speed_long_x;
  out_object.velocity.y = in_object.speed_lat_y;
  out_object.velocity_covariance.at(0) = 0.1;

  out_object.acceleration.x = in_object.rel_acceleration_long_x;
  out_object.acceleration.y = in_object.rel_acceleration_lat_y;

  out_object.size.x = size_x_[tbl_idx];
  out_object.size.y = size_y_[tbl_idx];
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
  uint8_t radar_id,
  const rclcpp::Time & stamp)
{
  uint32_t tbl_idx = ars408::RADAR_CONNECTIONS_MAX;  // initialize with invalid index
  for (uint32_t i = 0; i < connection_count_; i++) {
    if (radar_id == radar_id_[i]) {
      tbl_idx = i;
      break;
    }
  }
  if (tbl_idx >= ars408::RADAR_CONNECTIONS_MAX) {
    RCLCPP_WARN(get_logger(), "Unregistered RadarID detected: %u", radar_id);
    return;
  }

  can_receive_last_time_[tbl_idx] = this->now();
  radar_msgs::msg::RadarTracks output_objects;
  output_objects.header.frame_id = output_frame_[tbl_idx];
  output_objects.header.stamp = stamp;

  radar_msgs::msg::RadarScan output_scan;
  output_scan.header.frame_id = output_frame_[tbl_idx];
  output_scan.header.stamp = stamp;

  for (const auto & object : detected_objects) {
    if (publish_radar_track_[tbl_idx]) {
      output_objects.tracks.emplace_back(ConvertRadarObjectToRadarTrack(object.second, tbl_idx));
    };
    if (publish_radar_scan_[tbl_idx]) {
      output_scan.returns.emplace_back(ConvertRadarObjectToRadarReturn(object.second));
    }
  }

  if (publish_radar_track_[tbl_idx]) {
    publisher_radar_tracks_[tbl_idx]->publish(output_objects);
  }
  if (publish_radar_scan_[tbl_idx]) {
    publisher_radar_scan_[tbl_idx]->publish(output_scan);
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
  UUID_table_.resize(connection_count_);
  for (size_t j = 0; j < connection_count_; j++) {
    for (size_t i = 0; i <= max_radar_id; i++) {
      UUID_table_[j].emplace_back(PeContinentalArs408Node::GenerateRandomUUID());
    }
  }
}

void PeContinentalArs408Node::SetParameter()
{
  std::unordered_set<std::string> seen;

  connection_count_ = this->declare_parameter<uint8_t>("connection_count");
  if ((0u == connection_count_) || (ars408::RADAR_CONNECTIONS_MAX < connection_count_)) {
    throw std::invalid_argument("'connection_count' out of range[expected 1–8]: " + std::to_string(connection_count_));
  }

  output_frame_ = this->declare_parameter<std::vector<std::string>>("output_frame");
  if (connection_count_ != output_frame_.size()) {
    throw std::invalid_argument("parameter error: mismatch in number of 'output_frame' ("
      + std::to_string(output_frame_.size()) + ") and 'connection_count' ("
      + std::to_string(static_cast<int>(connection_count_)) + ")");
    return;
  }
  for (const auto &s : output_frame_) {
    auto [it, inserted] = seen.insert(s);
    if (!inserted) {
      throw std::invalid_argument("parameter error: duplicate name found in 'output_frame' array");
      return;
    }
  }

  publish_radar_track_ = this->declare_parameter<std::vector<bool>>("publish_radar_track");
  if (connection_count_ != publish_radar_track_.size()) {
    throw std::invalid_argument("parameter error: mismatch in number of 'publish_radar_track' ("
      + std::to_string(publish_radar_track_.size()) + ") and 'connection_count' ("
      + std::to_string(static_cast<int>(connection_count_)) + ")");
    return;
  }

  publish_radar_scan_ = this->declare_parameter<std::vector<bool>>("publish_radar_scan");
  if (connection_count_ != publish_radar_scan_.size()) {
    throw std::invalid_argument("parameter error: mismatch in number of 'publish_radar_scan' ("
      + std::to_string(publish_radar_scan_.size()) + ") and 'connection_count' ("
      + std::to_string(static_cast<int>(connection_count_)) + ")");
    return;
  }

  sequential_publish_ = this->declare_parameter<std::vector<bool>>("sequential_publish");
  if (connection_count_ != sequential_publish_.size()) {
    throw std::invalid_argument("parameter error: mismatch in number of 'sequential_publish' ("
      + std::to_string(sequential_publish_.size()) + ") and 'connection_count' ("
      + std::to_string(static_cast<int>(connection_count_)) + ")");
    return;
  }

  size_x_ = this->declare_parameter<std::vector<double>>("size_x");
  if (connection_count_ != size_x_.size()) {
    throw std::invalid_argument("parameter error: mismatch in number of 'size_x' ("
      + std::to_string(size_x_.size()) + ") and 'connection_count' ("
      + std::to_string(static_cast<int>(connection_count_)) + ")");
    return;
  }

  size_y_ = this->declare_parameter<std::vector<double>>("size_y");
  if (connection_count_ != size_y_.size()) {
    throw std::invalid_argument("parameter error: mismatch in number of 'size_y' ("
      + std::to_string(size_y_.size()) + ") and 'connection_count' ("
      + std::to_string(static_cast<int>(connection_count_)) + ")");
    return;
  }

  auto radar_id = this->declare_parameter<std::vector<int>>("radar_id");
  radar_id_.resize(radar_id.size());
  for (size_t i = 0; i < connection_count_; i++) {
    radar_id_[i] = static_cast<uint8_t>(radar_id[i]);
  }
  if (connection_count_ != radar_id_.size()) {
    throw std::invalid_argument("parameter error: mismatch in number of 'radar_id' ("
      + std::to_string(radar_id_.size()) + ") and 'connection_count' ("
      + std::to_string(static_cast<int>(connection_count_)) + ")");
    return;
  }
  for (size_t i = 0; i < connection_count_; i++) {
    for (size_t j = i + 1; j < connection_count_; j++) {
      if (radar_id_[i] == radar_id_[j]) {
        throw std::invalid_argument("parameter error: duplicate name found in 'radar_id' array");
        return;
      }
    }
  }
  for (size_t i = 0u; i < radar_id_.size(); i++) {
    if (radar_id_[i] >= ars408::RADAR_CONNECTIONS_MAX) {
      throw std::invalid_argument(
        "parameter error: 'radar_id' array has out-of-range entries (expected 0–7)");
      return;
    }
  }

  publish_radar_tracks_name_ = this->declare_parameter<std::vector<std::string>>("publish_objects_name");
  if (connection_count_ != publish_radar_tracks_name_.size()) {
    throw std::invalid_argument("parameter error: mismatch in number of 'publish_objects_name' ("
      + std::to_string(publish_radar_tracks_name_.size()) + ") and 'connection_count' ("
      + std::to_string(static_cast<int>(connection_count_)) + ")");
    return;
  }
  for (const auto &s : publish_radar_tracks_name_) {
    auto [it, inserted] = seen.insert(s);
    if (!inserted) {
      throw std::invalid_argument("parameter error: duplicate name found in 'publish_objects_name' array");
      return;
    }
  }

  publish_radar_scan_name_ = this->declare_parameter<std::vector<std::string>>("publish_scan_name");
  if (connection_count_ != publish_radar_scan_name_.size()) {
    throw std::invalid_argument("parameter error: mismatch in number of 'publish_scan_name' ("
      + std::to_string(publish_radar_scan_name_.size()) + ") and 'connection_count' ("
      + std::to_string(static_cast<int>(connection_count_)) + ")");
    return;
  }
  for (const auto &s : publish_radar_scan_name_) {
    auto [it, inserted] = seen.insert(s);
    if (!inserted) {
      throw std::invalid_argument("parameter error: duplicate name found in 'publish_scan_name' array");
      return;
    }
  }

  can_receive_check_rate_hz_ = this->declare_parameter<double>("can_receive_check_rate_hz");
  can_receive_check_timeout_sec_ = this->declare_parameter<double>("can_receive_check_timeout_sec");
}

void PeContinentalArs408Node::Run()
{
  can_receive_last_time_.fill(std::nullopt);

  std::array<bool, ars408::RADAR_CONNECTIONS_MAX> sequential_publish_by_radar_order;
  sequential_publish_by_radar_order.fill(false);
  for (size_t i = 0; i < connection_count_; i++) {
    sequential_publish_by_radar_order[radar_id_[i]] = sequential_publish_[i];
  }
  ars408_driver_.RegisterDetectedObjectsCallback(
    std::bind(&PeContinentalArs408Node::RadarDetectedObjectsCallback,
      this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
    sequential_publish_by_radar_order);

  subscription_ = this->create_subscription<can_msgs::msg::Frame>(
    "~/input/frame", 10,
    std::bind(&PeContinentalArs408Node::CanFrameCallback, this, std::placeholders::_1));

  publisher_radar_tracks_.resize(connection_count_);
  publisher_radar_scan_.resize(connection_count_);

  for (size_t i = 0; i < connection_count_; i++) {
    publisher_radar_tracks_[i] =
      this->create_publisher<radar_msgs::msg::RadarTracks>(publish_radar_tracks_name_[i], 10);
    publisher_radar_scan_[i] =
      this->create_publisher<radar_msgs::msg::RadarScan>(publish_radar_scan_name_[i], 10);
  }
  diagnostics_pub_ = this->create_publisher<DiagnosticArray>("~/output/diagnostics", rclcpp::QoS{1});

  can_receive_check_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(1000.0 / can_receive_check_rate_hz_)),
    std::bind(&PeContinentalArs408Node::OnCanReceiveCheck, this));

  last_warn_times_.assign(connection_count_, rclcpp::Time(0, 0, this->get_clock()->get_clock_type()));

  diagnostic_published_.assign(connection_count_, false);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PeContinentalArs408Node)
