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

#include "ars408_ros/ars408_can_encoder.hpp"
#include "ars408_ros/ars408_can_parser.hpp"
#include "ars408_ros/ars408_constants.hpp"
#include "ars408_ros/ars408_radar_cfg_verify.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace
{
constexpr double kRadToDeg = 180.0 / M_PI;

ars408::can_encoder::OutputType ParseOutputType(const std::string & value)
{
  if (value == "none") {
    return ars408::can_encoder::OutputType::NONE;
  }
  if (value == "objects") {
    return ars408::can_encoder::OutputType::OBJECTS;
  }
  if (value == "clusters") {
    return ars408::can_encoder::OutputType::CLUSTERS;
  }
  throw std::invalid_argument("radar_cfg.output_type must be none, objects, or clusters");
}

ars408::can_encoder::SortIndex ParseSortIndex(const std::string & value)
{
  if (value == "no_sort") {
    return ars408::can_encoder::SortIndex::NO_SORT;
  }
  if (value == "by_range") {
    return ars408::can_encoder::SortIndex::BY_RANGE;
  }
  if (value == "by_rcs") {
    return ars408::can_encoder::SortIndex::BY_RCS;
  }
  throw std::invalid_argument("radar_cfg.sort_index must be no_sort, by_range, or by_rcs");
}

ars408::can_encoder::RcsThreshold ParseRcsThreshold(const std::string & value)
{
  if (value == "normal") {
    return ars408::can_encoder::RcsThreshold::NORMAL;
  }
  if (value == "high_sensitivity") {
    return ars408::can_encoder::RcsThreshold::HIGH_SENSITIVITY;
  }
  throw std::invalid_argument("radar_cfg.rcs_threshold must be normal or high_sensitivity");
}

}  // namespace

PeContinentalArs408Node::PeContinentalArs408Node(const rclcpp::NodeOptions & node_options)
: Node("ars408_node", node_options)
{
  SetParameter();
  GenerateUUIDTable();
  Run();
}

bool PeContinentalArs408Node::IsRadarOutputEnabled() const
{
  return !require_radar_cfg_sync_ || radar_cfg_applied_;
}

void PeContinentalArs408Node::CanFrameCallback(const can_msgs::msg::Frame::SharedPtr can_msg)
{
  if (can_msg->data.empty()) {
    return;
  }

  if (ars408::can_parser::SensorIdFromCanId(can_msg->id) == radar_id_) {
    can_receive_last_time_ = this->now();
  }

  ars408_driver_.Parse(can_msg->id, can_msg->data, can_msg->dlc, can_msg->header.stamp);
}

void PeContinentalArs408Node::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double linear_x = msg->twist.twist.linear.x;

  if (standstill_ && std::abs(linear_x) > speed_moving_threshold_mps_) {
    standstill_ = false;
  } else if (!standstill_ && std::abs(linear_x) < speed_standstill_threshold_mps_) {
    standstill_ = true;
  }

  std::lock_guard<std::mutex> lock(odometry_mutex_);
  latest_odometry_ = *msg;
}

void PeContinentalArs408Node::PublishMotionCanFrames()
{
  if (!IsRadarOutputEnabled()) {
    return;
  }

  std::optional<nav_msgs::msg::Odometry> odometry;
  {
    std::lock_guard<std::mutex> lock(odometry_mutex_);
    odometry = latest_odometry_;
  }

  if (!odometry) {
    return;
  }

  const double linear_x = odometry->twist.twist.linear.x;
  const double yaw_rate_deg_s = odometry->twist.twist.angular.z * kRadToDeg;

  ars408::can_encoder::SpeedDirection direction = ars408::can_encoder::SpeedDirection::STANDSTILL;
  if (!standstill_) {
    direction = linear_x >= 0.0 ?
      ars408::can_encoder::SpeedDirection::FORWARD :
      ars408::can_encoder::SpeedDirection::BACKWARD;
  }

  const float speed_mps = static_cast<float>(std::abs(linear_x));
  const auto speed_payload = ars408::can_encoder::EncodeSpeedInformation(speed_mps, direction);
  const auto yaw_payload = ars408::can_encoder::EncodeYawRateInformation(
    static_cast<float>(yaw_rate_deg_s));

  const rclcpp::Time stamp = odometry->header.stamp;

  can_msgs::msg::Frame speed_frame;
  speed_frame.header.stamp = stamp;
  speed_frame.id = ars408::can_encoder::CanIdForSensor(ars408::SPEED_INFORMATION_00, radar_id_);
  speed_frame.dlc = 8;
  speed_frame.is_extended = false;
  speed_frame.is_rtr = false;
  speed_frame.is_error = false;
  speed_frame.data = speed_payload;

  can_msgs::msg::Frame yaw_frame;
  yaw_frame.header.stamp = stamp;
  yaw_frame.id = ars408::can_encoder::CanIdForSensor(ars408::YAW_RATE_INFORMATION_00, radar_id_);
  yaw_frame.dlc = 8;
  yaw_frame.is_extended = false;
  yaw_frame.is_rtr = false;
  yaw_frame.is_error = false;
  yaw_frame.data = yaw_payload;

  can_tx_publisher_->publish(speed_frame);
  can_tx_publisher_->publish(yaw_frame);
}

void PeContinentalArs408Node::PublishRadarCfg()
{
  if (!can_tx_publisher_) {
    return;
  }

  const auto payload = ars408::can_encoder::EncodeRadarCfg(radar_cfg_params_);

  can_msgs::msg::Frame cfg_frame;
  cfg_frame.header.stamp = this->now();
  cfg_frame.id = ars408::can_encoder::CanIdForSensor(ars408::RADAR_CFG_00, radar_id_);
  cfg_frame.dlc = 8;
  cfg_frame.is_extended = false;
  cfg_frame.is_rtr = false;
  cfg_frame.is_error = false;
  cfg_frame.data = payload;

  can_tx_publisher_->publish(cfg_frame);
  last_radar_cfg_send_time_ = this->now();

  RCLCPP_INFO(
    get_logger(),
    "Published RadarCfg (0x200) for sensor ID %d (max_distance=%u m)",
    radar_id_, radar_cfg_params_.max_distance_m);
}

void PeContinentalArs408Node::UpdateRadarCfgSync()
{
  if (!require_radar_cfg_sync_ || radar_cfg_applied_) {
    return;
  }

  ars408::RadarState state;
  if (ars408_driver_.GetCurrentRadarState(state)) {
    std::string detail;
    if (ars408::radar_cfg_verify::RadarStateMatchesConfig(
        state, radar_cfg_params_, radar_id_, &detail))
    {
      radar_cfg_applied_ = true;
      radar_cfg_mismatch_detail_.clear();
      RCLCPP_INFO(
        get_logger(), "RadarCfg verified on sensor ID %d (matches YAML)", radar_id_);
      return;
    }
    radar_cfg_mismatch_detail_ = detail;
  } else {
    radar_cfg_mismatch_detail_ = "RadarState (0x201) not received yet";
  }

  const rclcpp::Time now = this->now();
  const bool should_send = !last_radar_cfg_send_time_.has_value() ||
    (now - last_radar_cfg_send_time_.value()).seconds() >= radar_cfg_retry_interval_sec_;
  if (should_send) {
    RCLCPP_WARN(
      get_logger(), "RadarCfg mismatch or pending (%s); re-sending 0x200",
      radar_cfg_mismatch_detail_.c_str());
    PublishRadarCfg();
  }
}

void PeContinentalArs408Node::PublishRadarCfgDiagnostics()
{
  if (!require_radar_cfg_sync_) {
    return;
  }

  DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  DiagnosticStatus diag;
  diag.name = "ars408_radar_cfg";
  diag.hardware_id = output_frame_ + "_id" + std::to_string(radar_id_);

  if (radar_cfg_applied_) {
    diag.level = DiagnosticStatus::OK;
    diag.message = "RadarCfg matches YAML parameters";
  } else if (radar_cfg_mismatch_detail_.find("not received") != std::string::npos) {
    diag.level = DiagnosticStatus::STALE;
    diag.message = "Waiting for RadarCfg to apply (no RadarState yet)";
  } else {
    diag.level = DiagnosticStatus::WARN;
    diag.message = "RadarCfg does not match YAML; re-sending 0x200";
  }

  auto add_kv = [&](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    diag.values.push_back(kv);
  };

  add_kv("cfg_applied", radar_cfg_applied_ ? "true" : "false");
  add_kv("mismatch_detail", radar_cfg_mismatch_detail_);
  if (last_radar_cfg_send_time_.has_value()) {
    add_kv(
      "last_cfg_send_age_sec",
      std::to_string((this->now() - last_radar_cfg_send_time_.value()).seconds()));
  }

  diag_array.status.push_back(diag);
  diagnostics_pub_->publish(diag_array);
}

void PeContinentalArs408Node::PublishRadarStateDiagnostics()
{
  if (!publish_radar_state_diagnostics_ || !IsRadarOutputEnabled()) {
    return;
  }

  DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  DiagnosticStatus diag;
  diag.name = "ars408_radar_state";
  diag.hardware_id = output_frame_ + "_id" + std::to_string(radar_id_);

  ars408::RadarState state;
  if (!ars408_driver_.GetCurrentRadarState(state)) {
    diag.level = DiagnosticStatus::STALE;
    diag.message = "RadarState (0x201) not received yet";
    diag_array.status.push_back(diag);
    diagnostics_pub_->publish(diag_array);
    return;
  }

  diag.level = DiagnosticStatus::OK;
  diag.message = "RadarState OK";

  if (
    state.PersistentError || state.Interference || state.TemperatureError ||
    state.TemporaryError || state.VoltageError)
  {
    diag.level = DiagnosticStatus::ERROR;
    diag.message = "Radar reported hardware or environment error";
  } else if (state.EgoMotionRxStatus != ars408::RadarState::INPUT_OK) {
    diag.level = DiagnosticStatus::WARN;
    diag.message = "Ego motion input not OK on radar";
  }

  auto add_kv = [&](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    diag.values.push_back(kv);
  };

  add_kv("sensor_id", std::to_string(state.SensorID));
  add_kv("max_distance_m", std::to_string(state.MaxDistance));
  add_kv("output_type", std::to_string(static_cast<int>(state.OutputType)));
  add_kv("send_quality", state.SendQuality == ars408::RadarState::ACTIVE ? "true" : "false");
  add_kv("send_ext_info", state.SendExtInfo == ars408::RadarState::ACTIVE ? "true" : "false");
  add_kv(
    "ego_motion_rx_status", std::to_string(static_cast<int>(state.EgoMotionRxStatus)));
  add_kv("persistent_error", state.PersistentError ? "true" : "false");
  add_kv("interference", state.Interference ? "true" : "false");
  add_kv("voltage_error", state.VoltageError ? "true" : "false");
  add_kv("nvm_read_status", state.NvmReadStatus ? "success" : "failed");
  add_kv("nvm_write_status", state.NvmWriteStatus ? "success" : "failed");

  ars408::can_parser::VersionId version;
  if (ars408_driver_.GetVersionId(version)) {
    add_kv(
      "firmware_version",
      std::to_string(version.major) + "." + std::to_string(version.minor) + "." +
      std::to_string(version.patch));
    add_kv("extended_range", version.extended_range ? "true" : "false");
  }

  diag_array.status.push_back(diag);
  diagnostics_pub_->publish(diag_array);
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

  UpdateRadarCfgSync();
  PublishRadarCfgDiagnostics();
  PublishRadarStateDiagnostics();
}

ars408::can_encoder::RadarCfgParams PeContinentalArs408Node::LoadRadarCfgParams()
{
  ars408::can_encoder::RadarCfgParams params;

  const bool update_sensor_id =
    declare_parameter<bool>("radar_cfg.update_sensor_id", false);
  params.update_sensor_id = update_sensor_id;
  params.update_max_distance = true;
  params.update_radar_power = true;
  params.update_output_type = true;
  params.update_send_quality = true;
  params.update_send_ext_info = true;
  params.update_sort_index = true;
  params.update_store_in_nvm = true;
  params.update_ctrl_relay = true;
  params.update_rcs_threshold = true;

  params.max_distance_m = static_cast<uint16_t>(
    declare_parameter<int>("radar_cfg.max_distance_m", 260));
  params.sensor_id = radar_id_;
  params.output_type = ParseOutputType(
    declare_parameter<std::string>("radar_cfg.output_type", "objects"));
  // Standard (0 dB) is not selectable (Japan radio regulations); attenuated levels only.
  params.radar_power = ars408::can_encoder::ParseRadarPowerSetting(
    declare_parameter<std::string>("radar_cfg.radar_power", "minus_3db"));
  params.send_quality = declare_parameter<bool>("radar_cfg.send_quality", true);
  params.send_ext_info = declare_parameter<bool>("radar_cfg.send_ext_info", true);
  params.sort_index = ParseSortIndex(
    declare_parameter<std::string>("radar_cfg.sort_index", "by_range"));
  params.store_in_nvm = declare_parameter<bool>("radar_cfg.store_in_nvm", false);
  params.rcs_threshold = ParseRcsThreshold(
    declare_parameter<std::string>("radar_cfg.rcs_threshold", "normal"));
  params.ctrl_relay = declare_parameter<bool>("radar_cfg.ctrl_relay", false);

  if (params.max_distance_m < 2 || params.max_distance_m > 2046) {
    throw std::invalid_argument("radar_cfg.max_distance_m must be in [2, 2046]");
  }

  return params;
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
  auto track = ars408::radar_msgs_conversion::ToRadarTrack(
    in_object, track_conversion_options_,
    [this](const ars408::Obj_3_Extended::ObjectClassProperty & radar_class) {
      return ConvertRadarClassToAwSemanticClass(radar_class);
    });
  track.uuid = UUID_table_[in_object.id];
  return track;
}

radar_msgs::msg::RadarReturn PeContinentalArs408Node::ConvertRadarObjectToRadarReturn(
  const ars408::RadarObject & in_object)
{
  return ars408::radar_msgs_conversion::ToRadarReturn(in_object);
}

void PeContinentalArs408Node::RadarDetectedObjectsCallback(
  const std::unordered_map<uint8_t, ars408::RadarObject> & detected_objects,
  const rclcpp::Time & stamp)
{
  if (!IsRadarOutputEnabled()) {
    return;
  }

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
  track_conversion_options_.fallback_size_x = size_x_;
  track_conversion_options_.fallback_size_y = size_y_;
  track_conversion_options_.use_radar_reported_dimensions =
    declare_parameter<bool>("use_radar_reported_dimensions", true);
  track_conversion_options_.inflate_covariance_by_existence_probability =
    declare_parameter<bool>("inflate_covariance_by_existence_probability", true);
  publish_objects_name_ = this->declare_parameter<std::string>("publish_objects_name");
  publish_scan_name_ = this->declare_parameter<std::string>("publish_scan_name");
  can_receive_check_rate_hz_ = this->declare_parameter<double>("can_receive_check_rate_hz");
  can_receive_check_timeout_sec_ = this->declare_parameter<double>("can_receive_check_timeout_sec");

  publish_motion_input_ = this->declare_parameter<bool>("publish_motion_input", true);
  motion_publish_rate_hz_ = this->declare_parameter<double>("motion_publish_rate_hz", 50.0);
  speed_standstill_threshold_mps_ =
    declare_parameter<double>("speed_standstill_threshold_mps", 0.5);
  speed_moving_threshold_mps_ = declare_parameter<double>("speed_moving_threshold_mps", 2.0);

  if (motion_publish_rate_hz_ <= 0.0) {
    throw std::invalid_argument("motion_publish_rate_hz must be positive");
  }

  require_radar_cfg_sync_ = declare_parameter<bool>("publish_radar_cfg_on_startup", true);
  radar_cfg_startup_delay_sec_ =
    declare_parameter<double>("radar_cfg_startup_delay_sec", 1.0);
  radar_cfg_retry_interval_sec_ =
    declare_parameter<double>("radar_cfg_retry_interval_sec", 1.0);
  publish_radar_state_diagnostics_ =
    declare_parameter<bool>("publish_radar_state_diagnostics", true);

  if (radar_cfg_retry_interval_sec_ <= 0.0) {
    throw std::invalid_argument("radar_cfg_retry_interval_sec must be positive");
  }

  if (require_radar_cfg_sync_) {
    radar_cfg_applied_ = false;
    radar_cfg_params_ = LoadRadarCfgParams();
  } else {
    radar_cfg_applied_ = true;
  }
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

  can_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
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

  const bool needs_can_tx = publish_motion_input_ || require_radar_cfg_sync_;
  if (needs_can_tx) {
    can_tx_publisher_ =
      this->create_publisher<can_msgs::msg::Frame>("~/output/to_can_bus", rclcpp::QoS(10));
  }

  if (publish_motion_input_) {
    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "~/input/odometry", rclcpp::QoS(10),
      std::bind(&PeContinentalArs408Node::OdometryCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / motion_publish_rate_hz_);
    motion_publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PeContinentalArs408Node::PublishMotionCanFrames, this));
  }

  if (require_radar_cfg_sync_) {
    radar_cfg_startup_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(radar_cfg_startup_delay_sec_)),
      [this]() {
        if (!radar_cfg_applied_) {
          PublishRadarCfg();
        }
        radar_cfg_startup_timer_->cancel();
      });
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PeContinentalArs408Node)
