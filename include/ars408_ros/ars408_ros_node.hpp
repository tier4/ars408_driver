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

#ifndef ARS408_ROS__ARS408_ROS_NODE_HPP_
#define ARS408_ROS__ARS408_ROS_NODE_HPP_

#include "ars408_ros/ars408_can_encoder.hpp"
#include "ars408_ros/ars408_driver.hpp"
#include "ars408_ros/ars408_filter_signals.hpp"
#include "ars408_ros/ars408_radar_msgs_conversion.hpp"
#include "can_msgs/msg/frame.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "radar_msgs/msg/radar_scan.hpp"
#include "radar_msgs/msg/radar_tracks.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using DiagnosticArrayPublisher = rclcpp::Publisher<DiagnosticArray>;

class PeContinentalArs408Node : public rclcpp::Node
{
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_publisher_;
  rclcpp::Publisher<radar_msgs::msg::RadarTracks>::SharedPtr publisher_radar_tracks_;
  rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr publisher_radar_scan_;
  DiagnosticArrayPublisher::SharedPtr diagnostics_pub_;

  std::string output_frame_;
  bool publish_radar_track_;
  bool publish_radar_scan_;
  bool sequential_publish_;
  double size_x_;
  double size_y_;
  ars408::radar_msgs_conversion::TrackConversionOptions track_conversion_options_;
  uint8_t radar_id_;
  std::string publish_objects_name_;
  std::string publish_scan_name_;
  double can_receive_check_rate_hz_;
  double can_receive_check_timeout_sec_;

  bool publish_motion_input_;
  double motion_publish_rate_hz_;
  double speed_standstill_threshold_mps_;
  double speed_moving_threshold_mps_;
  bool standstill_{true};
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;
  std::mutex odometry_mutex_;

  bool require_radar_cfg_sync_{false};
  bool radar_cfg_applied_{true};
  double radar_cfg_startup_delay_sec_;
  double radar_cfg_retry_interval_sec_;
  bool publish_radar_state_diagnostics_;
  ars408::can_encoder::RadarCfgParams radar_cfg_params_;
  std::optional<rclcpp::Time> last_radar_cfg_send_time_;
  std::string radar_cfg_mismatch_detail_;

  bool send_filter_cfg_on_startup_{false};
  bool filter_cfg_applied_{true};
  double filter_cfg_startup_delay_sec_;
  double filter_cfg_inter_send_delay_sec_;
  double filter_cfg_retry_interval_sec_;
  std::vector<ars408::filter_signals::FilterCfgEntry> filter_cfg_entries_;
  size_t filter_cfg_send_index_{0};
  std::optional<rclcpp::Time> last_filter_cfg_send_time_;
  std::optional<rclcpp::Time> filter_cfg_sequence_start_time_;
  std::string filter_cfg_mismatch_detail_;

  const uint8_t max_radar_id = 255;
  std::vector<unique_identifier_msgs::msg::UUID> UUID_table_;
  rclcpp::TimerBase::SharedPtr can_receive_check_timer_;
  rclcpp::TimerBase::SharedPtr motion_publish_timer_;
  rclcpp::TimerBase::SharedPtr radar_cfg_startup_timer_;
  rclcpp::TimerBase::SharedPtr filter_cfg_startup_timer_;
  std::optional<rclcpp::Time> can_receive_last_time_;
  rclcpp::Time last_warn_time_;

  ars408::Ars408Driver ars408_driver_{};

  bool IsRadarOutputEnabled() const;

  void CanFrameCallback(const can_msgs::msg::Frame::SharedPtr can_msg);
  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void PublishMotionCanFrames();
  void PublishRadarCfg();
  void UpdateRadarCfgSync();
  void PublishFilterCfgEntry(const ars408::filter_signals::FilterCfgEntry & entry);
  void PublishFilterCfgSequenceStep();
  void UpdateFilterCfgSync();
  void OnCanReceiveCheck();
  void PublishRadarCfgDiagnostics();
  void PublishFilterCfgDiagnostics();
  void PublishRadarStateDiagnostics();
  void GenerateUUIDTable();
  void SetParameter();
  ars408::can_encoder::RadarCfgParams LoadRadarCfgParams();
  std::vector<ars408::filter_signals::FilterCfgEntry> LoadFilterCfgEntries();
  std::vector<std::string> ListFilterCriteriaNames();

  radar_msgs::msg::RadarTrack ConvertRadarObjectToRadarTrack(const ars408::RadarObject & in_object);
  radar_msgs::msg::RadarReturn ConvertRadarObjectToRadarReturn(
    const ars408::RadarObject & in_object);
  radar_msgs::msg::RadarReturn ConvertRadarClusterToRadarReturn(
    const ars408::RadarCluster & cluster);

  unique_identifier_msgs::msg::UUID GenerateRandomUUID();
  uint32_t ConvertRadarClassToAwSemanticClass(
    const ars408::Obj_3_Extended::ObjectClassProperty & in_radar_class);

public:
  explicit PeContinentalArs408Node(const rclcpp::NodeOptions & node_options);
  void RadarDetectedObjectsCallback(
    const std::unordered_map<uint8_t, ars408::RadarObject> & detected_objects,
    const rclcpp::Time & stamp);
  void ClusterListCallback(
    const std::unordered_map<uint8_t, ars408::RadarCluster> & detected_clusters,
    const rclcpp::Time & stamp);
  void Run();
};

#endif  // ARS408_ROS__ARS408_ROS_NODE_HPP_
