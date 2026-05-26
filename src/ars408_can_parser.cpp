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

#include "ars408_ros/ars408_can_parser.hpp"

#include <algorithm>

namespace ars408
{
namespace can_parser
{
namespace
{

uint32_t unpackSignalIntel(
  const std::array<uint8_t, 8> & data, const uint16_t start_bit, const uint8_t length)
{
  uint32_t raw = 0;
  for (uint8_t i = 0; i < length; ++i) {
    const uint16_t bit_index = start_bit + i;
    if ((data[bit_index / 8] >> (bit_index % 8)) & 0x01u) {
      raw |= (1u << i);
    }
  }
  return raw;
}

// Table 45 upper bounds (Standard Radar Interface v1.12).
constexpr float kRmsDistanceM[] = {
  0.005f, 0.007f, 0.010f, 0.014f, 0.020f, 0.029f, 0.041f, 0.058f, 0.082f, 0.116f, 0.165f,
  0.234f, 0.332f, 0.471f, 0.669f, 0.949f, 1.346f, 1.909f, 2.709f, 3.843f, 5.452f, 7.732f,
  10.98f, 15.57f, 22.10f, 31.35f, 44.44f, 63.00f, 89.32f, 126.7f, 179.7f, 255.0f};
constexpr float kRmsVelocityMps[] = {
  0.005f, 0.006f, 0.008f, 0.011f, 0.014f, 0.018f, 0.023f, 0.029f, 0.038f, 0.049f, 0.063f,
  0.081f, 0.105f, 0.135f, 0.174f, 0.224f, 0.288f, 0.371f, 0.478f, 0.616f, 0.794f, 1.023f,
  1.317f, 1.697f, 2.187f, 2.817f, 3.630f, 4.676f, 6.025f, 7.762f, 10.00f, 12.90f};
constexpr float kRmsOrientationDeg[] = {
  0.005f, 0.007f, 0.010f, 0.014f, 0.020f, 0.029f, 0.041f, 0.058f, 0.082f, 0.116f, 0.165f,
  0.234f, 0.332f, 0.471f, 0.669f, 0.949f, 1.346f, 1.909f, 2.709f, 3.843f, 5.452f, 7.732f,
  10.98f, 15.57f, 22.10f, 31.35f, 44.44f, 63.00f, 89.32f, 126.7f, 179.7f, 255.0f};

float LookupRms(const float * table, const size_t table_size, const uint8_t index)
{
  const size_t idx = std::min(static_cast<size_t>(index), table_size - 1u);
  return table[idx];
}

float DecodeExistenceProbability(const uint8_t prob_index)
{
  switch (prob_index) {
    case 0x00u: return 0.f;
    case 0x01u: return 0.25f;
    case 0x02u: return 0.5f;
    case 0x03u: return 0.75f;
    case 0x04u: return 0.9f;
    case 0x05u: return 0.99f;
    case 0x06u: return 0.999f;
    case 0x07u: return 1.f;
    default: return 0.f;
  }
}

}  // namespace

bool HasMinimumDlc(const uint8_t dlc, const uint8_t min_length)
{
  return dlc >= min_length;
}

uint8_t SensorIdFromCanId(const uint32_t can_id)
{
  return static_cast<uint8_t>((can_id & 0x00000070u) / 0x10u);
}

void ParseRadarState(const std::array<uint8_t, 8> & in_can_data, RadarState & out_state)
{
  out_state.NvmWriteStatus = ((in_can_data[0] & 0x80u) >> 7u);
  out_state.NvmReadStatus = ((in_can_data[0] & 0x40u) >> 6u);

  const uint16_t distance =
    static_cast<uint16_t>(
    ((((in_can_data[1] & 0xFFu) << 2u) & 0xFFFFu) + ((in_can_data[2] & 0xC0u) >> 6u)) << 1u);
  out_state.MaxDistance = distance;
  out_state.PersistentError = (in_can_data[2] & 0x20u) >> 5u;
  out_state.Interference = (in_can_data[2] & 0x10u) >> 4u;
  out_state.TemperatureError = (in_can_data[2] & 0x08u) >> 3u;
  out_state.TemporaryError = (in_can_data[2] & 0x04u) >> 2u;
  out_state.VoltageError = (in_can_data[2] & 0x02u) >> 1u;
  out_state.SensorID = (in_can_data[4] & 0x07u);
  out_state.SortingMode = RadarState::SortingConfig((in_can_data[4] & 0x70u) >> 4u);
  out_state.PowerMode =
    RadarState::PowerConfig((in_can_data[3] << 1u) + ((in_can_data[4] & 0x80u) >> 7u));
  out_state.EgoMotionRxStatus = RadarState::MotionRx((in_can_data[5] & 0xC0u) >> 6u);
  out_state.SendExtInfo = RadarState::Config((in_can_data[5] & 0x20u) >> 5u);
  out_state.SendQuality = RadarState::Config((in_can_data[5] & 0x10u) >> 4u);
  out_state.OutputType = RadarState::OutputTypeConfig((in_can_data[5] & 0x0Cu) >> 2u);
  out_state.CtrlRelay = RadarState::Config((in_can_data[5] & 0x02u) >> 1u);
  // RadarState_RCS_Threshold: start bit 58, length 3 (Intel / little-endian)
  out_state.Rcs_Threshold =
    RadarState::Rcs_ThresholdConfig((in_can_data[7] >> 2u) & 0x07u);
}

void ParseObjectListStatus(const std::array<uint8_t, 8> & in_can_data, Obj_0_Status & out_status)
{
  out_status.NumberOfObjects = in_can_data[0];
  out_status.MeasurementCounter =
    static_cast<uint16_t>(in_can_data[2]) |
    (static_cast<uint16_t>(in_can_data[3]) << 8u);
  out_status.InterfaceVersion = static_cast<uint8_t>((in_can_data[3] >> 4u) & 0x0Fu);
}

RadarObject ParseObjectGeneral(
  const std::array<uint8_t, 8> & in_can_data, const uint16_t measurement_counter)
{
  RadarObject current_object;
  current_object.sequence_id = measurement_counter;
  current_object.id = in_can_data[0];
  current_object.dynamic_property = Obj_1_General::DynamicProperty(in_can_data[6] & 0x07u);
  current_object.rcs = (in_can_data[7] * 0.5) - 64.0;

  const uint16_t dist_x_tmp = static_cast<uint16_t>(in_can_data[1] << 5u) +
    static_cast<uint16_t>((in_can_data[2] & 0xF8u) >> 3u);
  current_object.distance_long_x = dist_x_tmp * 0.2f - 500.0f;

  const uint16_t dist_y_tmp = static_cast<uint16_t>((in_can_data[2] & 0x07u) << 8u) +
    static_cast<uint16_t>(in_can_data[3]);
  current_object.distance_lat_y = dist_y_tmp * 0.2f - 204.6f;

  const uint16_t speed_x_tmp = static_cast<uint16_t>(in_can_data[4] << 2u) +
    static_cast<uint16_t>((in_can_data[5] & 0xC0u) >> 6u);
  current_object.speed_long_x = (speed_x_tmp * 0.25f) - 128.0f;

  const uint16_t speed_y_tmp = static_cast<uint16_t>((in_can_data[5] & 0x3Fu) << 3u) +
    static_cast<uint16_t>((in_can_data[6] & 0xE0u) >> 5u);
  current_object.speed_lat_y = (speed_y_tmp * 0.25f) - 64.0f;
  return current_object;
}

Obj_2_Quality ParseObjectQuality(const std::array<uint8_t, 8> & in_can_data)
{
  Obj_2_Quality obj_quality;
  obj_quality.Id = static_cast<uint8_t>(unpackSignalIntel(in_can_data, 0, 8));

  const uint8_t dist_long_idx = static_cast<uint8_t>(unpackSignalIntel(in_can_data, 11, 5));
  const uint8_t vrel_long_idx = static_cast<uint8_t>(unpackSignalIntel(in_can_data, 17, 5));
  const uint8_t dist_lat_idx = static_cast<uint8_t>(unpackSignalIntel(in_can_data, 22, 5));
  const uint8_t vrel_lat_idx = static_cast<uint8_t>(unpackSignalIntel(in_can_data, 28, 5));
  const uint8_t arel_lat_idx = static_cast<uint8_t>(unpackSignalIntel(in_can_data, 34, 5));
  const uint8_t arel_long_idx = static_cast<uint8_t>(unpackSignalIntel(in_can_data, 39, 5));
  const uint8_t orient_idx = static_cast<uint8_t>(unpackSignalIntel(in_can_data, 45, 5));

  obj_quality.LongitudinalDistanceXRms =
    LookupRms(kRmsDistanceM, sizeof(kRmsDistanceM) / sizeof(float), dist_long_idx);
  obj_quality.LateralDistanceYRms =
    LookupRms(kRmsDistanceM, sizeof(kRmsDistanceM) / sizeof(float), dist_lat_idx);
  obj_quality.RelativeLongitudinalVelocityXRms =
    LookupRms(kRmsVelocityMps, sizeof(kRmsVelocityMps) / sizeof(float), vrel_long_idx);
  obj_quality.RelativeLateralVelocityYRms =
    LookupRms(kRmsVelocityMps, sizeof(kRmsVelocityMps) / sizeof(float), vrel_lat_idx);
  obj_quality.RelativeLongitudinalAccelerationXRms =
    LookupRms(kRmsVelocityMps, sizeof(kRmsVelocityMps) / sizeof(float), arel_long_idx);
  obj_quality.RelativeLateralAccelerationYRms =
    LookupRms(kRmsVelocityMps, sizeof(kRmsVelocityMps) / sizeof(float), arel_lat_idx);
  obj_quality.OrientationAngleRms =
    LookupRms(
    kRmsOrientationDeg, sizeof(kRmsOrientationDeg) / sizeof(float), orient_idx);

  obj_quality.MeasState = static_cast<uint8_t>(unpackSignalIntel(in_can_data, 50, 3));
  const uint8_t prob_index = static_cast<uint8_t>(unpackSignalIntel(in_can_data, 53, 3));
  obj_quality.ExistenceProbability = DecodeExistenceProbability(prob_index);

  return obj_quality;
}

Obj_3_Extended ParseObjectExtended(const std::array<uint8_t, 8> & in_can_data)
{
  Obj_3_Extended obj_extended;
  obj_extended.Id = in_can_data[0];
  const uint16_t tmp_rel_acc_x = static_cast<uint16_t>(in_can_data[1] << 3u) +
    static_cast<uint16_t>((in_can_data[2] & 0xE0u) >> 5u);
  obj_extended.RelativeLongitudinalAccelerationX = tmp_rel_acc_x * 0.01f - 10.f;

  const uint16_t tmp_rel_acc_y = static_cast<uint16_t>((in_can_data[2] & 0x1Fu) << 4u) +
    static_cast<uint16_t>((in_can_data[3] & 0xF0u) >> 4u);
  obj_extended.RelativeLateralAccelerationY = tmp_rel_acc_y * 0.01f - 2.5f;

  const uint8_t tmp_class = in_can_data[3] & 0x07u;
  switch (tmp_class) {
    case 0x00u:
      obj_extended.ObjectClass = Obj_3_Extended::ObjectClassProperty::POINT;
      break;
    case 0x01u:
      obj_extended.ObjectClass = Obj_3_Extended::ObjectClassProperty::CAR;
      break;
    case 0x02u:
      obj_extended.ObjectClass = Obj_3_Extended::ObjectClassProperty::TRUCK;
      break;
    case 0x04u:
      obj_extended.ObjectClass = Obj_3_Extended::ObjectClassProperty::MOTORCYCLE;
      break;
    case 0x05u:
      obj_extended.ObjectClass = Obj_3_Extended::ObjectClassProperty::BICYCLE;
      break;
    case 0x06u:
      obj_extended.ObjectClass = Obj_3_Extended::ObjectClassProperty::WIDE;
      break;
    default:
      obj_extended.ObjectClass = Obj_3_Extended::ObjectClassProperty::RESERVED_01;
      break;
  }

  const uint16_t tmp_angle = static_cast<uint16_t>(in_can_data[4] << 2u) +
    static_cast<uint16_t>((in_can_data[5] & 0xC0u) >> 6u);
  obj_extended.OrientationAngle = tmp_angle * 0.4f - 180.f;
  obj_extended.Length = in_can_data[6] * 0.2f;
  obj_extended.Width = in_can_data[7] * 0.2f;
  return obj_extended;
}

VersionId ParseVersionId(const std::array<uint8_t, 8> & in_can_data)
{
  VersionId version;
  version.major = in_can_data[0];
  version.minor = in_can_data[1];
  version.patch = in_can_data[2];
  version.country_code_restricted = (in_can_data[3] & 0x01u) != 0u;
  version.extended_range = (in_can_data[3] & 0x02u) != 0u;
  return version;
}

filter_signals::FilterStateHeader ParseFilterStateHeader(
  const std::array<uint8_t, 8> & in_can_data)
{
  filter_signals::FilterStateHeader header;
  header.cluster_filter_count =
    static_cast<uint8_t>(unpackSignalIntel(in_can_data, 3, 5));
  header.object_filter_count =
    static_cast<uint8_t>(unpackSignalIntel(in_can_data, 11, 5));
  return header;
}

filter_signals::FilterStateCfg ParseFilterStateCfg(const std::array<uint8_t, 8> & in_can_data)
{
  filter_signals::FilterStateCfg state;
  const uint8_t raw_index = static_cast<uint8_t>(unpackSignalIntel(in_can_data, 3, 4));
  state.index = filter_signals::FilterIndexFromRaw(raw_index);
  state.active = unpackSignalIntel(in_can_data, 2, 1) != 0u;
  state.for_objects = unpackSignalIntel(in_can_data, 7, 1) != 0u;

  const uint8_t value_bits = filter_signals::FilterIndexUses13BitRange(state.index) ? 13u : 12u;
  const uint32_t raw_min = unpackSignalIntel(in_can_data, 16, value_bits);
  const uint32_t raw_max = unpackSignalIntel(in_can_data, 32, value_bits);
  state.min_value = filter_signals::DecodeRawMin(state.index, raw_min);
  state.max_value = filter_signals::DecodeRawMax(state.index, raw_max);
  return state;
}

}  // namespace can_parser
}  // namespace ars408
