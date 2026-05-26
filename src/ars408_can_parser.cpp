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

namespace ars408
{
namespace can_parser
{

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
  obj_quality.Id = in_can_data[0];
  const uint8_t prob_tmp = (in_can_data[6] & 0x1Cu) >> 2u;
  switch (prob_tmp) {
    case 0x00u: obj_quality.ExistenceProbability = 0; break;
    case 0x01u: obj_quality.ExistenceProbability = 0.25; break;
    case 0x02u: obj_quality.ExistenceProbability = 0.5; break;
    case 0x03u: obj_quality.ExistenceProbability = 0.75; break;
    case 0x04u: obj_quality.ExistenceProbability = 0.9; break;
    case 0x05u: obj_quality.ExistenceProbability = 0.99; break;
    case 0x06u: obj_quality.ExistenceProbability = 0.999; break;
    case 0x07u: obj_quality.ExistenceProbability = 1; break;
    default: obj_quality.ExistenceProbability = 0; break;
  }
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

}  // namespace can_parser
}  // namespace ars408
