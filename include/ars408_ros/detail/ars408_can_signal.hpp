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

#ifndef ARS408_ROS__DETAIL__ARS408_CAN_SIGNAL_HPP_
#define ARS408_ROS__DETAIL__ARS408_CAN_SIGNAL_HPP_

#include <array>
#include <cstdint>

namespace ars408
{
namespace can_signal
{

/// Maps a signal bit offset to a CAN payload byte/bit (ARS408 Standard Radar Interface layout).
///
/// ARS408 encoding rule:
///   - Within a byte : bit positions go small → large (LSB first, standard).
///   - Across bytes  : overflow always continues at bit 0 of the PREVIOUS (lower-index) byte.
///                     i.e. bytes go large-index → small-index (high CAN address → low CAN address).
///
/// This applies uniformly whether the start bit is byte-aligned or not.
/// Example: RadarDevice_Speed LSB@8, MSB@4 → bit sequence 8,9,…,15,0,1,2,3,4
///          RadarCfg_MaxDistance   LSB@22,MSB@15 → bit sequence 22,23,8,9,…,15
inline void SignalBitPosition(
  const uint16_t start_bit, const uint8_t offset, uint8_t & out_byte_index, uint8_t & out_bit_index)
{
  const uint8_t start_byte = static_cast<uint8_t>(start_bit / 8u);
  const uint8_t start_bit_in_byte = static_cast<uint8_t>(start_bit % 8u);
  const uint8_t bits_until_byte_end = static_cast<uint8_t>(8u - start_bit_in_byte);
  if (offset < bits_until_byte_end) {
    out_byte_index = start_byte;
    out_bit_index = static_cast<uint8_t>(start_bit_in_byte + offset);
    return;
  }

  // Overflow: always move to the previous (lower-index) byte.
  const uint8_t remaining = static_cast<uint8_t>(offset - bits_until_byte_end);
  out_byte_index = static_cast<uint8_t>(start_byte - 1u - remaining / 8u);
  out_bit_index = static_cast<uint8_t>(remaining % 8u);
}

inline uint32_t UnpackSignalIntel(
  const std::array<uint8_t, 8> & data, const uint16_t start_bit, const uint8_t length)
{
  uint32_t raw = 0;
  for (uint8_t i = 0; i < length; ++i) {
    uint8_t byte_index = 0;
    uint8_t bit_index = 0;
    SignalBitPosition(start_bit, i, byte_index, bit_index);
    if ((data[byte_index] >> bit_index) & 0x01u) {
      raw |= (1u << i);
    }
  }
  return raw;
}

inline void PackSignalIntel(
  std::array<uint8_t, 8> & data, const uint16_t start_bit, const uint8_t length,
  const uint32_t raw)
{
  for (uint8_t i = 0; i < length; ++i) {
    if ((raw >> i) & 0x01u) {
      uint8_t byte_index = 0;
      uint8_t bit_index = 0;
      SignalBitPosition(start_bit, i, byte_index, bit_index);
      data[byte_index] |= static_cast<uint8_t>(1u << bit_index);
    }
  }
}

}  // namespace can_signal
}  // namespace ars408

#endif  // ARS408_ROS__DETAIL__ARS408_CAN_SIGNAL_HPP_
