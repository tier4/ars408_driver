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

#include "ars408_ros/ars408_can_encoder.hpp"
#include "ars408_ros/ars408_can_parser.hpp"
#include "ars408_ros/ars408_filter_cfg_verify.hpp"
#include "ars408_ros/ars408_filter_signals.hpp"

#include <gtest/gtest.h>

#include <array>

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
}  // namespace

TEST(Ars408FilterCfg, EncodeDistanceFilter)
{
  ars408::filter_signals::FilterCfgEntry entry;
  entry.index = ars408::filter_signals::FilterIndex::DISTANCE;
  entry.active = true;
  entry.for_objects = true;
  entry.min_value = 0.0;
  entry.max_value = 260.0;

  const auto data = ars408::can_encoder::EncodeFilterCfg(entry);

  EXPECT_EQ(unpackSignalIntel(data, 1, 1), 1u);
  EXPECT_EQ(unpackSignalIntel(data, 2, 1), 1u);
  EXPECT_EQ(unpackSignalIntel(data, 3, 4), 1u);
  EXPECT_EQ(unpackSignalIntel(data, 7, 1), 1u);
  EXPECT_EQ(unpackSignalIntel(data, 32, 12), 2600u);
}

TEST(Ars408FilterCfg, EncodeNofObjMaxOnly)
{
  ars408::filter_signals::FilterCfgEntry entry;
  entry.index = ars408::filter_signals::FilterIndex::NOF_OBJ;
  entry.active = true;
  entry.for_objects = true;
  entry.max_value = 64.0;

  const auto data = ars408::can_encoder::EncodeFilterCfg(entry);

  EXPECT_EQ(unpackSignalIntel(data, 16, 12), 0u);
  EXPECT_EQ(unpackSignalIntel(data, 32, 12), 64u);
}

TEST(Ars408FilterCfg, ParseFilterStateCfgRoundTrip)
{
  ars408::filter_signals::FilterCfgEntry entry;
  entry.index = ars408::filter_signals::FilterIndex::POS_X;
  entry.active = true;
  entry.for_objects = true;
  entry.min_value = 0.0;
  entry.max_value = 200.0;

  const auto encoded = ars408::can_encoder::EncodeFilterCfg(entry);
  const auto state = ars408::can_parser::ParseFilterStateCfg(encoded);

  EXPECT_EQ(state.index, entry.index);
  EXPECT_TRUE(state.active);
  EXPECT_TRUE(state.for_objects);
  EXPECT_NEAR(state.max_value, 200.0, 0.25);
}

TEST(Ars408FilterCfg, ParseFilterStateHeader)
{
  std::array<uint8_t, 8> data{};
  data[0] = 0x08u;  // 1 cluster filter at bit 3
  data[1] = 0x10u;  // 2 object filters at bit 11

  const auto header = ars408::can_parser::ParseFilterStateHeader(data);
  EXPECT_EQ(header.cluster_filter_count, 1u);
  EXPECT_EQ(header.object_filter_count, 2u);
}

TEST(Ars408FilterCfg, VerifyDeactivatedEntryIgnoresMinMax)
{
  ars408::filter_signals::FilterCfgEntry expected;
  expected.index = ars408::filter_signals::FilterIndex::DISTANCE;
  expected.active = false;
  expected.for_objects = true;
  expected.min_value = 0.0;
  expected.max_value = 999.0;

  ars408::filter_signals::FilterStateCfg state;
  state.index = expected.index;
  state.active = false;
  state.for_objects = true;
  state.min_value = 10.0;
  state.max_value = 50.0;

  EXPECT_TRUE(ars408::filter_cfg_verify::FilterStateMatchesEntry(state, expected));
}

TEST(Ars408FilterCfg, EncodeDeactivateDistanceFilter)
{
  ars408::filter_signals::FilterCfgEntry entry;
  entry.index = ars408::filter_signals::FilterIndex::DISTANCE;
  entry.active = false;
  entry.for_objects = true;

  const auto data = ars408::can_encoder::EncodeFilterCfg(entry);
  EXPECT_EQ(unpackSignalIntel(data, 2, 1), 0u);
}

TEST(Ars408FilterCfg, VerifyMatchingEntries)
{
  ars408::filter_signals::FilterCfgEntry expected;
  expected.index = ars408::filter_signals::FilterIndex::DISTANCE;
  expected.active = true;
  expected.for_objects = true;
  expected.min_value = 0.0;
  expected.max_value = 100.0;

  ars408::filter_signals::FilterStateCfg state;
  state.index = expected.index;
  state.active = expected.active;
  state.for_objects = expected.for_objects;
  state.min_value = 0.0;
  state.max_value = 100.0;

  EXPECT_TRUE(ars408::filter_cfg_verify::FilterStateMatchesEntry(state, expected));
  EXPECT_TRUE(ars408::filter_cfg_verify::AllFilterEntriesMatch({expected}, {state}));
}

TEST(Ars408FilterCfg, ParseFilterIndexNames)
{
  EXPECT_EQ(
    ars408::filter_signals::ParseFilterIndexSetting("distance"),
    ars408::filter_signals::FilterIndex::DISTANCE);
  EXPECT_EQ(
    ars408::filter_signals::ParseFilterIndexSetting("pos_x"),
    ars408::filter_signals::FilterIndex::POS_X);
}
