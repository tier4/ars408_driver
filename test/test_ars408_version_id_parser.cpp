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

#include <gtest/gtest.h>

#include <array>

TEST(Ars408CanParser, ParseVersionId)
{
  std::array<uint8_t, 8> data{4, 30, 2, 0x02, 0, 0, 0, 0};
  const ars408::can_parser::VersionId version = ars408::can_parser::ParseVersionId(data);

  EXPECT_EQ(version.major, 4u);
  EXPECT_EQ(version.minor, 30u);
  EXPECT_EQ(version.patch, 2u);
  EXPECT_TRUE(version.extended_range);
  EXPECT_FALSE(version.country_code_restricted);
}
