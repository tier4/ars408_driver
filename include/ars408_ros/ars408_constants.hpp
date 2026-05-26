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

#ifndef ARS408_ROS__ARS408_CONSTANTS_HPP_
#define ARS408_ROS__ARS408_CONSTANTS_HPP_

#include <cstdint>

namespace ars408
{
const uint32_t RADAR_CFG_00 = 0x200;
const uint32_t RADAR_CFG_01 = 0x210;
const uint32_t RADAR_CFG_02 = 0x220;
const uint32_t RADAR_CFG_03 = 0x230;
const uint32_t RADAR_CFG_04 = 0x240;
const uint32_t RADAR_CFG_05 = 0x250;
const uint32_t RADAR_CFG_06 = 0x260;
const uint32_t RADAR_CFG_07 = 0x270;
const uint8_t RADAR_CFG_BYTES = 8;

const uint32_t RADAR_STATE_00 = 0x201;
const uint32_t RADAR_STATE_01 = 0x211;
const uint32_t RADAR_STATE_02 = 0x221;
const uint32_t RADAR_STATE_03 = 0x231;
const uint32_t RADAR_STATE_04 = 0x241;
const uint32_t RADAR_STATE_05 = 0x251;
const uint32_t RADAR_STATE_06 = 0x261;
const uint32_t RADAR_STATE_07 = 0x271;
const uint8_t RADAR_STATE_BYTES = 8;

const uint32_t FILTER_CFG_00 = 0x202;
const uint32_t FILTER_CFG_01 = 0x212;
const uint32_t FILTER_CFG_02 = 0x222;
const uint32_t FILTER_CFG_03 = 0x232;
const uint32_t FILTER_CFG_04 = 0x242;
const uint32_t FILTER_CFG_05 = 0x252;
const uint32_t FILTER_CFG_06 = 0x262;
const uint32_t FILTER_CFG_07 = 0x272;
const uint8_t FILTER_CFG_BYTES = 8;

const uint32_t FILTER_STATE_HEADER_00 = 0x203;
const uint32_t FILTER_STATE_HEADER_01 = 0x213;
const uint32_t FILTER_STATE_HEADER_02 = 0x223;
const uint32_t FILTER_STATE_HEADER_03 = 0x233;
const uint32_t FILTER_STATE_HEADER_04 = 0x243;
const uint32_t FILTER_STATE_HEADER_05 = 0x253;
const uint32_t FILTER_STATE_HEADER_06 = 0x263;
const uint32_t FILTER_STATE_HEADER_07 = 0x273;
const uint8_t FILTER_STATE_HEADER_BYTES = 2;

const uint32_t FILTER_STATE_CFG_00 = 0x204;
const uint32_t FILTER_STATE_CFG_01 = 0x214;
const uint32_t FILTER_STATE_CFG_02 = 0x224;
const uint32_t FILTER_STATE_CFG_03 = 0x234;
const uint32_t FILTER_STATE_CFG_04 = 0x244;
const uint32_t FILTER_STATE_CFG_05 = 0x254;
const uint32_t FILTER_STATE_CFG_06 = 0x264;
const uint32_t FILTER_STATE_CFG_07 = 0x274;
const uint8_t FILTER_STATE_CFG_BYTES = 5;

const uint32_t COLL_DET_CFG_00 = 0x400;
const uint32_t COLL_DET_CFG_01 = 0x410;
const uint32_t COLL_DET_CFG_02 = 0x420;
const uint32_t COLL_DET_CFG_03 = 0x430;
const uint32_t COLL_DET_CFG_04 = 0x440;
const uint32_t COLL_DET_CFG_05 = 0x450;
const uint32_t COLL_DET_CFG_06 = 0x460;
const uint32_t COLL_DET_CFG_07 = 0x470;
const uint8_t COLL_DET_CFG_BYTES = 2;

const uint32_t COLL_DET_REGION_CFG_00 = 0x401;
const uint32_t COLL_DET_REGION_CFG_01 = 0x411;
const uint32_t COLL_DET_REGION_CFG_02 = 0x421;
const uint32_t COLL_DET_REGION_CFG_03 = 0x431;
const uint32_t COLL_DET_REGION_CFG_04 = 0x441;
const uint32_t COLL_DET_REGION_CFG_05 = 0x451;
const uint32_t COLL_DET_REGION_CFG_06 = 0x461;
const uint32_t COLL_DET_REGION_CFG_07 = 0x471;
const uint8_t COLL_DET_REGION_CFG_BYTES = 8;

const uint32_t COLL_DET_STATE_00 = 0x408;
const uint32_t COLL_DET_STATE_01 = 0x418;
const uint32_t COLL_DET_STATE_02 = 0x428;
const uint32_t COLL_DET_STATE_03 = 0x438;
const uint32_t COLL_DET_STATE_04 = 0x448;
const uint32_t COLL_DET_STATE_05 = 0x458;
const uint32_t COLL_DET_STATE_06 = 0x468;
const uint32_t COLL_DET_STATE_07 = 0x478;
const uint8_t COLL_DET_STATE_BYTES = 4;

const uint32_t COLL_DET_REGION_STATE_00 = 0x402;
const uint32_t COLL_DET_REGION_STATE_01 = 0x412;
const uint32_t COLL_DET_REGION_STATE_02 = 0x422;
const uint32_t COLL_DET_REGION_STATE_03 = 0x432;
const uint32_t COLL_DET_REGION_STATE_04 = 0x442;
const uint32_t COLL_DET_REGION_STATE_05 = 0x452;
const uint32_t COLL_DET_REGION_STATE_06 = 0x462;
const uint32_t COLL_DET_REGION_STATE_07 = 0x472;
const uint8_t COLL_DET_REGION_STATE_BYTES = 8;

const uint32_t SPEED_INFORMATION_00 = 0x300;
const uint32_t SPEED_INFORMATION_01 = 0x310;
const uint32_t SPEED_INFORMATION_02 = 0x320;
const uint32_t SPEED_INFORMATION_03 = 0x330;
const uint32_t SPEED_INFORMATION_04 = 0x340;
const uint32_t SPEED_INFORMATION_05 = 0x350;
const uint32_t SPEED_INFORMATION_06 = 0x360;
const uint32_t SPEED_INFORMATION_07 = 0x370;
const uint8_t SPEED_INFORMATION_BYTES = 2;

const uint32_t YAW_RATE_INFORMATION_00 = 0x301;
const uint32_t YAW_RATE_INFORMATION_01 = 0x311;
const uint32_t YAW_RATE_INFORMATION_02 = 0x321;
const uint32_t YAW_RATE_INFORMATION_03 = 0x331;
const uint32_t YAW_RATE_INFORMATION_04 = 0x341;
const uint32_t YAW_RATE_INFORMATION_05 = 0x351;
const uint32_t YAW_RATE_INFORMATION_06 = 0x361;
const uint32_t YAW_RATE_INFORMATION_07 = 0x371;
const uint8_t YAW_RATE_INFORMATION_BYTES = 2;

const uint32_t CLUSTER_STATUS_00 = 0x600;
const uint32_t CLUSTER_STATUS_01 = 0x610;
const uint32_t CLUSTER_STATUS_02 = 0x620;
const uint32_t CLUSTER_STATUS_03 = 0x630;
const uint32_t CLUSTER_STATUS_04 = 0x640;
const uint32_t CLUSTER_STATUS_05 = 0x650;
const uint32_t CLUSTER_STATUS_06 = 0x660;
const uint32_t CLUSTER_STATUS_07 = 0x670;
const uint8_t CLUSTER_STATUS_BYTES = 5;

const uint32_t CLUSTER_GENERAL_00 = 0x701;
const uint32_t CLUSTER_GENERAL_01 = 0x711;
const uint32_t CLUSTER_GENERAL_02 = 0x721;
const uint32_t CLUSTER_GENERAL_03 = 0x731;
const uint32_t CLUSTER_GENERAL_04 = 0x741;
const uint32_t CLUSTER_GENERAL_05 = 0x751;
const uint32_t CLUSTER_GENERAL_06 = 0x761;
const uint32_t CLUSTER_GENERAL_07 = 0x771;
const uint8_t CLUSTER_GENERAL_BYTES = 8;

const uint32_t CLUSTER_QUALITY_00 = 0x702;
const uint32_t CLUSTER_QUALITY_01 = 0x712;
const uint32_t CLUSTER_QUALITY_02 = 0x722;
const uint32_t CLUSTER_QUALITY_03 = 0x732;
const uint32_t CLUSTER_QUALITY_04 = 0x742;
const uint32_t CLUSTER_QUALITY_05 = 0x752;
const uint32_t CLUSTER_QUALITY_06 = 0x762;
const uint32_t CLUSTER_QUALITY_07 = 0x772;
const uint8_t CLUSTER_QUALITY_BYTES = 5;

const uint32_t OBJ_STATUS_00 = 0x60A;
const uint32_t OBJ_STATUS_01 = 0x61A;
const uint32_t OBJ_STATUS_02 = 0x62A;
const uint32_t OBJ_STATUS_03 = 0x63A;
const uint32_t OBJ_STATUS_04 = 0x64A;
const uint32_t OBJ_STATUS_05 = 0x65A;
const uint32_t OBJ_STATUS_06 = 0x66A;
const uint32_t OBJ_STATUS_07 = 0x67A;
const uint8_t OBJ_STATUS_BYTES = 4;

const uint32_t OBJ_GENERAL_00 = 0x60B;
const uint32_t OBJ_GENERAL_01 = 0x61B;
const uint32_t OBJ_GENERAL_02 = 0x62B;
const uint32_t OBJ_GENERAL_03 = 0x63B;
const uint32_t OBJ_GENERAL_04 = 0x64B;
const uint32_t OBJ_GENERAL_05 = 0x65B;
const uint32_t OBJ_GENERAL_06 = 0x66B;
const uint32_t OBJ_GENERAL_07 = 0x67B;
const uint8_t OBJ_GENERAL_BYTES = 8;

const uint32_t OBJ_QUALITY_00 = 0x60C;
const uint32_t OBJ_QUALITY_01 = 0x61C;
const uint32_t OBJ_QUALITY_02 = 0x62C;
const uint32_t OBJ_QUALITY_03 = 0x63C;
const uint32_t OBJ_QUALITY_04 = 0x64C;
const uint32_t OBJ_QUALITY_05 = 0x65C;
const uint32_t OBJ_QUALITY_06 = 0x66C;
const uint32_t OBJ_QUALITY_07 = 0x67C;
const uint8_t OBJ_QUALITY_BYTES = 8;

const uint32_t OBJ_EXTENDED_00 = 0x60D;
const uint32_t OBJ_EXTENDED_01 = 0x61D;
const uint32_t OBJ_EXTENDED_02 = 0x62D;
const uint32_t OBJ_EXTENDED_03 = 0x63D;
const uint32_t OBJ_EXTENDED_04 = 0x64D;
const uint32_t OBJ_EXTENDED_05 = 0x65D;
const uint32_t OBJ_EXTENDED_06 = 0x66D;
const uint32_t OBJ_EXTENDED_07 = 0x67D;
const uint8_t OBJ_EXTENDED_BYTES = 8;

const uint32_t OBJ_WARNING_00 = 0x60E;
const uint32_t OBJ_WARNING_01 = 0x61E;
const uint32_t OBJ_WARNING_02 = 0x62E;
const uint32_t OBJ_WARNING_03 = 0x63E;
const uint32_t OBJ_WARNING_04 = 0x64E;
const uint32_t OBJ_WARNING_05 = 0x65E;
const uint32_t OBJ_WARNING_06 = 0x66E;
const uint32_t OBJ_WARNING_07 = 0x67E;
const uint8_t OBJ_WARNING_BYTES = 4;

const uint32_t VERSION_ID_00 = 0x700;
const uint32_t VERSION_ID_01 = 0x710;
const uint32_t VERSION_ID_02 = 0x720;
const uint32_t VERSION_ID_03 = 0x730;
const uint32_t VERSION_ID_04 = 0x740;
const uint32_t VERSION_ID_05 = 0x750;
const uint32_t VERSION_ID_06 = 0x760;
const uint32_t VERSION_ID_07 = 0x770;
const uint8_t VERSION_ID_BYTES = 4;

const uint32_t COLL_DET_RELAY_CTRL = 0x8;
const uint8_t COLL_DET_RELAY_CTRL_BYTES = 1;

const uint8_t RADAR_CONNECTIONS_MAX = 8;
}  // namespace ars408

#endif  // ARS408_ROS__ARS408_CONSTANTS_HPP_
