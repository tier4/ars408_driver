/*
 * Copyright 2021. Perception Engine Inc. All rights reserved.
 */
#ifndef PE_ARS408_ROS_ARS408_PARSER_H
#define PE_ARS408_ROS_ARS408_PARSER_H

#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <boost/array.hpp>

#include "ars408_ros/ars408_constants.h"
#include "ars408_ros/ars408_commands.h"


namespace ars408
{
  class CanParser
  {
  public:
    std::string Parse(const uint32_t& can_id, const boost::array<uint8_t, 8>& can_data, const uint8_t& data_length);
  };
}
#endif //PE_ARS408_ROS_ARS408_PARSER_H