/*
 * Copyright 2021. Perception Engine Inc. All rights reserved.
 */
#ifndef PE_ARS408_ROS_ARS408_DRIVER_H
#define PE_ARS408_ROS_ARS408_DRIVER_H

#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <boost/array.hpp>

#include "ars408_ros/ars408_constants.h"
#include "ars408_ros/ars408_commands.h"


namespace ars408
{
  class Ars408Driver
  {
  private:
    bool valid_radar_state_;
    ars408::RadarState current_state_;

    /**
    * Parses RadarState CAN 0x201
    * @param in_can_data boost::array<uint8_t, 8>
    * @return object filled with the current Radar State
    */
    ars408::RadarState ParseRadarState(const boost::array<uint8_t, 8>& in_can_data);

  public:
    /**
     * Parses incoming can_id and its byte array can_data
     * @param can_id CAN message Id
     * @param can_data Eight byte Array containing message content
     * @param data_length Number of bytes with valid data contained in can_data
     * @return String Message
     */
    std::string Parse(const uint32_t& can_id, const boost::array<uint8_t, 8>& can_data , const uint8_t& data_length);

    /**
     * Returns true if the RadarState has been received, if true it fills current_state with the valid state.
     * If invalid, current_state is empty.
     * @param current_state
     * @return True if the RadarState has been received.
     */
    bool GetCurrentRadarState(ars408::RadarState& current_state);

    /**
     * Generates the CAN message to configure the radar according to in_new_status
     * @param in_new_status object containing the desired configuration
     * @return array containing the can command to be send
     */
    boost::array<uint8_t, 8> GenerateRadarConfiguration(const ars408::RadarState &in_new_status);
  };
}
#endif //PE_ARS408_ROS_ARS408_DRIVER_H