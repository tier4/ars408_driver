#include "ars408_ros/ars408_driver.h"

namespace ars408
{

  bool Ars408Driver::GetCurrentRadarState(ars408::RadarState& current_state)
  {
    if(valid_radar_state_)
    {
      current_state = current_state_;
      return true;
    }
    return false;
  }

  ars408::RadarState Ars408Driver::ParseRadarState(const boost::array<uint8_t, 8>& in_can_data)
  {
    current_state_.NvmWriteStatus = ((in_can_data[0] & 0x80) >> 7);
    current_state_.NvmReadStatus = ((in_can_data[0] & 0x40) >> 6);

    uint16_t distance = ((((in_can_data[1] & 0xFF) << 2) & 0xFFFF)
                         + ((in_can_data[2] & 0xC0) >> 6)) << 1 ;
    current_state_.MaxDistance = distance;
    current_state_.PersistentError = (in_can_data[2] & 0x20) >> 5;
    current_state_.Interference = (in_can_data[2] & 0x10) >> 4;
    current_state_.TemperatureError = (in_can_data[2] & 0x08) >> 3;
    current_state_.TemporaryError = (in_can_data[2] & 0x04) >> 2;
    current_state_.VoltageError = (in_can_data[2] & 0x02) >> 1;
    current_state_.SensorID = (in_can_data[4] & 0x07);
    current_state_.SortingMode = ars408::RadarState::SortingConfig((in_can_data[4] & 0x70) >> 4);
    current_state_.PowerMode = ars408::RadarState::PowerConfig((in_can_data[3] << 1)
                                                     + ((in_can_data[4] & 0x80 ) >> 7));
    current_state_.EgoMotionRxStatus = ars408::RadarState::MotionRx( (in_can_data[5] & 0xC0) >> 6);
    current_state_.SendExtInfo = ars408::RadarState::Config((in_can_data[5] & 0x20) >> 5);
    current_state_.SendQuality = ars408::RadarState::Config((in_can_data[5] & 0x10) >> 4);
    current_state_.OutputType = ars408::RadarState::OutputTypeConfig((in_can_data[5] & 0x0C) >> 2);
    current_state_.CtrlRelay = ars408::RadarState::Config((in_can_data[5] & 0x02) >> 1);
    current_state_.Rcs_Threshold = ars408::RadarState::Rcs_ThresholdConfig((in_can_data[5] & 0x1C) >> 2);
    valid_radar_state_ = true;
    return current_state_;
  }

  boost::array<uint8_t, 8> Ars408Driver::GenerateRadarConfiguration(const ars408::RadarState &in_new_status)
  {

  }

  std::string Ars408Driver::Parse(const uint32_t& can_id, const boost::array<uint8_t, 8>& can_data , const uint8_t& data_length)
  {
    switch (can_id)
    {
      case ars408::RADAR_STATE:
        if (ars408::RADAR_STATE_BYTES == data_length)
        {
          ars408::RadarState state = ParseRadarState(can_data);

          std::cout << state.ToString() << std::endl << std::endl;
        }
        break;
    }
    return "";
  }
};