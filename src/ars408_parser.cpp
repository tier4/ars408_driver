#include "ars408_ros/ars408_parser.h"

namespace ars408
{

  /*
   * 0x201
   * */
  ars408::RadarState ParseRadarState(const boost::array<uint8_t, 8>& can_data)
  {
    ars408::RadarState state;
    state.NvmWriteStatus = ((can_data[0] & 0x80) >> 7);
    state.NvmReadStatus = ((can_data[0] & 0x40) >> 6);

    uint16_t distance = ((((can_data[1] & 0xFF) << 2) & 0xFFFF)
                         + ((can_data[2] & 0xC0) >> 6)) << 1 ;
    state.MaxDistance = distance;
    state.PersistentError = (can_data[2] & 0x20) >> 5;
    state.Interference = (can_data[2] & 0x10) >> 4;
    state.TemperatureError = (can_data[2] & 0x08) >> 3;
    state.TemporaryError = (can_data[2] & 0x04) >> 2;
    state.VoltageError = (can_data[2] & 0x02) >> 1;
    state.SensorID = (can_data[4] & 0x07);
    state.SortingMode = ars408::RadarState::SortingConfig((can_data[4] & 0x70) >> 4);
    state.PowerMode = ars408::RadarState::PowerConfig((can_data[3] << 1)
                                                     + ((can_data[4] & 0x80 ) >> 7));
    state.EgoMotionRxStatus = ars408::RadarState::MotionRx( (can_data[5] & 0xC0) >> 6);
    state.SendExtInfo = ars408::RadarState::Config((can_data[5] & 0x20) >> 5);
    state.SendQuality = ars408::RadarState::Config((can_data[5] & 0x10) >> 4);
    state.OutputType = ars408::RadarState::OutputTypeConfig((can_data[5] & 0x0C) >> 2);
    state.CtrlRelay = ars408::RadarState::Config((can_data[5] & 0x02) >> 1);
    state.Rcs_Threshold = ars408::RadarState::Rcs_ThresholdConfig((can_data[5] & 0x1C) >> 2);
    return state;
  }

  std::string CanParser::Parse(const uint32_t& can_id, const boost::array<uint8_t, 8>& can_data , const uint8_t& data_length)
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