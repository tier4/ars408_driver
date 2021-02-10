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
    current_state_.NvmWriteStatus = ((in_can_data[0] & 0x80u) >> 7u);
    current_state_.NvmReadStatus = ((in_can_data[0] & 0x40u) >> 6u);

    uint16_t distance = ((((in_can_data[1] & 0xFFu) << 2u) & 0xFFFFu)
                         + ((in_can_data[2] & 0xC0u) >> 6u)) << 1u ;
    current_state_.MaxDistance = distance;
    current_state_.PersistentError = (in_can_data[2] & 0x20u) >> 5u;
    current_state_.Interference = (in_can_data[2] & 0x10u) >> 4u;
    current_state_.TemperatureError = (in_can_data[2] & 0x08u) >> 3u;
    current_state_.TemporaryError = (in_can_data[2] & 0x04u) >> 2u;
    current_state_.VoltageError = (in_can_data[2] & 0x02u) >> 1u;
    current_state_.SensorID = (in_can_data[4] & 0x07u);
    current_state_.SortingMode = ars408::RadarState::SortingConfig((in_can_data[4] & 0x70u) >> 4u);
    current_state_.PowerMode = ars408::RadarState::PowerConfig((in_can_data[3] << 1u)
                                                     + ((in_can_data[4] & 0x80u ) >> 7u));
    current_state_.EgoMotionRxStatus = ars408::RadarState::MotionRx( (in_can_data[5] & 0xC0u) >> 6u);
    current_state_.SendExtInfo = ars408::RadarState::Config((in_can_data[5] & 0x20u) >> 5u);
    current_state_.SendQuality = ars408::RadarState::Config((in_can_data[5] & 0x10u) >> 4u);
    current_state_.OutputType = ars408::RadarState::OutputTypeConfig((in_can_data[5] & 0x0Cu) >> 2u);
    current_state_.CtrlRelay = ars408::RadarState::Config((in_can_data[5] & 0x02u) >> 1u);
    current_state_.Rcs_Threshold = ars408::RadarState::Rcs_ThresholdConfig((in_can_data[5] & 0x1Cu) >> 2u);
    valid_radar_state_ = true;
    return current_state_;
  }

  boost::array<uint8_t, 8> Ars408Driver::GenerateRadarConfiguration(const ars408::RadarCfg &in_new_status)
  {
    boost::array<uint8_t, 8> can_data = {0,0,0,0,
                                         0,0,0,0};

    if(in_new_status.UpdateStoreInNVM)
    {
      can_data[0] = 0x80;   /* X000 0000 */
      if(in_new_status.StoreInNVM){
        can_data[5] |= 0x80u;/* 1000 0000 */
      }
      else{
        can_data[5] |= 0x00u;/* 0000 0000 */
      }
    }
    if(in_new_status.UpdateSortIndex)
    {
      can_data[0] |= 0x40u;    /* 0XXX 0000 */
      switch(in_new_status.SortIndex){
        case ars408::RadarCfg::Sorting::NO_SORT:
          can_data[5] |= 0x00u;/* 0000 0000 */
          break;
        case ars408::RadarCfg::Sorting::BY_RANGE:
          can_data[5] |= 0x10u;/* 0001 0000 */
          break;
        case ars408::RadarCfg::Sorting::BY_RCS:
          can_data[5] |= 0x20u;/* 0010 0000 */
          break;
        default:
          can_data[5] |= 0x00u;
      }
    }
    if(in_new_status.UpdateSendExtInfo)
    {
      can_data[0] |= 0x20u;  /* 0000 X000 */
      if(in_new_status.SendExtInfo)
      {
        can_data[5] |= 0x08u;/* 0000 1000 */
      }
      else
      {
        can_data[5] |= 0x00u;/* 0000 0000 */
      }
    }
    if(in_new_status.UpdateSendQuality)
    {
      can_data[0] |= 0x10u;  /* 0000 0X00 */
      if(in_new_status.SendQuality)
      {
        can_data[5] |= 0x04u;/* 0000 0100 */
      }
      else
      {
        can_data[5] |= 0x00u;/* 0010 0000 */
      }
    }
    if(in_new_status.UpdateOutputType)
    {
      can_data[0] |= 0x08u;
      switch(in_new_status.OutputType)
      {                       /* 000X X000 */
        case ars408::RadarCfg::OutputTypeConfig::NONE:
          can_data[4] |= 0x00u;/* 0000 0000 */
          break;
        case ars408::RadarCfg::OutputTypeConfig::OBJECTS:
          can_data[4] |= 0x08u;/* 0000 1000 */
          break;
        case ars408::RadarCfg::OutputTypeConfig::CLUSTERS:
          can_data[4] |= 0x10u;/* 0001 0000 */
          break;
      }
    }
    if(in_new_status.UpdateRadarPower)
    {
      can_data[0] |= 0x04u;
      switch(in_new_status.RadarPower)
      {                       /* XXX0 0000 */
        case ars408::RadarCfg::RadarPowerConfig::STANDARD:
          can_data[4] |= 0x00u;/* 0000 0000 */
          break;
        case ars408::RadarCfg::RadarPowerConfig::MINUS_3dB_GAIN:
          can_data[4] |= 0x20u;/* 0010 0000 */
          break;
        case ars408::RadarCfg::RadarPowerConfig::MINUS_6dB_GAIN:
          can_data[4] |= 0x40u;/* 0100 0000 */
          break;
        case ars408::RadarCfg::RadarPowerConfig::MINUS_9dB_GAIN:
          can_data[4] |= 0x60u;/* 0110 0000 */
          break;
      }
    }
    if(in_new_status.UpdateSensorID
      && in_new_status.SensorID >=0
      && in_new_status.SensorID <= 7)
    {
      can_data[0] |= 0x02u;
      can_data[4] |= in_new_status.SensorID;/* 0000 0XXX */
    }
    if(in_new_status.UpdateMaxDistance)
    {
      can_data[0] |= 0x01u;    /* XXXX XXXX */
                              /* XX00 0000 */
      //ARS408:
      //Standard Range Version: 196 – 260 m
      //Extended Range Version: 196 – 1200 m
      uint16_t TempDistance = in_new_status.MaxDistance * 2;
      uint8_t low_byte = (TempDistance & 0x0002u) << 6u;
      uint8_t high_byte = (TempDistance & 0x0FFFu) >> 2u;
      can_data[1] = 0x00;    /* XXXX XXXX */
      can_data[2] = 0x00;    /* XX00 0000 */
    }
  }

  ars408::Obj_0_Status Ars408Driver::ParseObject0_Status(const boost::array<uint8_t, 8>& in_can_data)
  {
    current_objects_status_.NumberOfObjects = in_can_data[0];
    current_objects_status_.MeasurementCounter = (in_can_data[1] << 8u) + (in_can_data[0]);
    current_objects_status_.InterfaceVersion = (in_can_data[3] & 0xF0) >> 4u;
    return  current_objects_status_;
  }

  ars408::RadarObject Ars408Driver::ParseObject1_General(const boost::array<uint8_t, 8>& in_can_data)
  {
    ars408::RadarObject current_object;
    current_object.sequence = current_objects_status_.MeasurementCounter;
    current_object.id = in_can_data[0];
    current_object.dynamic_property = ars408::Obj_1_General::DynamicProperty(in_can_data[6] & 0x07u);
    current_object.rcs = (in_can_data[7] * 0.5) - 64.0;

    uint16_t  dist_x_tmp = (in_can_data[1] << 5u) + ((in_can_data[2] & 0xF8u) >> 3u);
    current_object.distance_long_x = dist_x_tmp * 0.2f - 500.0f;

    uint16_t dist_y_tmp = ((in_can_data[2] & 0x07u) << 8u) + (in_can_data[3]);
    current_object.distance_lat_y = dist_y_tmp *0.2f - 204.6f;

    uint16_t speed_x_tmp = (in_can_data[4] << 2u) + ((in_can_data[5] & 0xC0u) >> 6u);
    current_object.speed_long_x = (speed_x_tmp * 0.25f) - 128.0f;

    uint16_t speed_y_ymp = ( (in_can_data[5] & 0x3Fu) << 3u) + ( (in_can_data[6] & 0xE0u) >> 5u);
    current_object.speed_lat_y = ( speed_y_ymp * 0.25f - 64.0f);
    return current_object;
  }

  std::string Ars408Driver::Parse(const uint32_t& can_id, const boost::array<uint8_t, 8>& can_data , const uint8_t& data_length)
  {
    switch (can_id)
    {
      /// 0x201 the current configuration and
      //sensor state in message
      case ars408::RADAR_STATE:
        if (ars408::RADAR_STATE_BYTES == data_length)
        {
          ars408::RadarState state = ParseRadarState(can_data);

          std::cout << state.ToString() << std::endl;
        }
        break;
      /// 0x60A contains list header information,
      //i.e. the number of objects that are sent afterwards
      case ars408::OBJ_STATUS:
        if (ars408::OBJ_STATUS_BYTES == data_length)
        {
          ars408::Obj_0_Status object_status = ParseObject0_Status(can_data);
          std::cout << object_status.ToString() << std::endl;
        }
        break;
      /// 0x60B contains the position and velocity of
      //the objects and is sent repeatedly for all the tracked objects.
      case ars408::OBJ_GENERAL:
        if (ars408::OBJ_GENERAL_BYTES == data_length)
        {
          ars408::RadarObject object = ParseObject1_General(can_data);
          std::cout << object.ToString() << std::endl;
        }
        break;
    }
    return "";
  }
};