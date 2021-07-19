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

#ifndef ARS408_ROS__ARS408_COMMANDS_HPP_
#define ARS408_ROS__ARS408_COMMANDS_HPP_

#include <sstream>
#include <string>

#include "ars408_ros/ars408_constants.hpp"

namespace ars408
{

class RadarCommand
{
public:
  explicit RadarCommand(uint32_t CAN_ID)
  {
    CAN_ID_ = CAN_ID;
  }

  virtual uint32_t GetCommandType()
  {
    return CAN_ID_;
  }

protected:
  uint32_t CAN_ID_;
};
/*
 * CAN ID:    0x200
 * Direction: TO_DEVICE
 * Description: Radar sensor configuration
 */
class RadarCfg : RadarCommand
{
public:
  RadarCfg()
  : RadarCommand(ars408::RADAR_CFG) {}
  bool UpdateMaxDistance;  /*Allow change of maximum distance if true*/
  bool UpdateSensorID;  /*Allow change of sensor ID if true*/
  bool UpdateRadarPower;  /*Allow change of radar output power if true*/
  bool UpdateOutputType;  /*Allow change of output type if true*/
  bool UpdateSendQuality;  /*Allow change of quality message option if true*/
  bool UpdateSendExtInfo;  /*Allow change of extended info message option if true*/
  bool UpdateSortIndex;  /*Allow change of sorting index if true*/
  bool UpdateStoreInNVM;  /*Allow storing to non-volatile memory if true*/
  bool UpdateInvalidClusters;  /*Allows changing the invalid clusters parameter if true*/
  uint16_t MaxDistance;  /*Maximum detection distance*/
  uint8_t SensorID;  /*Sensor ID 0 – 7*/
  enum OutputTypeConfig {NONE,  /*0x0*/
    OBJECTS,   /*0x1*/
    CLUSTERS   /*0x2*/
  };
  /* Configures the data output to clusters (0x2) or objects (0x1) */
  OutputTypeConfig OutputType;
  /* Current configuration of transmitted radar power parameter */
  enum RadarPowerConfig {STANDARD, MINUS_3dB_GAIN, MINUS_6dB_GAIN, MINUS_9dB_GAIN};
  RadarPowerConfig RadarPower;
  /*
   * Configures the transmitted radar power.
   * The output RCS of cluster and objects will be compensated for this attenuation.
   * Reducing the output power can improve detection
   * in case of close distance scenarios or inside rooms.
   */
  bool
    UpdateCtrlRelay;
  /* Relay control message (0x8) is sent if true and the collision detection is activated */
  bool CtrlRelay;
  /* Cluster or object quality information (message 0x60C or 0x702) is sent if true */
  bool SendQuality;
  /*
   * Extended information (message 0x60D) is sent for objects
   * if true (if clusters are selected as output type this value is ignored)
   */
  bool SendExtInfo;
  /* Current configuration of sorting index for object list */
  enum Sorting {NO_SORT, BY_RANGE, BY_RCS};
  /*
   * Selects the sorting index for the object list
   * (ignored for clusters as they are always sorted by range)
   */
  Sorting SortIndex;
  /*
   * Stores the current configuration to non-volatile memory
   * to be read and set at sensor startup.
   */
  bool StoreInNVM;
  /* Sets the sensitivity of the cluster detection to standard (0x0) or high sensitivity (0x1) */
  bool UpdateRCS_Threshold;
  enum RCS_Threshold {NORMAL, HIGH_SENSITIVITY};
  /* If true, the sensor’s high sensitivity mode is active */
  RCS_Threshold RCS_Status;
  enum InvalidClustersConfig {DISABLE_INVALID = 0x00,
    ENABLE_INVALID = 0x01,
    ENABLE_LOW_RCS_DYNAMIC = 0x02,
    ENABLE_LOW_RCS_STATIC = 0x04,
    ENABLE_INVALID_RANGE = 0x08,
    ENABLE_RANGE_LESS_THAN_1M = 0x10,
    ENABLE_EGO_MIRROR = 0x20,
    ENABLE_WRAPPED_STATIONARY = 0x20, };
  InvalidClustersConfig InvalidClusters;
};

/*
 * CAN ID:    0x201
 * Direction: FROM_DEVICE
 * Description: Radar status
 */
class RadarState : public RadarCommand
{
public:
  RadarState()
  : RadarCommand(ars408::RADAR_STATE) {}
  std::string ToString()
  {
    std::ostringstream stream;
    stream << "NvmReadStatus: " << (uint16_t)NvmReadStatus << std::endl <<
      "NvmWriteStatus: " << (uint16_t)NvmWriteStatus << std::endl <<
      "MaxDistance: " << (uint16_t)MaxDistance << " m" << std::endl <<
      "PersistentError: " << (uint16_t)PersistentError << std::endl <<
      "Interference: " << (uint16_t)Interference << std::endl <<
      "TemperatureError: " << (uint16_t)TemperatureError << std::endl <<
      "TemporaryError: " << (uint16_t)TemporaryError << std::endl <<
      "VoltageError: " << (uint16_t)VoltageError << std::endl <<
      "SensorID: " << (uint16_t)SensorID << std::endl;
    stream << "SortingMode: ";
    switch (SortingMode) {
      case ars408::RadarState::SortingConfig::NO_SORT:
        stream << "NO_SORT";
        break;
      case ars408::RadarState::SortingConfig::BY_RANGE:
        stream << "BY_RANGE";
        break;
      case ars408::RadarState::SortingConfig::BY_RCS:
        stream << "BY_RCS";
        break;
    }
    stream << std::endl <<
      "PowerMode: ";
    switch (PowerMode) {
      case ars408::RadarState::PowerConfig::STANDARD:
        stream << "STANDARD";
        break;
      case ars408::RadarState::PowerConfig::MINUS_3dB_GAIN:
        stream << "MINUS_3dB_GAIN";
        break;
      case ars408::RadarState::PowerConfig::MINUS_6dB_GAIN:
        stream << "MINUS_6dB_GAIN";
        break;
      case ars408::RadarState::PowerConfig::MINUS_9dB_GAIN:
        stream << "MINUS_9dB_GAIN";
        break;
    }
    stream << std::endl <<
      "CtrlRelay: " << (uint16_t)CtrlRelay << std::endl

           << "OutputType: ";
    switch (OutputType) {
      case ars408::RadarState::OutputTypeConfig::NONE:
        stream << "NONE";
        break;
      case ars408::RadarState::OutputTypeConfig::OBJECTS:
        stream << "OBJECTS";
        break;
      case ars408::RadarState::OutputTypeConfig::CLUSTERS:
        stream << "CLUSTERS";
        break;
    }
    stream << std::endl <<
      "SendQuality: " << (uint16_t)SendQuality << std::endl <<
      "SendExtInfo: " << (uint16_t)SendExtInfo << std::endl

           << "EgoMotionRxStatus: ";
    switch (EgoMotionRxStatus) {
      case ars408::RadarState::MotionRx::INPUT_OK:
        stream << "INPUT_OK";
        break;
      case ars408::RadarState::MotionRx::SPEED_MISSING:
        stream << "SPEED_MISSING";
        break;
      case ars408::RadarState::MotionRx::YAW_MISSING:
        stream << "YAW_MISSING";
        break;
      case ars408::RadarState::MotionRx::SPEED_YAW_MISSING:
        stream << "SPEED_YAW_MISSING";
        break;
    }
    stream << std::endl <<
      "Rcs_Threshold: " << (uint16_t)Rcs_Threshold;

    return stream.str();
  }
  enum Status {FAILED, SUCCESS};  /*FAILED, SUCCESS*/
  enum Config {INACTIVE, ACTIVE};  /*INACTIVE, ACTIVE*/

  /* State of reading the configuration parameters from non-volatile memory at startup */
  bool NvmReadStatus;
  /*
   * State of storing a configuration parameter to non-volatile memory
   * (initially this value is set to 0x0 and set to 0x1
   * after a configuration has been sent and successfully stored)
   */
  bool NvmWriteStatus;
  uint16_t MaxDistance;
  /* An internal error which might not disappear after a reset has been detected. */
  bool PersistentError;
  bool Interference;        /*Interference with another radar sensor has been detected.*/
  /* Error will be active if the temperature is below or above the defined range. */
  bool TemperatureError;
  /*
   * A temporary error which will most probably disappear
   * after a sensor reset has been detected.
   */
  bool TemporaryError;
  /*
   * Error will be active
   * if the operating voltage is below or above the defined range for more than 5 seconds.
   */
  bool VoltageError;
  uint8_t SensorID;         /*Sensor ID 0 - 7*/
  enum SortingConfig {NO_SORT, BY_RANGE, BY_RCS, SORT_ERROR};
  SortingConfig SortingMode;  /*Current configuration of sorting index for object list*/
  /* Current configuration of transmitted radar power parameter */
  enum PowerConfig {STANDARD, MINUS_3dB_GAIN, MINUS_6dB_GAIN, MINUS_9dB_GAIN, POWER_ERROR};
  PowerConfig PowerMode;
  Config CtrlRelay;      /*True if relay control message is sent*/
  enum OutputTypeConfig {NONE, OBJECTS, CLUSTERS, OUTPUT_ERROR};   /*NONE, OBJECTS, CLUSTERS*/
  OutputTypeConfig OutputType;  /*Currently selected output type as either clusters or objects*/
  Config SendQuality;    /*True if quality information is sent for clusters or objects*/
  Config SendExtInfo;    /*True if extended information is sent for objects*/
  /* INPUT_OK, SPEED_MISSING, YAW_MISSING, SPEED_YAW_MISSING */
  enum MotionRx {INPUT_OK, SPEED_MISSING, YAW_MISSING, SPEED_YAW_MISSING, MOTION_ERROR};
  MotionRx EgoMotionRxStatus;  /*Shows the state of the speed and yaw rate input signals*/
  /* If true, the sensor’s high sensitivity mode is active */
  enum Rcs_ThresholdConfig {NORMAL, HIGH_SENSITIVITY, RCS_ERROR};
  Rcs_ThresholdConfig Rcs_Threshold;
};

/*
 * CAN ID:    0x202
 * Direction: TO_DEVICE
 * Description: Cluster and Object filter configuration
 */
class FilterCfg : RadarCommand
{
public:
  FilterCfg()
  : RadarCommand(ars408::FILTER_CFG) {}
  bool UpdateFilterConfig;  /*Allow change of filter configuration if true*/
  bool ActivateSelectedFilter;  /*De-/activate filter configuration for specified filter criterion*/
  enum FilterTypeConfig {NUM_OBJECTS_CO,
    DISTANCE_CO,  /*Radial distance in m [r = sqrt(x2 + y2)]*/
    AZIMUTH_CO,  /*Azimuth angle in degree [a = arc tan(y/x)]*/
    /*
     * Radial velocity in sensor line-of-sight in m/sec of oncoming clusters or objects
     * (all departing clusters and objects are ok)
     */
    ONCOMING_RADIAL_VELOCITY_CO,
    /*
     * Radial velocity in sensor line-of-sight in m/sec of departing clusters or objects
     * (all oncoming clusters and objects are ok)
     */
    DEPARTING_RADIAL_VELOCITY_CO,
    RADAR_CROSS_SECTION_CO,  /*RCS value (Radar cross section) in dBm2*/
    LIFETIME_O,  /*Life time (since first detection) in seconds*/
    AREA_SIZE_O,  /*Object size as area in m2 (length x width)*/
    /* probability for being a real target and not a sensor artifact caused by multipath */
    EXIST_PROBABILITY_O,
    POS_Y_O,  /*Y-position in m (lateral distance)*/
    POS_X_O,  /*X-position in m (longitudinal distance)*/
    /*
     * Lateral velocity component in m/sec for right-left moving objects
     * (all left-right moving objects are ok)
     */
    VEL_Y_RIGHT_LEFT_O,
    /*
     * Longitudinal velocity component in m/sec for oncoming objects
     * (all departing objects are ok)
     */
    VEL_X_ONCOMING_O,
    /*
     * Lateral velocity component in m/sec for left-right moving objects
     * (all right-left moving objects are ok)
     */
    VEL_Y_LEFT_RIGHT_O,
    /*
     * Longitudinal velocity component in m/sec for departing objects
     * (all oncoming objects are ok)
     */
    VEL_X_DEPARTING_O,
    /*
     * Filters objects having a specific classification
     * which is output in the signal Object_Class in the message Object_3_Extended.
     * Max value is used as a bitfield for the configuration.
     * Bit 0-7 are corresponding to the object classification 0-7. Min value is ignored.
     */
    CLASS_O, };
  FilterTypeConfig FilterType;
  /* Currently selected output type as either clusters or objects */
  enum OutputTypeConfig {NONE, OBJECTS, CLUSTERS};
  OutputTypeConfig OutputTypeFilter;
  uint8_t MinNumberOfObjects;      /*0     4095      1             */
  float MinDistance;               /*0     409.5     0.1     m     */
  float MinAzimuth;                /*-50   52.375    0.025   deg   */
  float MinVelocityOncoming;       /*0     128.993   0.0315  m/s   */
  float MinVelocityDeparting;      /*0     128.993   0.0315  m/s   */
  float MinRadarCrossSection;      /*-50   52.375    0.025   dBm2  */
  float MinLifetime;               /*0     409.5     0.1     s     */
  float MinAreaSize;               /*0     102.375   0.025   m2*/
  float MinExistProbability;       /*0     7         1       % {0,25,50,75,90,99,99.9,100}*/
  float MinPosY;                   /*-409.5 409.5    0.2     m     */
  float MinPosX;                   /*-500  1138.2    0.2     m     */
  float MinVelYRightLeft;          /*0     128.993   0.0315  m/s   */
  float MinVelXOncoming;           /*0     128.993   0.0315  m/s   */
  float MinVelYLeftRight;          /*0     128.993   0.0315  m/s   */
  float MinVelXDeparting;          /*0     128.993   0.0315  m/s   */
  uint8_t MinClass;                /*0     4095      1             */

  uint8_t MaxNumberOfObjects;      /*0     4095      1             */
  float MaxDistance;               /*0     409.5     0.1     m     */
  float MaxAzimuth;                /*-50   52.375    0.025   deg   */
  float MaxVelocityOncoming;       /*0     128.993   0.0315  m/s   */
  float MaxVelocityDeparting;      /*0     128.993   0.0315  m/s   */
  float MaxRadarCrossSection;      /*-50   52.375    0.025   dBm2  */
  float MaxLifetime;               /*0     409.5     0.1     s     */
  float MaxAreaSize;               /*0     102.375   0.025   m2*/
  float MaxExistProbability;       /*0     7         1       % {0,25,50,75,90,99,99.9,100}*/
  float MaxPosY;                   /*-409.5 409.5    0.2     m     */
  float MaxPosX;                   /*-500  1138.2    0.2     m     */
  float MaxVelYRightLeft;          /*0     128.993   0.0315  m/s   */
  float MaxVelXOncoming;           /*0     128.993   0.0315  m/s   */
  float MaxVelYLeftRight;          /*0     128.993   0.0315  m/s   */
  float MaxVelXDeparting;          /*0     128.993   0.0315  m/s   */
  uint8_t MaxClass;                /*0     4095      1             */
};

/*
 * CAN ID:    0x203
 * Direction: FROM_DEVICE
 * Description: Filter status header
 */
class FilterState_Header : RadarCommand
{
public:
  FilterState_Header()
  : RadarCommand(ars408::FILTER_STATE_HEADER) {}
};

/*
 * CAN ID:    0x204
 * Direction: TO_DEVICE
 * Description: Filter configuration status
 */
class FilterState_Cfg : RadarCommand
{
public:
  FilterState_Cfg()
  : RadarCommand(ars408::FILER_STATE_CFG) {}
  const int CAN_ID = 0x204;
};

/*
 * CAN ID:    0x400
 * Direction: TO_DEVICE
 * Description: Collision detection configuration
 */
class CollDetCfg : RadarCommand
{
public:
  CollDetCfg()
  : RadarCommand(ars408::COLL_DET_CFG) {}
};

/*
 * CAN ID:    0x401
 * Direction: FROM_DEVICE
 * Description: Collision detection region configuration
 */
class CollDetRegionCfg : RadarCommand
{
public:
  CollDetRegionCfg()
  : RadarCommand(ars408::COLL_DET_REGION_CFG) {}
};

/*
 * CAN ID:    0x408
 * Direction: FROM_DEVICE
 * Description: Collision detection status
 */
class CollDetState : RadarCommand
{
public:
  CollDetState()
  : RadarCommand(ars408::COLL_DET_STATE) {}
};

/*
 * CAN ID:    0x402
 * Direction: FROM_DEVICE
 * Description: Collision detection region status
 */
class CollDetRegionState : RadarCommand
{
public:
  CollDetRegionState()
  : RadarCommand(ars408::COLL_DET_REGION_STATE) {}
};

/*
 * CAN ID:    0x300
 * Direction: TO_DEVICE
 * Description: Vehicle (sensor platform) speed
 */
class SpeedInformation : RadarCommand
{
public:
  SpeedInformation()
  : RadarCommand(ars408::SPEED_INFORMATION) {}
  /*
   * Indicates the direction of the radar movement
   * while looking into positive straight ahead direction
   */
  enum SpeedDirection {STATIC, MOVING_FORWARD, MOVING_BACKWARDS};
  /* 0      163.8      0.02    m/s */
  /*
   * Absolute magnitude of speed in the direction the radar is moved
   * while looking into positive straight ahead direction
   */
  float Speed;
};

/*
 * CAN ID:    0x301
 * Direction: TO_DEVICE
 * Description: Vehicle (sensor platform) yaw rate
 */
class YawRateInformation : RadarCommand
{
public:
  YawRateInformation()
  : RadarCommand(ars408::YAW_RATE_INFORMATION) {}
  /* -327.68 327.68    0.01    deg/s */
  /*
   * Rate of change of angular velocity looking into positive straight ahead direction.
   * The center of rotation is assumed to be 1.95 m behind the sensor.
   */
  float YawRate;
};

/*
 * CAN ID:    0x600
 * Direction: FROM_DEVICE
 * Description: Cluster status (list header)
 */
class Cluster_0_Status : RadarCommand
{
public:
  Cluster_0_Status()
  : RadarCommand(ars408::CLUSTER_STATUS) {}
};

/*
 * CAN ID:    0x701
 * Direction: FROM_DEVICE
 * Description: Cluster general information
 */
class Cluster_1_General : RadarCommand
{
public:
  Cluster_1_General()
  : RadarCommand(ars408::CLUSTER_GENERAL) {}
};

/*
 * CAN ID:    0x702
 * Direction: FROM_DEVICE
 * Description: Cluster quality information
 */
class Cluster_2_Quality : RadarCommand
{
public:
  Cluster_2_Quality()
  : RadarCommand(ars408::CLUSTER_QUALITY) {}
};

/*
 * CAN ID:    0x60A
 * Direction: FROM_DEVICE
 * Description: Object status (list header).
 * Contains list header information, i.e. the number of objects that are sent afterwards.
 */
class Obj_0_Status : RadarCommand
{
public:
  Obj_0_Status()
  : RadarCommand(ars408::OBJ_STATUS) {}
  std::string ToString()
  {
    std::ostringstream stream;
    stream << "NumberOfObjects: " << (uint16_t)NumberOfObjects <<
      ", MeasurementCounter: " << (unsigned int)MeasurementCounter <<
      ", InterfaceVersion: " << (uint16_t)InterfaceVersion << std::endl;
    return stream.str();
  }
  uint8_t NumberOfObjects;    /*Number of objects (max. 100 Objects)*/
  /*
   * Measurement cycle counter
   * (counting up since startup of sensor and restarting at 0 when > 65535)
   */
  uint16_t MeasurementCounter;
  uint8_t InterfaceVersion;   /*Object list CAN interface version*/
};

/*
 * CAN ID:    0x60B
 * Direction: FROM_DEVICE
 * Description: Object general information.
 * This message contains the position and velocity of the objects and is sent repeatedly for all the
 * tracked objects.
 */
class Obj_1_General : RadarCommand
{
public:
  Obj_1_General()
  : RadarCommand(ars408::OBJ_GENERAL) {}

  /* 0       255       1 */
  /*
   * Object ID (since objects are tracked,
   * the ID is kept throughout measurement cycles and does not have to be consecutive)
   */
  uint8_t Id;
  /* -500    +1138.2   0.2   m */
  /* Longitudinal (x) coordinate */
  float LongitudinalDistanceX;
  /* -204.6  +204.8    0.2   m */
  /* Lateral (y) coordinate */
  float LateralDistanceY;
  /* -128.00 127.75    0.2   m */
  /* Relative velocity in longitudinal direction (x) */
  float RelativeLongitudinalVelocityX;
  /* -64.00  63.75     0.25  m/s */
  /* Relative velocity in lateral direction (y) */
  float RelativeLateralVelocityY;
  enum DynamicProperty {MOVING, STATIONARY, ONCOMING, CROSSING_LEFT, CROSSING_RIGHT, UNKNOWN,
    STOPPED};
  /*
   * Dynamic property of the object indicating if the object is moving or stationary
   * (this value can only be determined correctly if the speed and yaw rate is given correctly)
   */
  DynamicProperty ObjectStatus;
  /* -64.0   63.5      0.5   dBm2 */
  /* Radar cross section */
  float RadarCrossSection;
};

/*
 * CAN ID:    0x60C
 * Direction: FROM_DEVICE
 * Description: Object quality information.
 * This message contains the quality information of the objects
 * and is only sent if it was activated in
 * signal RadarCfg_SendQuality (0x200)
 */
class Obj_2_Quality : RadarCommand
{
public:
  uint8_t Id;
  float LongitudinalDistanceXRms;
  float RelativeLongitudinalVelocityXRms;
  float LateralDistanceYRms;
  float RelativeLateralVelocityYRms;
  float RelativeLongitudinalAccelerationXRms;
  float RelativeLateralAccelerationYRms;
  float ExistenceProbability;  /*Probability of existence*/
  Obj_2_Quality()
  : RadarCommand(ars408::OBJ_QUALITY) {}
};

/*
 * CAN ID:    0x60D
 * Direction: FROM_DEVICE
 * Description: Object extended information.
 * This message contains additional object properties and is only sent
 * if it was activated in signal
 * RadarCfg_SendExtInfo (0x200). It is sent repeatedly for all objects in the same way as message.
 */
class Obj_3_Extended : RadarCommand
{
public:
  Obj_3_Extended()
  : RadarCommand(ars408::OBJ_EXTENDED) {}
  /* 0       255       1 */
  /*
   * Object ID
   * (since objects are tracked,
   *  the ID is kept throughout measurement cycles and does not have to be consecutive)
   */
  uint8_t Id;
  /* -10.00 10.47 0.01 m/s2 */
  /* Relative acceleration in longitudinal direction */
  float RelativeLongitudinalAccelerationX;
  /*-2.50 2.61 0.01 m/s2 */
  /* Relative acceleration in lateral direction */
  float RelativeLateralAccelerationY;
  enum ObjectClassProperty {POINT, CAR, TRUCK, RESERVED_01, MOTORCYCLE, BICYCLE, WIDE,
    RESERVED_02};
  ObjectClassProperty ObjectClass;  /*Class of the object*/
  float OrientationAngle;  /*-180  180 0.4 degOrientation angle of the object in degrees */
  float Length;  /*0.0 51.0 0.2 m Length of the tracked object*/
  float Width;  /*0.0 51.0 0.2 m  Length of the tracked object*/
};

/*
 * CAN ID:    0x60E
 * Direction: FROM_DEVICE
 * Description: Object collision detection warnings.
 * This message contains the collision detection
 * warning state and is only sent if collision detection was activated in message
 * CollDetCfg (0x400). It is sent repeatedly for all objects in the same way as
 * message Object_1_General (0x60B).
 */
class Obj_4_Warning : RadarCommand
{
public:
  Obj_4_Warning()
  : RadarCommand(ars408::OBJ_WARNING) {}
};

/*
 * CAN ID:    0x700
 * Direction: FROM_DEVICE
 * Description: Software Version Identification
 */
class VersionID : RadarCommand
{
public:
  VersionID()
  : RadarCommand(ars408::VERSION_ID) {}
};

/*
 * CAN ID:    0x8
 * Direction: FROM_DEVICE
 * Description: Control message for relay
 * The ARS40X Collision detection supports a Wilke CAN Bus Relay for direct switching of
 * external hardware. When collision detection is active and the relay control output is
 * activated with RadarCfg_CtrlRelay, the sensor sends the relay control message
 * CollDetRelayCtrl (0x8) cyclically (once per second).
 */
class CollDetRelayCtrl : RadarCommand
{
public:
  CollDetRelayCtrl()
  : RadarCommand(ars408::COLL_DET_RELAY_CTRL) {}
};
}  // namespace ars408

#endif  // ARS408_ROS__ARS408_COMMANDS_HPP_
