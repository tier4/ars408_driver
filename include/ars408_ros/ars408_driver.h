/*
 * Copyright 2021. Perception Engine Inc. All rights reserved.
 */
namespace ars408
{
  /*
   * CAN ID:    0x200
   * Direction: TO_DEVICE
   * Description: Radar sensor configuration
   */
  class RadarCfg
  {

  };

  /*
   * CAN ID:    0x201
   * Direction: FROM_DEVICE
   * Description: Radar status
   */
  class RadarState
  {
    const int CAN_ID = 0x201;
    enum Status {FAILED, SUCCESS};
    enum Config {INACTIVE, ACTIVE};

    Status ReadStatus;      /*State of reading the configuration parameters from non-volatile memory at startup*/
    Status WriteStatus;     /* State of storing a configuration parameter to non-volatile memory (initially this value is set to 0x0 and set to 0x1 after a configuration has been sent and successfully stored)*/
    uint8_t MaxDistance;    /**/
    bool PersistentError;   /*An internal error which might not disappear after a reset has been detected.*/
    bool Interference;      /*Interference with another radar sensor has been detected.*/
    bool TemperatureError;  /*Error will be active if the temperature is below or above the defined range.*/
    bool TemporaryError;    /*A temporary error which will most probably disappear after a sensor reset has been detected.*/
    bool VoltageError;      /*Error will be active if the operating voltage is below or above the defined range for more than 5 seconds.*/
    uint8_t SensorID;       /*Sensor ID 0 - 7*/
    enum Sorting{NO_SORT, BY_RANGE, BY_RCS}; /*Current configuration of sorting index for object list*/
    enum PowerConfig{STANDARD, MINUS_3dB_GAIN, MINUS_6dB_GAIN, MINUS_9dB_GAIN}; /*Current configuration of transmitted radar power parameter*/
    Config CtrlRelayCfg;    /*True if relay control message is sent*/
    enum OutputTypeCfg{NONE, OBJECTS, CLUSTERS}; /*Currently selected output type as either clusters or objects*/
    Config SendQualityCfg;  /*True if quality information is sent for clusters or objects*/
    Config SendExtInfoCfg;  /*True if extended information is sent for objects*/
    enum MotionRxState{INPUT_OK, SPEED_MISSING, YAW_MISSING, SPEED_YAW_MISSING}; /*Shows the state of the speed and yaw rate input signals*/
    enum RCS_Threshold{NORMAL, HIGH_SENSITIVITY}; /*If true, the sensor’s high sensitivity mode is active*/
  };

  /*
   * CAN ID:    0x202
   * Direction: TO_DEVICE
   * Description: Cluster and Object filter configuration
   */
  class FilterCfg
  {
    const int CAN_ID = 0x202;
  };

  /*
   * CAN ID:    0x203
   * Direction: FROM_DEVICE
   * Description: Filter status header
   */
  class FilterState_Header
  {
    const int CAN_ID = 0x203;
  };

  /*
   * CAN ID:    0x204
   * Direction: TO_DEVICE
   * Description: Filter configuration status
   */
  class FilterState_Cfg
  {
    const int CAN_ID = 0x204;
  };

  /*
   * CAN ID:    0x400
   * Direction: TO_DEVICE
   * Description: Collision detection configuration
   */
  class CollDetCfg
  {
    const int CAN_ID = 0x400;
  };

  /*
   * CAN ID:    0x401
   * Direction: FROM_DEVICE
   * Description: Collision detection region configuration
   */
  class CollDetRegionCfg
  {
    const int CAN_ID = 0x401;
  };

  /*
   * CAN ID:    0x408
   * Direction: FROM_DEVICE
   * Description: Collision detection status
   */
  class CollDetState
  {
    const int CAN_ID = 0x408;
  };

  /*
   * CAN ID:    0x402
   * Direction: FROM_DEVICE
   * Description: Collision detection region status
   */
  class CollDetRegionState
  {
    const int CAN_ID = 0x402;
  };

  /*
   * CAN ID:    0x300
   * Direction: TO_DEVICE
   * Description: Vehicle (sensor platform) speed
   */
  class SpeedInformation
  {
    const int CAN_ID = 0x300;
  };

  /*
   * CAN ID:    0x301
   * Direction: TO_DEVICE
   * Description: Vehicle (sensor platform) yaw rate
   */
  class YawRateInformation
  {
    const int CAN_ID = 0x301;
  };

  /*
   * CAN ID:    0x600
   * Direction: FROM_DEVICE
   * Description: Cluster status (list header)
   */
  class Cluster_0_Status
  {
    const int CAN_ID = 0x600;
  };

  /*
   * CAN ID:    0x701
   * Direction: FROM_DEVICE
   * Description: Cluster general information
   */
  class Cluster_1_General
  {
    const int CAN_ID = 0x701;
  };

  /*
   * CAN ID:    0x702
   * Direction: FROM_DEVICE
   * Description: Cluster quality information
   */
  class Cluster_2_Quality
  {
    const int CAN_ID = 0x702;
  };

  /*
   * CAN ID:    0x60A
   * Direction: FROM_DEVICE
   * Description: Object status (list header)
   */
  class Obj_0_Status
  {
    const int CAN_ID = 0x60A;
  };

  /*
   * CAN ID:    0x60B
   * Direction: FROM_DEVICE
   * Description: Object general information
   */
  class Obj_1_General
  {
    const int CAN_ID = 0x60B;
  };

  /*
   * CAN ID:    0x60C
   * Direction: FROM_DEVICE
   * Description: Object quality information
   */
  class Obj_2_Quality
  {
    const int CAN_ID = 0x60C;
  };

  /*
   * CAN ID:    0x60D
   * Direction: FROM_DEVICE
   * Description: Object extended information
   */
  class Obj_3_Extended
  {
    const int CAN_ID = 0x60D;
  };

  /*
   * CAN ID:    0x60E
   * Direction: FROM_DEVICE
   * Description: Object collision detection warnings
   */
  class Obj_4_Warning
  {
    const int CAN_ID = 0x60E;
  };

  /*
   * CAN ID:    0x700
   * Direction: FROM_DEVICE
   * Description: Software Version Identification
   */
  class VersionID
  {
    const int CAN_ID = 0x700;
  };

  /*
   * CAN ID:    0x8
   * Direction: FROM_DEVICE
   * Description: Control message for relay
   */
  class CollDetRelayCtrl
  {
    const int CAN_ID = 0x8;
  };

}