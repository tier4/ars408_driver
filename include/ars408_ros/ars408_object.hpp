/*
 * Copyright 2021. Perception Engine Inc. All rights reserved.
 */

#ifndef PE_ARS408_ROS_ARS408_OBJECT_H
#define PE_ARS408_ROS_ARS408_OBJECT_H
namespace ars408
{
  class RadarObject
  {
  public:
    std::string ObjectClassToString()
    {
      switch(object_class)
      {
        case ars408::Obj_3_Extended::ObjectClassProperty::WIDE:
          return "WIDE ";
          break;
        case ars408::Obj_3_Extended::ObjectClassProperty::MOTORCYCLE:
          return "MOTORCYCLE ";
          break;
        case ars408::Obj_3_Extended::ObjectClassProperty::BICYCLE:
          return "BICYCLE ";
          break;
        case ars408::Obj_3_Extended::ObjectClassProperty::RESERVED_01:
          return "RESERVED_01";
          break;
        case ars408::Obj_3_Extended::ObjectClassProperty::POINT:
          return "POINT";
          break;
        case ars408::Obj_3_Extended::ObjectClassProperty::CAR:
          return "CAR ";
          break;
        case ars408::Obj_3_Extended::ObjectClassProperty::RESERVED_02:
          return "RESERVED_02";
          break;
        case ars408::Obj_3_Extended::ObjectClassProperty::TRUCK:
          return "TRUCK";
          break;
        default:
          return "UNDEFINED";
          break;
      }
    }
    std::string DynamicPropertyToString()
    {
      switch(dynamic_property)
      {
        case ars408::Obj_1_General::DynamicProperty::MOVING:
          return "MOVING";
          break;
        case ars408::Obj_1_General::DynamicProperty::STATIONARY:
          return "STATIONARY";
          break;
        case ars408::Obj_1_General::DynamicProperty::ONCOMING:
          return "ONCOMING";
          break;
        case ars408::Obj_1_General::DynamicProperty::CROSSING_LEFT:
          return "CROSSING_LEFT";
          break;
        case ars408::Obj_1_General::DynamicProperty::CROSSING_RIGHT:
          return "CROSSING_RIGHT";
          break;
        case ars408::Obj_1_General::DynamicProperty::UNKNOWN:
          return "UNKNOWN";
          break;
        case ars408::Obj_1_General::DynamicProperty::STOPPED:
          return "STOPPED";
          break;
        default:
          return "UNDEFINED";
          break;
      }
    }
    std::string ToString(){
      std::ostringstream stream;
      stream << "Sequence: " << sequence_id << ", ID: " << (unsigned int)id << ", X: " << distance_long_x
             << ", Y: " << distance_lat_y << ", SpeedX: " << speed_long_x << ", SpeedY: " << speed_lat_y
             << ", rcs: " << rcs  << ", Status: ";
      stream << ObjectClassToString();
      stream << std::endl;
      return stream.str();
    }
    uint16_t sequence_id;
    uint8_t id;
    float distance_long_x;
    float distance_lat_y;
    float speed_long_x;
    float speed_lat_y;
    ars408::Obj_1_General::DynamicProperty dynamic_property;
    float rcs;
    float rel_acceleration_long_x;/*Relative acceleration in longitudinal direction in m/s^2 */
    ars408::Obj_3_Extended::ObjectClassProperty object_class;
    float rel_acceleration_lat_y;/*Relative acceleration in lateral direction in m/s^2*/
    float orientation_angle;/*Orientation angle of the object in degrees*/
    float length;/*Length of the tracked object*/
    float width;/*Width of the tracked object*/
    float probability_existence;
  };
}
#endif //PE_ARS408_ROS_ARS408_OBJECT_H
