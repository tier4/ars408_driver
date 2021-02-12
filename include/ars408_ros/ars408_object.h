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
    std::string ToString(){
      std::ostringstream stream;
      stream << "Sequence: " << sequence_id << ", ID: " << (unsigned int)id << ", X: " << distance_long_x
      << ", Y: " << distance_lat_y << ", SpeedX: " << speed_long_x << ", SpeedY: " << speed_lat_y
      << ", rcs: " << rcs  << ", Status: ";
      switch(dynamic_property)
      {
        case ars408::Obj_1_General::DynamicProperty::MOVING:
          stream << "MOVING ";
          break;
        case ars408::Obj_1_General::DynamicProperty::STATIONARY:
          stream << "STATIONARY ";
          break;
        case ars408::Obj_1_General::DynamicProperty::ONCOMING:
          stream << "ONCOMING ";
          break;
        case ars408::Obj_1_General::DynamicProperty::CROSSING_LEFT:
          stream << "CROSSING_LEFT ";
          break;
        case ars408::Obj_1_General::DynamicProperty::CROSSING_RIGHT:
          stream << "CROSSING_RIGHT ";
          break;
        case ars408::Obj_1_General::DynamicProperty::UNKNOWN:
          stream << "UNKNOWN ";
          break;
        case ars408::Obj_1_General::DynamicProperty::STOPPED:
          stream << "STOPPED ";
          break;
      }
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
