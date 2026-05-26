# Continental ARS408 Driver

This is Continental ARS408 Driver for ROS2.

## How to use

1. build environment

```sh
$ rosdep install --from-paths src --ignore-src -r -y
```

2. build

```sh
$ colcon build
```

3. Enable can port

```sh
# Configure only the necessary CAN ports
# Replace X with any value
sudo ip link set canX up type can bitrate 500000
```

4. setup hardware (Only the first time or when the hardware configuration changes)

```sh
# Object detection with all extended properties
# Please configure the Sensor ID and related settings for the CAN interfaces (as in steps 3 and 4)
# Set Y to the value (0-7) that corresponds to the Sensor ID configured in the hardware
# On first-time hardware setup, the Sensor ID is 0, so set Y to 0
cansend canX 2Y0#FA000000089C0000  # Set Sensor ID from Y to 0
cansend canX 2Y0#FA000000099C0000  # Set Sensor ID from Y to 1
cansend canX 2Y0#FA0000000A9C0000  # Set Sensor ID from Y to 2
cansend canX 2Y0#FA0000000B9C0000  # Set Sensor ID from Y to 3
cansend canX 2Y0#FA0000000C9C0000  # Set Sensor ID from Y to 4
cansend canX 2Y0#FA0000000D9C0000  # Set Sensor ID from Y to 5
cansend canX 2Y0#FA0000000E9C0000  # Set Sensor ID from Y to 6
cansend canX 2Y0#FA0000000F9C0000  # Set Sensor ID from Y to 7
```

5. Launch CAN bridge (outside this package, e.g. `ros2_socketcan`) and the driver

```sh
# Example: start socket_can_bridge on your vehicle launch stack, then:
ros2 launch pe_ars408_ros continental_ars408.launch.xml
```

## Design
### Input

- `~/input/frame` (remap to `from_can_bus` from `socket_can_bridge`)
  - `can_msgs` <https://github.com/ros-industrial/ros_canopen/tree/melodic-devel/can_msgs>
- `~/input/odometry` (default remap: `/localization/kinematic_state`)
  - `nav_msgs/Odometry` — used to publish motion CAN 0x300/0x301 when `publish_motion_input` is true

### Output

- `~/output/to_can_bus` (remap to `to_can_bus` for `socket_can_bridge`)
  - Motion CAN frames (0x300 Speed Information, 0x301 Yaw Rate Information)
- `output/objects`
  - `RadarTrack`: <https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTrack.msg>
  - If you want to visualize, you should choose `RadarTrack` and visualize in rviz using [radar_tracks_msgs_converter](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/radar_tracks_msgs_converter) with autoware.universe.
- `output/return`
  - `RadarReturn`: <https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarReturn.msg>

### parameters

- `publish_radar_track`
  - The bool parameter to publish `output/objects` topic
- `publish_radar_return`
  - The bool parameter to publish `output/return` topic
- `output_frame`
  - The string parameter of the output frame id
- `sequential_publish`
  - The bool parameter to determine output publishing behavior.
  - If this parameter is set to false (default value), the driver will publish output after receiving a complete cycle of sequential data from the CAN data topic.
  - If this parameter is set to true, the driver will publish output every time data is received from the CAN data topic.
- `size_x`
  - The assumed x-axis size of output objects [m]. The default parameter is 1.8, which derive from distance resolution measuring of ARS408 for far range.
- `size_y`
  - The assumed y-axis size of output objects [m]. The default parameter is 1.8, which derive from distance resolution measuring of ARS408 for far range.
- `radar_id`
  - Sensor ID of this node instance (0–7). Launch one node per radar with a distinct `radar_id`.
- `publish_objects_name` / `publish_scan_name`
  - Topic names for radar tracks and scan output.
- `publish_motion_input`
  - When true, publish 0x300/0x301 on `~/output/to_can_bus` from odometry at `motion_publish_rate_hz`.
- `motion_publish_rate_hz`, `speed_standstill_threshold_mps`, `speed_moving_threshold_mps`
  - Motion CAN encoding options (see `ars408_driver.param.yaml`).
- `can_receive_check_rate_hz`
  - The parameter specifies the check/poll rate of the CAN receive status [Hz].
- `can_receive_check_timeout_sec`
  - The parameter specifies the CAN receive status check/poll timeout [sec].

### launcher

- `continental_ars408.launch.xml`
  - Single `pe_ars408_node` (RX + TX + object publishing). Remaps:
    - `input/frame` → CAN RX (`from_can_bus`)
    - `output/to_can_bus` → CAN TX (`to_can_bus`)
    - `input/odometry` → vehicle odometry

## Reference

- This repository fork from original package [Perception Engine's Continental ARS408 Driver](https://gitlab.com/perceptionengine/pe-drivers/ars408_ros)
