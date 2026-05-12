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
# Set Y to the value (0?7) that corresponds to the Sensor ID configured in the hardware
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

5. Launch the driver

```sh
ros2 launch pe_ars408_ros continental_ars408_socket_can.launch.xml receiver_interval_sec:=1.0

## Design
### Input

- `input/frame`
  - `can_msgs` <https://github.com/ros-industrial/ros_canopen/tree/melodic-devel/can_msgs>

### Output

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
- `connection_count`
  - The parameter specifies the Radar (ARS408) connection count.
  - The default value is 2, which derives from the number of connected ARS408 units.
- `radar_id`
  - The parameter specifies the Radar ID.
  - The default parameter values are [0, 1].
  - The number of settings derives from connection_count, and each can be configured with a unique value from 0 to 7.
- `publish_radar_tracks_name`
  - The string parameter specifies the topic name to publish radar tracks.
  - The default parameter values are ["~/output/objects1", "~/output/objects2"].
  - The number of settings derives from connection_count, and each can be configured with a unique topic name.
- `publish_radar_scan_name`
  - The string parameter specifies the topic name to publish scan radar.
  - The default parameter values are ["~/output/scan1", "~/output/scan2"].
  - The number of settings derives from connection_count, and each can be configured with a unique topic name.
- `can_receive_check_rate_hz`
  - The parameter specifies the check/poll rate of the CAN receive status [Hz].
- `can_receive_check_timeout_sec`
  - The parameter specifies the CAN receive status check/poll timeout [sec].

### launcher

- continental_ars408.xml
  - Base launcher
- continental_ars408_socket_can.xml
  - The launch file will initiate two nodes:
    1. socketcan_bridge to read from `canN` and publish the CAN msg in `can_raw`
    1. Continental ARS408 driver will read the `can_raw`, parse and publish `RadarTrack` or `RadarReturn`

## Reference

- This repository fork from original package [Perception Engine's Continental ARS408 Driver](https://gitlab.com/perceptionengine/pe-drivers/ars408_ros)
