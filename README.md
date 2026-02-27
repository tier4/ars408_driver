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
sudo ip link set can0 up type can bitrate 500000
```

4. setup hardware (only first time)

```sh
# Objects detection with all extended properties
cansend can0 200#F8000000089C0000
```

5. Launch the driver

```sh
ros2 launch continental_ars408_socket_can.launch.xml
```

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

### launcher

- continental_ars408.xml
  - Base launcher
- continental_ars408_socket_can.xml
  - The launch file will initiate two nodes:
    1. socketcan_bridge to read from `can0` and publish the CAN msg in `can_raw`
    1. Continental ARS408 driver will read the `can_raw`, parse and publish `RadarTrack` or `RadarReturn`

## Reference

- This repository fork from original package [Perception Engine's Continental ARS408 Driver](https://gitlab.com/perceptionengine/pe-drivers/ars408_ros)
