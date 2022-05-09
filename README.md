# Continental ARS408 Driver

This is Continental ARS408 Driver for ROS2.

## How to use

1. build environment

```sh
#$ vcs import . < build_depends.repos
$ rosdep install --from-paths src --ignore-src -r -y
```

2. build

```
$ colcon build
```

3. Enable can port

```
sudo ip link set can0 up type can bitrate 500000
```

4. Launch the driver

```
roslaunch pe_ars408_ros continental_ars408_socket_can.launch
```

## Design
### Input / Output

- Input
   - `can_msgs` <https://github.com/ros-industrial/ros_canopen/tree/melodic-devel/can_msgs>
- Output
   - `RadarTrack`: <https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTrack.msg>

### Parameters

-  `can_device`
    - Device name of the can interface
    - Default is `can0`
-  `can_topic`
    - Topic on which socketcan will publish the can raw msg
    - Default is  `can_raw`

### Launch

The launch file will initiate two nodes:

1. socketcan_bridge to read from `can0` and publish the CAN msg in `can_raw`
1. Continental ARS408 driver will read the `can_raw`, parse and publish the detected objects using the Autoware.universe
    - `autoware_auto_msgs/autoware_auto_perception_msgs/TrackedObjects` in the topic `/objects`.

If you want to visualize, you should choose `TrackedObjects` and visualize in rviz using autoware.

## Reference

- Fork from original package [Perception Engine's Continental ARS408 Driver](https://gitlab.com/perceptionengine/pe-drivers/ars408_ros)
