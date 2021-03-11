# Perception Engine's Continental ARS408 Driver

## How to compile

```
$ git clone https://gitlab.com/perceptionengine/pe-drivers/ars408_ros.git pe_ars408_ws/src && cd pe_ars408_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin build
```

## How to run
1. Enable can port
`sudo ip link set can0 up type can bitrate 500000`

2. Launch the driver
`roslaunch pe_ars408_ros continental_ars408.launch`
   
### Parameters

|Parameter|Description|Default|
|---|---|---|
|`can_device`|Device name of the can interface|`can0`|
|`can_topic`|Topic on which socketcan will publish the can raw msg|`can_raw`|

# Driver

The launch file will initiate two nodes:
1. socketcan_bridge to read from `can0` and publish the CAN msg in `can_raw`
1. Continental ARS408 driver will read the `can_raw`, parse and publish the detected objects using the Autoware.IV
   `autoware_perception_msgs/DynamicObjectArray` in the topic `/detection/radar/objects`.

# Visualization

To visualize the objects the Autoware's `dynamic_object_visualizer` needs to be launched and subscribe to `/detection/radar/objects`.

i.e.`roslaunch dynamic_object_visualization dynamic_object_visualizer.launch with_feature:=False input:=/detection/radar/objects`