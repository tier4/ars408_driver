# Perception Engine's ARS408 Driver

## How to run
1. Enable can port

`sudo ip link set can0 up type can bitrate 500000`

2. Launch the driver

`roslaunch pe_ars408_ros continental_ars408.launch`

Defaults to can0

3. Example output:

```
[/pe_ars408_node_ne0x8k_177881_8280997784855069204] CAN ID: 60b DATA: 0 50 14 11 80 20 1 79 
[/pe_ars408_node_ne0x8k_177881_8280997784855069204] CAN ID: 60c DATA: 5 84 63 3a 2 20 e8 
[/pe_ars408_node_ne0x8k_177881_8280997784855069204] CAN ID: 60c DATA: 2 b6 27 8d 82 20 64 
[/pe_ars408_node_ne0x8k_177881_8280997784855069204] CAN ID: 60d DATA: 5 7d 8f a1 70 80 19 c 
[/pe_ars408_node_ne0x8k_177881_8280997784855069204] CAN ID: 60d DATA: 3 7d 4f a1 70 80 d c 
[/pe_ars408_node_ne0x8k_177881_8280997784855069204] CAN ID: 60d DATA: 2 7d f a0 70 80 e 7 
[/pe_ars408_node_ne0x8k_177881_8280997784855069204] CAN ID: 60a DATA: 6 ad 67 10 
```
#Driver


1. Node
1. Driver

## node

ROS topic subscription, publication

## Driver
CAN parsing, conversion

Radar configuration