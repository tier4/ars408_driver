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

4. Configure the radar (optional if using driver startup RadarCfg)

By default the driver sends **RadarCfg (0x200)** once at startup from `config/ars408_driver.param.yaml` (`radar_cfg.*`). Adjust `max_distance_m`, `output_type`, `send_quality`, `send_ext_info`, etc. there.

For first-time **Sensor ID** programming on the bench only, set `radar_cfg.update_sensor_id: true` and use the factory default ID in `radar_id`, or use manual `cansend` as in the Continental documentation.

5. Launch CAN bridge (outside this package, e.g. `ros2_socketcan`) and the driver

```sh
# Example: start socket_can_bridge on your vehicle launch stack, then:
ros2 launch pe_ars408_ros continental_ars408.launch.xml
```

## Design
### Input

- `~/from_can_bus` (remap to `from_can_bus` from `socket_can_bridge`)
  - `can_msgs` <https://github.com/ros-industrial/ros_canopen/tree/melodic-devel/can_msgs>
- `~/odometry` (default remap: `/localization/kinematic_state`)
  - `nav_msgs/Odometry` — used to publish motion CAN 0x300/0x301 when `publish_motion_input` is true

### Output

- `~/to_can_bus` (remap to `to_can_bus` for `socket_can_bridge`)
  - RadarCfg (0x200) at startup when `publish_radar_cfg_on_startup` is true
  - FilterCfg (0x202) sequence when `filter_cfg.send_on_startup` is true (after RadarCfg is verified)
  - Motion CAN frames (0x300 Speed Information, 0x301 Yaw Rate Information)
- `/diagnostics` — radar state (`0x201`), filter cfg (`0x203`/`0x204`), and firmware version (`0x700`) when enabled
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
- `size_x` / `size_y`
  - Fallback object dimensions [m] when 0x60D length/width are not available.
- `use_radar_reported_dimensions`
  - When true, `RadarTrack.size` uses length/width from 0x60D.
- `inflate_covariance_by_existence_probability`
  - When true, scales position/velocity covariance by `(1 / existence_probability)^2` from 0x60C (no dedicated field in `radar_msgs`).
- **Note:** `radar_msgs/RadarTrack` has no orientation field; Autoware derives heading from velocity in `radar_tracks_msgs_converter`.
- `radar_id`
  - Sensor ID of this node instance (0–7). Launch one node per radar with a distinct `radar_id`.
- `publish_objects_name` / `publish_scan_name`
  - Topic names for radar tracks and scan output.
- `publish_motion_input`
  - When true, publish 0x300/0x301 on `~/to_can_bus` from odometry at `motion_publish_rate_hz`. Remap the topic in launch only (not a ROS parameter).
- `motion_publish_rate_hz`, `speed_standstill_threshold_mps`, `speed_moving_threshold_mps`
  - Motion CAN encoding options (see `ars408_driver.param.yaml`).
- `publish_radar_cfg_on_startup`, `radar_cfg_startup_delay_sec`, `radar_cfg_retry_interval_sec`
  - Send RadarCfg (0x200) after launch; verify against `0x201` RadarState; re-send on mismatch until YAML matches.
  - While not verified, `RadarTracks` / `RadarScan` and motion CAN TX are suppressed; `ars408_radar_cfg` is published on `/diagnostics`.
- `publish_radar_state_diagnostics`
  - Publish `ars408_radar_state` on `/diagnostics` from 0x201 / 0x700.
- `radar_cfg.*` (nested, see `config/ars408_driver.param.yaml`)
  - `max_distance_m`, `output_type` (`none` | `objects` | `clusters`), `send_quality`, `send_ext_info`, `sort_index` (`no_sort` | `by_range` | `by_rcs`), `radar_power` (`minus_3db` | `minus_6db` | `minus_9db`; `standard` not allowed for Japan radio regulations), `store_in_nvm`, `rcs_threshold` (`normal` | `high_sensitivity`), `ctrl_relay`, `update_sensor_id`.
- **`filter_cfg`** (nested block in `config/ars408_driver.param.yaml`):
  1. **`filter_cfg.send_on_startup`** — send FilterCfg (0x202) after RadarCfg is verified (`false` = leave radar filter state unchanged).
  2. **`filter_cfg.criteria.<criterion>.active`** — enable (`true`) or disable (`false`) that criterion on the radar. Criterion keys are discovered automatically (no separate index list). **Do not** write `criteria: {}` when empty (ROS 2 registers it as a parameter with no value and the node aborts on launch).
  3. **`filter_cfg.criteria.<criterion>.target`** (`objects` | `clusters`), **`min` / `max`** — pass-through range when `active` is true (`min` ignored for `nof_obj` and `class`). Optional **`index`** overrides the criterion id (default: YAML key name).
- Timing: `filter_cfg.startup_delay_sec`, `filter_cfg.inter_send_delay_sec`, `filter_cfg.retry_interval_sec`. Does not gate `RadarTracks` output (unlike RadarCfg).
- Criterion key names: `distance`, `nof_obj`, `pos_x`, `azimuth`, `rcs`, etc. (or set `index` to `0`–`15` / `0xA`).
- `can_receive_check_rate_hz`
  - The parameter specifies the check/poll rate of the CAN receive status [Hz].
- `can_receive_check_timeout_sec`
  - The parameter specifies the CAN receive status check/poll timeout [sec].

### launcher

- `continental_ars408.launch.xml`
  - Single `pe_ars408_node` (RX + TX + object publishing). Launch args / remaps (same names as `ros2_socketcan`):
    - `from_can_bus` — CAN RX
    - `to_can_bus` — CAN TX
    - `odometry` — vehicle odometry

## Reference

- This repository fork from original package [Perception Engine's Continental ARS408 Driver](https://gitlab.com/perceptionengine/pe-drivers/ars408_ros)
