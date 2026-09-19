# Pan Tilt Bringup

System-level launch files for the Pan Tilt mechanism (`pt100`, `pt101`): the full bringup (control and camera), the OAK-D S2 camera on its own, and the camera configuration.

## Contents

| Path | Purpose |
|------|---------|
| `launch/pantilt.launch.py` | Full system: includes `pt_control`'s launch file and, on real hardware, the camera |
| `launch/oakd.launch.py` | The OAK-D S2 driver, the `/oak/scan` slice and the optional point cloud and octomap nodes, in one composable node container |
| `config/oakd_vio.yaml` | Camera base profile: RGB, IMU, VIO and depth, no point cloud |
| `config/oakd_vio_pcl.yaml` | Overlay for `pointcloud:=true`: aligned depth, RGBD point cloud and its compression |
| `config/depthimage_to_laserscan.yaml` | The `/oak/scan` slice |
| `config/octomap.yaml` | `octomap_server` settings |
| `src/pcl_compressor_node.cpp` | Composable node that compresses the point cloud with [cloudini](https://github.com/facontidavide/cloudini) |

## Requirements

ROS 2 Kilted with `depthai-ros` and `octomap_server` at runtime, plus `pt_control`. Building the C++ point cloud node needs [cloudini](https://github.com/facontidavide/cloudini) and `point_cloud_interfaces`, which are not plain apt packages; clone cloudini into the workspace as described in the [repository README](../README.md#installation). The package has no tests of its own and CI does not build it. On a Raspberry Pi 5 the camera also needs a raised USB current limit (see the [repository README](../README.md#raspberry-pi-5)).

## Running

```bash
ros2 launch pt_bringup pantilt.launch.py                        # control + camera
ros2 launch pt_bringup pantilt.launch.py pointcloud:=true       # plus aligned depth and a compressed point cloud
ros2 launch pt_bringup pantilt.launch.py sim:=true              # MuJoCo simulation, no camera driver
ros2 launch pt_bringup oakd.launch.py                           # camera only
```

The `joy` node is not started; run `ros2 run joy joy_node` before using a joystick. `sim:=true` needs the simulation packages listed in the [repository README](../README.md#simulation-optional).

### Launch arguments

| Argument | Default | Meaning |
|----------|---------|---------|
| `sts_serial_port`, `use_mock`, `diagnostics`, `pantilt_config` | as in [`pt_control`](../pt_control/README.md#launch-arguments) | Passed to `pt_control` |
| `use_sim_time` | `false` | Use `/clock` instead of system time |
| `pointcloud` | `false` | Layer `oakd_vio_pcl.yaml` on the base profile |
| `octomap` | `false` | Also run `octomap_server` on the point cloud; needs `pointcloud:=true` |
| `tf_parent_frame` | `tilt_link` | TF frame the camera is mounted to |
| `sim` | `false` | Run in MuJoCo instead of on hardware; forces `use_sim_time` and mock motors, and skips the camera driver |
| `mujoco_gui` | `false` | `sim` only: open the MuJoCo viewer (needs a display) |
| `mujoco_scene` | `flat` | `sim` only: `flat`, `none` or a scene file |

`oakd.launch.py` takes only `pointcloud`, `octomap` and `tf_parent_frame`. `octomap` and `tf_parent_frame` are also accepted by `pantilt.launch.py` and passed on.

## Configuration

| File | Common changes |
|------|----------------|
| `oakd_vio.yaml` | Camera resolution and frame rates, IMU rates, depth threshold, USB speed |
| `depthimage_to_laserscan.yaml` | `scan_height` (rows sampled), `range_min` and `range_max` for `/oak/scan` |
| `octomap.yaml` | Octree `resolution`, `frame_id`, sensor `max_range` |

## Camera modes

| Mode | Publishes | Config |
|------|-----------|--------|
| Default | RGB and depth at 640x400 and 30 Hz, IMU, VIO at 60 Hz, `/oak/scan` | `oakd_vio.yaml` |
| `pointcloud:=true` | The above with depth aligned to RGB, plus `/oak/rgbd/points` and `/oak/rgbd/points/compressed` | `oakd_vio_pcl.yaml` on top of `oakd_vio.yaml` |
| `pointcloud:=true octomap:=true` | Also a persistent 3D occupancy octree | `octomap.yaml` |

- **Default mode** leaves depth unaligned, because `depthai_ros_driver` 3.1.0 crashes on the alignment code when only a ROS publisher consumes the output. The unaligned image is enough for `/oak/scan`. A threshold filter keeps depth between 0.45 and 4 m, and the infrared emitter is disabled since the OAK-D S2 has none.
- **Point cloud mode** pins the RGB size and turns decimation off, because changing either crashes the driver together with aligned depth and the RGBD output. It has a higher CPU load. Depth and RGB must run at the same rate, or depth, scan and point cloud silently stop.
- **Compression** uses cloudini at 1 mm resolution and leaves colour uncompressed; the resolution is in `oakd_vio_pcl.yaml`.
- **Octomap** builds its tree in the `odom` frame at 5 cm resolution. It looks up the camera's TF for each cloud, so points land correctly while the pan-tilt sweeps. It expects a robot that publishes `odom` and `base_footprint`; adjust `octomap.yaml` if yours differs.

## Troubleshooting

Set `DEPTHAI_DEBUG=1` before launching for verbose camera driver logs:

```bash
DEPTHAI_DEBUG=1 ros2 launch pt_bringup pantilt.launch.py
```

## Using it on another robot

To use the camera on a robot without the pan-tilt, launch `oakd.launch.py` with `tf_parent_frame` set to that robot's camera mount link. `pantilt.launch.py` runs its own controller manager, so a robot that shares the servo bus does not include it; see [`pt_control`](../pt_control/README.md#using-it-on-another-robot).
