# Pan Tilt Bringup

System-level launch files for the Pan Tilt mechanism (`pt100`, `pt101`): the full bringup (control with the Gemini 2 driver by default, or the supported OAK-D S2 alternative), either camera on its own, and the camera configuration.

## Contents

| Path | Purpose |
|------|---------|
| `launch/pantilt.launch.py` | Full system: includes `pt_control`'s launch file and the selected real camera driver; simulation skips both drivers |
| `launch/gemini2.launch.py` | Upstream Orbbec Gemini 2 RGB/depth/IMU driver, a depth-derived `/gemini2/scan`, optional colored cloud and Cloudini compression, and configurable mount TF |
| `launch/oakd.launch.py` | The OAK-D S2 driver, the `/oak/scan` slice and the optional point cloud and octomap nodes, in one composable node container |
| `config/oakd_vio.yaml` | Camera base profile: RGB, depth and IMU; VIO and point cloud disabled |
| `config/oakd_vio_pcl.yaml` | OAK-D overlay for `pointcloud:=true`: aligned depth, RGBD point cloud and its compression |
| `config/gemini2_pcl.yaml` | Gemini 2 Cloudini input, output and resolution when `pointcloud:=true` |
| `config/depthimage_to_laserscan.yaml` | The `/oak/scan` depth slice |
| `config/gemini2_depth_to_scan.yaml` | The `/gemini2/scan` depth slice |
| `config/octomap.yaml` | `octomap_server` settings |
| `patches/orbbec-kilted-qos.patch` | Kilted QoS API compatibility fix for the pinned upstream image-sync example |
| `src/pcl_compressor_node.cpp` | Composable node that compresses the point cloud with [cloudini](https://github.com/facontidavide/cloudini) |

## Requirements

ROS 2 Kilted with `orbbec_camera`, `tf2_ros`, `depthai-ros` and `octomap_server` at runtime, plus `pt_control`. Build OrbbecSDK_ROS2 from source in the workspace and install its USB rules as described in the [repository installation instructions](../README.md#installation). Building the C++ point cloud node needs [cloudini](https://github.com/facontidavide/cloudini) and `point_cloud_interfaces`, which are not plain apt packages; clone cloudini into the workspace as described in the [repository README](../README.md#installation). Camera launch tests live in `pt_control/test/test_camera_launch.py`; CI does not build this package's C++ component. On a Raspberry Pi 5 the alternative OAK-D S2 camera also needs a raised USB current limit (see the [repository README](../README.md#raspberry-pi-5)).

## Running

```bash
ros2 launch pt_bringup pantilt.launch.py                        # control + Gemini 2 driver
ros2 launch pt_bringup pantilt.launch.py pointcloud:=true       # Gemini 2 colored cloud
ros2 launch pt_bringup pantilt.launch.py sim:=true              # MuJoCo simulation, no camera driver
ros2 launch pt_bringup gemini2.launch.py publish_mount_tf:=false # Gemini 2 camera only, no model required
ros2 launch pt_bringup oakd.launch.py                           # alternative OAK-D S2 camera only
```

The `joy` node is not started; run `ros2 run joy joy_node` before using a joystick. `sim:=true` needs the simulation packages listed in the [repository README](../README.md#simulation-optional).

To keep the selected camera in the URDF while disabling its real driver and streaming pipeline:

```bash
ros2 launch pt_bringup pantilt.launch.py camera_config:=gemini2 enable_camera:=false
ros2 launch pt_bringup pantilt.launch.py camera_config:=oakd_s2 enable_camera:=false
```

`enable_camera` defaults to `true` and affects only real-camera bringup. When false, control, the selected camera meshes and URDF TF remain available; camera-driver TF and streaming helpers are not started. It does not disable the MuJoCo simulated camera. Driver launch files are included lazily: Gemini 2 loads only Orbbec, OAK-D S2 loads only DepthAI, and disabled streaming loads neither driver. This is runtime selection; package manifests still declare dependencies for the supported features.

### Launch arguments

| Argument | Default | Meaning |
|----------|---------|---------|
| `sts_serial_port`, `use_mock`, `diagnostics`, `pantilt_config`, `camera_config` | as in [`pt_control`](../pt_control/README.md#launch-arguments) | Passed to `pt_control` |
| `enable_camera` | `true` | Start the selected real camera driver; false retains URDF geometry and skips streaming |
| `use_sim_time` | `false` | Use `/clock` instead of system time |
| `camera_fps` | `15` | Shared RGB/depth rate for Gemini 2 or OAK-D S2: 5, 10, 15, or 30 Hz |
| `pointcloud` | `false` | Gemini 2: one native colored cloud plus Cloudini; OAK-D S2: layer `oakd_vio_pcl.yaml` on the base profile |
| `octomap` | `false` | OAK-D S2 only: run `octomap_server` on the point cloud; needs `pointcloud:=true` |
| `tf_parent_frame` | empty | Full bringup chooses `oak_link` for Gemini 2 or `tilt_link` for OAK-D S2 |
| `serial_number`, `usb_port` | empty | Gemini 2 device selectors |
| `camera_mount_xyz` | `0 0 0` | Gemini parent-to-driver translation in meters; provisional until measured |
| `camera_mount_rpy` | `3.141592653589793 0 0` | Gemini parent-to-driver roll, pitch, yaw in radians |
| `publish_mount_tf` | `true` | Publish Gemini mount TF; disable if supplied elsewhere |
| `sim` | `false` | Run in MuJoCo instead of on hardware; forces `use_sim_time` and mock motors, and skips the camera driver |
| `mujoco_gui` | `false` | `sim` only: open the MuJoCo viewer (needs a display) |
| `mujoco_scene` | `flat` | `sim` only: `flat`, `none` or a scene file |

Both camera-only launches accept `enable_camera` (default `true`); false returns without loading driver dependencies or starting streaming helpers. LeKiwi forwards this flag directly. `oakd.launch.py` also takes `camera_fps`, `pointcloud`, `octomap` and `tf_parent_frame`. `octomap` and `tf_parent_frame` are also accepted by `pantilt.launch.py` and passed on.

## Configuration

| File | Common changes |
|------|----------------|
| `oakd_vio.yaml` | OAK-D resolution and frame rates, IMU rates, depth threshold, USB speed |
| `gemini2_pcl.yaml` | Gemini Cloudini input, output and 1 mm resolution |
| `depthimage_to_laserscan.yaml` | `scan_height` (rows sampled), `range_min` and `range_max` for `/oak/scan` |
| `octomap.yaml` | Octree `resolution`, `frame_id`, sensor `max_range` |

## OAK-D S2 camera modes

These modes apply to `camera_config:=oakd_s2` or the standalone `oakd.launch.py`. Gemini 2 uses its own Orbbec pipeline (see below).

| Mode | Publishes | Config |
|------|-----------|--------|
| Default | RGB and depth at 640×400 and 15 Hz, 100 Hz IMU and `/oak/scan`; VIO disabled | `oakd_vio.yaml` |
| `pointcloud:=true` | The above with depth aligned to RGB, plus `/oak/rgbd/points` and `/oak/rgbd/points/compressed` | `oakd_vio_pcl.yaml` on top of `oakd_vio.yaml` |
| `pointcloud:=true octomap:=true` | Also a persistent 3D occupancy octree | `octomap.yaml` |

- **Default mode** leaves depth unaligned, because `depthai_ros_driver` 3.1.0 crashes on the alignment code when only a ROS publisher consumes the output. The unaligned image is enough for `/oak/scan`. A threshold filter keeps depth between 0.45 and 4 m, and the infrared emitter is disabled since the OAK-D S2 has none.
- **Point cloud mode** pins the RGB size and turns decimation off, because changing either crashes the driver together with aligned depth and the RGBD output. It has a higher CPU load. Depth and RGB must run at the same rate, or depth, scan and point cloud silently stop.
- **Compression** uses cloudini at 1 mm resolution and leaves colour uncompressed; the resolution is in `oakd_vio_pcl.yaml`.
- **Octomap** builds its tree in the `odom` frame at 5 cm resolution. It looks up the camera's TF for each cloud, so points land correctly while the pan-tilt sweeps. It expects a robot that publishes `odom` and `base_footprint`; adjust `octomap.yaml` if yours differs.

## Troubleshooting

Set `DEPTHAI_DEBUG=1` before launching for verbose camera driver logs:

```bash
DEPTHAI_DEBUG=1 ros2 launch pt_bringup pantilt.launch.py camera_config:=oakd_s2
```

## Using it on another robot

To use either camera without pan-tilt control, launch `gemini2.launch.py` or `oakd.launch.py` with `tf_parent_frame` set to the host mount. For Gemini 2, also set the measured `camera_mount_xyz` and `camera_mount_rpy`; use `publish_mount_tf:=false` if the host already supplies `gemini2_link`. `pantilt.launch.py` runs its own controller manager, so a robot that shares the servo bus does not include it; see [`pt_control`](../pt_control/README.md#using-it-on-another-robot).

## Camera selection

Pass `camera_config:=gemini2` independently of `pantilt_config:=pt100|pt101`.
The default is `gemini2`; pass `camera_config:=oakd_s2` for the supported OAK-D S2 alternative. The same selection is forwarded to URDF generation and,
in simulation, to the MuJoCo builder. When launching `pt_control` directly, its explicit `mujoco_model` option overrides
model building and must already match the selected camera geometry.

Gemini 2 uses its own camera and tilt-mount meshes, aligned mounting faces, equal
motor-side clearance, and the upright camera-body rotation. Camera/IMU frame names
remain available for compatibility. Real Gemini bringup uses the upstream Orbbec driver and its native sensor frames. See [Gemini 2 topics and calibration](../README.md#gemini-2-camera). Both real cameras publish depth-derived scans and support Cloudini compression. Octomap remains OAK-only, and Gemini rejects `octomap:=true`. OAK VIO is disabled by default.

For simulation, `camera_config` selects `oakd_s2_subtree.xml` or
`gemini2_subtree.xml` through the builder. See the [MuJoCo camera documentation](../pt_mujoco/README.md#gemini-2-models)
for regeneration, standalone tools, and the remaining simulated sensor limitations.

## Gemini 2 driver and TF

`gemini2.launch.py` includes the upstream driver with namespace `gemini2`, RGB and registered depth enabled, IR disabled, and synchronized accelerometer/gyroscope output enabled. The Pi-oriented defaults request 640×360 RGB and 640×400 depth at 15 Hz; `camera_fps` selects 5, 10, 15 or 30 Hz. `pointcloud:=true` enables only `/gemini2/depth_registered/points`; uncolored `/gemini2/depth/points` stays off. Cloudini also publishes `/gemini2/depth_registered/points/compressed` at 1 mm resolution. It provides no VIO. `/gemini2/scan` is generated directly from the depth image in both point-cloud modes; publishing a cloud is unnecessary for the scan.

The upstream driver owns all calibrated sensor transforms below `gemini2_link`. This wrapper only publishes `tf_parent_frame -> gemini2_link`; standalone `tf_parent_frame` defaults to `oak_link`. The zero translation is provisional, and the default π roll corrects the legacy inverted mount orientation. Measure the physical sensor offset before relying on cloud-to-robot registration. The old URDF `oak_imu_frame` is not used by the real driver.

Check detection with `ros2 run orbbec_camera list_devices_node`, then inspect `/gemini2/color/image_raw`, `/gemini2/depth/image_raw`, and `/gemini2/gyro_accel/sample`. For USB permission errors, install the upstream rules and reconnect the camera; see the [installation instructions](../README.md#installation).
