# Pan Tilt 100

![Project Status](https://img.shields.io/badge/Status-Active-brightgreen)
![ROS 2](https://img.shields.io/badge/ROS%202-Kilted%20(Ubuntu%2024.04)-blue?style=flat&logo=ros&logoSize=auto)
[![CI](https://github.com/adityakamath/pantilt_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/adityakamath/pantilt_ros2/actions/workflows/ci.yml)
[![Ask DeepWiki (Experimental)](https://deepwiki.com/badge.svg)](https://deepwiki.com/adityakamath/pantilt_ros2)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

ROS 2 software stack for a 2-DOF pan-tilt camera mount built from [SO-100 or SO-101](https://github.com/TheRobotStudio/SO-ARM100) parts, two [Feetech STS3215](https://www.feetechrc.com/2020-05-13_56655.html) servo motors and an [Orbbec Gemini 2](https://www.seeedstudio.com/Orbbec-Gemini-2-3D-Camera-p-6464.html?sensecap_affiliate=8fjl172&referring_service=link) camera. [OAK-D S2](https://docs.luxonis.com/hardware/products/OAK-D%20S2) is also supported as an alternative. It provides position control with joystick teleop, Gemini 2 depth camera bringup, an alternative OAK-D S2 pipeline, a MuJoCo simulation, and an embeddable xacro module for other robots such as [lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2).

<p align="center">
  <img width="500" height="575" alt="Screenshot 2026-04-28 at 15 56 11" src="https://github.com/user-attachments/assets/c9520454-7523-44a7-bcb3-8b6428437759" />
</p>

## Contents

| Package | Purpose |
|---------|----------|
| [`pt_description`](pt_description/) | URDF/xacro model (standalone entry, embeddable macros, all PT100/PT101 × Gemini 2/OAK-D S2 combinations pre-generated), meshes, `urdf.launch.py` |
| [`pt_control`](pt_control/) | `ros2_control` setup and configuration (`urdf_config.yaml`, `pantilt_controller.yaml`, `teleop_config.yaml`) and `pantilt.launch.py` |
| [`pt_bringup`](pt_bringup/) | Full-system launch with the Gemini 2 driver by default, alternative OAK-D S2 driver and configuration |
| [`pt_mujoco`](pt_mujoco/) | MuJoCo model generated from the URDF, camera plugin config, standalone viewer, benchmark |

## Hardware

| Component    | Details                                                                                             |
|--------------|-----------------------------------------------------------------------------------------------------|
| Pan motor    | [Feetech STS3215](https://www.feetechrc.com/2020-05-13_56655.html), motor ID `1`                    |
| Tilt motor   | Feetech STS3215, motor ID `2`                                                                       |
| Servo driver | [Waveshare Bus Servo Adapter A](https://www.waveshare.com/bus-servo-adapter-a.htm)                  |
| Camera | [Orbbec Gemini 2](https://www.seeedstudio.com/Orbbec-Gemini-2-3D-Camera-p-6464.html?sensecap_affiliate=8fjl172&referring_service=link) (default; real driver, geometry and simulation), or OAK-D S2 (supported alternative with driver) |
| Structure    | 3D printed base and shoulder parts from [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) or [SO-ARM101](https://github.com/TheRobotStudio/SO-ARM101) |
| Camera mount | 3D printed camera-specific bracket (STL in [`pt_description/meshes/`](pt_description/meshes/))             |

Both motors share one serial bus at 1 Mbaud, connected through the Waveshare driver. **PT100** uses the SO-ARM100 base and shoulder parts and **PT101** uses the SO-ARM101 ones; you pick the variant with the `pantilt_config` launch argument (`pt101`, the default, is recommended).

## Installation

Requires [ROS 2 Kilted](https://docs.ros.org/en/kilted/). Gemini 2 uses the upstream [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2) `v2-main` branch. The official Noble/ARM64 apt index has no `ros-kilted-orbbec-camera` package as checked on 2026-09-27, so build the driver from source in the same workspace. Its system dependencies and the alternative OAK-D S2 driver can be installed with apt-get:

```bash
sudo apt-get update
sudo apt-get install libgflags-dev nlohmann-json3-dev libgoogle-glog-dev libdw-dev libssl-dev \
  ros-kilted-backward-ros ros-kilted-image-transport ros-kilted-image-transport-plugins \
  ros-kilted-image-publisher ros-kilted-camera-info-manager ros-kilted-diagnostic-updater \
  ros-kilted-statistics-msgs ros-kilted-xacro ros-kilted-depthai-ros
```

Clone the packages and build the Orbbec driver first:

```bash
source /opt/ros/kilted/setup.bash
cd <your workspace>/src
git clone https://github.com/adityakamath/pantilt_ros2.git
git clone https://github.com/adityakamath/sts_hardware_interface.git
git clone https://github.com/facontidavide/cloudini.git
git clone --branch v2-main https://github.com/orbbec/OrbbecSDK_ROS2.git
git -C OrbbecSDK_ROS2 checkout 8e7cad2bfa2c4a6ac4e779be99c64e72166043af
git -C OrbbecSDK_ROS2 apply ../pantilt_ros2/pt_bringup/patches/orbbec-kilted-qos.patch
cd ..
rosdep install --from-paths src/OrbbecSDK_ROS2 --ignore-src -r -y --rosdistro kilted
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --packages-select orbbec_camera_msgs orbbec_description orbbec_camera \
  --executor sequential --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_CXX_FLAGS=-Wno-error=cpp
source install/setup.bash
colcon build --packages-select cloudini_lib cloudini_ros pt_description pt_mujoco pt_control pt_bringup sts_hardware_interface
source install/setup.bash
```

The `-Wno-error=cpp` flag allows Kilted's deprecated-header warnings. The included [QoS compatibility patch](pt_bringup/patches/orbbec-kilted-qos.patch) switches the image-sync example to the current `rclcpp::QoS` API. When using pantilt as a LeKiwi submodule, change the patch path to `../lekiwi_ros2/payloads/pantilt_ros2/pt_bringup/patches/orbbec-kilted-qos.patch`. The integration targets upstream revision `8e7cad2bfa2c4a6ac4e779be99c64e72166043af` (2.9.3).

Install the [upstream USB permission rules](https://github.com/orbbec/OrbbecSDK_ROS2#registration-script-required), then reconnect the camera:

```bash
sudo cp src/OrbbecSDK_ROS2/orbbec_camera/scripts/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
ros2 run orbbec_camera list_devices_node
```

The teleop uses [`joy_teleop`](https://index.ros.org/p/joy_teleop/), which is required by `pt_control`, but the [`joy`](https://github.com/ros-drivers/joystick_drivers) node is not started for you. Run `ros2 run joy joy_node` (on this or another machine on the network) before using a controller.

### Simulation (optional)

The MuJoCo simulation (`sim:=true`) needs a few more packages:

```bash
sudo apt install ros-kilted-mujoco-ros2-control ros-kilted-mujoco-ros2-control-plugins ros-kilted-image-transport-plugins
pip install -r src/pantilt_ros2/pt_mujoco/requirements.txt # MuJoCo and the model builder's Python dependencies
cd <your workspace>/src
git clone https://github.com/adityakamath/mujoco_ros2_plugins.git      # simulated /emergency_stop
cd ..
colcon build --packages-select mujoco_ros2_plugins
```

### Raspberry Pi 5

The following setup applies to the alternative OAK-D S2 camera.

The OAK-D S2 needs USB 3.0 and more current than the Pi 5's default 600 mA USB limit. Add this to `/boot/firmware/config.txt` and reboot:

```
usb_max_current_enable=1
```

> **⚠️ Note:** This raises the per-port limit to 1200 mA. On an inadequate power supply, or with several high-power USB devices, it can cause brownouts. Use a supply that meets the Raspberry Pi's recommended minimum and keep the OAK-D S2 as the only high-current USB device.

### As part of lekiwi_ros2

pantilt_ros2 is a git submodule under `payloads/pantilt_ros2/` in [lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2). After cloning it, run `git submodule update --init --recursive`.

## Running

```bash
ros2 launch pt_bringup pantilt.launch.py                       # control + Gemini 2 driver
ros2 launch pt_bringup pantilt.launch.py camera_config:=oakd_s2 # control + alternative OAK-D S2 driver
ros2 launch pt_control pantilt.launch.py                       # control only
ros2 launch pt_bringup gemini2.launch.py                       # Gemini 2 camera only (expects the mount TF)
ros2 launch pt_bringup oakd.launch.py                          # alternative OAK-D S2 camera only
ros2 launch pt_control pantilt.launch.py use_mock:=true        # control without hardware
ros2 launch pt_description urdf.launch.py                      # only the model and TF, for RViz
```

Send a position (pan, tilt in radians) directly, or use a joystick:

```bash
ros2 topic pub /pantilt_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5, -0.3]}"
```

| Joystick input | Action |
|----------------|--------|
| L1 (hold)      | Deadman: commands are only sent while it is held |
| Left stick X / Y | Pan / tilt to an absolute position (full deflection = ±π/2 rad) |
| A / B          | Enable / disable `/emergency_stop` |

The button mapping is in [`pt_control/config/teleop_config.yaml`](pt_control/config/teleop_config.yaml).

To keep the selected camera in the URDF while disabling its real driver and streaming pipeline:

```bash
ros2 launch pt_bringup pantilt.launch.py camera_config:=gemini2 enable_camera:=false
ros2 launch pt_bringup pantilt.launch.py camera_config:=oakd_s2 enable_camera:=false
```

`enable_camera` defaults to `true` and affects only real-camera bringup. When false, control, the selected camera meshes and URDF TF remain available; camera-driver TF and streaming helpers are not started. It does not disable the MuJoCo simulated camera. Driver launch files are included lazily: Gemini 2 loads only Orbbec, OAK-D S2 loads only DepthAI, and disabled streaming loads neither driver. This is runtime selection; package manifests still declare dependencies for the supported features.

### Launch arguments

| Argument           | Used by                    | Default         | Description |
|--------------------|----------------------------|-----------------|-------------|
| `sts_serial_port`  | `pt_control`, `pt_bringup` | from yaml       | Serial port; empty uses the value in `urdf_config.yaml` |
| `use_mock`         | `pt_control`, `pt_bringup` | from yaml       | Mock motors only; real bringup still starts the selected camera |
| `enable_camera` | `pt_bringup` | `true` | Enable the real camera driver and streaming pipeline; retain URDF geometry when false |
| `camera_config` | `pt_control`, `pt_bringup` | `gemini2` | `gemini2` or `oakd_s2`; selects geometry and real camera driver |
| `pantilt_config`   | `pt_control`, `pt_bringup` | `pt101`         | Mesh variant: `pt100` or `pt101` |
| `diagnostics`      | `pt_control`, `pt_bringup` | `false`         | Publish motor temperature, voltage and current |
| `pointcloud`       | `pt_bringup`               | `false`         | One colored cloud and Cloudini compression for either real camera; OAK-D S2 also aligns depth to RGB |
| `octomap`          | `pt_bringup`               | `false`         | OAK-D S2 only: build a persistent 3D octree (needs `pointcloud:=true`); rejected for Gemini 2 |
| `tf_parent_frame` | `pt_bringup` | empty | Full bringup selects `oak_link` for Gemini 2 or `tilt_link` for OAK-D S2 |
| `serial_number`, `usb_port` | `pt_bringup`, `gemini2.launch.py` | empty | Optional Orbbec device selectors |
| `camera_mount_xyz`, `camera_mount_rpy` | `pt_bringup`, `gemini2.launch.py` | `0 0 0`, `π 0 0` | Gemini mount-to-driver translation (meters) and rotation (radians); calibrate translation on hardware |
| `publish_mount_tf` | `pt_bringup`, `gemini2.launch.py` | `true` | Disable if another component supplies the Gemini mount transform |
| `sim`              | `pt_bringup`               | `false`         | Run in MuJoCo instead of on hardware (see [Simulation](#simulation)) |
| `mujoco_gui`       | `pt_bringup`               | `false`         | `sim` only: open the MuJoCo viewer (needs a display) |
| `mujoco_scene`     | `pt_control`, `pt_bringup` | `flat`          | `sim` only: `flat`, `none` or the path to a scene file |
| `use_sim_time`     | `pt_control`, `pt_bringup` | `false`         | Use `/clock` from a simulator |

## Configuration

### Serial port

The default port is `/dev/ttySERVO`, a udev symlink you create yourself. Find your device (commonly `/dev/ttyACM0` or `/dev/ttyUSB0`), then either pass it at launch with `sts_serial_port:=/dev/ttyACM0` or set it permanently in [`pt_control/config/urdf_config.yaml`](pt_control/config/urdf_config.yaml).

### Motor calibration

Each motor has a centre position, in raw steps (0–4095), that maps to 0 rad in the URDF. The defaults match the reference build; every physical assembly differs, so recalibrate yours. Motor IDs, centres and joint limits are constants of the mechanism, so they live in the macro defaults of [`pt_description/urdf/pantilt.joints.xacro`](pt_description/urdf/pantilt.joints.xacro). Edit them there:

```xml
<xacro:macro name="pantilt_joints" params="
    shoulder_pan_motor_id:=1
    tilt_motor_id:=2
    shoulder_pan_center_steps:=2048
    tilt_center_steps:=2048
    ...">
```

### Servo speed profile

`internal_max_vel`, `internal_max_acc` and `internal_acc_coeff` in [`urdf_config.yaml`](pt_control/config/urdf_config.yaml) are written to each servo at start-up and set how fast it accelerates and moves. The default, 65 / 50 / 0, is slow and smooth. Raise the values to make the mount faster and snappier. The same file holds the baud rate and other bus settings.

### Files you may want to edit

| File | What it sets |
|------|--------------|
| [`pt_control/config/urdf_config.yaml`](pt_control/config/urdf_config.yaml) | Serial port, baud rate, mock mode, servo speed profile |
| [`pt_control/config/teleop_config.yaml`](pt_control/config/teleop_config.yaml) | Joystick buttons and axes |
| [`pt_control/config/pantilt_controller.yaml`](pt_control/config/pantilt_controller.yaml) | The position controller (joints and interface); rarely changed |
| [`pt_description/urdf/pantilt.joints.xacro`](pt_description/urdf/pantilt.joints.xacro) | Motor IDs, centre steps and joint limits (calibration) |
| [`pt_bringup/config/oakd_vio.yaml`](pt_bringup/config/oakd_vio.yaml), `oakd_vio_pcl.yaml` | OAK-D S2 RGB/depth/IMU profile and optional aligned cloud |
| [`pt_bringup/config/gemini2_pcl.yaml`](pt_bringup/config/gemini2_pcl.yaml) | Gemini 2 Cloudini input, output and 1 mm resolution |
| [`pt_bringup/config/depthimage_to_laserscan.yaml`](pt_bringup/config/depthimage_to_laserscan.yaml) | Range and height of the `/oak/scan` slice |

## Gemini 2 camera

The default real bringup includes the upstream `orbbec_camera/gemini2.launch.py`. It enables RGB, registered depth and synchronized accelerometer/gyroscope output, requesting 640×360 RGB and 640×400 depth at 15 Hz by default (`camera_fps` accepts 5, 10, 15 or 30). `pointcloud:=true` enables one native colored cloud and Cloudini compression at 1 mm resolution. `serial_number` and `usb_port` select a particular device.

| Topic | Data |
|-------|------|
| `/gemini2/color/image_raw`, `/gemini2/color/camera_info` | RGB image and intrinsics |
| `/gemini2/depth/image_raw`, `/gemini2/depth/camera_info` | Registered depth and intrinsics |
| `/gemini2/gyro_accel/sample` | Synchronized IMU sample |
| `/gemini2/scan` | LaserScan generated directly from the depth image in both point-cloud modes |
| `/gemini2/depth_registered/points` | One colored cloud with `pointcloud:=true` |
| `/gemini2/depth_registered/points/compressed` | Cloudini-compressed colored cloud with `pointcloud:=true` |

The driver publishes its sensor TF tree below `gemini2_link`. A separate static transform attaches that root to the existing `oak_link` mount frame. Its default roll of π compensates for the inverted legacy frame; its zero translation is provisional, not a measured sensor origin. Set `camera_mount_xyz` and `camera_mount_rpy` after measuring the mount-to-sensor transform. The old `oak_imu_frame` is retained for compatibility and is not used for Gemini IMU data.

```bash
ros2 launch pt_bringup pantilt.launch.py pointcloud:=true
# Camera alone, with no pan-tilt TF provider:
ros2 launch pt_bringup gemini2.launch.py publish_mount_tf:=false
```

Gemini bringup does not supply VIO or octomap. `octomap:=true` is rejected for Gemini rather than silently ignored. Standalone pantilt MuJoCo still uses `/oak/*` compatibility topics and nominal Gemini optics; it does not model the device's measured calibration.

## OAK-D S2 camera modes

These modes require `camera_config:=oakd_s2` on the full bringup, or the standalone `oakd.launch.py`. Gemini 2 uses the separate Orbbec driver described below.

| Mode | What it publishes | Config |
|------|-------------------|--------|
| Default | RGB and depth at 640×400 and 15 Hz, 100 Hz IMU and `/oak/scan`; VIO disabled | `oakd_vio.yaml` |
| `pointcloud:=true` | Adds an RGB-aligned depth image, colored point cloud and Cloudini-compressed cloud, for 3D mapping | `oakd_vio_pcl.yaml` on top of the default |

`/oak/scan` (a laser scan sliced from the depth image) is published in both modes. In the default mode depth is left unaligned, which avoids a `depthai_ros_driver` 3.1.0 crash and is enough for the scan. Set `DEPTHAI_DEBUG=1` for verbose driver logs. With `octomap:=true` the point clouds are accumulated into a 3D map as the pan-tilt sweeps.

## Simulation

```bash
ros2 launch pt_bringup pantilt.launch.py sim:=true                    # headless
ros2 launch pt_bringup pantilt.launch.py sim:=true mujoco_gui:=true   # with the MuJoCo viewer
```

The same controllers and teleop run against a MuJoCo model instead of the hardware, so the commands above work unchanged. It needs no display; to watch it from another machine, run `foxglove_bridge` and connect [Foxglove](https://foxglove.dev/) to `ws://<host>:8765`. The standalone simulated camera publishes `/oak/rgb/image_raw`, `/oak/rgb/image_raw/compressed`, `/oak/stereo/image_raw`, `/oak/rgb/camera_info` and `/oak/scan` regardless of the selected camera geometry, and `/emergency_stop` works as on the robot. The model is generated from the URDF by [`pt_mujoco`](pt_mujoco/README.md), which also has a standalone (no ROS) viewer and the details of the simulated servo. The servo profile is identified from a real STS3215, but the simulation as a whole has not been compared against the hardware.

## Using it on another robot

The pan-tilt is designed to be mounted on another robot. Which way you embed it depends on whether it shares a serial bus with the host's motors.

**Dedicated bus.** Include the control block and the module in the host's URDF:

```xml
<xacro:include filename="$(find pt_description)/urdf/pantilt.control.xacro"/>
<xacro:include filename="$(find pt_description)/urdf/pantilt.module.xacro"/>

<xacro:pantilt_module parent="base_link">
  <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
</xacro:pantilt_module>
```

**Shared bus.** Call the joint macro inside the host's own `<ros2_control>` block, and include the module for the links:

```xml
<xacro:include filename="$(find pt_description)/urdf/pantilt.joints.xacro"/>
<xacro:include filename="$(find pt_description)/urdf/pantilt.module.xacro"/>

<ros2_control name="host_control" type="system">
  <hardware>...</hardware>
  <!-- host joints here -->
  <xacro:pantilt_joints sts3215_max_vel_steps="${sts3215_max_vel_steps}"/>
</ros2_control>

<xacro:pantilt_module parent="base_link">
  <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
</xacro:pantilt_module>
```

Motor IDs, centres and limits come from the macro defaults, so you only pass them if your unit is calibrated differently. The launch files in this repository start their own `controller_manager`, so a shared-bus host does not include them. It runs one `controller_manager` for everything and gives the spawner this package's controller definition, [`pt_control/config/pantilt_controller.yaml`](pt_control/config/pantilt_controller.yaml), with `--param-file`. [lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2) is the reference for this, including how it mounts the simulated pan-tilt in its own MuJoCo model. The camera-only launch files, `gemini2.launch.py` and `oakd.launch.py`, have no bus coupling and can be included directly.

## ROS interfaces

Real Gemini 2 topics use `/gemini2/*` (see [Gemini 2 camera](#gemini-2-camera)). The `/oak/*` topics below belong to the alternative OAK-D S2 driver; simulation retains its compatible image and scan topics.

| Topic                          | Type                                   | Description |
|--------------------------------|----------------------------------------|-------------|
| `/pantilt_controller/commands` | `std_msgs/Float64MultiArray`           | Subscribed: position command `[pan_rad, tilt_rad]` |
| `/joint_states`                | `sensor_msgs/JointState`               | Pan and tilt position, velocity, effort |
| `/dynamic_joint_states`        | `control_msgs/DynamicJointState`       | Voltage, temperature, current and moving flag per joint |
| `/base/diagnostics`            | `diagnostic_msgs/DiagnosticArray`      | Motor health (with `diagnostics:=true`) |
| `/joy`                         | `sensor_msgs/Joy`                      | Subscribed: joystick input from the external `joy` node |
| `/oak/rgb/image_raw`           | `sensor_msgs/Image`                    | RGB stream |
| `/oak/stereo/image_raw`        | `sensor_msgs/Image`                    | Depth stream |
| `/oak/scan`                    | `sensor_msgs/LaserScan`                | Laser scan sliced from the depth image |
| `/oak/imu/data`                | `sensor_msgs/Imu`                      | IMU data |
| `/oak/vio/transform`           | `geometry_msgs/TransformStamped`       | Available only if VIO is enabled in the OAK-D profile; disabled by default |
| `/oak/rgbd/points`, `/oak/rgbd/points/compressed` | `sensor_msgs/PointCloud2`, `point_cloud_interfaces/CompressedPointCloud2` | Point cloud and its 1 mm compressed version (`pointcloud:=true` only) |

| Service           | Type               | Description |
|-------------------|--------------------|-------------|
| `/emergency_stop` | `std_srvs/SetBool` | `true` stops the motors, `false` releases them. Served by the hardware interface (or the simulation plugin) |

TF tree:

```text
base_footprint                  ← standalone root only
└── pantilt_base_link           ← mount to the host robot when embedded
    └── shoulder_link           ← shoulder_pan_joint (±90°)
        └── tilt_link           ← tilt_joint (±90°)
            └── oak_link        ← compatibility mount frame; real Gemini sensor tree attaches here
                └── oak_imu_frame
```

## Troubleshooting

- **Joystick does nothing.** Start `ros2 run joy joy_node`, and hold **L1**; without the deadman button `joy_teleop` publishes nothing.
- **No hardware.** Use `use_mock:=true`. Topics, TF and controllers behave the same, with synthesised motor feedback.
- **A motor does not reach the commanded position.** It may be obstructed or the centre calibration may be wrong. High `effort` or `current` in `/dynamic_joint_states` means the motor is stalled.
- **Serial port not found.** Check the device name and pass it with `sts_serial_port` (see [Configuration](#serial-port)).

## License

Apache License 2.0 — see [LICENSE](LICENSE).

## Camera mesh variants

Camera selection is independent of the PT100/PT101 body selection:

```bash
ros2 launch pt_bringup pantilt.launch.py pantilt_config:=pt101 camera_config:=gemini2
```

`camera_config` accepts `gemini2` (the default) or `oakd_s2` (the supported alternative), and is also available
on `pt_control`'s `pantilt.launch.py` and the description visualization launch.
Both cameras can be combined with either `pantilt_config:=pt100` or `pt101`.
The MuJoCo builder defaults to Gemini 2 and accepts `--camera oakd_s2` for the alternative; the control launch forwards the
selection when generating its simulation model.

Gemini 2 uses its own `gemini2.stl` and `tilt_joint_gemini2.stl` meshes.
The Gemini tilt mount is centered along the motor shaft, leaving approximately
0.225 mm clearance at each mounting face. The camera follows the same lateral
shift so its mounting holes remain aligned with the bracket. The camera has a Gemini-specific mount offset, seating its rear face on
the bracket. The two camera mounting holes have 45 mm spacing. The Gemini body is rotated
180 degrees about its front-to-back mesh axis to mount upright.
Existing `oak_link`/IMU frame names and simulated camera settings remain for
compatibility; they are not calibrated Gemini optical/IMU properties. Real Gemini
bringup launches OrbbecSDK_ROS2 with native `/gemini2/*` topics and calibrated sensor TF. The mount-to-sensor translation remains provisional and must be measured for accurate registration. The parent LeKiwi bringup forwards `camera_config` to its URDF, simulation builder and selected real driver.

The MuJoCo source selects `pt_mujoco/mjcf/oakd_s2_subtree.xml` or
`pt_mujoco/mjcf/gemini2_subtree.xml` for the camera-specific tilt assembly. The
shared `pt_description/urdf/camera.module.xacro` supplies the URDF geometry.
Generated assets are `pt{100,101}_{oakd_s2,gemini2}.urdf` and the corresponding
`.xml` models. See [URDF regeneration](pt_description/README.md#regenerating-the-pre-built-urdfs)
and [MuJoCo regeneration](pt_mujoco/README.md#gemini-2-models).
