# Pan Tilt 100

![Project Status](https://img.shields.io/badge/Status-Active-brightgreen)
![ROS 2](https://img.shields.io/badge/ROS%202-Kilted%20%7C%20Jazzy-blue?style=flat&logo=ros&logoSize=auto)
[![CI](https://github.com/adityakamath/pantilt_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/adityakamath/pantilt_ros2/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

ROS 2 software stack for a 2-DOF pan-tilt camera mount built from [SO-100 or SO-101](https://github.com/TheRobotStudio/SO-ARM100) parts, two [Feetech STS3215](https://www.feetechrc.com/2020-05-13_56655.html) servos and an [Orbbec Gemini 2](https://www.seeedstudio.com/Orbbec-Gemini-2-3D-Camera-p-6464.html?sensecap_affiliate=8fjl172&referring_service=link) camera. The [Luxonis OAK-D S2](https://docs.luxonis.com/hardware/products/OAK-D%20S2) is supported as an alternative.

- Position control with joystick teleop
- Gemini 2 (default) or OAK-D S2 camera bringup
- MuJoCo simulation
- An embeddable xacro module for other robots, such as [lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2)

<p align="center">
  <img height="350" alt="image" src="https://github.com/user-attachments/assets/1275a39b-a3c3-4d39-832a-4220b96da997" />
  <img height="350" alt="image" src="https://github.com/user-attachments/assets/d7bfdb5b-2cc9-430c-923e-275f186895f4" />
</p>

## Packages

| Package | Purpose |
|---------|---------|
| [`pt_description`](pt_description/) | URDF/xacro model (standalone, embeddable macros, all PT100/PT101 × Gemini 2/OAK-D S2 combinations pre-generated), meshes, `urdf.launch.py` |
| [`pt_control`](pt_control/) | `ros2_control` setup, configuration and `pantilt.launch.py` |
| [`pt_bringup`](pt_bringup/) | Full-system launch with the camera driver (Gemini 2 by default, OAK-D S2 optional) |
| [`pt_mujoco`](pt_mujoco/) | MuJoCo model generated from the URDF, camera plugin config, standalone viewer, benchmark |

## Hardware

| Component | Details |
|-----------|---------|
| Motors | Two Feetech STS3215 on one serial bus at 1 Mbaud: pan is ID `1`, tilt is ID `2` |
| Servo driver | [Waveshare Bus Servo Adapter A](https://www.waveshare.com/bus-servo-adapter-a.htm) |
| Camera | Orbbec Gemini 2 (default), or Luxonis OAK-D S2 |
| Structure | 3D printed base and shoulder parts from [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) (**PT100**) or [SO-ARM101](https://github.com/TheRobotStudio/SO-ARM101) (**PT101**, recommended) |
| Camera mount | 3D printed bracket (STL in [`pt_description/meshes/`](pt_description/meshes/)) |

Pick the body variant with `pantilt_config:=pt100|pt101` and the camera with `camera_config:=gemini2|oakd_s2`. Any combination works.

## Installation

Requires [ROS 2](https://docs.ros.org/) Kilted or Jazzy. CI builds and tests the description, control and MuJoCo packages on both, and on Lyrical. The camera drivers below are documented for Kilted.

Clone the packages:

```bash
source /opt/ros/kilted/setup.bash
cd <your workspace>/src
git clone https://github.com/adityakamath/pantilt_ros2.git
git clone https://github.com/adityakamath/sts_hardware_interface.git
git clone https://github.com/facontidavide/cloudini.git
```

Teleop uses [`joy_teleop`](https://index.ros.org/p/joy_teleop/), but the [`joy`](https://github.com/ros-drivers/joystick_drivers) node is not started for you. Run `ros2 run joy joy_node` (on this or another machine) before using a controller.

### Gemini 2

The upstream [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2) driver has no `ros-kilted-orbbec-camera` apt package (checked 2026-09-27), so build it from source.

```bash
sudo apt-get update
sudo apt-get install libgflags-dev nlohmann-json3-dev libgoogle-glog-dev libdw-dev libssl-dev \
  ros-kilted-backward-ros ros-kilted-image-transport ros-kilted-image-transport-plugins \
  ros-kilted-image-publisher ros-kilted-camera-info-manager ros-kilted-diagnostic-updater \
  ros-kilted-statistics-msgs ros-kilted-xacro

cd <your workspace>/src
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

The [QoS patch](pt_bringup/patches/orbbec-kilted-qos.patch) moves the image-sync example to the current `rclcpp::QoS` API. `-Wno-error=cpp` allows Kilted's deprecated-header warnings.

Install the [upstream USB permission rules](https://github.com/orbbec/OrbbecSDK_ROS2#registration-script-required), then reconnect the camera:

```bash
sudo cp src/OrbbecSDK_ROS2/orbbec_camera/scripts/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
ros2 run orbbec_camera list_devices_node
```

### OAK-D S2

The OAK-D S2 uses the packaged DepthAI driver, so there is no source build, QoS patch or USB rules.

```bash
sudo apt-get install ros-kilted-depthai-ros
cd <your workspace>
colcon build --packages-select cloudini_lib cloudini_ros pt_description pt_mujoco pt_control pt_bringup sts_hardware_interface
source install/setup.bash
```

Select it at launch with `camera_config:=oakd_s2`.

On a Raspberry Pi 5, the OAK-D S2 needs more than the default 600 mA USB limit. Add this to `/boot/firmware/config.txt` and reboot:

```
usb_max_current_enable=1
```

> **⚠️ Note:** This raises the per-port limit to 1200 mA. On an inadequate power supply, or with several high-power USB devices, it can cause brownouts. Use a supply that meets the Raspberry Pi's recommended minimum and keep the OAK-D S2 as the only high-current USB device.

### Simulation (optional)

`sim:=true` needs a few more packages:

```bash
sudo apt install ros-kilted-mujoco-ros2-control ros-kilted-mujoco-ros2-control-plugins ros-kilted-image-transport-plugins
pip install -r src/pantilt_ros2/pt_mujoco/requirements.txt
cd <your workspace>/src
git clone https://github.com/adityakamath/mujoco_ros2_plugins.git      # simulated /emergency_stop
cd ..
colcon build --packages-select mujoco_ros2_plugins
```

### As part of lekiwi_ros2

`pantilt_ros2` is a git submodule under `payloads/pantilt_ros2/` in [lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2). After cloning, run `git submodule update --init --recursive`.

## Usage

```bash
ros2 launch pt_bringup pantilt.launch.py                        # control + Gemini 2 driver
ros2 launch pt_bringup pantilt.launch.py camera_config:=oakd_s2 # control + OAK-D S2 driver
ros2 launch pt_control pantilt.launch.py                        # control only
ros2 launch pt_control pantilt.launch.py use_mock:=true         # control without hardware
ros2 launch pt_bringup gemini2.launch.py                        # Gemini 2 driver only (host Xacro supplies its mount)
ros2 launch pt_bringup oakd.launch.py                           # OAK-D S2 driver only
ros2 launch pt_description urdf.launch.py                       # model and TF only, for RViz
```

Send a position (pan, tilt in radians) directly, or use a joystick:

```bash
ros2 topic pub /pantilt_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5, -0.3]}"
```

| Joystick input | Action |
|----------------|--------|
| L1 (hold) | Deadman: commands are sent only while held |
| Left stick X / Y | Pan / tilt to an absolute position (full deflection = ±π/2 rad) |
| A / B | Enable / disable `/emergency_stop` |

The mapping is in [`pt_control/config/teleop_config.yaml`](pt_control/config/teleop_config.yaml).

### Launch arguments

| Argument | Used by | Default | Description |
|----------|---------|---------|-------------|
| `sts_serial_port` | `pt_control`, `pt_bringup` | from yaml | Serial port; empty uses `urdf_config.yaml` |
| `use_mock` | `pt_control`, `pt_bringup` | from yaml | Mock motors only; real bringup still starts the camera |
| `camera_config` | `pt_control`, `pt_bringup` | `gemini2` | `gemini2` or `oakd_s2`: geometry and real camera driver |
| `pantilt_config` | `pt_control`, `pt_bringup` | `pt101` | Mesh variant: `pt100` or `pt101` |
| `enable_camera` | `pt_bringup` | `true` | `false` keeps the camera in the URDF but starts no real driver or streaming (the MuJoCo camera is unaffected) |
| `diagnostics` | `pt_control`, `pt_bringup` | `false` | Publish motor temperature, voltage and current (real hardware) |
| `pointcloud` | `pt_bringup` | `false` | One colored cloud plus Cloudini compression; OAK-D S2 also aligns depth to RGB |
| `octomap` | `pt_bringup` | `false` | OAK-D S2 only: persistent 3D octree (needs `pointcloud:=true`); rejected for Gemini 2 |
| `serial_number`, `usb_port` | `pt_bringup`, `gemini2.launch.py` | empty | Optional Orbbec device selectors |
| `sim` | `pt_bringup` | `false` | Run in MuJoCo instead of on hardware |
| `mujoco_gui` | `pt_bringup` | `false` | `sim` only: open the viewer (needs a display) |
| `mujoco_scene` | `pt_control`, `pt_bringup` | `flat` | `sim` only: `flat`, `none` or a scene file path |
| `use_sim_time` | `pt_control`, `pt_bringup` | `false` | Use `/clock` from a simulator |

## Configuration

| File | Sets |
|------|------|
| [`pt_control/config/urdf_config.yaml`](pt_control/config/urdf_config.yaml) | Serial port, baud rate, mock mode, servo speed profile |
| [`pt_control/config/teleop_config.yaml`](pt_control/config/teleop_config.yaml) | Joystick buttons and axes |
| [`pt_control/config/pantilt_controller.yaml`](pt_control/config/pantilt_controller.yaml) | The position controller (rarely changed) |
| [`pt_description/urdf/pantilt.joints.xacro`](pt_description/urdf/pantilt.joints.xacro) | Motor IDs, centre steps, joint limits |
| [`pt_bringup/config/oakd_vio.yaml`](pt_bringup/config/oakd_vio.yaml), `oakd_vio_pcl.yaml` | OAK-D S2 RGB/depth/IMU profile and optional aligned cloud |
| [`pt_bringup/config/gemini2_pcl.yaml`](pt_bringup/config/gemini2_pcl.yaml) | Gemini 2 Cloudini input, output and 1 mm resolution |
| [`pt_bringup/config/depthimage_to_laserscan.yaml`](pt_bringup/config/depthimage_to_laserscan.yaml) | Range and height of the `/oak/scan` slice |

**Serial port.** The default is `/dev/ttySERVO`, a udev symlink you create yourself. Find your device (commonly `/dev/ttyACM0` or `/dev/ttyUSB0`), then pass `sts_serial_port:=/dev/ttyACM0` or set it in `urdf_config.yaml`.

**Motor calibration.** Each motor has a centre position, in raw steps (0-4095), that maps to 0 rad in the URDF. The defaults match the reference build, so recalibrate yours. Motor IDs, centres and limits are macro defaults in `pantilt.joints.xacro`:

```xml
<xacro:macro name="pantilt_joints" params="
    shoulder_pan_motor_id:=1
    tilt_motor_id:=2
    shoulder_pan_center_steps:=2048
    tilt_center_steps:=2048
    ...">
```

**Servo speed profile.** `internal_max_vel`, `internal_max_acc` and `internal_acc_coeff` in `urdf_config.yaml` are written to each servo at start-up. The default, 65 / 50 / 0, is slow and smooth; raise the values for a faster, snappier mount.

## Cameras

### Gemini 2

Real bringup includes the upstream `orbbec_camera/gemini2.launch.py`. It enables RGB, registered depth and synchronized accelerometer/gyroscope output, at 640×360 RGB and 640×400 depth at 15 Hz by default (`camera_fps` accepts 5, 10, 15 or 30).

| Topic | Data |
|-------|------|
| `/gemini2/color/image_raw`, `/gemini2/color/camera_info` | RGB image and intrinsics |
| `/gemini2/depth/image_raw`, `/gemini2/depth/camera_info` | Registered depth and intrinsics |
| `/gemini2/gyro_accel/sample` | Synchronized IMU sample |
| `/gemini2/scan` | LaserScan generated from the depth image |
| `/gemini2/depth_registered/points`, `.../points/compressed` | Colored cloud and its Cloudini-compressed version (`pointcloud:=true`, 1 mm resolution) |

The driver publishes its sensor TF tree below `gemini2_link`. The pan-tilt Xacro mounts the camera to `tilt_link`, and the driver launch does not publish a competing mount transform. A standalone `gemini2.launch.py` therefore needs a host Xacro to provide `gemini2_link`.

Gemini bringup has no VIO or octomap; `octomap:=true` is rejected rather than ignored.

### OAK-D S2

Requires `camera_config:=oakd_s2` (or the standalone `oakd.launch.py`).

| Mode | Publishes | Config |
|------|-----------|--------|
| Default | RGB and depth at 640×400 and 15 Hz, 100 Hz IMU, `/oak/scan`; VIO disabled | `oakd_vio.yaml` |
| `pointcloud:=true` | Adds an RGB-aligned depth image, colored cloud and Cloudini-compressed cloud | `oakd_vio_pcl.yaml` on top of the default |

`/oak/scan` is published in both modes. In the default mode depth is left unaligned, which avoids a `depthai_ros_driver` 3.1.0 crash and is enough for the scan. Set `DEPTHAI_DEBUG=1` for verbose driver logs. With `octomap:=true`, the clouds are accumulated into a 3D map as the mount sweeps.

### Mesh variants

Camera and body selection are independent. For example:

```bash
ros2 launch pt_bringup pantilt.launch.py pantilt_config:=pt101 camera_config:=gemini2
```

Gemini 2 uses its own `gemini2.stl` and `tilt_joint_gemini2.stl` meshes. The camera mounts with 45 mm hole spacing, rotated 180° about its front-to-back axis to sit upright. The Xacro carries this pose, and the real camera launch publishes no mount transform. The Gemini frames, IMU and simulated camera settings are nominal, not calibrated.

The shared `pt_description/urdf/camera.module.xacro` supplies the URDF geometry. MuJoCo selects `pt_mujoco/mjcf/oakd_s2_subtree.xml` or `gemini2_subtree.xml` for the tilt assembly, and its builder takes `--camera oakd_s2` (default Gemini 2). Generated assets are `pt{100,101}_{oakd_s2,gemini2}.urdf` and matching `.xml` models; see [URDF regeneration](pt_description/README.md#regenerating-the-pre-built-urdfs) and [MuJoCo regeneration](pt_mujoco/README.md#gemini-2-models).

## Simulation

```bash
ros2 launch pt_bringup pantilt.launch.py sim:=true                    # headless
ros2 launch pt_bringup pantilt.launch.py sim:=true mujoco_gui:=true   # with the viewer
```

The same controllers and teleop run against a MuJoCo model, so the commands above work unchanged, and `/emergency_stop` works as on the robot. No display is needed; to watch from another machine, run `foxglove_bridge` and connect [Foxglove](https://foxglove.dev/) to `ws://<host>:8765`.

The simulated Gemini 2 publishes `/gemini2/color/image_raw`, `.../image_raw/compressed` and `.../camera_info`. The simulated OAK-D S2 publishes `/oak/rgb/image_raw`, `.../image_raw/compressed`, `/oak/stereo/image_raw`, `/oak/rgb/camera_info` and `/oak/scan`.

The model is generated from the URDF by [`pt_mujoco`](pt_mujoco/README.md), which also has a standalone (no ROS) viewer. The servo profile is identified from a real STS3215, but the simulation as a whole has not been compared against the hardware.

## Using it on another robot

Which way you embed the pan-tilt depends on whether it shares a serial bus with the host's motors.

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

Motor IDs, centres and limits come from the macro defaults; pass them only if your unit is calibrated differently.

The launch files here start their own `controller_manager`, so a shared-bus host does not include them. It runs one `controller_manager` for everything and gives the spawner [`pt_control/config/pantilt_controller.yaml`](pt_control/config/pantilt_controller.yaml) with `--param-file`. [lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2) is the reference, including how it mounts the simulated pan-tilt in its own MuJoCo model. The camera-only launch files (`gemini2.launch.py`, `oakd.launch.py`) have no bus coupling and can be included directly.

## ROS interfaces

| Topic | Type | Description |
|-------|------|-------------|
| `/pantilt_controller/commands` | `std_msgs/Float64MultiArray` | Subscribed: position command `[pan_rad, tilt_rad]` |
| `/joint_states` | `sensor_msgs/JointState` | Pan and tilt position, velocity, effort |
| `/state_interfaces_broadcaster/values` | `control_msgs/Float64Values` | Effort, voltage, temperature, current and moving flag per joint; keys on `/state_interfaces_broadcaster/names` (real hardware with `diagnostics:=true`) |
| `/base/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Motor health (`diagnostics:=true`) |
| `/joy` | `sensor_msgs/Joy` | Subscribed: joystick input from the external `joy` node |

Camera topics are listed under [Cameras](#cameras), and the OAK-D S2 driver also publishes `/oak/rgb/image_raw`, `/oak/stereo/image_raw`, `/oak/scan`, `/oak/imu/data`, `/oak/rgbd/points` (with `/compressed`, `pointcloud:=true` only) and `/oak/vio/transform` (only if VIO is enabled; off by default).

| Service | Type | Description |
|---------|------|-------------|
| `/emergency_stop` | `std_srvs/SetBool` | `true` stops the motors, `false` releases them. Served by the hardware interface or the simulation plugin |

TF tree:

```text
base_footprint                  ← standalone root only
└── pantilt_base_link           ← mount to the host robot when embedded
    └── shoulder_link           ← shoulder_pan_joint (±90°)
        └── tilt_link           ← tilt_joint (±90°)
            └── gemini2_link    ← Gemini 2 mount frame; the real Gemini sensor tree attaches here
                └── oak_imu_frame
```

## Troubleshooting

- **Joystick does nothing.** Start `ros2 run joy joy_node` and hold **L1**; without the deadman button `joy_teleop` publishes nothing.
- **No hardware.** Use `use_mock:=true`. Topics, TF and controllers behave the same, with synthesised motor feedback.
- **A motor does not reach the commanded position.** It may be obstructed or its centre calibration may be wrong. High `effort` or `current` in `/state_interfaces_broadcaster/values` means it is stalled.
- **Serial port not found.** Check the device name and pass it with `sts_serial_port`.

## License

Apache License 2.0. See [LICENSE](LICENSE).
