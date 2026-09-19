# Pan Tilt 100

![Project Status](https://img.shields.io/badge/Status-Active-brightgreen)
![ROS 2](https://img.shields.io/badge/ROS%202-Kilted%20(Ubuntu%2024.04)-blue?style=flat&logo=ros&logoSize=auto)
[![CI](https://github.com/adityakamath/pantilt_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/adityakamath/pantilt_ros2/actions/workflows/ci.yml)
[![Ask DeepWiki (Experimental)](https://deepwiki.com/badge.svg)](https://deepwiki.com/adityakamath/pantilt_ros2)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

ROS 2 software stack for a 2-DOF pan-tilt camera mount built from [SO-100 or SO-101](https://github.com/TheRobotStudio/SO-ARM100) parts, two [Feetech STS3215](https://www.feetechrc.com/2020-05-13_56655.html) servo motors and an [OAK-D S2](https://docs.luxonis.com/hardware/products/OAK-D%20S2) camera. It provides position control with joystick teleop, visual-inertial odometry (VIO) bringup, a MuJoCo simulation, and an embeddable xacro module for other robots such as [lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2).

<p align="center">
  <img width="500" height="575" alt="Screenshot 2026-04-28 at 15 56 11" src="https://github.com/user-attachments/assets/c9520454-7523-44a7-bcb3-8b6428437759" />
</p>

## Hardware

| Component    | Details                                                                                             |
|--------------|-----------------------------------------------------------------------------------------------------|
| Pan motor    | [Feetech STS3215](https://www.feetechrc.com/2020-05-13_56655.html), motor ID `1`                    |
| Tilt motor   | Feetech STS3215, motor ID `2`                                                                       |
| Servo driver | [Waveshare Bus Servo Adapter A](https://www.waveshare.com/bus-servo-adapter-a.htm)                  |
| Camera       | [OAK-D S2](https://docs.luxonis.com/hardware/products/OAK-D%20S2)                                   |
| Structure    | 3D printed base and shoulder parts from [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) or [SO-ARM101](https://github.com/TheRobotStudio/SO-ARM101) |
| Camera mount | 3D printed OAK-D S2 bracket (STL in [`pt_description/meshes/`](pt_description/meshes/))             |

Both motors share one serial bus at 1 Mbaud, connected through the Waveshare driver. **PT100** uses the SO-ARM100 base and shoulder parts and **PT101** uses the SO-ARM101 ones; you pick the variant with the `pantilt_config` launch argument (`pt101`, the default, is recommended).

## Installation

Requires [ROS 2 Kilted](https://docs.ros.org/en/kilted/) (other distributions are untested) and the Kilted packages for [depthai-ros](https://github.com/luxonis/depthai-ros):

```bash
sudo apt install ros-kilted-depthai-ros
```

Clone this repository with its dependencies into a workspace and build:

```bash
cd <your workspace>/src
git clone https://github.com/adityakamath/pantilt_ros2.git
git clone https://github.com/adityakamath/sts_hardware_interface.git   # servo driver (ros2_control)
git clone https://github.com/facontidavide/cloudini.git                # point cloud compression
cd ..
colcon build --packages-select cloudini_lib cloudini_ros pt_description pt_mujoco pt_control pt_bringup sts_hardware_interface
source install/setup.bash
```

The teleop uses [`joy_teleop`](https://index.ros.org/p/joy_teleop/), which is installed with the packages above, but the [`joy`](https://github.com/ros-drivers/joystick_drivers) node is not started for you. Run `ros2 run joy joy_node` (on this or another machine on the network) before using a controller.

### Simulation (optional)

The MuJoCo simulation (`sim:=true`) needs a few more packages:

```bash
sudo apt install ros-kilted-mujoco-ros2-control ros-kilted-mujoco-ros2-control-plugins ros-kilted-image-transport-plugins
pip install -r pantilt_ros2/pt_mujoco/requirements.txt     # MuJoCo and the model builder's Python dependencies
cd <your workspace>/src
git clone https://github.com/adityakamath/mujoco_ros2_plugins.git      # simulated /emergency_stop
cd ..
colcon build --packages-select mujoco_ros2_plugins
```

### Raspberry Pi 5

The OAK-D S2 needs USB 3.0 and more current than the Pi 5's default 600 mA USB limit. Add this to `/boot/firmware/config.txt` and reboot:

```
usb_max_current_enable=1
```

> **⚠️ Note:** This raises the per-port limit to 1200 mA. On an inadequate power supply, or with several high-power USB devices, it can cause brownouts. Use a supply that meets the Raspberry Pi's recommended minimum and keep the OAK-D S2 as the only high-current USB device.

### As part of lekiwi_ros2

pantilt_ros2 is a git submodule under `payloads/pantilt_ros2/` in [lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2). After cloning it, run `git submodule update --init --recursive`.

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

### Launch arguments

| Argument           | Used by                    | Default         | Description |
|--------------------|----------------------------|-----------------|-------------|
| `sts_serial_port`  | `pt_control`, `pt_bringup` | from yaml       | Serial port; empty uses the value in `urdf_config.yaml` |
| `use_mock`         | `pt_control`, `pt_bringup` | from yaml       | Run with simulated motors and no hardware |
| `pantilt_config`   | `pt_control`, `pt_bringup` | `pt101`         | Mesh variant: `pt100` or `pt101` |
| `diagnostics`      | `pt_control`, `pt_bringup` | `true`          | Publish motor temperature, voltage and current |
| `pointcloud`       | `pt_bringup`               | `false`         | Aligned depth plus a compressed point cloud (higher CPU load) |
| `octomap`          | `oakd.launch.py`           | `false`         | Build a persistent 3D octree from the point cloud (needs `pointcloud:=true`) |
| `tf_parent_frame`  | `oakd.launch.py`           | `tilt_link`     | TF frame the camera is mounted to; change it to use the camera on a robot without the pan-tilt |
| `sim`              | `pt_bringup`               | `false`         | Run in MuJoCo instead of on hardware (see [Simulation](#simulation)) |
| `mujoco_gui`       | `pt_bringup`               | `false`         | `sim` only: open the MuJoCo viewer (needs a display) |
| `mujoco_scene`     | `pt_control`, `pt_bringup` | `flat`          | `sim` only: `flat`, `none` or the path to a scene file |
| `use_sim_time`     | `pt_control`, `pt_bringup` | `false`         | Use `/clock` from a simulator |

## Usage

```bash
ros2 launch pt_bringup pantilt.launch.py                       # full system: control + camera
ros2 launch pt_control pantilt.launch.py                       # control only
ros2 launch pt_bringup oakd.launch.py                          # camera only
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

### Camera modes

| Mode | What it publishes | Config |
|------|-------------------|--------|
| Default | RGB and depth at 640x400, 30 Hz, IMU, VIO at 60 Hz | `oakd_vio.yaml` |
| `pointcloud:=true` | Adds an RGB-aligned depth image and a compressed point cloud, for 3D mapping | `oakd_vio_pcl.yaml` on top of the default |

`/oak/scan` (a laser scan sliced from the depth image) is published in both modes. In the default mode depth is left unaligned, which avoids a `depthai_ros_driver` 3.1.0 crash and is enough for the scan. Set `DEPTHAI_DEBUG=1` for verbose driver logs. With `octomap:=true` the point clouds are accumulated into a 3D map as the pan-tilt sweeps.

## Simulation

```bash
ros2 launch pt_bringup pantilt.launch.py sim:=true                    # headless
ros2 launch pt_bringup pantilt.launch.py sim:=true mujoco_gui:=true   # with the MuJoCo viewer
```

The same controllers and teleop run against a MuJoCo model instead of the hardware, so the commands above work unchanged. It needs no display; to watch it from another machine, run `foxglove_bridge` and connect [Foxglove](https://foxglove.dev/) to `ws://<host>:8765`. The simulated camera publishes `/oak/rgb/image_raw`, `/oak/stereo/image_raw`, `/oak/rgb/camera_info` and `/oak/scan`, and `/emergency_stop` works as on the robot. The model is generated from the URDF by [`pt_mujoco`](pt_mujoco/README.md), which also has a standalone (no ROS) viewer and the details of the simulated servo. The servo profile is identified from a real STS3215, but the simulation as a whole has not been compared against the hardware.

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

Motor IDs, centres and limits come from the macro defaults, so you only pass them if your unit is calibrated differently. The launch files in this repository start their own `controller_manager`, so a shared-bus host does not include them. It runs one `controller_manager` for everything and gives the spawner this package's controller definition, [`pt_control/config/pantilt_controller.yaml`](pt_control/config/pantilt_controller.yaml), with `--param-file`. [lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2) is the reference for this, including how it mounts the simulated pan-tilt in its own MuJoCo model. The camera launch file, `oakd.launch.py`, has no bus coupling and can be included directly.

## ROS interfaces

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
| `/oak/vio/transform`           | `geometry_msgs/TransformStamped`       | Visual-inertial odometry |
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
            └── oak_link        ← OAK-D S2 optical centre
                └── oak_imu_frame
```

## Repository layout

| Package | Contents |
|---------|----------|
| [`pt_description`](pt_description/) | URDF/xacro model (standalone entry, embeddable macros, `pt100.urdf` and `pt101.urdf` pre-generated), meshes, `urdf.launch.py` |
| [`pt_control`](pt_control/) | `ros2_control` setup and configuration (`urdf_config.yaml`, `pantilt_controller.yaml`, `teleop_config.yaml`) and `pantilt.launch.py` |
| [`pt_bringup`](pt_bringup/) | Full-system launch, OAK-D S2 driver launch and configuration |
| [`pt_mujoco`](pt_mujoco/README.md) | MuJoCo model generated from the URDF, camera plugin config, standalone viewer, benchmark |

## Troubleshooting

- **Joystick does nothing.** Start `ros2 run joy joy_node`, and hold **L1**; without the deadman button `joy_teleop` publishes nothing.
- **No hardware.** Use `use_mock:=true`. Topics, TF and controllers behave the same, with synthesised motor feedback.
- **A motor does not reach the commanded position.** It may be obstructed or the centre calibration may be wrong. High `effort` or `current` in `/dynamic_joint_states` means the motor is stalled.
- **Serial port not found.** Check the device name and pass it with `sts_serial_port` (see [Configuration](#serial-port)).

## License

Apache License 2.0 — see [LICENSE](LICENSE).
