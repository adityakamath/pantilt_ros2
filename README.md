# Pan Tilt 100

![Project Status](https://img.shields.io/badge/Status-Active-brightgreen)
![ROS 2](https://img.shields.io/badge/ROS%202-Kilted%20(Ubuntu%2024.04)-blue?style=flat&logo=ros&logoSize=auto)
[![CI](https://github.com/adityakamath/pantilt_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/adityakamath/pantilt_ros2/actions/workflows/ci.yml)
[![Ask DeepWiki (Experimental)](https://deepwiki.com/badge.svg)](https://deepwiki.com/adityakamath/pantilt_ros2)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

ROS 2 software stack for a 2-DOF pan-tilt camera mount using [SO-100 or SO-101](https://github.com/TheRobotStudio/SO-ARM100) parts, [Feetech STS3215](https://www.feetechrc.com/2020-05-13_56655.html) servo motors and an [OAK-D S2](https://docs.luxonis.com/hardware/products/OAK-D%20S2) camera. Provides position control with joystick teleop, visual-inertial odometry (VIO) bringup, and an embeddable xacro module for integration into other robots like the [lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2) project.

<p align="center">
  <img width="500" height="575" alt="Screenshot 2026-04-28 at 15 56 11" src="https://github.com/user-attachments/assets/c9520454-7523-44a7-bcb3-8b6428437759" />
</p>

## Hardware Requirements

| Component      | Details                                                                                          |
|----------------|--------------------------------------------------------------------------------------------------|
| Pan motor      | [Feetech STS3215](https://www.feetechrc.com/2020-05-13_56655.html), Motor ID `1`                 |
| Tilt motor     | Feetech STS3215, Motor ID `2`                                                                    |
| Servo driver   | [Waveshare Bus Servo Adapter A](https://www.waveshare.com/bus-servo-adapter-a.htm)               |
| Camera         | [OAK-D S2](https://docs.luxonis.com/hardware/products/OAK-D%20S2)                                |
| Structural     | 3D printed Base and shoulder parts from [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) or [SO-ARM101](https://github.com/TheRobotStudio/SO-ARM101) |
| Camera mount   | 3D printed OAK-D S2 bracket (STL in [`pt_description/meshes/`](pt_description/meshes/))    |

Both motors are chained together on a single serial bus at 1 Mbaud, connected to the host via the Waveshare servo driver. The URDF is simply the SO-ARM URDF, but only till the second joint. This is also the project's naming convention: **PT100** uses Base and shoulder parts from **SO-ARM100**, and **PT101** uses the equivalent parts from **SO-ARM101** — selected via the `pantilt_config` arg (see [Mesh variants](#mesh-variants-pantilt_config)).

| Parameter   | Launch argument   | Default value   |
|-------------|-------------------|-----------------|
| Serial port | `sts_serial_port` | `/dev/ttySERVO` |

`/dev/ttySERVO` is a user-created udev symlink. Identify your actual device (commonly `/dev/ttyACM0` or `/dev/ttyUSB0`) and either pass it with the `sts_serial_port` argument at launch, or set it permanently in [`pt_control/config/urdf_config.yaml`](pt_control/config/urdf_config.yaml).

### Motor Calibration

Each motor has a **center position** (in raw steps, 0–4095) that maps to 0 rad in the URDF. The defaults below match my reference hardware build; recalibration is needed since each physical assembly will differ.

| Joint | Parameter                     | Default |
|-------|--------------------------------|---------|
| Pan   | `shoulder_pan_center_steps`   | `2048`  |
| Tilt  | `tilt_center_steps`           | `2048`  |

Motor IDs, step-centering, and joint limits are physical-calibration constants for this mechanism, not per-deployment settings — they're baked directly into [`pt_description/urdf/pantilt.joints.xacro`](pt_description/urdf/pantilt.joints.xacro)'s macro defaults (single source of truth, the same convention `so_arm_ros2` uses for its own motor IDs/joint limits). Recalibrate by editing that file's macro parameter defaults directly:

```xml
<xacro:macro name="pantilt_joints" params="
    shoulder_pan_motor_id:=1
    tilt_motor_id:=2
    shoulder_pan_center_steps:=2048
    tilt_center_steps:=2048
    ...">
```

A host embedding this mechanism on a shared bus (e.g. `lekiwi_ros2`) can still override any of these explicitly on its own `<xacro:pantilt_joints .../>` call if that specific unit is ever calibrated differently from these defaults.

## Dependencies

- **[ROS 2 Kilted](https://docs.ros.org/en/kilted/)** — other distributions untested
- **[ros2_control](https://control.ros.org/)** — controller manager, joint state broadcaster, forward command controller
- **[sts_hardware_interface](https://github.com/adityakamath/sts_hardware_interface)** — hardware interface for Feetech STS servo motors
- **[depthai-ros](https://github.com/luxonis/depthai-ros)** — DepthAI ROS 2 driver for the OAK-D S2 Camera
- **[cloudini](https://github.com/facontidavide/cloudini)** — high-performance point cloud compression library; required by `pt_bringup` for the PCL compressor node (point cloud mode only)
- **[joy_teleop](https://index.ros.org/p/joy_teleop/)** — joystick-to-topic bridge (included in this package's launch)
- **[mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control)** / **mujoco_ros2_control_plugins** — `sim:=true` only (see [Simulation](#simulation)); `sudo apt install ros-kilted-mujoco-ros2-control ros-kilted-mujoco-ros2-control-plugins ros-kilted-image-transport-plugins` (0.1.2 or newer)
- **[mujoco](https://pypi.org/project/mujoco/)** (pip, 3.13.0) — used by [`pt_mujoco`](pt_mujoco/README.md) to generate the simulation model and for its standalone viewer; `pip install -r pt_mujoco/requirements.txt`

> **⚠️ Joystick:** `joy_teleop` is included but the [`joy`](https://github.com/ros-drivers/joystick_drivers) node is **not** — it must be started separately (on the same or a networked device) before the system will respond to controller input:
> ```bash
> ros2 run joy joy_node
> ```

## Raspberry Pi System Setup

The OAK-D S2 requires USB 3.0 (5 Gbps) for its combined stereo depth, RGB, and IMU streams, and draws more current than the Raspberry Pi 5's default 600 mA USB cap allows. Add the following to `/boot/firmware/config.txt` and reboot to raise the cap:

```
usb_max_current_enable=1
```

> **⚠️ Note:** This raises the per-port USB current limit from 600 mA to 1200 mA. On an inadequate power supply, or with multiple high-power USB devices connected, this can cause brownouts. For best performance, use an adequate power supply that satisfies the recommended minimum for the Raspberry Pi and ensure the OAK-D S2 is the only high-current USB device on the bus.

## Installation

### Standalone

Install [depthai-ros](https://docs.luxonis.com/software/ros/depthai-ros/) via apt:

```bash
sudo apt install ros-kilted-depthai-ros
```

Optional, only needed for `sim:=true` (see [Simulation](#simulation)):

```bash
sudo apt install ros-kilted-mujoco-ros2-control ros-kilted-mujoco-ros2-control-plugins ros-kilted-image-transport-plugins
pip install -r pantilt_ros2/pt_mujoco/requirements.txt   # after the clone below
```

Then clone and build this package and its dependencies:

```bash
cd <your workspace>/src
git clone https://github.com/adityakamath/pantilt_ros2.git
git clone https://github.com/adityakamath/sts_hardware_interface.git
git clone https://github.com/facontidavide/cloudini.git

cd ..
colcon build --packages-select cloudini_lib cloudini_ros pt_description pt_mujoco pt_control pt_bringup sts_hardware_interface

source install/setup.bash
```

### As part of lekiwi_ros2

pantilt_ros2 is included as a git submodule under `payloads/pantilt_ros2/` in [lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2). After cloning lekiwi_ros2, initialise the submodule:

```bash
git submodule update --init --recursive
```

## Usage
The package provides multiple launch files for different use cases:

### Visualization only (no hardware, just URDF/TF)

```bash
ros2 launch pt_description urdf.launch.py
```

This launches only the robot_state_publisher node with the Pan Tilt 100 URDF, for visualization or model inspection in RViz or other tools. No hardware, controllers, or teleop nodes are started. Useful for:
- Viewing the robot model and TF tree
- Debugging or editing the URDF
- Integrating the model into other systems or simulation environments

### Control stack only

```bash
ros2 launch pt_control pantilt.launch.py
```

### Camera driver only

```bash
ros2 launch pt_bringup oakd.launch.py
```

### Full PT100 bringup (control + camera)

```bash
ros2 launch pt_bringup pantilt.launch.py
```

### Mock control stack (no hardware required)

```bash
ros2 launch pt_control pantilt.launch.py use_mock:=true
```

### MuJoCo simulation

```bash
ros2 launch pt_bringup pantilt.launch.py sim:=true
```

Runs the pan-tilt in MuJoCo instead of real hardware — same controllers, same teleop, only the hardware layer differs. It runs headless (no display needed); add `mujoco_gui:=true` for the MuJoCo viewer, or connect Foxglove through a `foxglove_bridge`. `oakd` (real camera driver) is skipped; the simulated OAK-D RGB and depth images are published by `pt_mujoco`'s camera plugin instead, see [Simulation](#simulation) below.

### Launch arguments

| Argument          | Package                          | Default | Description                                        |
|-------------------|----------------------------------|---------|----------------------------------------------------|
| `sts_serial_port` | `pt_control`, `pt_bringup` | `""`    | Serial port override; empty means use `urdf_config.yaml` value |
| `use_mock`        | `pt_control`, `pt_bringup` | `""`    | Mock mode override; empty means use `urdf_config.yaml` value   |
| `diagnostics`     | `pt_control`, `pt_bringup` | `true`  | Launch motor diagnostics node                      |
| `pantilt_config`  | `pt_control`, `pt_bringup` | `pt101` | Pan-tilt mesh variant: `pt100` or `pt101` (`pt101` is recommended and default, see [Mesh variants](#mesh-variants-pantilt_config)) |
| `pointcloud`      | `pt_bringup`                  | `false` | Use `oakd_vio_pcl.yaml` (depth aligned to RGB + point cloud) instead of `oakd_vio.yaml` (depth unaligned, no point cloud). Also gates point cloud compression. |
| `octomap`         | `pt_bringup` (`oakd.launch.py` only) | `false` | Run `octomap_server` on `/oak/rgbd/points` to build a persistent 3D octree. Only takes effect when `pointcloud:=true`. Not forwarded by `pantilt.launch.py` - launch `oakd.launch.py` directly to use it. |
| `tf_parent_frame` | `pt_bringup` (`oakd.launch.py` only) | `tilt_link` | TF frame the OAK-D S2 is mounted to. Override when reusing `oakd.launch.py` to mount the camera elsewhere (e.g. directly on a host robot without the pan-tilt). Not forwarded by `pantilt.launch.py`. |
| `use_sim_time`    | `pt_control`, `pt_bringup` | `false` | Use `/clock` from a simulator instead of system time |
| `sim`              | `pt_bringup`                  | `false` | Run against MuJoCo instead of real hardware: forces `use_sim_time`/`use_mock`, and skips `oakd` (the simulated camera comes from `pt_mujoco`) |
| `mujoco_gui`       | `pt_bringup`                  | `false` | [`sim` only] Open the MuJoCo viewer window (headless otherwise; needs a display) |
| `mujoco_scene`     | `pt_control`, `pt_bringup`    | `flat`  | [`sim` only] `flat`, `none` or a scene MJCF path |
| `mujoco_model`     | `pt_control`                  | `""`    | [`sim` only] Path to a pre-built MJCF file; empty means generate one with `pt_mujoco` at launch time (so `pantilt_config` alone picks the pt100/pt101 model) |
| `mujoco_headless`  | `pt_control`                  | `false` | [`sim` only] Run without the MuJoCo Simulate viewer window (set automatically from `mujoco_gui` when launched via `pt_bringup`) |

> **Note:** All hardware parameters (serial port, baud rate, motor IDs, center steps, joint limits, etc.) are configured in [`pt_control/config/urdf_config.yaml`](pt_control/config/urdf_config.yaml). `sts_serial_port` and `use_mock` can be overridden at launch time; all other parameters must be changed in the yaml file directly.

## Package Structure

```text
pantilt_ros2/
├── pt_description/             # URDF model, meshes, and visualization launch
│   ├── urdf/
│   │   ├── pantilt.common.xacro   # Geometry constants and visual offsets
│   │   ├── pantilt.control.xacro  # ros2_control block, motor parameters, joint limits
│   │   ├── pantilt.joints.xacro   # Pan/tilt joint declarations as an embeddable macro (shared-bus use)
│   │   ├── pantilt.module.xacro   # Links and joints as an embeddable xacro macro
│   │   ├── pantilt.urdf.xacro     # Standalone entry point (includes all above)
│   │   ├── pt100.urdf             # Pre-generated standalone URDF (pantilt_config=pt100)
│   │   ├── pt101.urdf             # Pre-generated standalone URDF (pantilt_config=pt101, default, recommended)
│   │   └── oakd_s2.module.xacro   # OAK-D S2 camera and IMU macro
│   ├── meshes/                    # STL files for pan-tilt body and OAK-D S2
│   └── launch/
│       └── urdf.launch.py         # Visualization-only launch file (robot_state_publisher)
│
├── pt_mujoco/                  # MuJoCo models generated from the URDF, standalone viewer, benchmark, tests
│   ├── config/
│   │   ├── mujoco.yaml                       # Physics settings and pan/tilt servo profile (BAM-identified STS3215)
│   │   └── mujoco_ros2_control_plugins.yaml  # CameraPlugin config for oak_rgb (topics, optical frame, 30 Hz)
│   ├── mjcf/                      # pt.mjcf.xacro entry, pantilt_body macro, shared defaults, OAK-D S2 mount, scenes/
│   ├── pt_mujoco/                 # build_mujoco_models, simulation, mujoco_preview, benchmark_mujoco
│   └── test/
│
├── pt_control/                 # Controllers, config, and launch files
│   ├── config/
│   │   ├── urdf_config.yaml       # Hardware parameters (serial port, motor IDs, center steps, joint limits)
│   │   ├── pantilt_config.yaml    # Controller manager, spawner types, joint limits
│   │   └── teleop_config.yaml     # joy_teleop axis/button mapping
│   ├── launch/
│   │   ├── pantilt.launch.py      # Control stack (RSP, controller_manager, spawners, teleop); sim extras
│   │   └── teleop.launch.py       # joy_teleop node in isolation
│   └── test/
│       └── test_launch.py         # Launch argument checks
│
└── pt_bringup/                 # System-level launch files and camera config
  ├── config/
  │   ├── oakd_vio.yaml            # OAK-D S2: shared base - RGB + IMU + VIO + depth, no point cloud
  │   ├── oakd_vio_pcl.yaml        # OAK-D S2: overlay on the base - RGBD point cloud + RGB-aligned depth
  │   ├── depthimage_to_laserscan.yaml  # Slices a 2D LaserScan from the depth image (both modes)
  │   └── octomap.yaml             # octomap_server params (pointcloud:=true + octomap:=true only)
  ├── src/
  │   └── pcl_compressor_node.cpp  # Cloudini PCL compression composable node (point cloud mode)
  └── launch/
    ├── pantilt.launch.py          # Full PT100 system: includes pt_control + oakd
    └── oakd.launch.py             # OAK-D S2 camera driver only
```

## Package Details

### pt_description

The URDF is split across several xacro files with distinct responsibilities:

| File                     | Purpose                                                                       |
|--------------------------|-------------------------------------------------------------------------------|
| `pantilt.common.xacro`   | Visual offsets, mesh colours, joint origins, camera mount position            |
| `pantilt.control.xacro`  | Launch args, motor velocity/torque limits, joint limits, `ros2_control` block |
| `pantilt.joints.xacro`   | Pan/tilt joint declarations as a macro — for embedding on a shared serial bus |
| `pantilt.module.xacro`   | All links and joints wrapped in a `pantilt_module` xacro macro                |
| `oakd_s2.module.xacro`   | OAK-D S2 camera and IMU links as a reusable macro (`oakd_s2_camera`)          |
| `pantilt.urdf.xacro`     | Standalone robot: creates `base_footprint`, instantiates the macro            |

Both pan and tilt joints use `velocity="1e6"` in their URDF `<limit>` elements. See [Design](#design) for the reason.

The MuJoCo model is built from this URDF by [`pt_mujoco`](pt_mujoco/README.md) (see [Simulation](#simulation)); this package no longer holds any MJCF.

#### Mesh variants (`pantilt_config`)

This is the source of the project's naming convention: **PT100** is built with [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) base/shoulder parts, and **PT101** is built with the equivalent [SO-ARM101](https://github.com/TheRobotStudio/SO-ARM101) parts. `pantilt.urdf.xacro` (and `pantilt.common.xacro`) accept a `pantilt_config` arg — `pt101` (default) or `pt100` — that selects the matching mesh set for `pantilt_base_link` and `shoulder_link`.

> **PT101 is the recommended configuration and is set as the default** for `pantilt_config` across this project's launch files.

All link/joint origins, axes, and the mount to the host robot are identical between configs — only the body meshes (and their visual-origin corrections) differ. `pt100.urdf` and `pt101.urdf` are pre-generated standalone URDFs for each variant, regenerated with:

```bash
xacro pantilt.urdf.xacro pantilt_config:=pt100 -o pt100.urdf
sed -i 's#package://pt_description/meshes/#../meshes/#g' pt100.urdf
```

### pt_mujoco

Generates the MuJoCo model of the pan-tilt (frames, inertias and limits synced from `pt_description`'s URDF), and provides a standalone no-ROS viewer and benchmark, the ROS camera plugin configuration, and `build_payload_spec` for host robots that mount the pan-tilt in their own simulation. See [`pt_mujoco/README.md`](pt_mujoco/README.md).

### pt_control

Starts whichever control-node process matches `ros2_control_hardware_type`: the standard `controller_manager`/`ros2_control_node` for `real`, or `mujoco_ros2_control`'s own `ros2_control_node` for `mujoco` (it hosts the MuJoCo simulation itself, so this package owns that dependency). In `mujoco` mode it generates the model with `pt_mujoco`, loads the camera plugin config, and adds a compressed-image republisher and the camera optical-frame TF.

Hardware parameters are read from [`config/urdf_config.yaml`](pt_control/config/urdf_config.yaml) at launch time. `sts_serial_port` and `use_mock` can be overridden on the command line (empty string = use yaml value); all other parameters (motor IDs, center steps, joint limits, etc.) must be edited in the yaml directly.

The `pantilt_controller` uses a [`ForwardCommandController`](https://control.ros.org/kilted/doc/ros2_controllers/forward_command_controller/doc/userdoc.html) on the `position` interface. Each cycle it forwards the commanded position directly to the hardware interface, which translates it to motor steps. Velocity profiling is handled by the STS3215 motor firmware, not in software. The controller manager runs at **50 Hz** (set in `pantilt_config.yaml`).

The [`JointStateBroadcaster`](https://control.ros.org/kilted/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html) publishes standard joint states plus extended per-joint diagnostics (voltage, temperature, current, moving flag) to `/dynamic_joint_states`.

**Teleop mapping** (`teleop_config.yaml`):

| Input            | Button / Axis | Action                                      |
|------------------|---------------|---------------------------------------------|
| Deadman          | L1 (button 9) | Enable motion — hold to send commands       |
| Left stick X     | Axis 0        | Pan position (±π/2 rad at full deflection)  |
| Left stick Y     | Axis 1        | Tilt position (±π/2 rad at full deflection) |
| A (button 0)     | —             | Call `/emergency_stop` → enable             |
| B (button 1)     | —             | Call `/emergency_stop` → disable            |

Joystick axes map directly to **absolute** joint positions, not velocities. The full axis range ±1 maps to ±π/2 rad.

### pt_bringup

`pt_bringup/pantilt.launch.py` composes `pt_control/pantilt.launch.py` with either `oakd.launch.py` (real hardware) or nothing further at all (`sim:=true` - `pt_control`'s own launch file already starts everything MuJoCo needs), forwarding the relevant arguments to each.

`oakd.launch.py` launches the OAK-D S2 as a composable node container. `depth_to_scan` (`/oak/scan`) runs in both modes. A `PCLCompressorNode` (subscribes to `/oak/rgbd/points`, compresses using [cloudini](https://github.com/facontidavide/cloudini) at 1 mm resolution, publishes to `/oak/rgbd/points/compressed`) is only loaded when `pointcloud:=true` — the point cloud topic it depends on doesn't exist otherwise. The camera's TF parent is `tf_parent_frame` (default `tilt_link`), so this launch file can be reused as-is to bring up the OAK-D S2 on a host robot that doesn't have the pan-tilt, by overriding `tf_parent_frame` to the host's camera mount link.

`oakd_vio.yaml` is the shared base config (RGB, IMU, VIO, stereo depth settings) - always loaded. `oakd_vio_pcl.yaml` is a small overlay layered on top of it in `oak`'s composable node parameters list when `pointcloud:=true`, rather than duplicating the base into a second full config file. It sets the two keys that actually differ (`i_enable_rgbd`, `i_aligned`) plus its own `cloudini_compressor` block; RGB resolution and decimation are also pinned there defensively (see below) even though they currently match the base.

| Config file       | Pipeline                                                | Use case                        |
|-------------------|----------------------------------------------------------|---------------------------------|
| `oakd_vio.yaml`   | RGB 640x400 @ 30 Hz, IMU, depth 640x400 @ 30 Hz (full resolution, unaligned to RGB), VIO 60 Hz - no point cloud | Default — odometry and tracking |
| `oakd_vio_pcl.yaml`| RGB 640x400 @ 30 Hz, depth 640x400 @ 30 Hz (full resolution, aligned to RGB), VIO 60 Hz, point cloud | 3D mapping (higher CPU load)    |

`oakd_vio.yaml` publishes depth with `stereo.i_aligned: false` rather than the more common RGB-aligned depth — a `depthai_ros_driver` 3.1.0 bug crashes `oak_container` on the RGB-alignment code path when only the ROS publisher (not also a point cloud) consumes its output. Unaligned depth bypasses that code path entirely and is sufficient for `depth_to_scan`, which only needs a depth image + matching `camera_info`, not RGB alignment. A threshold filter (450-4000mm) clamps noisy readings. `oakd_vio_pcl.yaml` additionally pins RGB to 640x400 and decimation off as an explicit guard against a separate crash that only manifests in combination with `i_aligned: true` + `i_enable_rgbd: true` active together. `driver.i_enable_ir` is disabled in the base config since the OAK-D S2 (non-Pro) has no IR emitter hardware to drive.

When `octomap:=true` (requires `pointcloud:=true`), `octomap_server` subscribes to `/oak/rgbd/points` and accumulates a persistent 3D occupancy octree, looking up the camera's TF pose (driven by the pan-tilt's live joint states) at each cloud's timestamp so points land at their correct 3D position as the pan-tilt sweeps through different angles over time.

Set `DEPTHAI_DEBUG=1` in the environment before launching to enable debug-level logging from the camera driver.

## ROS Interfaces

### Topics

| Topic                            | Type                                    | Direction  | Description                                                                      |
|----------------------------------|-----------------------------------------|------------|----------------------------------------------------------------------------------|
| `/joint_states`                  | `sensor_msgs/JointState`                | Published  | Pan and tilt position, velocity, effort                                          |
| `/dynamic_joint_states`          | `control_msgs/DynamicJointState`        | Published  | Extended states: voltage, temperature, current, is_moving                        |
| `/pantilt_controller/commands`   | `std_msgs/Float64MultiArray`            | Subscribed | Position commands `[pan_rad, tilt_rad]`                                          |
| `/joy`                           | `sensor_msgs/Joy`                       | Subscribed | Joystick input (published by external `joy` node)                                |
| `/oak/rgb/image_raw`             | `sensor_msgs/Image`                     | Published  | OAK-D S2 RGB stream                                                              |
| `/oak/stereo/image_raw`          | `sensor_msgs/Image`                     | Published  | OAK-D S2 depth stream (aligned to RGB when `pointcloud:=true`, unaligned otherwise - see Package Details) |
| `/oak/scan`                      | `sensor_msgs/LaserScan`                 | Published  | Laser scan sliced from the depth image                                          |
| `/oak/imu/data`                  | `sensor_msgs/Imu`                       | Published  | OAK-D S2 IMU data                                                                |
| `/oak/vio/transform`             | `geometry_msgs/TransformStamped`        | Published  | Visual-inertial odometry output                                                  |
| `/oak/rgbd/points`               | `sensor_msgs/PointCloud2`               | Published  | Raw RGBD point cloud (only when `pointcloud:=true`)                              |
| `/oak/rgbd/points/compressed`    | `point_cloud_interfaces/CompressedPointCloud2` | Published | Cloudini-compressed point cloud at 1 mm resolution (only when `pointcloud:=true`) |
| `/base/diagnostics`              | `diagnostic_msgs/DiagnosticArray`       | Published  | Per-joint motor health: temperature, voltage, current (when `diagnostics:=true`) |

### Services

| Service             | Type                       | Direction | Description                                                                 |
|---------------------|----------------------------|-----------|-----------------------------------------------------------------------------|
| `/emergency_stop`   | `std_srvs/SetBool`         | Provided  | Enable (`true`) or disable (`false`) emergency stop — provided by the hardware interface, mapped to joystick buttons A and B |

### TF Frames

```text
base_footprint                             ← standalone root (pantilt.urdf.xacro only)
└── pantilt_base_link                      ← physical mount base (pantilt_mount_joint when embedded)
    └── shoulder_link                      ← rotates about Z (shoulder_pan_joint, ±90°)
        └── tilt_link                      ← rotates about Z in reoriented frame (tilt_joint, ±90°)
            └── oak_link                   ← OAK-D S2 optical centre
                ├── oak_link_model_origin  ← mesh visual origin
                └── oak_imu_frame          ← OAK-D S2 IMU frame
```

## Embedding as a Module

The pan-tilt is designed to be mounted on another robot. Two embedding strategies are available depending on whether the host robot shares the same serial bus as the pan-tilt motors.

### Dedicated serial bus

Include `pantilt.control.xacro` (which defines a standalone `<ros2_control>` hardware block) and `pantilt.module.xacro` in the host robot's URDF:

```xml
<xacro:include filename="$(find pt_description)/urdf/pantilt.control.xacro"/>
<xacro:include filename="$(find pt_description)/urdf/pantilt.module.xacro"/>

<xacro:pantilt_module parent="base_link">
  <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
</xacro:pantilt_module>
```

### Shared serial bus

If all motors (host + pan-tilt) share one serial bus and a single `<ros2_control>` hardware block, include `pantilt.joints.xacro` instead and call the `pantilt_joints` macro inside the host's existing hardware block:

```xml
<xacro:include filename="$(find pt_description)/urdf/pantilt.joints.xacro"/>

<ros2_control name="host_control" type="system">
  <hardware>...</hardware>
  <!-- host joints here -->
  <xacro:pantilt_joints sts3215_max_vel_steps="${sts3215_max_vel_steps}"/>
</ros2_control>
```

Motor IDs, step-centering, and joint limits aren't passed here — `pantilt_joints`' own macro defaults are their single source of truth (see [Motor Calibration](#motor-calibration)). Override any of them explicitly on this call only if this specific unit is calibrated differently.

Then include `pantilt.module.xacro` separately for the visual model:

```xml
<xacro:include filename="$(find pt_description)/urdf/pantilt.module.xacro"/>

<xacro:pantilt_module parent="base_link">
  <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
</xacro:pantilt_module>
```

In the host robot's controller config, add the pantilt joint limits under `controller_manager.ros__parameters.joint_limits`:

```yaml
controller_manager:
  ros__parameters:
    joint_limits:
      shoulder_pan_joint:
        has_position_limits: true
        min_position: -1.5708
        max_position: 1.5708
        has_velocity_limits: false
      tilt_joint:
        has_position_limits: true
        min_position: -1.5708
        max_position: 1.5708
        has_velocity_limits: false
```

### Launch-time bring-up on a shared bus

The xacro embedding above only covers the URDF/`ros2_control` description. `pt_control/launch/pantilt.launch.py` + `pt_bringup/launch/pantilt.launch.py` are the **dedicated-bus** bring-up (standalone `controller_manager`, own serial port) and aren't included by a shared-bus host, since a shared bus needs one `controller_manager` merging the host's own controllers with `pantilt_controller` - not two.

[lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2) is the reference shared-bus integration: its `lekiwi_control/launch/control.launch.py` re-implements the same xacro-merge → `controller_manager` → spawner sequence `pt_control/launch/pantilt.launch.py` shows for a dedicated bus, natively, rather than including it. Any other shared-bus host will need to do the same. What *is* shared automatically is the joint/limit xacro (`pantilt.joints.xacro`, single source); `urdf_config.yaml` motor IDs and joint limits are **not** - lekiwi_ros2 keeps its own copy under `lekiwi_control/config/payloads/pantilt/urdf_config.yaml`, so re-check it against this package's own `pt_control/config/urdf_config.yaml` after any motor recalibration. `pt_bringup/launch/oakd.launch.py` (the camera) has no bus coupling and is included directly by both bring-up paths. For simulation, a host builds the payload with `pt_mujoco.build_mujoco_models.build_payload_spec` from its own URDF and attaches it at its mount frame (see [`pt_mujoco/README.md`](pt_mujoco/README.md)), which keeps the pan-tilt's model in this repository and the mount in the host.

## Design

The URDF is split into `common` (geometry), `control` (ros2_control + motor parameters), `joints` (embeddable joint declarations), and `module` (links and joints) so the pan-tilt can be embedded into a host robot either with its own hardware block or as joints added to a shared bus - see [Embedding as a Module](#embedding-as-a-module) above. `pantilt.urdf.xacro` is a thin standalone wrapper around the `pantilt_module` macro.

`ForwardCommandController` sends raw position targets directly to the hardware; the STS3215 firmware handles velocity profiling, not software. Both joints use `velocity="1e6"` in real mode (physically unreachable, so it never clips) since `joy_teleop`'s absolute position commands can jump enough in one cycle to otherwise trip spurious `ros2_control` limit errors. Position limits (±π/2) remain enforced; the real speed ceiling comes from `max_velocity` (85% of the STS3215 hardware max).

## Simulation

`ros2_control_hardware_type` swaps the `<hardware>` plugin between real (`sts_hardware_interface`), Gazebo, and MuJoCo. **Only MuJoCo is wired into the launch files** (`sim:=true`); Gazebo support exists at the xacro level only, with no launch-side plumbing to start it.

The MuJoCo model is generated by [`pt_mujoco`](pt_mujoco/README.md) from the URDF (`pantilt_config` picks the pt100/pt101 variant), so no offset is copied by hand. `mujoco_ros2_control` hosts the physics in-process, so `sim:=true` starts nothing beyond `pt_control`'s own launch file plus two small helpers (the compressed image republisher and the optical-frame static TF). A commanded position on `/pantilt_controller/commands` drives the simulated physics, `/joint_states` converges to it (about 0.3 s with a few percent overshoot), `shoulder_link`/`tilt_link` TF updates, and the simulated camera's image changes with pan and tilt: `/oak/rgb/image_raw`, `/oak/stereo/image_raw`, `/oak/rgb/camera_info` and `/oak/rgb/image_raw/compressed`, in frame `oak_rgb_camera_optical_frame`. The camera is configured for the real 30 Hz pipeline; headless software rendering on a Raspberry Pi 5 delivered about 19 Hz at a real-time factor of 1.0. The same payload model is what [lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2) mounts in its own simulation. The servo profile is BAM-identified from a real STS3215; the simulation as a whole was not compared against this hardware.

## Notes and Troubleshooting

**Joystick not working** — The `joy` node is not launched by this package. Start it separately before launching the control stack, or in a second terminal alongside it:

```bash
ros2 run joy joy_node
```

Hold **L1** to enable motion. Without the deadman button held, `joy_teleop` does not publish commands.

**No hardware available** — Use `use_mock:=true` to run the full control stack with simulated motor responses. All topics, TF frames, and controllers behave identically; motor feedback values are synthesised.

**Camera driver debug logging** — Set `DEPTHAI_DEBUG=1` before launching to enable verbose output from `depthai_ros_driver`:

```bash
DEPTHAI_DEBUG=1 ros2 launch pt_bringup pantilt.launch.py
```

**Motor not reaching commanded position** — If a motor is mechanically obstructed or the center step calibration is wrong, the reported position will diverge from the command. Check `/dynamic_joint_states` for elevated `effort` or `current` values, which indicate the motor is stalled.

## License

Apache License 2.0 — see [LICENSE](LICENSE).
