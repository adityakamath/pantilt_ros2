# Pan Tilt Description

URDF/xacro model of the Pan Tilt mechanisms - `pt100` and `pt101` (two STS3215 servos and an Orbbec Gemini 2 camera by default, or the supported OAK-D S2 alternative), its meshes, and a launch file for viewing it. The same files describe the standalone pan-tilt and the module that other robots embed. Everything else in the `pantilt_ros2` repository, including the MuJoCo model, is generated from this description.

## Contents

| Path | Purpose |
|------|---------|
| `urdf/pantilt.urdf.xacro` | Standalone robot: adds a `base_footprint` root and instantiates the module |
| `urdf/pantilt.module.xacro` | The links and joints as a `pantilt_module` macro, for embedding |
| `urdf/pantilt.control.xacro` | The `<ros2_control>` block for a dedicated serial bus, plus the launch arguments below |
| `urdf/pantilt.joints.xacro` | The `pantilt_joints` macro: only the ros2_control joints, for a host that shares its bus |
| `urdf/pantilt.common.xacro` | Geometry constants, mesh variant selection, joint origins and limits |
| `urdf/camera.module.xacro` | Shared `camera` macro for both camera meshes and compatibility IMU frames |
| `urdf/pt{100,101}_{oakd_s2,gemini2}.urdf` | Four pre-generated standalone body/camera combinations |
| `meshes/` | STL files for the base, shoulder, motors and camera |
| `launch/urdf.launch.py` | Starts `robot_state_publisher` with the standalone URDF |

## Running

```bash
ros2 launch pt_description urdf.launch.py
```

This starts only `robot_state_publisher`, with no hardware, controllers or teleop, so you can inspect the model and TF tree in RViz or other tools. The launch accepts `pantilt_config:=pt100|pt101` and `camera_config:=gemini2|oakd_s2`; defaults are `pt101` and `gemini2`. You can also expand the xacro directly:

```bash
xacro pantilt.urdf.xacro pantilt_config:=pt100
```

## Configuration

### Arguments

Pass these to `xacro` as `name:=value`. The launch files in `pt_control` fill them in from `urdf_config.yaml`.

| Argument | Default | Meaning |
|----------|---------|---------|
| `camera_config` | `gemini2` | Camera mesh: `gemini2` or `oakd_s2` |
| `pantilt_config` | `pt101` | Mesh variant: `pt100` (SO-ARM100 parts) or `pt101` (SO-ARM101 parts, recommended) |
| `serial_port` | `/dev/ttySERVO` | Servo bus serial port |
| `baud_rate` | `1000000` | Servo bus baud rate |
| `use_mock` | `false` | Use simulated motor responses instead of hardware |
| `use_sync_write` | `true` | Send both servo commands in one bus write |
| `sts3215_max_vel_steps` | `3400` | Servo maximum speed in steps/s (STS3215: 3400, STS3032: 2900) |
| `proportional_vel_max` | `0` | Bus-level velocity cap; `0` uses each joint's commanded velocity |
| `internal_max_vel`, `internal_max_acc`, `internal_acc_coeff` | `65`, `50`, `0` | Servo speed profile written to each servo's EEPROM at start-up |
| `ros2_control_hardware_type` | `real` | Hardware plugin: `real` (`sts_hardware_interface`), `gazebo` or `mujoco`; the joints and controllers are the same for all three |
| `mujoco_model`, `mujoco_headless` | `""`, `false` | `mujoco` only: the generated MJCF to load and whether to skip the viewer |

### Mesh variants

PT100 is built from SO-ARM100 base and shoulder parts and PT101 from the SO-ARM101 equivalents. Only the body meshes and their visual offsets differ; every link origin, joint axis and the mount to a host robot are identical, so PT101 is a drop-in replacement for PT100.

### Calibration and limits

Motor IDs, step centres and joint limits belong to the mechanism, not to a deployment, so they are the macro defaults in `pantilt.joints.xacro`:

| Parameter | Default |
|-----------|---------|
| `shoulder_pan_motor_id`, `tilt_motor_id` | `1`, `2` |
| `shoulder_pan_center_steps`, `tilt_center_steps` | `2048`, `2048` (raw steps 0–4095 that map to 0 rad) |
| `shoulder_pan_lower`/`upper`, `tilt_joint_lower`/`upper` | ±1.5708 rad (±90°) |

Recalibrate a physical unit by editing these defaults. A host robot on a shared bus can instead pass different values on its own `<xacro:pantilt_joints .../>` call.

The joint `<limit>` velocity is `1e6` (effectively unlimited) for `real` and `mujoco`, because `joy_teleop` sends absolute positions that can jump far in one control cycle and would otherwise trigger spurious limit errors. The real speed ceiling is the servo's own: 85% of `sts3215_max_vel_steps`, plus the EEPROM profile above. Only `gazebo` uses the computed limit. The torque limit is 2.942 N·m (30 kgf·cm, the 12 V STS3215).

## Frames

```text
base_footprint                  ← standalone root only
└── pantilt_base_link           ← mount to the host when embedded
    └── shoulder_link           ← shoulder_pan_joint
        └── tilt_link           ← tilt_joint
            └── oak_link        ← compatibility mount frame; real Gemini sensor tree attaches here
                ├── oak_link_model_origin   ← mesh visual origin
                └── oak_imu_frame
```

## Regenerating the pre-built URDFs

`pt100_oakd_s2.urdf`, `pt101_oakd_s2.urdf`, `pt100_gemini2.urdf`, and
`pt101_gemini2.urdf` are checked in for tools that want plain URDF without running xacro. Regenerate them from the `urdf/` directory after any xacro change:

```bash
for v in pt100 pt101; do
  for camera in oakd_s2 gemini2; do
    output="${v}_${camera}.urdf"
    xacro pantilt.urdf.xacro pantilt_config:=$v camera_config:=$camera -o "$output"
    python3 -c 'from pathlib import Path; import sys; p=Path(sys.argv[1]); p.write_text(p.read_text().replace("package://pt_description/meshes/", "../meshes/"))' "$output"
  done
done
```

## Using it on another robot

Include the macros in the host's URDF and call them at the mount point. See the [repository README](../README.md#using-it-on-another-robot) for the dedicated-bus and shared-bus examples. [lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2) is the reference host.

## Tests

```bash
pytest test -q
```

The tests run `xacro` as a subprocess and check the output for both variants and hardware types: that the correct hardware plugin is selected, the servo profile defaults are 65 / 50 / 0, and the velocity limits are unlimited except where a simulator enforces them.

Camera geometry is selected independently with `camera_config:=gemini2|oakd_s2`
(default `gemini2`), for either PT100 or PT101. Gemini 2 uses dedicated meshes and a camera-specific mesh mounting offset, while
retaining the existing camera frame names; see the [camera variant notes](../README.md#camera-mesh-variants).

Camera STL assets must omit embedded binary-STL color headers and per-face colors,
so viewers use the URDF materials: charcoal for the camera and light grey for
the tilt mount, matching the other printed parts. Both Gemini meshes retain their
geometry; their CAD-export color metadata has been removed.

## MuJoCo camera subtrees

`pt_mujoco` selects `oakd_s2_subtree.xml` or `gemini2_subtree.xml` from the
expanded URDF's camera mesh. These define the corresponding tilt assembly and
simulation sensor. The builder then synchronizes mesh origins, transforms and
physical parameters from the URDF. Regenerate both camera sets after changing
this description; see [the builder instructions](../pt_mujoco/README.md#gemini-2-models).
Plain `.urdf` snapshots use `../meshes/` paths; Xacro retains ROS package paths.

The real Gemini 2 driver owns calibrated sensor frames below `gemini2_link`. `pt_bringup` attaches that frame to `oak_link` with a configurable mount transform; its translation must be measured on hardware. The legacy `oak_imu_frame` is not a Gemini IMU calibration. See [Gemini 2 driver and TF](../pt_bringup/README.md#gemini-2-driver-and-tf).
