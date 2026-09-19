# Pan-Tilt MuJoCo

MuJoCo models of the pan-tilt payload (`pt100`, `pt101`) with its OAK-D S2 camera, generated from
`pt_description`'s URDF, plus a standalone (no ROS) viewer and benchmark. The ROS simulation
(`sim:=true`) runs these models in `mujoco_ros2_control`, and host robots such as
[lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2) mount the same payload on their own model.

## Requirements

- Python: `pip install -r requirements.txt` (`mujoco==3.13.0`, `numpy`, `xacro`, `PyYAML`) into the
  interpreter used by colcon/ROS launch. Add `pytest<8` for the tests.
- `pt_description` (URDF and meshes) and `pt_control` (`config/`, data only) from the same repository.
- ROS sim only (Kilted): `sudo apt install ros-kilted-mujoco-ros2-control
  ros-kilted-mujoco-ros2-control-plugins ros-kilted-image-transport-plugins` (0.1.2 or newer; older
  releases have no camera plugin).

## Standalone use

From this directory (no install, no ROS; `mjpython` on macOS for the viewer):

```sh
python3 -m pt_mujoco.mujoco_preview --variant pt101      # native viewer
python3 -m pt_mujoco.build_mujoco_models --variant pt101 --output /tmp/pt.xml --absolute
python3 -m pt_mujoco.benchmark_mujoco --output step_response.json
```

The same tools are installed as `mujoco_preview`, `build_mujoco_models` and `benchmark_mujoco`
(`pip install -e .` or `ros2 run pt_mujoco <tool>`). `build_mujoco_models` takes
`--scene flat|none|<path>`; with no arguments it regenerates the committed
`mjcf/pt100_oakd_s2.xml` and `mjcf/pt101_oakd_s2.xml` snapshots (do this after any URDF, config or
MJCF edit). Use it instead of plain xacro: it syncs frames, mesh origins, inertias and limits from
the URDF.

Viewer keys: Left/Right pan, Up/Down tilt, X reset, P pause; click the viewport first.

```python
import mujoco
from pt_mujoco.build_mujoco_models import build
from pt_mujoco.simulation import Simulation

sim = Simulation(mujoco.MjModel.from_xml_path(str(build("pt101", "/tmp/pt.xml", absolute=True))))
sim.reset()
sim.command(0.5, -0.3)     # absolute [pan, tilt] targets in radians, slew-limited and clipped to range
sim.step(500)              # or sim.step(500, action=[1., 0.]) for normalized [pan, tilt] rates
print(sim.positions())
```

## ROS simulation

Build the workspace as in the [repository README](../README.md#installation), then:

```sh
ros2 launch pt_bringup pantilt.launch.py sim:=true                    # headless, no display needed
ros2 launch pt_bringup pantilt.launch.py sim:=true mujoco_gui:=true   # with the MuJoCo viewer
```

`mujoco_scene` (`flat`, `none` or a scene path) and `pantilt_config` (`pt100`/`pt101`) select the
model. To watch a headless sim from another machine, run `foxglove_bridge` and connect Foxglove to
`ws://<host>:8765`. Drive it with `ros2 topic pub /pantilt_controller/commands
std_msgs/msg/Float64MultiArray "{data: [0.5, -0.3]}"` (pan, tilt in radians) or the joystick.

| Topic | Source |
|---|---|
| `/joint_states`, `/pantilt_controller/commands` | ros2_control on the simulated hardware |
| `/oak/rgb/image_raw`, `/oak/stereo/image_raw`, `/oak/rgb/camera_info` | `CameraPlugin`, headless EGL; frame `oak_rgb_camera_optical_frame` (static TF from `oak_link`); the image is upside down like the real, inverted OAK-D mount |
| `/oak/rgb/image_raw/compressed` | `image_transport` republisher of the raw image |

The camera is configured for the real pipeline's 30 Hz; headless software rendering on a Raspberry
Pi 5 delivered about 19 Hz. The real-time factor was 1.0.

## Use as a payload in another robot

```python
from pt_mujoco.build_mujoco_models import build_payload_spec

payload = build_payload_spec(variant, urdf, joint_limits, description_dir)  # world -> pantilt_base_link
payload.default.name = 'pt_payload'                                          # named root default
frame = host.body('base_link').add_frame(name='pantilt_mount')              # place it from the host URDF
host.attach(payload, prefix='', frame=frame)
```

`urdf` is the host's own expanded URDF (it must contain the pan-tilt chain, and its ros2_control
block), so the host stays the source of truth for the mount and frames; `joint_limits` is the
controller `joint_limits` block of the host's control config. Frames, inertias, effort limits, servo
parameters, the camera and its orientation all come from this package. lekiwi_mujoco does exactly
this. A host loads `config/mujoco_ros2_control_plugins.yaml` for the camera plugin and may override
`camera_publish_rate`.

## Configuration

| File | Controls |
|---|---|
| `config/mujoco.yaml` | Timestep, integrator and solver iterations; pan/tilt servo armature, friction and position gain (uncalibrated STS3215 approximation) |
| `config/mujoco_ros2_control_plugins.yaml` | ROS camera plugin: topics, optical frame, 30 Hz rate |
| `mjcf/pt.mjcf.xacro`, `pantilt.mjcf.xacro`, `pantilt_shared.xml`, `oakd_s2_subtree.xml` | Payload MJCF (entry file, `pantilt_body` macro, defaults and actuators, tilt link and camera); frames, mesh origins, inertias and limits are overwritten from the URDF at build time |
| `mjcf/scenes/flat.xml` | Floor only |

Asset lookup uses the source checkout, ROS prefixes or the Python `share` directory; override with
`PT_MUJOCO_SHARE`, `PT_DESCRIPTION_SHARE`, `PT_CONTROL_SHARE` or the builder's `--control-package`,
`--description-package`. Joint limits come from `pt_control/config/pantilt_config.yaml`, the same file
the real controller uses.

## Limitations

Servo dynamics and inertias are uncalibrated approximations (no torque-speed curve, backlash or
sensor noise); a step response settles in about 0.55 s without overshoot. Camera rendering is
headless (EGL); the MuJoCo viewer (`mujoco_gui:=true`, `mujoco_preview`) needs a display.

## Tests

`pytest pt_mujoco/test -q` (about 15 seconds). `test_without_ros.py` runs the tools with ROS imports
blocked and needs `mujoco` and `xacro` pip-installed (skipped otherwise). The launch arguments are
checked by `pt_control/test/test_launch.py`.
