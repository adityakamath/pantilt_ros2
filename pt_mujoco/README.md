# Pan-Tilt MuJoCo

MuJoCo models of the Pan Tilt mechanism (`pt100`, `pt101`) and its OAK-D S2 camera, generated from the URDF in `pt_description`. The package gives you three ways to use them:

- **Standalone:** a native viewer, a model builder and a benchmark, with no ROS needed.
- **ROS simulation:** `sim:=true` runs the same controllers and teleop as the real pan-tilt against the model in `mujoco_ros2_control`.
- **Payload for another robot:** hosts such as [lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2) attach the model to their own.

## Requirements

- Python packages, installed into the interpreter that ROS and colcon use: `pip install -r requirements.txt` (`mujoco`, `numpy`, `xacro`, `PyYAML`). Add `pytest<8` to run the tests.
- `pt_description` (URDF and meshes) and `pt_control` (configuration files) from this repository.
- For the ROS simulation only, on Kilted:
  - `sudo apt install ros-kilted-mujoco-ros2-control ros-kilted-mujoco-ros2-control-plugins ros-kilted-image-transport-plugins` (0.1.2 or newer; older releases have no camera plugin)
  - [mujoco_ros2_plugins](https://github.com/adityakamath/mujoco_ros2_plugins), cloned into the same workspace. It provides the simulated `/emergency_stop`, and launch fails if it is missing.

## Standalone use

Run these from this directory (`mjpython` instead of `python3` on macOS for the viewer):

```sh
python3 -m pt_mujoco.mujoco_preview --variant pt101      # native viewer
python3 -m pt_mujoco.build_mujoco_models --variant pt101 --output /tmp/pt.xml --absolute
python3 -m pt_mujoco.benchmark_mujoco --output step_response.json
```

In the viewer, click the window first, then use Left/Right for pan, Up/Down for tilt, X to reset and P to pause. The same tools are installed as commands (`ros2 run pt_mujoco <tool>` or `pip install -e .`).

Always build models with `build_mujoco_models` rather than plain xacro: it takes frames, mesh origins, inertias and joint limits from the URDF. `--scene` selects the environment (`flat`, `none` or a scene file). With no arguments it regenerates the committed `mjcf/pt100_oakd_s2.xml` and `mjcf/pt101_oakd_s2.xml`, which you should do after any change to the URDF, the config or the MJCF.

From Python:

```python
import mujoco
from pt_mujoco.build_mujoco_models import build
from pt_mujoco.simulation import Simulation

sim = Simulation(mujoco.MjModel.from_xml_path(str(build("pt101", "/tmp/pt.xml", absolute=True))))
sim.reset()
sim.command(0.5, -0.3)     # pan, tilt targets in radians
sim.step(500)
print(sim.positions())
```

## ROS simulation

Build the workspace as described in the [repository README](../README.md#installation), then:

```sh
ros2 launch pt_bringup pantilt.launch.py sim:=true                    # headless, no display needed
ros2 launch pt_bringup pantilt.launch.py sim:=true mujoco_gui:=true   # with the MuJoCo viewer
```

`pantilt_config` (`pt100` or `pt101`) and `mujoco_scene` choose the model. To watch a headless run from another machine, start `foxglove_bridge` and connect Foxglove to `ws://<host>:8765`. Command it as on the real pan-tilt, with the joystick or:

```sh
ros2 topic pub /pantilt_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5, -0.3]}"
```

| Topic or service | What it is |
|---|---|
| `/joint_states`, `/pantilt_controller/commands` | Same as the real robot, from ros2_control on the simulated motors |
| `/oak/rgb/image_raw`, `/oak/stereo/image_raw`, `/oak/rgb/camera_info` | Simulated OAK-D S2 camera in frame `oak_rgb_camera_optical_frame`. The image is upside down, like the real, inverted camera mount |
| `/oak/rgb/image_raw/compressed` | Compressed version of the RGB image |
| `/oak/scan` | Laser scan sliced from the depth image, as on the real bringup |
| `/emergency_stop` (`std_srvs/SetBool`) | While enabled the pan-tilt holds its position and ignores commands; releasing it hands control back |

The camera is set to the real pipeline's 30 Hz. Headless rendering on a Raspberry Pi 5 delivered about 19 Hz, with the simulation still running in real time.

## Use as a payload in another robot

```python
from pt_mujoco.build_mujoco_models import build_payload_spec

payload = build_payload_spec(variant, urdf, joint_limits, description_dir)
payload.default.name = 'pt_payload'
frame = host.body('base_link').add_frame(name='pantilt_mount')   # placed from the host's URDF
host.attach(payload, prefix='', frame=frame)
```

`urdf` is the host's own expanded URDF, which must contain the pan-tilt, so the host stays the source of truth for where the pan-tilt is mounted. `joint_limits` is an optional dictionary of joint limit overrides (`{}` to use the URDF's limits). Everything else, meaning inertias, torque limits, servo parameters and the camera with its orientation, comes from this package. lekiwi_mujoco does exactly this. A host also loads `config/mujoco_ros2_control_plugins.yaml` for the camera plugin, and may lower the camera rate with `camera_publish_rate`.

## Configuration

| File | What it sets |
|---|---|
| `config/mujoco.yaml` | Timestep, integrator and solver settings, and the pan and tilt servo profile (gain, damping, armature, friction) |
| `config/mujoco_ros2_control_plugins.yaml` | The ROS plugins: camera (topics, frame, rate) and the emergency stop |
| `config/mujoco_depth_to_scan.yaml` | Range and height of the `/oak/scan` slice |
| `mjcf/` | The MJCF model sources (entry file, pan-tilt body, servo defaults, camera) and the `scenes/` floor. Frames, inertias and limits in them are overwritten from the URDF at build time |

Changes to `config/mujoco.yaml` are applied when the model is built, so rebuild (or relaunch) to see them. The servo profile was identified from a real STS3215 servo; change the gains there if you want a different response.

If the package cannot find its files, point it at them with `PT_MUJOCO_SHARE`, `PT_DESCRIPTION_SHARE` or `PT_CONTROL_SHARE`.

## Limitations

Only the servo profile is measured (a real STS3215 at 12 V). Inertias, the torque limit and everything else are approximations, and there is no torque-speed curve, backlash or sensor noise. A step response settles in about 0.3 s with 4% (pan) to 8% (tilt) overshoot. The camera renders headless; the MuJoCo viewer (`mujoco_gui:=true` and `mujoco_preview`) needs a display.

## Tests

```sh
pytest pt_mujoco/test -q       # about 15 seconds
```

The launch arguments are covered by `pt_control/test/test_launch.py`.
