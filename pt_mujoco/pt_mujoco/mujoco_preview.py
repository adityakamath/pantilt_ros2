#!/usr/bin/env python3
"""Inspect a pan-tilt MJCF in the passive native viewer (mjpython on macOS): python3 -m pt_mujoco.mujoco_preview --variant pt101."""
import argparse
from pathlib import Path
from queue import SimpleQueue
from threading import Lock
import time
from tempfile import TemporaryDirectory

from pt_mujoco.build_mujoco_models import build
from pt_mujoco.simulation import PAYLOAD, PayloadControl, Simulation

import glfw
import mujoco
import mujoco.viewer


class KeyboardControl:
    """Compute pan/tilt rates from held keys, independent of keyboard repeat settings."""
    def __init__(self, model, control=None):
        self.model = model
        self.control = control or PayloadControl(model)
        self.payload_limits = self.control.payload_limits

    def update(self, data, held, dt):
        held = {ord(chr(k).upper()) if 97 <= k <= 122 else k for k in held}
        tilt = int(glfw.KEY_UP in held) - int(glfw.KEY_DOWN in held)
        pan = int(glfw.KEY_LEFT in held) - int(glfw.KEY_RIGHT in held)
        rates = {'shoulder_pan_joint': pan, 'tilt_joint': tilt}
        self.control.integrate_payload(data, [rates[name] for name in PAYLOAD], dt)


class HeldKeys:
    """Attach GLFW key callbacks on the viewer's UI thread; other events go to MuJoCo's callbacks."""
    BOUND = {glfw.KEY_UP, glfw.KEY_DOWN, glfw.KEY_LEFT, glfw.KEY_RIGHT, *map(ord, 'XP')}

    def __init__(self):
        self.lock = Lock()
        self.held = set()
        self.events = SimpleQueue()
        self.window = None
        self.previous_key = None
        self.previous_focus = None

    def snapshot(self):
        with self.lock:
            return self.held.copy()

    def clear(self):
        with self.lock:
            self.held.clear()

    def install_callbacks(self, window):
        # Python GLFW's public setters discard callbacks installed by C++.
        # Retain both native function pointers and Python callback lifetimes.
        self.key_callback = glfw._GLFWkeyfun(self.on_key)
        self.focus_callback = glfw._GLFWwindowfocusfun(self.on_focus)
        self.previous_key = glfw._glfw.glfwSetKeyCallback(window, self.key_callback)
        self.previous_focus = glfw._glfw.glfwSetWindowFocusCallback(window, self.focus_callback)

    def bootstrap(self, key):
        if self.window is not None:
            return
        window = glfw.get_current_context()
        if not window:
            return
        self.window = window
        self.install_callbacks(window)
        with self.lock:
            self.held = {k for k in self.BOUND if glfw.get_key(window, k) == glfw.PRESS}
        if key in self.BOUND:
            self.record(key, glfw.PRESS)
        print('Held-key input attached: press/release and focus tracking enabled', flush=True)

    def record(self, key, action):
        with self.lock:
            if action == glfw.RELEASE:
                self.held.discard(key)
            elif action == glfw.PRESS:
                self.held.add(key)
        if action == glfw.PRESS and key in (ord('P'), ord('X')):
            self.events.put(key)

    def on_key(self, window, key, scancode, action, mods):
        if key in self.BOUND:
            self.record(key, action)
        elif self.previous_key:
            self.previous_key(window, key, scancode, action, mods)

    def on_focus(self, window, focused):
        if not focused:
            self.clear()
            self.events.put('focus_lost')
        if self.previous_focus:
            self.previous_focus(window, focused)


CONTROLS = 'Left/Right: pan | Up/Down: tilt | X: reset | P: pause'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=['pt100', 'pt101'], default='pt101')
    parser.add_argument('--control-package', type=Path)
    parser.add_argument('--description-package', type=Path)
    parser.add_argument('--scene', default='flat', help='flat, none, or scene MJCF path')
    parser.add_argument('--model', type=Path, help='Use an explicit prebuilt XML and its embedded limits instead of regenerating')
    parser.add_argument('--island-colors', action='store_true', help='Debug constraint islands instead of displaying robot materials')
    args = parser.parse_args()
    filename = f'{args.variant}_oakd_s2.xml'
    # Keep generated files alive for the viewer lifetime; mesh paths are absolute.
    generated = TemporaryDirectory(prefix='pt_preview_') if args.model is None else None
    path = (args.model.resolve() if args.model else
            build(args.variant, Path(generated.name) / filename, absolute=True, scene=args.scene,
                  control_dir=args.control_package, description_dir=args.description_package))
    model = mujoco.MjModel.from_xml_path(str(path))
    simulation = Simulation(model)
    simulation.reset()
    data = simulation.data
    keyboard = KeyboardControl(model, simulation.control)
    keys = HeldKeys()
    paused = False
    with mujoco.viewer.launch_passive(model, data, key_callback=keys.bootstrap) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = data.xpos[simulation.payload.base] + [0.05, 0, .1]
            viewer.cam.distance = .6
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -22
            viewer.opt.geomgroup[3] = 0  # Hide collision proxies, keep CAD visuals.
        print(f'Native viewer ready: {path}', flush=True)
        print(CONTROLS, flush=True)
        steps = max(1, round(1 / (60 * model.opt.timestep)))
        period = steps * model.opt.timestep
        while viewer.is_running():
            start = time.monotonic()
            with viewer.lock():
                # Island debug coloring strips robot materials; keep normal colors by default.
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_ISLAND] = args.island_colors
                while not keys.events.empty():
                    key = keys.events.get()
                    if key in (ord('P'), ord('p')):
                        paused = not paused
                        keys.clear()
                        simulation.stop()
                    elif key in (ord('X'), ord('x')):
                        simulation.stop()
                        keys.clear()
                        simulation.reset()
                    elif key == 'focus_lost':
                        simulation.stop()
                if not paused:
                    held = keys.snapshot()
                    for _ in range(steps):
                        keyboard.update(data, held, model.opt.timestep)
                        simulation.step()
            pan, tilt = simulation.positions()
            viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100,
                               mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                               f'{Path(path).stem} | {data.time:.2f} s | pan {pan:+.2f} tilt {tilt:+.2f} rad | '
                               f'{"PAUSED" if paused else "RUNNING"}\n' + CONTROLS, '')])
            viewer.sync()
            time.sleep(max(0, period - (time.monotonic() - start)))


if __name__ == '__main__':
    main()
