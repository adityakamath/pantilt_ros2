#!/usr/bin/env python3
"""Measure pan and tilt step responses: python3 -m pt_mujoco.benchmark_mujoco --output out.json."""
import argparse
import json
from pathlib import Path
from tempfile import TemporaryDirectory

import mujoco
import numpy as np

from pt_mujoco.build_mujoco_models import VARIANTS, build
from pt_mujoco.simulation import PAYLOAD, Simulation


def benchmark(directory):
    rows = []
    steps = [('pan_positive', 0, .6), ('pan_negative', 0, -.6), ('tilt_positive', 1, .3), ('tilt_negative', 1, -.3)]
    for variant in VARIANTS:
        runtime = Simulation(mujoco.MjModel.from_xml_path(str(directory / f'{variant}_oakd_s2.xml')))
        for motion, joint, target in steps:
            runtime.reset()
            goal = [0., 0.]
            goal[joint] = target
            runtime.command(*goal)
            trace = []
            for _ in range(round(2 / runtime.model.opt.timestep)):
                runtime.step()
                trace.append(runtime.positions()[joint])
            trace = np.array(trace)
            settled = np.flatnonzero(np.abs(trace - target) > .02 * abs(target))
            rows.append({'variant': variant, 'motion': motion, 'joint': PAYLOAD[joint], 'target': target,
                         'seconds': 2, 'final_error': float(target - trace[-1]),
                         'overshoot_fraction': float(max(0., np.max(trace * np.sign(target)) - abs(target)) / abs(target)),
                         'settle_seconds': float((settled[-1] + 1) * runtime.model.opt.timestep) if settled.size else 0.,
                         'total_mass_kg': float(runtime.model.body_mass.sum())})
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--control-package', type=Path)
    parser.add_argument('--description-package', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with TemporaryDirectory(prefix='pt_benchmark_') as directory:
        for variant in VARIANTS:
            build(variant, Path(directory) / f'{variant}_oakd_s2.xml', absolute=True,
                  control_dir=args.control_package, description_dir=args.description_package)
        result = {'updated': benchmark(Path(directory))}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2))
    print(args.output)


if __name__ == '__main__':
    main()
