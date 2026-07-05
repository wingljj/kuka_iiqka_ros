#!/usr/bin/env python3
# scripts/run_scenarios.py
"""Batch-run all validation scenarios, print report, exit non-zero on FAIL."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from kuka_mujoco_sim.mujoco_world import MujocoWorld  # noqa: E402
from kuka_mujoco_sim import scenarios  # noqa: E402

MODEL = os.path.join(os.path.dirname(__file__), '..', 'models',
                     'kuka_kr20_scene.xml')


def world_factory(**kw):
    return MujocoWorld(os.path.abspath(MODEL), **kw)


def main():
    results = [fn(world_factory) for fn in scenarios.ALL_SCENARIOS]
    report = scenarios.format_report(results)
    print(report)
    sys.exit(0 if all(r.passed for r in results) else 1)


if __name__ == '__main__':
    main()
