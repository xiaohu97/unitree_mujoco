#!/usr/bin/env python3
"""Launch the Humanoid Ultra Mimic-Pick policy with repo-local assets.

By default this loads walk + stand + Pick and waits for the normal one-shot
Mimic trigger.  Pass ``--direct`` to initialize directly in the Pick clip.
All remaining arguments are forwarded to ``humanoid_ultra/sim2sim.py``.
"""

import runpy
import sys
from pathlib import Path


DIRECT_FLAG = "--direct"


def main():
    script_dir = Path(__file__).resolve().parent
    core_script = script_dir.parent / "humanoid_ultra" / "sim2sim.py"
    policy_path = script_dir / "assets" / "policy.pt"
    motion_path = script_dir / "assets" / "ustc1_pick.npz"

    for path in (core_script, policy_path, motion_path):
        if not path.is_file():
            raise FileNotFoundError("Required Pick sim2sim asset is missing: {}".format(path))

    forwarded_args = [arg for arg in sys.argv[1:] if arg != DIRECT_FLAG]
    direct_mode = DIRECT_FLAG in sys.argv[1:]

    default_args = ["--dof", "27"]
    if direct_mode:
        default_args.extend(
            ["--mode", "mimic", "--policy", str(policy_path)]
        )
    else:
        default_args.extend(["--mimic-policy", str(policy_path)])
    default_args.extend(["--motion-file", str(motion_path)])

    # Pick starts and ends far from the walk pose.  Enable the guarded
    # walk -> pick_prepare -> pick_play -> pick_recover -> walk path by
    # default while still allowing every value to be overridden on the CLI.
    transition_defaults = {
        "--mimic-prepare-time": "4.0",
        "--mimic-recover-time": "4.0",
        "--mimic-transition-target-speed": "1.0",
        "--mimic-policy-target-speed": "4.0",
        "--mimic-ready-rms": "0.12",
        "--mimic-ready-max-error": "0.25",
        "--mimic-ready-max-velocity": "0.8",
    }
    for option, value in transition_defaults.items():
        if not any(
            arg == option or arg.startswith(option + "=") for arg in forwarded_args
        ):
            default_args.extend([option, value])

    if not any(
        option in forwarded_args for option in ("--elastic-band", "--no-elastic-band")
    ):
        default_args.append("--no-elastic-band")

    sys.argv = [str(core_script), *default_args, *forwarded_args]
    runpy.run_path(str(core_script), run_name="__main__")


if __name__ == "__main__":
    main()
