"""Push OSC PD controller gains to the running controller without restarting.

Edit rim_controller.yaml (or any other config), then run this script to apply
the changes immediately — no controller restart or robot rehoming needed.

Usage:
    python set_gains.py [config_name]

config_name defaults to "rim_controller" and resolves to
configs/controllers/osc_pd/<config_name>.yaml.
"""

from __future__ import annotations

import sys
from pathlib import Path

from arm_client.robot import Robot

from arm_client import CONFIG_DIR


def main() -> None:
    config_name = sys.argv[1] if len(sys.argv) > 1 else "rim_controller"
    config_path = CONFIG_DIR / "controllers" / "osc_pd" / f"{config_name}.yaml"
    if not config_path.exists():
        print(f"Config not found: {config_path}")
        sys.exit(1)

    robot = Robot(namespace="fr3")
    client = robot.osc_pd_controller_parameters_client
    client.wait_until_ready()

    # Print current values for the gains we're about to change
    gain_params = [p for p in client.list_parameters() if p.startswith("gains.")]
    before = dict(zip(gain_params, client.get_parameters(gain_params)))

    client.load_param_config(file_path=config_path)
    print(f"Applied: {config_path.name}")

    after = dict(zip(gain_params, client.get_parameters(gain_params)))
    changed = {k: (before[k], after[k]) for k in gain_params if before.get(k) != after.get(k)}
    if changed:
        print("Changed gains:")
        for name, (old, new) in sorted(changed.items()):
            print(f"  {name}: {old} → {new}")
    else:
        print("No gain values changed.")

    robot.shutdown()


if __name__ == "__main__":
    main()
