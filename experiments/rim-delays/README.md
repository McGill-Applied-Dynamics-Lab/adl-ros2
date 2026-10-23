# RIM under communication delay

- [EXP_PLAN.md](EXP_PLAN.md): the experiments, metrics and parameter-selection rules.
- [SCRIPTS_PLAN.md](SCRIPTS_PLAN.md): the scripts that run them, and the order they are built in.
- [FORMULATION.md](FORMULATION.md): the methods' equations.
- `pretests/`: interactive pre-tests T1–T5 (one preset per script, Ctrl-C ends, metrics follow).
- `sim_teleop.py`, `sim_plant.py`: `fr3_teleop` against a simulated FR3 (constant task inertia).
- `runs.py`: loads a saved run.
- `old/`: the retired scripted-grid design (`run_conditions.py`, `conditions.yaml`, `analyze.py`).

Runs are saved under `~/data/franka/rim-delays/<robot|sim>/`.
