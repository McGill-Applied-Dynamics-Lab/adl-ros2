# RIM under communication delay

- [EXP_PLAN.md](EXP_PLAN.md): the experiments, metrics and parameter-selection rules.
- [SCRIPTS_PLAN.md](SCRIPTS_PLAN.md): the scripts that run them, and the order they are built in.
- [FORMULATION.md](FORMULATION.md): the methods' equations.
- `exp/`: one script per experiment (`e1_free_motion.py` … `e6_taps.py`): parameters and the
  method × condition lists at the top; each run is saved as `<time>_<exp>_<condition>`, analysed
  (`<run>/analysis/check.md`, `metrics.json`) and listed in a session file
  (`~/data/franka/rim-delays/<robot|sim>/<exp>/sessions/`). One notebook per experiment
  (`exp/eN_*.ipynb`) takes runs, session files or a folder and plots the metrics. Shared:
  `exp/common.py` (runner), `exp/exp_metrics.py` (metrics), `exp/plots.py`.
- `pretests/`: interactive pre-tests T1–T5 (one preset per script, Ctrl-C ends, metrics follow).
- `sim_teleop.py`, `sim_plant.py`: `fr3_teleop` against a simulated FR3 (constant task inertia).
- `analysis/talk_figures.ipynb`: every talk figure (F1–F9), into `analysis/figures/`. Runs on any
  laptop in the light `analysis` environment (`pixi run -e analysis figures`); how to move the data
  there: [analysis/README.md](analysis/README.md).
- `runs.py`: loads a saved run.
- `old/`: the retired scripted-grid design (`run_conditions.py`, `conditions.yaml`, `analyze.py`).

Runs are saved under `~/data/franka/rim-delays/<robot|sim>/` (elsewhere: `RIM_DELAYS_DATA`).
