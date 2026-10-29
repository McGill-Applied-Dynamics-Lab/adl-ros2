# Analysis — reproducing the talk figures on another machine

`talk_figures.ipynb` renders every talk figure (F1–F9) into `figures/` (PDF and PNG). It runs in the
light `analysis` pixi environment: no ROS, no robot stack, no CUDA (Linux or macOS).

## On the robot PC: pack the data

```bash
pixi run -e analysis pack-data      # -> rim-delays-analysis-data.tar.gz (~2 GB, 152 runs)
```

It packs only what the figures read: the runs of the latest session of E1–E4 and E6, E5's guided
steps (`kmax.csv`), the session files and `kmax.csv`, and per run `experiment.json`,
`metadata.json`, `samples.mcap` and `analysis/metrics.json`.

## On the laptop

```bash
git clone <adl-ros2> && cd adl-ros2 && git checkout feat/rim-delay-experiments
git submodule update --init external/adl-python          # the run loader (experiment_logger)
pixi install -e analysis

mkdir -p ~/data/franka/rim-delays
tar xzf rim-delays-analysis-data.tar.gz -C ~/data/franka/rim-delays
# elsewhere: export RIM_DELAYS_DATA=/path/to/the/extracted/folder

pixi run -e analysis figures        # executes the notebook in place, figures -> analysis/figures/
pixi run -e analysis jupyter lab    # or open it interactively
```

In VS Code, pick the kernel `.pixi/envs/analysis/bin/python`; the notebook finds `experiment_logger`
and `fr3_haptic` by path. Set `RIM_DELAYS_DATA` before starting VS Code if the data is not in
`~/data/franka/rim-delays`.

## What differs from the robot PC

- **Paths.** Session files and `kmax.csv` store the robot PC's absolute paths; `common.relocate`
  re-roots them under `RIM_DELAYS_DATA` (default `~/data/franka/rim-delays`).
- **The arm's model.** F4 and F6 need the RIM's Λ at two start positions, which needs the robot
  stack (arm_client, PyRoki, pinocchio). It is read from `model_cache.json` (committed). A new
  position is computed and added to the cache only in the `humble` environment.
- **LaTeX.** With a TeX install the figures use LaTeX text, exactly as on the robot PC (on macOS:
  MacTeX, or BasicTeX plus `tlmgr install type1cm cm-super dvipng`). Without one they fall back to
  matplotlib's Computer Modern mathtext: close, not identical. `FIG_USETEX=0` (or `1`) forces it.
- **Superseded runs.** The pack holds only the sessions' runs, so the notebook's session check
  reports 0 superseded tries (on the robot PC: the interrupted and replaced runs).
