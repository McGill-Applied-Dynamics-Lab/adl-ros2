# osc_controller benchmarks

Reproducible measurements of `osc_controller` (franka-server) from the controller's own 1 kHz topics.

| Script | Motion | Measures |
|---|---|---|
| `hold.py` | none, holds the current pose | static error, drift, torque/velocity chatter |
| `step_response.py` | 3 cm steps out and back along x, y, z | rise, overshoot, settling, steady-state error, off-axis and orientation error, chatter |
| `tracking.py` | figure eight (8 s period, 0.4 x 0.16 m), streamed at 500 Hz with twist feedforward | RMS/max error, lag, orientation error, chatter |
| `compare.py` | – | table of runs: gains, differing parameters, headline metrics |
| `plots.py` | – | per-run figures (made automatically) and `--compare` overlays |

## Workflow

```bash
cd experiments/benchmarks/osc_controller
# Gains: a preset from configs/controllers/osc/ and/or overrides. Restored after the run unless --keep.
python step_response.py --config default --tag base
python step_response.py --config default --set gains.k_pos_x=1000 --set gains.k_pos_y=1000 --set gains.k_pos_z=1000 --tag kp1000
python compare.py step_response
python plots.py --compare results/step_response_base_*.yaml results/step_response_kp1000_*.yaml
```

Always pass `--config` (or `--set`) for runs you want to compare: without it the run uses whatever
gains are live, and a franka-server restart resets those to the server defaults. The gains that
were actually used are in each result YAML (`params`) and in every plot label.

## Results

`results/<benchmark>_<tag>_<time>.{yaml,npz,png}` (git-ignored).

- `.yaml`: CLI args, git commit, all live controller parameters, stream health, metrics
- `.npz`: raw streams `ee`, `err`, `wrench`, `tau`, `state`, each with `<k>_t` (local arrival time,
  same clock as the event marks) and `<k>_stamp` (controller stamp); `event_t`/`event_label`;
  step targets (`step_*`) or the streamed reference (`ref_*`)
- `.png`: response figure and per-joint spectra

Chatter is the RMS of a signal above 15 Hz (all commanded motion is below that). In step runs it
includes the step transient itself; compare the hold portion or the spectra for steady chatter.

Gain units: with `control.inertia_decoupling: true` gains are in 1/s² and 1/s, so the physical
stiffness is `Λ·k` and depends on the configuration (very low for rotation about the tool axis).
