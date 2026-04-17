# Rolling Contact Experiment — Force Control

Acoustic sensing experiment where the FR3 robot rolls a finray-gripper sensor along a cylinder surface while maintaining a constant contact force. RF data from all 4 waveguide channels is collected throughout the slide.

## Scripts

| Script | Purpose |
|--------|---------|
| `rolling_contact_experiment_force.py` | Main experiment — adaptive-PID force-controlled slide |
| `rolling_contact_experiment_spline.py` | Spline feedforward slide (uses pre-learned Z(Y) profile) |
| `rolling_force_config.py` | All tunable parameters for both scripts |
| `read_force_results.py` | Replay and visualise saved `.pkl` results |
| `build_depth_trajectory.py` | Build the Z(Y) spline from prior force-control runs |
| `extract_depth_profile.py` | Extract Z-depth profile from a result for spline fitting |

---

## Motion Sequence

```
(1) approach_pose     ← CONTACT_POSITION_M + APPROACH_HEIGHT_M
      │  move_to  (APPROACH_SPEED_M_S)
(2) contact_pose      ← sensor just touching cylinder
      │  execute_cartesian_traj compress  (position-controlled)
(3) compressed_pose   ← COMPRESS_DEPTH_M below contact
      │  force-acquisition phase: press slowly until fz ≤ FORCE_UPPER_N
      │  RF streaming starts
      │  force-controlled (or spline-tracked) slide at FORCE_CTRL_HZ
      │    Y: y_start + SLIDE_SPEED_M_S × t              (constant)
      │    Z: adjusted by PID on Fz error  (force) or spline  (spline)
(4) compressed_end    ← after slide terminates
      │  RF streaming stops
      │  execute_cartesian_traj decompress  (position-controlled)
(5) surface_end       ← back up COMPRESS_DEPTH_M
      │  move_to approach  (RETURN_SPEED_M_S)
(6) approach_pose     ← ready for next trial
```

Slide terminates on whichever condition fires first:
- Y travel ≥ `roll_distance_m` (computed from `FINRAY_LENGTH_M × ROLL_LENGTH_PERCENT`)
- Elapsed time ≥ `SLIDE_TIMEOUT_S`
- Contact lost: fz rises above `CONTACT_FORCE_THRESHOLD_N`

---

## Force Control (PID variant)

Sign convention: `end_effector_wrench["force"][2]` is **negative** when pressing into the surface; more negative = harder contact.

```
Force dead-band:
  fz > FORCE_UPPER_N  →  f_error = fz − FORCE_UPPER_N  (press down)
  fz < FORCE_LOWER_N  →  f_error = fz − FORCE_LOWER_N  (lift up)
  FORCE_LOWER_N ≤ fz ≤ FORCE_UPPER_N  →  f_error = 0   (hold)

PID output → Z velocity:
  vz_cmd = −(Kp × f_error + Ki × ∫f_error dt + Kd × df_error/dt)
  z_cmd  += vz_cmd × dt      (clamped to acquired_z ± MAX_Z_CORRECTION_M)
  z_cmd velocity also capped at ±MAX_VZ_SPEED_M_S

Y always advances independently:
  y_cmd += SLIDE_SPEED_M_S × dt   (never blocked by Z corrections)
```

### Adaptive Kp

The fin-ray stiffness varies significantly along the slide direction (stiffer near the clamped base). Kp is estimated online each tick:

```
k_est = ΔFz / ΔZ  over last STIFFNESS_WINDOW samples  [N/m]
Kp    = clip(1 / k_est, KP_MIN, KP_MAX)               [m/N]
```

If the Z displacement window is too small (`< STIFFNESS_MIN_DZ_M`), the estimate is noise-dominated and `KP_INITIAL` is used instead.

---

## Spline Feedforward Variant

`rolling_contact_experiment_spline.py` replaces the PID with a pre-learned Z(Y) spline:

1. Run `build_depth_trajectory.py` on one or more prior force-control results.
2. The spline maps `y_traveled_mm → z_world_m`.
3. During the slide, Z tracks the spline directly: `z_cmd = interp(y_traveled, spline_y_mm, spline_z_world_m)`.
4. Fz is still monitored for contact loss.

Spline files live in `depth_profiles/`.

---

## Key Configuration (`rolling_force_config.py`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `CONTACT_POSITION_M` | `[0.531, -0.121, 0.098]` | XYZ where sensor first touches cylinder |
| `COMPRESS_DEPTH_M` | `0.010 m` | Initial position-controlled press depth |
| `SLIDE_SPEED_M_S` | `0.005 m/s` | Constant Y advance rate |
| `FORCE_UPPER_N` | `−25 N` | Upper dead-band edge (press down above this) |
| `FORCE_LOWER_N` | `−26 N` | Lower dead-band edge (lift up below this) |
| `KP_INITIAL` | `0.0002 m/N` | Fallback Kp before stiffness estimate is valid |
| `KP_MIN / KP_MAX` | `0.0001 / 0.0007 m/N` | Adaptive Kp clamp |
| `MAX_Z_CORRECTION_M` | `0.020 m` | Max Z travel from acquired contact depth |
| `MAX_VZ_SPEED_M_S` | `0.002 m/s` | Max Z correction velocity |
| `FORCE_CTRL_HZ` | `50 Hz` | Control loop rate |
| `SERIAL_PORT` | `/dev/ttyACM1` | Teensy serial port |

Set `DRY_RUN = True` to skip all serial/RF and test robot motion only.

---

## Running an Experiment

```bash
cd src/acoustic_sensing/scripts/rolling_testing/force
python rolling_contact_experiment_force.py     # force-PID variant
python rolling_contact_experiment_spline.py    # spline feedforward variant
```

Results are saved in `results/trial_NN_depthX.Xmm/`.

---

## Viewing Results

```bash
python read_force_results.py                    # interactive trial/file picker
python read_force_results.py --fps 10           # slower replay
```

The viewer shows:
- 4 RF channel waveforms with Hilbert envelope and detected contact peak
- Fz tracking plot (target band shaded) vs time
- Force-control diagnostics panel (in-band %, Kp, z correction, f_error)
- EEF pose, velocity, acceleration table
- Phase timeline (compress / slide / decompress)

Both force-PID and spline result files are supported.

---

## Output File Format

Each `.pkl` contains a dict with `"trial"` and `"session"` keys.

**`trial` keys (force-PID):**

| Key | Description |
|-----|-------------|
| `frames` | List of `(4-ch RF arrays, timestamp_s)` |
| `ee_poses_slide` | EEF pose/wrench samples during slide |
| `ee_poses_slide_force` | Force-control telemetry (one entry per tick) |
| `roll_distance_m` | Y distance actually travelled |
| `termination_reason` | `"distance"`, `"timeout"`, or `"contact_lost"` |
| `compress_duration_s` / `decompress_duration_s` | Phase durations |

**`ee_poses_slide_force` entry keys (force-PID):**

```
t, elapsed_slide_s, y_traveled_m, z_cmd, z_correction_m,
fz_measured, fz_target, f_error, kp, in_band, vy_cmd, vz_cmd
```

**`ee_poses_slide_force` entry keys (spline):**

```
t, elapsed_slide_s, y_traveled_m, z_cmd, z_spline_target,
z_error_m, vy_cmd, fz_measured, position_actual
```
