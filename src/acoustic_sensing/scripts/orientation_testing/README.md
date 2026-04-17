# Orientation Experiment

Acoustic sensing experiment where the FR3 robot presses the finray-gripper sensor onto a bar held at various angles. The goal is to characterise how the sensor RF response changes with bar inclination.

At each angle the robot performs `N_REPEATS` press-hold-lift cycles automatically. A live GUI viewer (`live_orientation_view.py`) shows all 4 RF channels, a bar-angle diagram with expected vs. measured contact positions, and EEF metrics in real time.

## Scripts

| Script | Purpose |
|--------|---------|
| `orientation_experiment.py` | Headless experiment runner (terminal prompts) |
| `live_orientation_view.py` | Live pyqtgraph GUI runner — shows RF + angle diagram live |
| `orientation_config.py` | All tunable parameters |
| `read_orientation_results.py` | Replay and visualise saved `.pkl` results |

---

## Motion Sequence (per repeat)

```
(1) approach_pose     ← CONTACT_POSITION_M + APPROACH_HEIGHT_M above
      │  [angle entered once before all N_REPEATS]
      │  move_to contact  (APPROACH_SPEED_M_S)
(2) contact_pose      ← sensor just touching bar
      │  execute_cartesian_traj compress  (position-controlled)
(3) compressed_pose   ← COMPRESS_DEPTH_M below contact
      │  hold phase  (HOLD_AT_CONTACT_SEC)
      │    force control (if HOLD_FORCE_CONTROL):
      │      Z nudged to maintain Fz at the snapshot taken after settling
      │    rotational compliance (if HOLD_TORQUE_ADAPT):
      │      Ry, Rz adapt from torque feedback; Rx=180° always fixed
      │  lift back to contact  (APPROACH_SPEED_M_S)
(4) contact_pose
      │  move_to approach  (RETURN_SPEED_M_S)
(5) approach_pose     ← repeat N_REPEATS times, then wait for bar rotation
```

---

## Hold-Phase Force Control

The robot can be pushed upward by the reaction force during the hold. A P-controller corrects Z each tick to maintain the force level sampled just after compression:

```
Sign convention: Fz < 0 when pressing into surface; more negative = harder.

After settling (HOLD_SETTLE_SEC):
  fz_target = Fz snapshot

Each tick at HOLD_FORCE_CTRL_HZ:
  f_error = Fz − fz_target
  if |f_error| > HOLD_FORCE_DEADBAND_N:
      z_cmd -= HOLD_FORCE_KP × f_error
  z_cmd clamped to compressed_z ± HOLD_FORCE_MAX_Z_CORR_M
```

Set `HOLD_FORCE_CONTROL = False` to use a plain `time.sleep()` hold instead.

---

## Hold-Phase Rotational Compliance

Rather than hardcoding a surface-tilt angle, the robot adapts Ry and Rz from torque feedback during the hold. Rx (≈ 180°) is always kept fixed.

```
Each tick at HOLD_FORCE_CTRL_HZ:
  if HOLD_TORQUE_ADAPT:
      if |Ty| > HOLD_TORQUE_DEADBAND_NM:
          Ry_cmd += HOLD_TORQUE_KP_DEG_PER_NM × Ty
      if |Tz| > HOLD_TORQUE_DEADBAND_NM:
          Rz_cmd += HOLD_TORQUE_KP_DEG_PER_NM × Tz
      Ry_cmd, Rz_cmd clamped to initial ± HOLD_TORQUE_MAX_CORR_DEG

  orientation = Rotation.from_euler("xyz", [Rx_fixed, Ry_cmd, Rz_cmd])
  robot.set_target(pose=Pose([x, y, z_cmd], orientation))
```

This removes the need to hardcode a surface inclination angle — `BASE_ORI_EULER_DEG = [180, 0, 0]` and the robot finds the surface tilt automatically.

---

## Key Configuration (`orientation_config.py`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `CONTACT_POSITION_M` | `[0.750, -0.071, 0.112]` | XYZ where sensor just touches bar |
| `APPROACH_HEIGHT_M` | `0.030 m` | Height above contact to start/return to |
| `COMPRESS_DEPTH_M` | `0.013 m` | How far to press past contact |
| `HOLD_AT_CONTACT_SEC` | `3.0 s` | Hold duration per press |
| `N_REPEATS` | `4` | Number of press-hold-lift cycles per bar angle |
| `APPROACH_SPEED_M_S` | `0.005 m/s` | Descent and compression speed |
| `RETURN_SPEED_M_S` | `0.02 m/s` | Lift and return speed |
| `BASE_ORI_EULER_DEG` | `[180, 0, 0]` | Starting EEF orientation (Rx=180° flips the sensor) |
| `HOLD_FORCE_CONTROL` | `True` | Enable Z force feedback during hold |
| `HOLD_FORCE_KP` | `0.0001 m/N` | Force control proportional gain |
| `HOLD_FORCE_DEADBAND_N` | `0.1 N` | Force dead-band (no correction inside) |
| `HOLD_FORCE_MAX_Z_CORR_M` | `0.010 m` | Max Z correction from compressed depth |
| `HOLD_SETTLE_SEC` | `0.5 s` | Settle time before snapshotting fz_target |
| `HOLD_TORQUE_ADAPT` | `True` | Enable Ry/Rz torque-based compliance |
| `HOLD_TORQUE_KP_DEG_PER_NM` | `2.0 °/Nm` | Rotational compliance gain |
| `HOLD_TORQUE_DEADBAND_NM` | `0.05 Nm` | Torque dead-band |
| `HOLD_TORQUE_MAX_CORR_DEG` | `8.0°` | Max angular correction from initial |
| `SERIAL_PORT` | `/dev/ttyACM1` | Teensy serial port |

Set `DRY_RUN = True` to skip all serial/RF and verify robot motion only.

---

## Running an Experiment

### Headless (terminal)

```bash
cd src/acoustic_sensing/scripts/orientation_testing
python orientation_experiment.py
```

Prompts:
1. Dry run? `[y/N]`
2. Enter bar angle in degrees → N_REPEATS presses happen automatically
3. After all repeats: rotate bar, press Enter for next angle

### Live GUI

```bash
python live_orientation_view.py [--dry-run] [--port /dev/ttyACM0]
```

Toolbar controls:
- **Continue ▶** — replaces all Enter prompts
- Angle input dialog opens automatically before each angle's set of repeats

The GUI shows:
- 4 RF channel waveforms (live, updates each frame)
- Angular diagram: expected contact markers (open circles) and measured Hilbert-peak positions (filled circles)
- EEF metrics: measured vs. commanded XYZ and Euler, Fz, Fy, Fz, velocity

Results are saved automatically after all repeats for each angle.

---

## Viewing Results

```bash
python read_orientation_results.py
python read_orientation_results.py --fps 5   # slower replay
```

Interactive prompts select trial folder and then individual `.pkl` files. After closing a plot, it asks whether to open another file from the same trial, pick a new trial, or quit.

---

## Output File Format

Results are saved in `results/trial_NN_depthX.Xmm/` with two files per repeat:

### `full_exp/angle_A.A_NN.pkl`

Full experiment data:

| Key | Description |
|-----|-------------|
| `angle_deg` | Bar angle entered by user |
| `repeat_idx` / `n_repeats` | Which repeat this is |
| `compress_depth_m` | Compression depth |
| `frames` | Full RF frame list (all channels, all timestamps) |
| `ee_poses` | EEF pose/wrench/vel/acc sampled at EE_SAMPLE_PERIOD_SEC |
| `commanded_ee_poses` | Commanded target pose samples |
| `hold_force_telemetry` | Per-tick force controller log |
| `commanded_segments` | Phase timestamps (compress / hold / lift) |
| `contact_position`, `compressed_position`, `approach_position` | Key poses |

### `20frames/20_angle_A.A_NN.pkl`

Compact file — same as `full_exp` but `ee_poses` is replaced by `ee_poses_during_rf` (only samples within the RF window) and RF is limited to 20 frames.

---

## Physical Setup Notes

- Robot orientation `[180, 0, 0]` flips the sensor face downward.
- The bar is mounted horizontally below the sensor; rotate it manually between angle sessions.
- `CONTACT_POSITION_M` should be jogged and read from the EEF when the sensor just touches the bar at 0° — update this in `orientation_config.py` before each session.
- The surface is physically level; any tilt is compensated automatically by `HOLD_TORQUE_ADAPT`.
