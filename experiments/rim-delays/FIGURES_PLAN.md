# Delayed RIM Teleoperation — Talk Figures

Oct 27, 2026 · from the E1–E5 notebooks (2026-10-26 sessions); E6 pending.

## Verdict

The data carry both halves of the claim, but each half needs the right view.

- **Transparency is set by the model, not the delay: strong.** M3–M5 are flat in T everywhere (E1 within 1.5 N, E2 within ~1 N, E3 88–89 ms to the hold force at every delay, E4 within ~0.5 kg). M1 grows as predicted: K·2A|sin(ωT/2)|/√2 within ~10 % at 0.25–0.5 Hz, and the contact arrives ≈ T late (116 → 2719 ms).
- **Better model, smaller residual: holds once friction is separated out.** RMS e\_F is friction-dominated (E1 floor ~9 N heavy / ~4.6 N light, E2 ~11 N tangential, E4 M4 10.3 vs M5 9.3 N), so RMS e\_F alone barely separates M4 from M5. The views that do separate them: E2's mean radial e\_F and felt force against phase, and E4's phase-separated masses (m\_err RMS M4 7.4 kg vs M5 0.85–1.24 kg).
- **E5 does not deliver its planned plot yet.** E5-lite has 6 steps. M1 was stable at K = 1000 held at 850 ms and at 2.6 s, so there is no falling K\_max(T) line. M5 at 2.6 s was stable at 2000 held and unstable at 2000 released. The dropped-handle ladders (EXP\_PLAN E5, added 2026-10-27, ~15 min) are meant to fill this gap.
- **Unmodelled dynamics are an honest "limits" slide:** the arm is ~1.43× the held model (E1, both directions; E4 ×1.26 near the base → ×1.03 at reach; rotor inertia is a likely candidate), plus Coulomb friction.

## Main figures (story order)

| # | Slide message | Source | Content | Changes from the notebook |
| --- | --- | --- | --- | --- |
| F1 | P-P degrades with delay; local models don't | E1 cell 5 | RMS e\_F vs T, 4 methods, **heavy 0.25 Hz** (monotonic M1: 0.2 → 3 → 42 → 59 N) | One panel, large fonts. Y label "unfelt force e\_F". Avoid 0.5 Hz (non-monotonic M1) and the 5G 1 Hz points (chatter guard inflated them). |
| F2 | Under delay, contact arrives a round trip late, after a drag | E3 cell 13, T = 850 ms, 5 cm/s | Height and force time series, **M1 vs M5 only** (2 columns) | Drop M3/M4 columns and the metric boxes. Annotate "26 N drag", "969 ms vs 88 ms". |
| F3 | Same, across delays | E3 cell 9 | Time to the hold force vs T (+ drag at crossing), 4 methods | Keep 2 of 5 panels. Drop onset, overshoot (1.085 vs 1.0875 axis: noise) and robot peak. |
| F4 | A fixed mass cannot follow direction-dependent inertia | E2 cell 11 | Felt radial force vs circle phase: M5 held follows u'Λu·a (2f), M4 flat, M5 free ~7× too light, M3 zero | As is; maybe drop M3 (zero line). Optional inset: mean radial e\_F ladder M3 8.3 → M5-free 7.0 → M4 3.6 → M5 0.8 N (T = 0, 2.6 s side by side). |
| F5 | A mass tuned in one configuration is wrong elsewhere; RIM follows the arm | E4 cell 6 (+ cell 8 left) | Mass along the line: arm measured, M4 rendered (flat 16 kg), M5 rendered, model n'Λn | **The strongest model-fidelity figure.** One panel at T = 2.6 s (T = 0 dashed, or note "same at T = 0"). Optional second panel: m\_err (M4 −1.2 → −9.8 kg, M5 ~−1 kg). |
| F6 | What the model misses: friction and ~1.4× inertia | E1 cell 15 | Re Z vs ω², heavy and light, model dashed | Annotate fitted m (20.7 vs 14.5 kg; 6.8 vs 4.8 kg) and the Coulomb floor. Frame it as M5's honest floor. |
| F7 | Demo: contact with the hand in the loop | E6 | Video, 3 clips (M1 T = 0, M1 and M5 at 2.6 s), `tools/contact_video.py --mode side` | After E6 runs. Pair with a small tap/hold metrics table (hold force, time to hold, track\_rms\_mm). |

E5 → once the dropped ladders are run: **F6b, "Without a hand, P-P under delay throws the handle; RIM holds it"**, from `e5_kmax.ipynb` (dropped handle, height and felt force from the first touch, M1 vs M5 at K = 1000, 2.6 s), with M1's K\_max per delay as a caption or the K\_max plot as backup. If M1 turns out stable when dropped, fall back to one line: "At the frozen K = 1000 N/m, M5 stays stable at 2.6 s (and at 2000 N/m with the handle held)", and do not claim a K\_max(T) trend.

## Backup figures

- E1 cell 11: M1 measured vs delayed-sine prediction (answers "why is M1 non-monotonic in T").
- E1 cell 7: e\_F vs frequency (M3 grows with ω², M4/M5 cut it by ~⅓ at 1 Hz: 23.6 vs 34.2 N heavy).
- E1 cell 13: coupling-resonance amplitude ratio (caveat for 1 Hz heavy: methods on different motions).
- E2 cell 13: robot radial/tangential force vs phase (why the 2f e\_F is friction, not inertia).
- E3 cell 7: speed sweep at 850 ms (M1 drag ~30 N at every speed; M4/M5 inertial drag at 20 cm/s).
- E3 cell 11: steady approach (M3–M5 coincide at contact; momentum goes into the table).

## Fixes before rendering

1. **E3 steady runs missing from the saved output.** `RUNS = [latest_session("e3")]` loads 28 runs; the discussion quotes 37 and the steady time series prints "no runs". Add the steady-sweep session to `RUNS` and re-run before using cell 11 or the steady time series.
2. **One run per condition** in E1, E2 and E4 (E3: 3 contacts per run, < 10 ms spread). T = 0 vs 2.6 s agreement serves as a repeat for M3–M5. Say so on the slide rather than drawing error bars.
3. **Slide styling:** one message per figure, ≥ 14 pt fonts, the existing method colours (`plots.color`), "e\_F" written out as "unfelt force" on first use, contact-limiter ✕ marks only where a 20 cm/s point is shown.
4. **Label force sources honestly:** robot force is F\_osc (`task_wrench`), not a sensor (EXP\_PLAN protocol).
