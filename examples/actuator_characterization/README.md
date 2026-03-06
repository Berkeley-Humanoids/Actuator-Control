# Actuator Characterization Test Plan (Torque Stand) — Updated

## Fixed experiment choices
- **Position-command max amplitude:** ±3.14 rad
- **Amplitude set (percent of max):** [10%, 25%, 50%, 100%]
  - 10% → ±0.314 rad
  - 25% → ±0.785 rad
  - 50% → ±1.570 rad
  - 100% → ±3.140 rad
- **Frequency set:** [0.25, 0.5, 1.0, 2.0] Hz
- **Per-test duration:** 8 s (active excitation)

---

## Goal
Collect data to fit:
- **ActuatorNet-style supervised model:** map command/state history → real output torque
- **UAN-style simulator correction model:** map history → corrective torque / improved transitions for sim2real

---

## Hardware setup
- Actuator drives controllable brake (constant-load setpoints per block)
- Shaft torque sensor (ground-truth torque)
- Encoders for angle/velocity (motor-side and/or output-side)

---

## Signals to command and log
**Logging:** ≥1 kHz preferred (≥500 Hz acceptable). Timestamp everything; ensure synchronization.

### Commanded
- `u_mode` (position / torque / current / impedance)
- **Position tests:** `q_cmd(t)` (rad)
- If impedance: `kp_cmd`, `kd_cmd`
- Brake: `τ_brake_cmd` (Nm) and (if available) `τ_brake_meas`

### Measured
- `θ_m`, `ω_m` (motor angle/velocity)
- `θ_out`, `ω_out` (output angle/velocity) if available
- `τ_meas` (torque sensor)
- Strongly recommended: `i_meas`, `V_bus`, temperatures (motor/driver/gearbox/brake)

### Derived (offline)
- Position error: `e_t = θ_out,t − q_cmd,t` (or motor-side proxy if needed)
- History windows for learning (short + longer)
- Transition tuples: `(s_t, u_t, s_{t+1})` for residual/UAN-style training

---

## Pre-flight (each session)
1. Zero torque sensor with motor disabled + brake disengaged.
2. Verify encoder reference/zero.
3. Brake check: slow sweep of `τ_brake_cmd` to verify stable behavior.
4. Log 2–3 min baseline temperatures idle.

---

## Safety / limits
- Enforce conservative bounds: `|q_cmd| ≤ 3.14 rad`, plus actuator current/torque and speed limits.
- E-stop if: torque sensor saturates, strong oscillation, overheating, or brake slip.

---

## Load conditions (brake levels)
Choose 5–8 constant levels spanning:
- ~0 (near free)
- light (10–20% rated)
- medium (40–60%)
- heavy (80–100%, use short runs if needed)

Hold brake torque constant during each excitation grid.

---

## Excitation grid (per brake level)
For each waveform type you run (square and/or sine), use the same grid:

### Grid definition
- Frequencies: **0.25, 0.5, 1.0, 2.0 Hz**
- Amplitudes: **±0.314, ±0.785, ±1.570, ±3.140 rad**
- Duration per (freq, amp): **8 s**

**Cycles per 8 s**
- 0.25 Hz: 2 cycles
- 0.5 Hz: 4 cycles
- 1.0 Hz: 8 cycles
- 2.0 Hz: 16 cycles

### Recommended rest between tests
- 2 s at `q_cmd = 0` (or a safe neutral position) between tests for repeatability.

### Suggested ordering
- Run low amplitude → high amplitude at a fixed frequency, then increase frequency
- Run 100% amplitude last; if it induces saturation/hard-stops, keep it only for ≤1 Hz or treat as a separate “limit test”

---

## Optional: stochastic excitation block (for transition coverage)
(Recommended if you will do UAN/residual learning)

- Position-command noise / PRBS: sample `q_cmd` from a bounded distribution and hold for `Δt_hold`
- Use hold times: 5–400 ms (log-spaced subset OK)
- Clip to ±3.14 rad; optionally low-pass command (5–10 ms) and log both raw+filtered

---

## Session sequencing (per brake level)
1. Warm-up: 30 s small sine (e.g., 10%, 0.5 Hz)
2. Sine grid (all amps × freqs) — 16 tests × 8 s = 128 s active
3. Square grid (optional, same grid) — +128 s active
4. Stochastic block (optional) — 1–5 min
5. Cool-down: 60 s at `q_cmd=0` (keep logging)

With 2 s rest between the 16 tests:
- 16 × (8 + 2) = 160 s per waveform per brake level

---

## Data packaging (deliverables)
- Time-synced logs (CSV/Parquet/HDF5):
  - `t, q_cmd, θ_out, ω_out, τ_meas, τ_brake_cmd, temps, V_bus, i_meas, ...`
- Run metadata:
  - brake level, waveform type, frequency, amplitude, PD gains (if any), filters used
- Quality plots:
  - tracking (q_cmd vs θ_out), torque response (τ_meas), drift vs temperature
- Train/val/test split:
  - hold out at least one brake level (and optionally one waveform type) as test
