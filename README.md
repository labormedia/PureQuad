# PureQuad

A complete 4-input (Thrust / Roll / Pitch / Yaw) quadcopter rigid-body
simulation written entirely in Pure Data, using `expr~` and `fexpr~` for
sample-rate physics integration. Twelve states — position, velocity, Euler
angles and body angular rates — are integrated under gravity at audio rate.

No visualization: all inputs and outputs are plain number boxes.
Computation is optimized over UX — each coupled ODE block is a single
multi-expression `fexpr~`, so the whole 6DOF model evaluates in two
per-sample expression passes with no DSP-graph feedback objects.

## Requirements

Pure Data vanilla (≥ 0.51 recommended; `expr`/`expr~`/`fexpr~` ship with it).

## Usage

1. Open `pd/quadcopter.pd`.
2. Turn DSP on (the `; pd dsp 1` message at top right, or Pd's Media menu).
3. Fly with the four control number boxes:
   - **Thrust T (N)** — total commanded thrust (default 11.772 N ≈ hover for the default 1.2 kg craft)
   - **Roll τx (N·m)** — body-x torque command
   - **Pitch τy (N·m)** — body-y torque command
   - **Yaw τz (N·m)** — body-z torque command
4. Watch the 12 state number boxes update every 50 ms.
5. Open the `[params]` abstraction to change the craft; bang **init** there to
   re-send all parameters *and reset the simulation to its initial conditions*.

## Parameters (all numeric, in `pd/params.pd`)

| Send name | Meaning | Default |
|---|---|---|
| `pq_mass` | mass (kg) | 1.2 |
| `pq_Ixx` `pq_Iyy` `pq_Izz` | inertia diagonal (kg·m²) | 0.015 / 0.015 / 0.024 |
| `pq_L` | actuator (arm) distance from CG (m) | 0.225 |
| `pq_km` | yaw torque-per-thrust coefficient (m) | 0.016 |
| `pq_g` | gravity (m/s²) | 9.81 |
| `pq_Tmin` `pq_Tmax` | per-motor thrust range (N) | 0 / 8 |
| `pq_Tslope` | actuator gain (commanded → produced thrust) | 1.0 |
| `pq_x0` `pq_y0` `pq_z0` | initial position (m) | 0 / 0 / 1 |
| `pq_vx0` `pq_vy0` `pq_vz0` | initial velocity (m/s) | 0 |
| `pq_phi0` `pq_theta0` `pq_psi0` | initial roll/pitch/yaw (rad) | 0 |

## Model

Frames: world z-up; body x forward, y left, z up. ZYX (yaw–pitch–roll)
Euler angles φ θ ψ; body rates p q r. Explicit Euler integration with
dt = 1/samplerate.

**Motor allocation** (X configuration — M1 FR/CW, M2 RL/CW, M3 FL/CCW, M4 RR/CCW),
in `pd/mixer.pd`:

```
Fi = clip( slope · (T ± τx/L ± τy/L ± τz/km) / 4 ,  Tmin, Tmax )
```

Saturation happens per motor, and `pd/force_torque.pd` re-derives the body
force/torques from the *clamped* thrusts — so thrust-range limits correctly
distort the torques the dynamics see, as on a real craft.

**Rotational** (`ROT` fexpr~ in `pd/dynamics.pd`), with gyroscopic coupling:

```
ṗ = (Tx + (Iyy−Izz)·q·r) / Ixx          φ̇ = p + sinφ·tanθ·q + cosφ·tanθ·r
q̇ = (Ty + (Izz−Ixx)·p·r) / Iyy          θ̇ = cosφ·q − sinφ·r
ṙ = (Tz + (Ixx−Iyy)·p·q) / Izz          ψ̇ = (sinφ·q + cosφ·r) / cosθ
```

**Translational** (`TRANS` fexpr~), thrust along body z under gravity:

```
ax = (cosψ·sinθ·cosφ + sinψ·sinφ) · Fz/m
ay = (sinψ·sinθ·cosφ − cosψ·sinφ) · Fz/m
az = cosθ·cosφ · Fz/m − g
```

## Repository layout

```
pd/quadcopter.pd     top-level patch: controls, chain, numeric state display
pd/params.pd         all parameters as number boxes + global sends; init/reset
pd/mixer.pd          T/R/P/Y → per-motor thrusts (slope + range clamp)
pd/force_torque.pd   clamped thrusts → body Fz, Tx, Ty, Tz
pd/dynamics.pd       two fexpr~ blocks integrating the 12-state model
tools/gen_patch.py   generator that emits all .pd files (the actual source of truth)
```

The `.pd` files are generated — edit `tools/gen_patch.py` and re-run
`python3 tools/gen_patch.py` rather than hand-editing patches.

## Known limitations

- Explicit Euler at 44.1/48 kHz: very accurate for rigid-body timescales, but
  no energy conservation guarantee over very long runs.
- Euler-angle singularity at θ = ±90° (guarded numerically, not resolved —
  no quaternions).
- No aerodynamic drag, motor lag, or ground contact; the craft falls through z = 0.
- `fexpr~` evaluates per sample: expect a few % CPU per block on a modern machine.
