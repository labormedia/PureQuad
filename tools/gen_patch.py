#!/usr/bin/env python3
"""Generate Pure Data patches for PureQuad — 4D quadcopter simulation.

Design notes
============
expr-family inlet conventions (Pd vanilla "expr" 0.5x):
  expr~ : $v<n> = signal inlet (vector), $f<n> = float inlet
  fexpr~: $x<n>[k] = input sample (k <= 0), $y<n>[k] = previous OUTPUT sample
          (k < 0), $f<n> = float inlet
  The digit IS the inlet number — indices must be unique across all variable
  types in one object. Max 9 inlets, multiple ';'-separated expressions give
  multiple outlets.

Feedback strategy: Pd forbids DSP graph cycles, so each coupled ODE block is
ONE multi-expression fexpr~ whose state feedback is internal via $y<n>[-1].
  - rotational fexpr~ : p q r phi theta psi   (torques in, angles out)
  - translational fexpr~: vx vy vz x y z      (thrust+angles in, position out)
This is also the fastest layout: one per-sample evaluation per block, no
redundant object-graph overhead.

State is reset with "set y<n> <val>" messages to the fexpr~ left inlet,
driven by the parameter [send]s, so banging "init" in params.pd re-seeds
the initial conditions.

Physics
=======
Frames: world ENU-style (z up), body x forward / y left / z up.
ZYX (yaw-pitch-roll) Euler angles phi theta psi; rates p q r in body frame.

Rotational (with gyroscopic cross-coupling):
  p' = (Tx + (Iyy-Izz) q r) / Ixx
  q' = (Ty + (Izz-Ixx) p r) / Iyy
  r' = (Tz + (Ixx-Iyy) p q) / Izz
  phi'   = p + sin(phi) tan(theta) q + cos(phi) tan(theta) r
  theta' = cos(phi) q - sin(phi) r
  psi'   = (sin(phi) q + cos(phi) r) / cos(theta)

Translational (thrust along body z, gravity along world -z):
  ax = (cos(psi) sin(theta) cos(phi) + sin(psi) sin(phi)) Fz/m
  ay = (sin(psi) sin(theta) cos(phi) - cos(psi) sin(phi)) Fz/m
  az = cos(theta) cos(phi) Fz/m - g

Motor layout (X config, top view), CW/CCW for yaw reaction:
       M3 FL CCW     M1 FR CW
              \\      /
               \\    /
               /    \\
              /      \\
       M2 RL CW      M4 RR CCW

Mixer (inverse allocation, slope = actuator gain, clamp = thrust range):
  F1 = clip(slope*(T - tx/L + ty/L - tz/km)/4, Tmin, Tmax)
  F2 = clip(slope*(T + tx/L - ty/L - tz/km)/4, Tmin, Tmax)
  F3 = clip(slope*(T + tx/L + ty/L + tz/km)/4, Tmin, Tmax)
  F4 = clip(slope*(T - tx/L - ty/L + tz/km)/4, Tmin, Tmax)
Forward force/torque (so saturation feeds back into the dynamics):
  Fz = F1+F2+F3+F4
  Tx = L (F2+F3-F1-F4),  Ty = L (F1+F3-F2-F4),  Tz = km (F3+F4-F1-F2)
"""

import os
import re

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pd")
os.makedirs(OUT, exist_ok=True)


def esc(text):
    """Escape $ , ; for the .pd file format (object/message box text)."""
    text = text.replace("\\", "")          # no stray backslashes expected
    text = text.replace("$", "\\$")
    text = text.replace(",", " \\, ")
    text = text.replace(";", " \\; ")
    return re.sub(r"\s+", " ", text).strip()


class Patch:
    def __init__(self, w=900, h=700):
        self.w, self.h = w, h
        self._objs, self._conns = [], []

    def _add(self, kind, x, y, body):
        self._objs.append((kind, x, y, body))
        return len(self._objs) - 1

    def obj(self, x, y, body):
        """body is escaped (use for expr~/fexpr~/anything with $ , ;)."""
        return self._add("obj", x, y, esc(body))

    def raw_obj(self, x, y, body):
        """body written verbatim (gui objects with fixed syntax)."""
        return self._add("obj", x, y, body)

    def msg(self, x, y, body):
        return self._add("msg", x, y, esc(body))

    def text(self, x, y, body):
        return self._add("text", x, y, esc(body))

    def floatatom(self, x, y, w=9, lo=0, hi=0):
        return self._add("floatatom", x, y, f"{w} {lo} {hi} 0 - - -")

    def connect(self, s, so, d, di):
        assert 0 <= s < len(self._objs) and 0 <= d < len(self._objs)
        self._conns.append((s, so, d, di))

    def save(self, name):
        lines = [f"#N canvas 60 60 {self.w} {self.h} 10;"]
        lines += [f"#X {k} {x} {y} {b};" for k, x, y, b in self._objs]
        lines += [f"#X connect {s} {so} {d} {di};" for s, so, d, di in self._conns]
        path = os.path.join(OUT, name)
        with open(path, "w") as f:
            f.write("\n".join(lines) + "\n")
        print(f"  {path}")


BNG = "bng 18 250 50 0 empty empty empty 17 7 0 10 -262144 -1 -1"

# parameter table: (label, send-name, default)
PARAMS = [
    ("mass (kg)",          "pq_mass",   1.2),
    ("Ixx (kg.m2)",        "pq_Ixx",    0.015),
    ("Iyy (kg.m2)",        "pq_Iyy",    0.015),
    ("Izz (kg.m2)",        "pq_Izz",    0.024),
    ("arm length L (m)",   "pq_L",      0.225),
    ("yaw coeff km (m)",   "pq_km",     0.016),
    ("gravity g (m/s2)",   "pq_g",      9.81),
    ("thrust min/motor N", "pq_Tmin",   0.0),
    ("thrust max/motor N", "pq_Tmax",   8.0),
    ("thrust slope",       "pq_Tslope", 1.0),
    ("x0 (m)",             "pq_x0",     0.0),
    ("y0 (m)",             "pq_y0",     0.0),
    ("z0 (m)",             "pq_z0",     1.0),
    ("vx0 (m/s)",          "pq_vx0",    0.0),
    ("vy0 (m/s)",          "pq_vy0",    0.0),
    ("vz0 (m/s)",          "pq_vz0",    0.0),
    ("phi0 (rad)",         "pq_phi0",   0.0),
    ("theta0 (rad)",       "pq_theta0", 0.0),
    ("psi0 (rad)",         "pq_psi0",   0.0),
]


def gen_params():
    """params.pd — every quadcopter parameter as a numeric box + global send.

    loadbang (or the init bang) pushes defaults through the number boxes,
    which both displays them and broadcasts on the pq_* send names.
    Re-banging init also re-seeds the integrator initial conditions.
    """
    p = Patch(760, 280)
    p.text(10,  5, "PARAMS - edit boxes or bang init to (re)send defaults")
    p.text(10, 17, "banging init also RESETS the simulation state to x0/v0/angles0")

    lb = p.obj(10, 38, "loadbang")
    bn = p.raw_obj(90, 38, BNG)
    p.text(115, 40, "init / reset")
    p.connect(lb, 0, bn, 0)

    COLS = [10, 260, 510]
    Y0, DY = 68, 26
    for i, (label, sname, default) in enumerate(PARAMS):
        x = COLS[i % 3]
        y = Y0 + (i // 3) * DY
        p.text(x, y + 2, label)
        nb = p.floatatom(x + 130, y, 9, -1e9, 1e9)
        m  = p.msg(x + 205, y, repr(default))
        sd = p.obj(x + 205, y + 12, f"s {sname}")
        # wiring: bang -> default msg -> number box -> send
        p.connect(bn, 0, m, 0)
        p.connect(m, 0, nb, 0)
        p.connect(nb, 0, sd, 0)

    p.save("params.pd")


def gen_mixer():
    """mixer.pd — T tx ty tz (signals) -> clamped per-motor thrusts F1..F4.

    One expr~ per motor: 4 signal inlets + 5 float inlets (L km slope Tmin
    Tmax) = 9, the expr maximum. Slope is applied before the clamp so the
    thrust range always bounds the physical motor output.
    """
    p = Patch(980, 330)
    p.text(10,  5, "MIXER: T tx ty tz -> F1..F4 (slope applied then clamped to thrust range)")
    p.text(10, 17, "X config: M1 FR/CW M2 RL/CW M3 FL/CCW M4 RR/CCW")

    iT  = p.obj(10,  45, "inlet~"); p.text(10,  30, "T (N)")
    itx = p.obj(90,  45, "inlet~"); p.text(90,  30, "tx (N.m)")
    ity = p.obj(170, 45, "inlet~"); p.text(170, 30, "ty (N.m)")
    itz = p.obj(250, 45, "inlet~"); p.text(250, 30, "tz (N.m)")

    rL  = p.obj(340, 45, "r pq_L")
    rkm = p.obj(420, 45, "r pq_km")
    rsl = p.obj(500, 45, "r pq_Tslope")
    rmn = p.obj(600, 45, "r pq_Tmin")
    rmx = p.obj(690, 45, "r pq_Tmax")

    # signs of (tx/L, ty/L, tz/km) per motor
    SIGNS = [("-", "+", "-"),   # F1 FR CW
             ("+", "-", "-"),   # F2 RL CW
             ("+", "+", "+"),   # F3 FL CCW
             ("-", "-", "+")]   # F4 RR CCW

    outs = []
    for i, (sx, sy_, sz) in enumerate(SIGNS):
        e = p.obj(10, 85 + i * 32,
                  f"expr~ max($f8, min($f9, $f7 * ($v1 "
                  f"{sx} $v2 / max($f5, 0.0001) "
                  f"{sy_} $v3 / max($f5, 0.0001) "
                  f"{sz} $v4 / max($f6, 0.0001)) * 0.25))")
        for src, inl in ((iT, 0), (itx, 1), (ity, 2), (itz, 3),
                         (rL, 4), (rkm, 5), (rsl, 6), (rmn, 7), (rmx, 8)):
            p.connect(src, 0, e, inl)
        outs.append(e)

    names = ["F1 FR", "F2 RL", "F3 FL", "F4 RR"]
    for i, (e, nm) in enumerate(zip(outs, names)):
        o = p.obj(10 + i * 90, 240, "outlet~")
        p.connect(e, 0, o, 0)
        p.text(10 + i * 90, 258, nm)

    p.save("mixer.pd")


def gen_force_torque():
    """force_torque.pd — F1..F4 (signals) -> body Fz Tx Ty Tz (signals).

    Forward allocation AFTER motor clamping, so saturation correctly
    distorts the torques the dynamics actually see.
    """
    p = Patch(700, 300)
    p.text(10, 5, "FORCE-TORQUE: clamped motor thrusts -> total thrust + body torques")

    ins = []
    for i, nm in enumerate(["F1 FR", "F2 RL", "F3 FL", "F4 RR"]):
        o = p.obj(10 + i * 90, 45, "inlet~")
        p.text(10 + i * 90, 30, nm)
        ins.append(o)
    rL  = p.obj(420, 45, "r pq_L")
    rkm = p.obj(490, 45, "r pq_km")

    Fz = p.obj(10,  90, "expr~ $v1 + $v2 + $v3 + $v4")
    Tx = p.obj(10, 120, "expr~ $f5 * ($v2 + $v3 - $v1 - $v4)")
    Ty = p.obj(10, 150, "expr~ $f5 * ($v1 + $v3 - $v2 - $v4)")
    Tz = p.obj(10, 180, "expr~ $f5 * ($v3 + $v4 - $v1 - $v2)")

    for e in (Fz, Tx, Ty, Tz):
        for j, src in enumerate(ins):
            p.connect(src, 0, e, j)
    p.connect(rL,  0, Tx, 4)
    p.connect(rL,  0, Ty, 4)
    p.connect(rkm, 0, Tz, 4)

    for i, (e, nm) in enumerate(zip((Fz, Tx, Ty, Tz),
                                    ("Fz (N)", "Tx (N.m)", "Ty (N.m)", "Tz (N.m)"))):
        o = p.obj(10 + i * 100, 230, "outlet~")
        p.connect(e, 0, o, 0)
        p.text(10 + i * 100, 248, nm)

    p.save("force_torque.pd")


def gen_dynamics():
    """dynamics.pd — Fz Tx Ty Tz -> 12-state output, Euler-integrated per sample.

    Two multi-expression fexpr~ blocks; all ODE feedback is internal via
    $y<n>[-1], so there are no DSP graph cycles:

      ROT  (in: Tx Ty Tz | f: Ixx Iyy Izz dt)      -> p q r phi theta psi
      TRANS(in: Fz phi theta psi | f: m g dt)      -> vx vy vz x y z

    Initial conditions arrive as "set y<n> <val>" messages whenever the
    matching pq_* parameter is (re)sent — banging init in params.pd resets
    the whole simulation.
    """
    p = Patch(1080, 620)
    p.text(10,  5, "DYNAMICS - 6DOF rigid body, explicit Euler at audio rate")
    p.text(10, 17, "outputs: x y z vx vy vz phi theta psi p q r")

    iFz = p.obj(10,  48, "inlet~"); p.text(10,  33, "Fz")
    iTx = p.obj(80,  48, "inlet~"); p.text(80,  33, "Tx")
    iTy = p.obj(150, 48, "inlet~"); p.text(150, 33, "Ty")
    iTz = p.obj(220, 48, "inlet~"); p.text(220, 33, "Tz")

    # dt = 1/samplerate, computed once at load (control rate, float)
    lb  = p.obj(310, 48, "loadbang")
    sr  = p.obj(310, 70, "samplerate~")
    dte = p.obj(310, 92, "expr 1.0 / $f1")
    p.connect(lb, 0, sr, 0)
    p.connect(sr, 0, dte, 0)

    rIxx  = p.obj(420, 48, "r pq_Ixx")
    rIyy  = p.obj(500, 48, "r pq_Iyy")
    rIzz  = p.obj(580, 48, "r pq_Izz")
    rmass = p.obj(660, 48, "r pq_mass")
    rg    = p.obj(740, 48, "r pq_g")

    # ── ROT: 6 expressions / 6 outlets ──────────────────────────────────────
    # inlets: $x1=Tx $x2=Ty $x3=Tz  $f4=Ixx $f5=Iyy $f6=Izz $f7=dt
    # y1=p y2=q y3=r y4=phi y5=theta y6=psi
    p.text(10, 135, "ROT: p q r phi theta psi (gyroscopic coupling + ZYX Euler kinematics)")
    rot = p.obj(10, 152,
        "fexpr~ "
        "$y1[-1] + ($x1[0] + ($f5 - $f6) * $y2[-1] * $y3[-1]) / max($f4, 0.000001) * $f7;"
        "$y2[-1] + ($x2[0] + ($f6 - $f4) * $y1[-1] * $y3[-1]) / max($f5, 0.000001) * $f7;"
        "$y3[-1] + ($x3[0] + ($f4 - $f5) * $y1[-1] * $y2[-1]) / max($f6, 0.000001) * $f7;"
        "$y4[-1] + ($y1[-1] + sin($y4[-1]) * tan($y5[-1]) * $y2[-1]"
        " + cos($y4[-1]) * tan($y5[-1]) * $y3[-1]) * $f7;"
        "$y5[-1] + (cos($y4[-1]) * $y2[-1] - sin($y4[-1]) * $y3[-1]) * $f7;"
        "$y6[-1] + (sin($y4[-1]) * $y2[-1] + cos($y4[-1]) * $y3[-1])"
        " / max(cos($y5[-1]), 0.001) * $f7")
    p.connect(iTx, 0, rot, 0)
    p.connect(iTy, 0, rot, 1)
    p.connect(iTz, 0, rot, 2)
    p.connect(rIxx, 0, rot, 3)
    p.connect(rIyy, 0, rot, 4)
    p.connect(rIzz, 0, rot, 5)
    p.connect(dte, 0, rot, 6)

    # ── TRANS: 6 expressions / 6 outlets ────────────────────────────────────
    # inlets: $x1=Fz $x2=phi $x3=theta $x4=psi  $f5=mass $f6=g $f7=dt
    # y1=vx y2=vy y3=vz y4=x y5=y y6=z
    p.text(10, 255, "TRANS: vx vy vz x y z (body-z thrust rotated to world, minus gravity)")
    trans = p.obj(10, 272,
        "fexpr~ "
        "$y1[-1] + (cos($x4[0]) * sin($x3[0]) * cos($x2[0])"
        " + sin($x4[0]) * sin($x2[0])) * $x1[0] / max($f5, 0.0001) * $f7;"
        "$y2[-1] + (sin($x4[0]) * sin($x3[0]) * cos($x2[0])"
        " - cos($x4[0]) * sin($x2[0])) * $x1[0] / max($f5, 0.0001) * $f7;"
        "$y3[-1] + (cos($x3[0]) * cos($x2[0]) * $x1[0] / max($f5, 0.0001) - $f6) * $f7;"
        "$y4[-1] + $y1[-1] * $f7;"
        "$y5[-1] + $y2[-1] * $f7;"
        "$y6[-1] + $y3[-1] * $f7")
    p.connect(iFz, 0, trans, 0)
    p.connect(rot, 3, trans, 1)   # phi
    p.connect(rot, 4, trans, 2)   # theta
    p.connect(rot, 5, trans, 3)   # psi
    p.connect(rmass, 0, trans, 4)
    p.connect(rg,    0, trans, 5)
    p.connect(dte,   0, trans, 6)

    # ── initial-condition resets via set messages ───────────────────────────
    p.text(10, 375, "initial conditions: pq_* sends -> set y<n> messages reset fexpr~ state")
    IC = [  # (param send, target fexpr~, state slot)
        ("pq_phi0",   rot,   4), ("pq_theta0", rot,   5), ("pq_psi0", rot,   6),
        ("pq_vx0",    trans, 1), ("pq_vy0",    trans, 2), ("pq_vz0",  trans, 3),
        ("pq_x0",     trans, 4), ("pq_y0",     trans, 5), ("pq_z0",   trans, 6),
    ]
    for i, (sname, target, slot) in enumerate(IC):
        x = 10 + (i % 5) * 210
        y = 395 + (i // 5) * 52
        r = p.obj(x, y, f"r {sname}")
        m = p.msg(x, y + 22, f"set y{slot} $1")
        p.connect(r, 0, m, 0)
        p.connect(m, 0, target, 0)
    # p q r restart at 0 whenever phi0 arrives (full attitude-rate reset)
    rz = p.obj(10, 505, "r pq_phi0")
    mz = p.msg(10, 527, "set y1 0, set y2 0, set y3 0")
    p.connect(rz, 0, mz, 0)
    p.connect(mz, 0, rot, 0)

    # ── outlets: x y z vx vy vz phi theta psi p q r ─────────────────────────
    ORDER = [(trans, 3, "x"), (trans, 4, "y"), (trans, 5, "z"),
             (trans, 0, "vx"), (trans, 1, "vy"), (trans, 2, "vz"),
             (rot, 3, "phi"), (rot, 4, "theta"), (rot, 5, "psi"),
             (rot, 0, "p"), (rot, 1, "q"), (rot, 2, "r")]
    for i, (src, outn, lbl) in enumerate(ORDER):
        o = p.obj(10 + i * 85, 565, "outlet~")
        p.connect(src, outn, o, 0)
        p.text(10 + i * 85, 583, lbl)

    p.save("dynamics.pd")


def gen_main():
    """quadcopter.pd — top level: params, 4 controls, chain, numeric readout."""
    p = Patch(1060, 640)
    p.text(10,  4, "PUREQUAD - Pure Data 4D quadcopter simulation (expr~ / fexpr~)")
    p.text(10, 16, "1. turn DSP ON  2. fly with Thrust / Roll / Pitch / Yaw  3. bang init in [params] to reset")
    p.text(10, 28, "physics at audio rate - numeric state display every 50 ms")

    # DSP switches
    don  = p.msg(720, 4, "; pd dsp 1")
    doff = p.msg(800, 4, "; pd dsp 0")
    p.text(880, 6, "<- DSP on/off")

    par = p.obj(10, 52, "params")
    p.text(70, 54, "<- all quadcopter parameters (open it / bang init to reset)")

    # ── controls ─────────────────────────────────────────────────────────────
    p.text(10, 84, "CONTROLS")
    CTRL = [  # (label, lo, hi, default) — default thrust = hover for m=1.2 g=9.81
        ("Thrust T (N)",    0,  32, 11.772),
        ("Roll tx (N.m)",  -4,   4, 0),
        ("Pitch ty (N.m)", -4,   4, 0),
        ("Yaw tz (N.m)",   -4,   4, 0),
    ]
    lbc = p.obj(950, 100, "loadbang")
    sigs = []
    for i, (lbl, lo, hi, dflt) in enumerate(CTRL):
        x = 10 + i * 235
        p.text(x, 100, lbl)
        nb = p.floatatom(x, 116, 9, lo, hi)
        sg = p.obj(x, 138, "sig~")
        m  = p.msg(x + 105, 116, repr(dflt))
        p.connect(lbc, 0, m, 0)
        p.connect(m, 0, nb, 0)
        p.connect(nb, 0, sg, 0)
        sigs.append(sg)

    # ── chain: mixer -> force_torque -> dynamics ────────────────────────────
    mix = p.obj(10, 175, "mixer")
    ft  = p.obj(10, 200, "force_torque")
    dyn = p.obj(10, 225, "dynamics")
    p.text(120, 177, "T/R/P/Y -> per-motor thrusts (slope + range clamp)")
    p.text(120, 202, "motor thrusts -> body Fz Tx Ty Tz")
    p.text(120, 227, "6DOF integration -> 12-state output")
    for i, sg in enumerate(sigs):
        p.connect(sg, 0, mix, i)
    for i in range(4):
        p.connect(mix, i, ft, i)
    for i in range(4):
        p.connect(ft, i, dyn, i)

    # ── numeric state readout ────────────────────────────────────────────────
    p.text(10, 262, "STATE (sampled every 50 ms)")
    met = p.obj(10, 280, "metro 50")
    lbm = p.obj(90, 280, "loadbang")
    p.connect(lbm, 0, met, 0)

    LABELS = ["x (m)", "y (m)", "z (m)",
              "vx (m/s)", "vy (m/s)", "vz (m/s)",
              "phi roll (rad)", "theta pitch (rad)", "psi yaw (rad)",
              "p (rad/s)", "q (rad/s)", "r (rad/s)"]
    for i, lbl in enumerate(LABELS):
        x = 10 + (i % 3) * 350
        y = 310 + (i // 3) * 62
        sn = p.obj(x, y, "snapshot~")
        nb = p.floatatom(x + 80, y, 12, -1e9, 1e9)
        p.text(x, y + 20, lbl)
        p.connect(dyn, i, sn, 0)
        p.connect(met, 0, sn, 0)
        p.connect(sn, 0, nb, 0)

    p.save("quadcopter.pd")


if __name__ == "__main__":
    print("Generating PureQuad patches into pd/ ...")
    gen_params()
    gen_mixer()
    gen_force_torque()
    gen_dynamics()
    gen_main()
    print("Done. Open pd/quadcopter.pd in Pure Data (vanilla, expr 0.5x included).")
