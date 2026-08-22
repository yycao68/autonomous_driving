# Benchmark Suite: Time-First Double-Integrator Planning vs Path-First TOTG

Companion experimental design for `motion_planning_double_integrator.md`.
Every number in this document is produced by the runnable harness in this folder
(`python3 benchmark_fr3.py`, `python3 benchmark_av.py`); no figures are projected.

---

## 0. Scope: motion *planning*, not path *tracking*

The treatment of motion planning in `h_inf/hinf.md` §6 frames the backbone as a
**feedforward + feedback tracking** command (`u = u_ff + u_fb`): a trajectory is
assumed and the backbone tracks it. That is path *tracking*. This suite is about
motion *planning*: **the waypoints are the only input**, and the planner must
*generate* the timed trajectory `(q, q_dot, q_ddot)` itself.

The contrast we benchmark is between two ways to turn waypoints into a timed
trajectory:

| | **Path-first (TOTG / TOPP family)** | **Time-first (this work)** |
|---|---|---|
| step 1 | fit a geometric path `q(s)` through waypoints (circular blends) | — |
| step 2 | time-parameterize: solve time-optimal `s(t)` (`t→s`) | — |
| step 3 | recover `q(t)=q(s(t))` by inverting `s(t)` (`s→t`) and differentiating | solve one receding-horizon QP directly in `t` |
| limits enforced in | arc-length `s` (cheap per-axis box *along a fixed path*) | time `t` (box on the QP decision variable) |
| output continuity | `C^1` path; `q_ddot` jumps at blend seams | `q_ddot` is the decision variable, bounded as a hard constraint |

The user's hypothesis, which the benchmarks confirm: **the two coordinate
conversions `t→s` and `s→t` are where the numerical trouble lives**, and skipping
them (planning directly in time) removes that trouble — at the cost of
time-optimality.

### Where the conversions break (the thing we measure)

The `s→t` reconstruction is
```
q_ddot(t) = q'(s) · s_ddot   +   q''(s) · s_dot^2
                                  └── curvature term
```
* `q''(s)` (path curvature) **jumps** at every blend seam — `0` on a straight,
  `1/r` on a circular arc. So `q_ddot` is **discontinuous**, and the jump scales
  with `s_dot^2`: the faster you take a tight corner, the bigger the acceleration
  step. Tightening the blend (small `max_deviation` ⇒ small `r`) amplifies it.
* `t(s) = ∫ ds / s_dot` is **singular wherever `s_dot → 0`** (rest points, tight
  near-reversal blends). Recovering a uniform-time sample there is ill-conditioned
  or outright fails (the trajectory time `T → ∞`).

The time-first planner never forms `s`, so neither pathology can occur: its output
`q_ddot` *is* the bounded decision variable.

---

## 1. Methods under test

* **DI-QP (time-first)** — `di_planner.py`. Per-joint virtual double integrator
  `x=[q,q_dot]`, input `u=q_ddot`. Constant `A_d` ⇒ prediction matrices `Φ,Γ`
  precomputed once; one strictly-convex OSQP per joint per cycle, warm-started.
  Hard box constraints on `q_dot` and `u`. Waypoints supplied as inputs; goal
  switches by proximity.
* **TOTG / TOPP (path-first)** — `di_totg.py`. Kunz–Stilman circular-blend path +
  numerical-integration time-optimal parameterization (forward/backward sweep on
  `x=s_dot^2`, the Bobrow/Slotine/Pham/TOPP-RA core), then `s→t` inversion and
  resampling. This is *not* a production TOTG (no exhaustive switch-point search)
  but it is the standard algorithmic core, sufficient to expose the conversions.

Both receive identical waypoints and identical kinematic limits and emit
`(q, q_dot, q_ddot)` sampled at the same `dt`. Fair comparison.

### Metrics

| metric | meaning | who should win |
|---|---|---|
| `T` time-to-goal | TOTG is time-*optimal* by construction | TOTG (lower bound) |
| `peak\|a\|/amax` | >1 ⇒ the accel limit is **broken on the emitted trajectory** | DI (hard constraint) |
| accel / vel violations | # output samples past the limit | DI |
| jerk RMS / peak | smoothness of the command | DI |
| accel jump | `max\|a_{k+1}-a_k\|`, the discontinuity magnitude | DI |
| min `s_dot`, max `1/s_dot` | `s→t` conditioning (TOTG only) | — (DI has no `s`) |
| compute | DI: ms/cycle *online*; TOTG: ms full build *offline* | context-dependent |

---

## 2. FR3 manipulator suite (7-DOF)

Run: `python3 benchmark_fr3.py`. Limits: FR3 joint velocity/acceleration sets.
Stresses are concentrated in joints 0–1 for interpretability; all 7 joints planned.

### B1 — point-to-point (optimality-gap baseline)
Two waypoints, no blends. Establishes the honest cost of smoothness.

| metric | DI-QP | TOTG |
|---|---|---|
| time-to-goal `T` | 2.020 s | **0.591 s** |
| peak\|a\|/amax | 1.000 | 1.000 |
| jerk RMS | **22.4** | 143.0 |
| accel jump | **1.43** | 12.50 |
| compute | 0.048 ms/cyc | 20.8 ms build |

Takeaway: TOTG is **3.4× faster** (time-optimal). But even on a straight move its
time-optimal profile is bang-bang in acceleration → an accel **jump of 12.5** and
**6.4× higher jerk**. DI trades time for a far smoother command.

### B2 — acute corner (~124°), sweep blend tightness `max_dev`
The centripetal `q''·s_dot^2` term at the corner.

| `max_dev` | DI peak\|a\|/amax | TOTG peak\|a\|/amax | TOTG accel viol | DI accel viol |
|---|---|---|---|---|
| 0.20 | 1.000 | 1.014 | 3 | 0 |
| 0.10 | 1.000 | 1.000 | 1 | 0 |
| 0.05 | 1.000 | 1.025 | 1 | 0 |
| 0.02 | 1.000 | **1.047** | 2 | 0 |

Takeaway: as the blend tightens, TOTG’s resampled trajectory **overshoots the
acceleration limit** (up to 1.047×) and posts violations; DI stays pinned at the
limit with **zero violations** (the hard box constraint).

### B3 — near-reversal / stop (~177°), sweep blend tightness
A near-180° turn forces the blend radius `r→0`, so `s_dot→0` on the arc.

| `max_dev` | DI peak\|a\|/amax | TOTG peak\|a\|/amax | min `s_dot` | `s→t` status |
|---|---|---|---|---|
| 0.10 | 1.000 | 1.000 | 0.000 | **STALLED (singular)** |
| 0.05 | 1.000 | **3.125** | 0.000 | **STALLED (singular)** |
| 0.02 | 1.000 | 1.000 | 0.000 | **STALLED (singular)** |

Takeaway: the **headline result**. `s_dot` collapses to 0, so `1/s_dot → ∞`,
`T → 10^6 s`, and the uniform-time resample would allocate ~`10^8` samples — the
`s→t` conversion *fails* (the harness flags it `STALLED` and caps it; when it does
emit samples they reach **3.1× the accel limit**). DI handles the reversal natively
(it is just a velocity sign change in a planned state): 1.000×, zero violations.

### B4 — dense noisy waypoints (24 points)
Many short segments + many blends — accumulated parameterization conditioning.

| metric | DI-QP | TOTG |
|---|---|---|
| time-to-goal `T` | 13.24 s | **3.83 s** |
| peak\|a\|/amax | 1.000 | **1.445** |
| accel violations | **0** | 49 |
| jerk RMS | **53.6** | 332.9 |
| compute | 0.036 ms/cyc | 122.8 ms build |

Takeaway: TOTG is fast in *time* but the resampled command **breaks the accel limit
by 45% at 49 samples**; DI stays feasible everywhere. (DI’s 13.2 s is its weakest
showing — see Limitations.)

### B5 — reactivity to an online change
A new goal appears mid-flight.

```
DI-QP : react within ONE QP cycle      -> 0.032 ms
TOTG  : must rebuild path + re-time     -> 19.2 ms
latency ratio (TOTG / DI)              -> 608x
```
Takeaway: DI’s constant `A_d` (hence fixed `Φ, H`) means a reaction is one
warm-started solve; TOTG must re-fit the path and re-run TOPP from scratch.

**FR3 verdict.** TOTG wins time-optimality (it is the lower bound, by design). DI
wins every smoothness/feasibility/conditioning/reactivity axis, and *decisively* at
the geometric degeneracies (B3) and dense-waypoint conditioning (B4) where the
`s↔t` conversions break.

---

## 3. Autonomous-vehicle variant

Run: `python3 benchmark_av.py`. Planar point mass `(x,y)`; the stressed
second-derivative quantity is now **front-wheel steering**, derived identically for
both planners from the kinematic bicycle model
`κ=(vx·ay−vy·ax)/|v|^3`, `δ=atan(L·κ)`, `a_lat=|v|^2·κ` (`L=2.7 m`).

### 3.1 Why the manipulator recipe does not transfer verbatim

A car does **not** stress the double-integrator *input*. Steering is curvature,
`κ ~ y''/V^2` — a function of acceleration **and** speed. Consequences, measured
with the naive Cartesian double-integrator planner:

* **Low-speed singularity**: `κ ~ 1/V^3`. Planning stop-to-stop, steering rate
  blows up near `V=0` (the naive DI posted steering rate 1.8–5.8 rad/s, *worse*
  than TOTG, when the maneuver starts/ends at rest).
* **Sharp turns at cruise are infeasible** without speed scheduling: AV-B3 (90°)
  and AV-B4 (U-turn) hit the time cap because a point mass at 13 m/s cannot make
  the corner under the lateral-accel bound.

But TOTG’s structural artifact is still loud at the seams of the sharp cases:

| sharp scenario | naive-DI steering jump | TOTG steering jump | TOTG steering rate |
|---|---|---|---|
| AV-B3 90° turn | 0.294 rad | **1.309 rad** | **13.1 rad/s** |
| AV-B4 U-turn | 0.331 rad | **1.309 rad** | **13.1 rad/s** |

The **1.309 rad single-sample steering jump** is the circular-blend curvature step
(`0 → 1/r` entered instantaneously) — a physically impossible steering command. It
is the `s→t` curvature-discontinuity artifact, now wearing a steering hat.

### 3.2 The correct AV backbone: match integrator order to output order

**Principle.** *The integrator order of the time-domain backbone must equal the
smoothness order of the stressed output.*

* Manipulator: stressed output = joint **acceleration** = the DI input ⇒ a **double**
  integrator suffices.
* Car: stressed output = **steering** (≈ curvature `~y''/V^2`), whose *rate* (the
  actuator limit) is `~y'''/V^2` ⇒ bound lateral **jerk** ⇒ a **triple** integrator
  (jerk input). Implemented in `di_lateral.py`, still a constant-`A_d` linear
  backbone (`A_c^3=0`), planned directly in time at cruise speed `V` (so `x=Vt` is
  trivial and there is *still no `s↔t` conversion*).

Cruise maneuvers, jerk-bounded lateral DI vs TOTG:

| metric | AV-B1 jerk-DI | AV-B1 TOTG | AV-B2 jerk-DI | AV-B2 TOTG |
|---|---|---|---|---|
| peak steering rate [rad/s] | **0.129** | 0.254 | **0.129** | 0.594 |
| steering jump/step [rad] | **0.0065** | 0.0254 | **0.0065** | 0.0594 |
| peak lateral accel [m/s²] | 3.99 | 2.47 | 4.00 | 4.05 |
| compute | 0.30 ms/cyc | 13.7 ms build | 0.42 ms/cyc | 41.7 ms build |

Takeaway: with the order-matched backbone, the time-first planner **bounds steering
rate by construction** (0.129 rad/s, comfortably under a 0.5 rad/s actuator cap that
TOTG *exceeds* in the moose test) and emits **continuous** steering (jump ≈ one `dt`
of smooth change, 4–9× smaller than TOTG’s seam steps). No `s↔t`, no curvature
discontinuity.

**AV verdict.** The thesis holds for the car, but the lesson is sharper: choose the
backbone order to match the stressed output. A double integrator in Cartesian
position is the *wrong* model for steering; a jerk-bounded lateral integrator is the
right one, and it removes TOTG’s curvature-step artifact while keeping everything in
the time domain.

---

## 4. Honest limitations

1. **DI is not time-optimal — but the gap is largely recoverable.** With a pure
   position-tracking cost it trades time for smoothness (B1 3.4×, B4 3.5× slower than
   TOTG), because it regulates to the goal like a damped second-order system. Enabling
   the **velocity-law** mode (`plan_di(..., velocity_law=True)`) feeds the analytic
   time-optimal double-integrator velocity reference
   `v_ref = sign(dq)·min(vmax, sqrt(2·amax·|dq|))` per horizon step, closing the gap to
   **1.06–1.32× TOTG** with **zero** limit violations (run `python3 improve_test.py`):

   | scenario | baseline gap | velocity-law gap | accel viol. (DI / TOTG) |
   |---|---|---|---|
   | B1 point-to-point | 3.42× | **1.32×** | 0 / 0 |
   | B2 acute corner | 1.59× | **1.06×** | 0 / 1 |
   | B4 dense (24 wp) | 3.46× | **1.14×** | 0 / 49 |

   The price is higher jerk (near-time-optimal → near-bang-bang; B4 jerk RMS 54 → 841),
   so the velocity reference is a tunable speed↔smoothness selector—both extremes stay
   feasible by construction, unlike TOTG which violates the accel limit on B2/B4.
2. **TOPP fidelity.** Our TOPP is the numerical-integration core, not a full
   switch-point solver; a production TOTG would be smoother in the *interior* but
   has the *same* two structural conversions, so the seam discontinuity (B2, AV) and
   the `s_dot→0` singularity (B3, AV-B4) are intrinsic, not artifacts of our
   implementation.
3. **AV obstacle/coupled constraints** (friction circle, nonholonomy) are
   approximated by per-axis boxes; a full treatment would add a friction-circle SOC
   constraint to the QP (still convex).
4. **Sharp AV turns need speed scheduling**, which the lateral-only jerk planner does
   not do; a joint longitudinal+lateral backbone is the natural extension.

---

## 5. Files

| file | contents |
|---|---|
| `di_planner.py` | time-first double-integrator QP planner (FR3) |
| `di_totg.py` | path-first circular-blend + TOPP reference, with `s→t` stall guard |
| `di_lateral.py` | jerk-bounded (triple-integrator) lateral backbone for AV |
| `benchmark_fr3.py` | FR3 suite B1–B5 + metrics table |
| `benchmark_av.py` | AV variant (naive DI + jerk-bounded backbone) vs TOTG |
| `improve_test.py` | velocity-law mode: closes the time-optimality gap to ~1.1–1.3× |
| `sweep_N.py` | horizon-length sweep (N = 10…40) |
| `analysis_totg.py` | TOTG compute-rate (∝K) and steering-rate (∝1/dt) drill-down |
| `demo_smooth_totg.py` | C² spline path: shows TOTG limitations (a)/(b) are repairable |
| `compare_topp.py` | NI-TOPP vs reachability TOPP-RA: structural findings are parameterizer-invariant |
| `topp_phaseplane.py` | TOPP-RA-style (s, s_dot) phase plane: MVC, controllable set, optimal profile (cf. arXiv:1707.07239) |
| `benchmark_dynamic.py` | moving-obstacle reactivity (bounded DI latency vs replan-latency sweep) + plot |
| `av_trajectory_output.py` | Apollo-style trajectory export (pose, κ, steering, a, jerk) from path+speed channels |
| `fr3_kinematics.py` | FR3 forward kinematics + geometric Jacobian (FD-verified, 1.7e-10) |
| `fr3_dynamic_obstacle.py` | 7-DOF FR3 task-space dynamic obstacle: QP (0 joint violations) vs reactive baseline (42/346); Jacobian-error study; plot |
| `mujoco_compare.py` | MuJoCo computed-torque execution on FR3: DI vs TOTG torque smoothness/tracking + plot |
| `fr3_render.py` | MuJoCo offscreen render of the FR3 avoiding the moving obstacle (scene frames) |
| `fr3_video.py` | MuJoCo MP4 video of the full FR3 obstacle-avoidance rollout (fr3_motion.mp4) |
| `verify_math.py` | numerical verification of every equation (14/14 pass) |
