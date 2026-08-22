"""
Autonomous-vehicle variant of the suite: the SAME time-first DI-QP vs path-first
TOTG comparison, but the stressed second-derivative quantity is now STEERING.

Model: planar point mass (x, y) with double-integrator dynamics per axis (the AV
"backbone"); longitudinal/lateral motion are the two virtual integrators.  The
front-wheel STEERING follows from the kinematic bicycle model:

    kappa(t) = (vx*ay - vy*ax) / |v|^3            (path curvature)
    delta(t) = atan(L * kappa)                     (steering angle)
    a_lat(t) = |v|^2 * kappa                       (lateral / friction load)

Both planners emit (x,y,v,a); steering is derived with the SAME formula, so the
comparison is apples-to-apples.

Why steering exposes the s<->t conversion:
  * TOTG path-first: curvature is PIECEWISE CONSTANT (0 on a straight, +/-1/r on a
    circular blend) -> steering angle STEPS at every seam -> infinite steering rate
    (a physically impossible command / actuator jerk).
  * Time-first DI: x(t), y(t) are smooth, so curvature -- hence steering -- varies
    continuously.  (Honest caveat: kappa ~ 1/|v|^3, so DI steering is only well
    conditioned while the car keeps moving; we report min speed too.)

Run:  python3 benchmark_av.py
"""

import numpy as np
from di_planner import plan_di
from di_totg import plan_totg
from di_lateral import plan_lateral_jerk

L_WHEELBASE = 2.7            # m
V_MAX = np.array([16.0, 16.0])     # m/s per axis (box approx of speed limit)
A_MAX = np.array([4.0, 4.0])       # m/s^2 (comfort/friction box)
DELTA_RATE_LIMIT = 0.5             # rad/s, steering actuator rate cap
EPS = 1e-6


def steering(res, L=L_WHEELBASE):
    """Derive steering angle/rate, curvature, lateral accel from (V, A)."""
    V, A, dt = res["V"], res["A"], res["dt"]
    vx, vy = V[:, 0], V[:, 1]
    ax, ay = A[:, 0], A[:, 1]
    speed = np.hypot(vx, vy)
    kappa = (vx * ay - vy * ax) / np.maximum(speed ** 3, EPS)
    delta = np.arctan(L * kappa)
    a_lat = (vx * ay - vy * ax) / np.maximum(speed, EPS)
    drate = np.gradient(delta, dt) if len(delta) > 1 else np.zeros_like(delta)
    # ignore the first/last samples where speed -> 0 (start/stop) for peak stats
    m = speed > 0.5
    return dict(speed=speed, kappa=kappa, delta=delta, a_lat=a_lat, drate=drate,
                mask=m, min_speed=float(speed[m].min()) if m.any() else 0.0,
                peak_delta=float(np.abs(delta[m]).max()) if m.any() else 0.0,
                peak_drate=float(np.abs(drate[m]).max()) if m.any() else 0.0,
                delta_jump=float(np.abs(np.diff(delta[m])).max()) if m.sum() > 1 else 0.0,
                peak_alat=float(np.abs(a_lat[m]).max()) if m.any() else 0.0)


V_CRUISE = 13.0   # m/s nominal forward speed (AV plans at speed, not stop-start)


def run_av(name, wps, dt=0.05, max_dev=0.4, di_kw=None):
    di_kw = di_kw or {}
    # Enter at cruise speed along the first leg and cruise through (no stop):
    # this is where the steering comparison is meaningful (kappa ~ 1/v^3 is only
    # singular near v=0).  Velocity tracking (wv>0) holds the cruise speed.
    leg0 = wps[1] - wps[0]
    v0 = V_CRUISE * leg0 / np.linalg.norm(leg0)
    di = plan_di(wps, dt=dt, vmax=V_MAX, amax=A_MAX, switch_radius=1.5,
                 v_pass=V_CRUISE / V_MAX[0], wv=4.0, wq=12.0, wr=0.3, gamma=6.0,
                 t_max=60.0, v0=v0, final_stop=False, **di_kw)
    tg = plan_totg(wps, dt=dt, vmax=V_MAX, amax=A_MAX, max_dev=max_dev)
    sd, st = steering(di), steering(tg)
    return name, di, tg, sd, st


def print_av(name, di, tg, sd, st):
    print(f"\n=== {name} ===")
    hdr = f"{'metric':<26}{'DI-QP (time-first)':>20}{'TOTG (path-first)':>20}"
    print(hdr); print("-" * len(hdr))
    rows = [
        ("time to goal [s]", f"{di['t'][-1]:.2f}", f"{tg['t'][-1]:.2f}"),
        ("peak steering [deg]", f"{np.degrees(sd['peak_delta']):.1f}",
         f"{np.degrees(st['peak_delta']):.1f}"),
        ("peak steering RATE [rad/s]", f"{sd['peak_drate']:.2f}",
         f"{st['peak_drate']:.2f}"),
        (f"  (actuator cap {DELTA_RATE_LIMIT})",
         "OK" if sd['peak_drate'] <= DELTA_RATE_LIMIT else "EXCEEDED",
         "OK" if st['peak_drate'] <= DELTA_RATE_LIMIT else "EXCEEDED"),
        ("steering JUMP/step [rad]", f"{sd['delta_jump']:.3f}",
         f"{st['delta_jump']:.3f}"),
        ("peak lateral accel [m/s^2]", f"{sd['peak_alat']:.2f}",
         f"{st['peak_alat']:.2f}"),
        ("min speed [m/s]", f"{sd['min_speed']:.2f}", f"{st['min_speed']:.2f}"),
        ("compute", f"{1e3*di['solve_t'].mean():.3f} ms/cyc",
         f"{1e3*tg['build_t']:.1f} ms build"),
    ]
    for r in rows:
        print(f"{r[0]:<26}{r[1]:>20}{r[2]:>20}")


# ---------------------------------------------------------------------------
# Scenarios (planar x-y waypoints, metres)
# ---------------------------------------------------------------------------
def av_single_lane_change():
    W = np.array([[0, 0], [20, 0], [40, 3.5], [70, 3.5]], float)
    return "AV-B1  single lane change", W, 0.4


def av_double_lane_change():
    # ISO 3888-style "moose test" offsets.
    W = np.array([[0, 0], [15, 0], [30, 3.5], [45, 3.5],
                  [60, 0], [80, 0]], float)
    return "AV-B2  double lane change (moose test)", W, 0.3


def av_intersection_turn():
    # 90-degree turn (sharp) into a cross street.
    W = np.array([[0, 0], [30, 0], [30, -30], [30, -60]], float)
    return "AV-B3  sharp 90-deg intersection turn", W, 0.3


def av_uturn():
    # near-reversal (U-turn) -> curvature blow-up / s_dot -> 0 (singular).
    W = np.array([[0, 0], [40, 0], [40, 6], [2, 6]], float)
    return "AV-B4  U-turn (near-reversal)", W, 0.3


def cruise_compare(name, wps, dt=0.05, max_dev=0.4, V=V_CRUISE):
    """Proper AV backbone: jerk-bounded lateral planner vs TOTG, at cruise speed.

    Both produce steering with the SAME kinematic formula.  The jerk-bounded DI
    bounds steering RATE by construction (|y'''| <= j_max -> |delta_dot| bounded);
    TOTG's piecewise-constant blend curvature steps the steering at every seam.
    """
    di = plan_lateral_jerk(wps, V=V, dt=dt)
    tg = plan_totg(wps, dt=dt, vmax=V_MAX, amax=A_MAX, max_dev=max_dev)
    sd, st = steering(di), steering(tg)
    print(f"\n=== {name}  (cruise {V} m/s, jerk-bounded lateral DI) ===")
    hdr = f"{'metric':<26}{'jerk-DI (time-first)':>20}{'TOTG (path-first)':>20}"
    print(hdr); print("-" * len(hdr))
    rows = [
        ("peak steering [deg]", f"{np.degrees(sd['peak_delta']):.1f}",
         f"{np.degrees(st['peak_delta']):.1f}"),
        ("peak steering RATE [rad/s]", f"{sd['peak_drate']:.3f}",
         f"{st['peak_drate']:.3f}"),
        ("steering JUMP/step [rad]", f"{sd['delta_jump']:.4f}",
         f"{st['delta_jump']:.4f}"),
        ("peak lateral accel [m/s^2]", f"{sd['peak_alat']:.2f}",
         f"{st['peak_alat']:.2f}"),
        ("compute", f"{1e3*di['solve_t'].mean():.3f} ms/cyc",
         f"{1e3*tg['build_t']:.1f} ms build"),
    ]
    for r in rows:
        print(f"{r[0]:<26}{r[1]:>20}{r[2]:>20}")


if __name__ == "__main__":
    print("##### Cartesian double-integrator DI (naive) vs TOTG #####")
    for fn in (av_single_lane_change, av_double_lane_change,
               av_intersection_turn, av_uturn):
        name, W, dev = fn()
        print_av(*run_av(name, W, max_dev=dev))

    print("\n\n##### Proper AV backbone: jerk-bounded lateral DI vs TOTG #####")
    print("(cruise maneuvers; bounds steering RATE by construction)")
    cruise_compare("AV-B1  single lane change", av_single_lane_change()[1],
                   max_dev=0.4)
    cruise_compare("AV-B2  double lane change (moose)",
                   av_double_lane_change()[1], max_dev=0.3)
