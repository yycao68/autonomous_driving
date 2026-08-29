"""
Apollo-style trajectory output for the AV planner.

A motion-control layer needs a trajectory of points, each carrying pose AND the
dynamic/geometric commands.  This mirrors Apollo's pnc_point.proto:

    PathPoint       : x, y, z, theta (heading), kappa (curvature), s (station),
                      dkappa (dκ/ds)
    TrajectoryPoint : path_point, v (speed), a (linear accel), da (long. jerk),
                      relative_time

Apollo stores kappa in PathPoint; the control module derives the steering
command from it via the kinematic bicycle  delta = atan(L*kappa).  We output both
kappa and delta (plus steer_rate and lateral_accel) so either controller works.

Path-speed DECOUPLED (Apollo EM/Lattice style):
  * lateral path  : jerk-bounded triple integrator -> y(x) -> kappa, theta, delta
                    (curvature is geometric, speed-independent; jerk bound => smooth,
                     continuous kappa and steering)
  * longitudinal  : double-integrator speed profile v(t), a(t) along the station s
  * combine        : at station s(t) look up the path geometry; lateral accel = v^2*kappa

Conversion formulas (planar kinematic bicycle, wheelbase L):
    theta   = atan2(y', x')                       (x'=dx/ds etc., here path tangent)
    kappa   = (x' y'' - y' x'') / (x'^2+y'^2)^{3/2}   = y''_xx / (1+y'^2_x)^{3/2}
    delta   = atan(L * kappa)                      (front-wheel steering)
    a_lat   = v^2 * kappa                          (lateral / centripetal accel)
    v, a    = speed and dv/dt from the longitudinal profile
    s       = ∫ |v| dt  (station / accumulated arc length)
    dkappa  = dκ/ds ;  steer_rate = dδ/dt ;  da = da/dt (longitudinal jerk)
"""
import numpy as np
import csv
import json
from pathlib import Path
import scipy.sparse as sp
import osqp
from di_lateral import plan_lateral_jerk, _tri_matrices
from di_planner import plan_di

RESULTS_DIR = Path(__file__).parent.parent / "simulationResults"

L_WHEELBASE = 2.7   # m


def build_lateral_path(wps, V_ref=15.0, dt=0.05):
    """Jerk-bounded lateral plan -> geometric path quantities (params by station s)."""
    r = plan_lateral_jerk(wps, V=V_ref, dt=dt)
    x = V_ref * r["t"]                     # x = V_ref * t (cruise param)
    y = r["Y"]
    dydx = r["V"][:, 1] / V_ref            # dy/dx = (dy/dt)/(dx/dt)
    d2ydx2 = r["A"][:, 1] / V_ref ** 2     # d2y/dx2 = (d2y/dt2)/V^2
    theta = np.arctan2(r["V"][:, 1], V_ref)
    kappa = d2ydx2 / (1.0 + dydx ** 2) ** 1.5         # geometric curvature
    # arc length s(x) = ∫ sqrt(1+(dy/dx)^2) dx
    ds_dx = np.sqrt(1.0 + dydx ** 2)
    s = np.concatenate([[0.0], np.cumsum(0.5 * (ds_dx[1:] + ds_dx[:-1]) * np.diff(x))])
    dkappa_ds = np.gradient(kappa, s)
    return dict(x=x, y=y, theta=theta, kappa=kappa, s=s, dkappa_ds=dkappa_ds,
                length=s[-1])


def build_speed_profile(path_len, v0=12.0, v_cruise=15.0, amax=2.0, dt=0.05):
    """Longitudinal DOUBLE integrator: bounded accel (a piecewise-constant -> da spikes)."""
    goal = path_len * 1.6                              # goal beyond path -> cruise, no braking
    W = np.array([[0.0], [goal]])
    r = plan_di(W, dt=dt, N=25, vmax=np.array([v_cruise]), amax=np.array([amax]),
                wq=40.0, wr=0.05, gamma=8.0, v0=np.array([v0]),
                final_stop=False, switch_radius=1.0, t_max=40.0)
    s = r["Q"][:, 0]; v = r["V"][:, 0]; a = r["A"][:, 0]; t = r["t"]
    m = s <= path_len + 1e-6                            # truncate at path end
    return dict(t=t[m], s=s[m], v=v[m], a=a[m])


def build_speed_profile_jerk(path_len, v0=12.0, v_cruise=15.0, amax=2.0,
                             jmax=2.0, dt=0.05, N=30):
    """Longitudinal TRIPLE integrator (state [s,v,a], input jerk): tracks cruise
    speed with BOUNDED jerk -> da is bounded (C^2 ride comfort).  Same constant-A_d
    backbone as the lateral channel, one order up (the integrator-order principle)."""
    Phi_p, Phi_v, Phi_a, Gp, Gv, Ga = _tri_matrices(dt, N)
    wv, wa, wj = 5.0, 0.1, 0.05
    H = wv * Gv.T @ Gv + wa * Ga.T @ Ga + wj * np.eye(N)
    P = sp.csc_matrix(0.5 * (H + H.T))
    A = sp.csc_matrix(np.vstack([Ga, np.eye(N), Gv]))     # accel box, jerk box, v>=0
    prob = osqp.OSQP()
    prob.setup(P, np.zeros(N), A, -np.inf * np.ones(3 * N), np.inf * np.ones(3 * N),
               verbose=False, warm_starting=True, eps_abs=1e-7, eps_rel=1e-7)
    z = np.array([0.0, v0, 0.0])
    T, S, V, Acc, J = [], [], [], [], []
    for _ in range(int(40.0 / dt)):
        v_free = Phi_v @ z; a_free = Phi_a @ z
        q = wv * Gv.T @ (v_free - v_cruise) + wa * Ga.T @ a_free
        l = np.concatenate([-amax - a_free, -jmax * np.ones(N), -v_free])
        u = np.concatenate([amax - a_free, jmax * np.ones(N), np.full(N, np.inf)])
        prob.update(q=q, l=l, u=u); r = prob.solve()
        j0 = r.x[0] if r.info.status_val in (1, 2) else 0.0
        s, v, a = z
        z = np.array([s + dt * v + 0.5 * dt**2 * a + dt**3 / 6 * j0,
                      v + dt * a + 0.5 * dt**2 * j0, a + dt * j0])
        T.append(len(T) * dt); S.append(z[0]); V.append(z[1]); Acc.append(z[2]); J.append(j0)
        if z[0] >= path_len:
            break
    return dict(t=np.array(T), s=np.array(S), v=np.array(V), a=np.array(Acc),
                jerk=np.array(J))


def to_apollo_trajectory(path, speed, L=L_WHEELBASE):
    """Combine path geometry + speed profile into Apollo-style TrajectoryPoints."""
    t, s_t, v_t, a_t = speed["t"], speed["s"], speed["v"], speed["a"]
    sp = path["s"]
    x = np.interp(s_t, sp, path["x"]);     y = np.interp(s_t, sp, path["y"])
    theta = np.interp(s_t, sp, path["theta"])
    kappa = np.interp(s_t, sp, path["kappa"])
    dkappa = np.interp(s_t, sp, path["dkappa_ds"])
    delta = np.arctan(L * kappa)                       # front-wheel steering angle
    a_lat = v_t ** 2 * kappa                           # lateral / centripetal accel
    steer_rate = np.gradient(delta, t)
    # longitudinal jerk: use the planned jerk if the speed channel provides it
    # (triple integrator), else finite-difference the acceleration (double integrator)
    da = speed["jerk"] if "jerk" in speed else np.gradient(a_t, t)
    pts = []
    for i in range(len(t)):
        pts.append(dict(
            relative_time=float(t[i]),
            # PathPoint
            x=float(x[i]), y=float(y[i]), z=0.0, theta=float(theta[i]),
            kappa=float(kappa[i]), s=float(s_t[i]), dkappa=float(dkappa[i]),
            # TrajectoryPoint
            v=float(v_t[i]), a=float(a_t[i]), da=float(da[i]),
            # steering (derived; what control consumes)
            steer=float(delta[i]), steer_rate=float(steer_rate[i]),
            lateral_accel=float(a_lat[i]),
        ))
    return pts


def print_trajectory(pts, every=10):
    cols = ["relative_time", "x", "y", "theta", "kappa", "dkappa", "s",
            "v", "a", "da", "steer", "steer_rate", "lateral_accel"]
    hdr = ("  t[s]    x[m]   y[m]  theta  kappa  dkap/ds   s[m]   v[m/s] a[m/s2] "
           "da[m/s3] steer[deg] dsteer[r/s] alat[m/s2]")
    print(hdr); print("-" * len(hdr))
    for p in pts[::every] + [pts[-1]]:
        print(f"{p['relative_time']:6.2f}{p['x']:8.2f}{p['y']:7.2f}{p['theta']:7.3f}"
              f"{p['kappa']:7.4f}{p['dkappa']:9.4f}{p['s']:7.2f}{p['v']:8.2f}"
              f"{p['a']:8.2f}{p['da']:9.2f}{np.degrees(p['steer']):10.2f}"
              f"{p['steer_rate']:11.3f}{p['lateral_accel']:11.2f}")


if __name__ == "__main__":
    # ISO 3888 double lane change (moose test) waypoints, metres
    wps = np.array([[0, 0], [15, 0], [30, 3.5], [45, 3.5], [60, 0], [80, 0]], float)
    path = build_lateral_path(wps, V_ref=15.0, dt=0.05)
    # triple-integrator longitudinal channel -> bounded longitudinal jerk (C^2)
    speed = build_speed_profile_jerk(path["length"], v0=12.0, v_cruise=15.0,
                                     amax=2.0, jmax=2.0, dt=0.05)
    pts = to_apollo_trajectory(path, speed)

    print(f"Maneuver: double lane change | path length {path['length']:.1f} m | "
          f"{len(pts)} trajectory points @ 20 Hz\n")
    print_trajectory(pts, every=8)

    # command envelopes
    K = np.array([p["kappa"] for p in pts]); D = np.array([p["steer"] for p in pts])
    DR = np.array([p["steer_rate"] for p in pts]); A = np.array([p["a"] for p in pts])
    AL = np.array([p["lateral_accel"] for p in pts])
    print(f"\nCommand envelopes:")
    print(f"  peak |kappa|       = {np.abs(K).max():.4f} 1/m  (min radius {1/max(np.abs(K).max(),1e-9):.1f} m)")
    print(f"  peak |steer|       = {np.degrees(np.abs(D).max()):.2f} deg")
    print(f"  peak |steer_rate|  = {np.abs(DR).max():.3f} rad/s")
    print(f"  longitudinal a     = [{A.min():.2f}, {A.max():.2f}] m/s^2")
    DA = np.array([p["da"] for p in pts])
    print(f"  longitudinal jerk  = [{DA.min():.2f}, {DA.max():.2f}] m/s^3  (bounded)")
    print(f"  peak |lateral acc| = {np.abs(AL).max():.2f} m/s^2")

    # export CSV + JSON (what a planner would publish)
    csv_out = RESULTS_DIR / "av_trajectory.csv"
    json_out = RESULTS_DIR / "av_trajectory.json"
    with open(csv_out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(pts[0].keys())); w.writeheader()
        w.writerows(pts)
    with open(json_out, "w") as f:
        json.dump(pts, f, indent=1)
    print(f"\nwrote {csv_out}  and  {json_out}")
