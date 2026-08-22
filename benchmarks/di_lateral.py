"""
Jerk-bounded lateral backbone for the autonomous-vehicle case.

Key insight from the AV experiments: the integrator order of the time-domain
backbone must match the smoothness order of the *stressed output*.

  * Manipulator: the stressed quantity is joint ACCELERATION, which is exactly the
    double-integrator input -> bounding the input bounds the output (di_planner).
  * Car: the stressed quantity is STEERING ~ curvature kappa ~ y''/V^2.  Its rate
    (steering rate, the actuator limit) is ~ y'''/V^2.  So to bound steering RATE
    we must bound lateral JERK -> the backbone is a TRIPLE integrator (jerk input),
    one order higher than the manipulator's double integrator.

This is still a constant-A_d linear backbone (A_c nilpotent, A_c^3 = 0), planned
directly in TIME at a constant cruise speed V -- no t->s / s->t conversion.  x = V t
is trivial, and we plan the lateral offset y(t) tracking the waypoint offsets.

State z = [y, y_dot, y_ddot], input u = y_dddot (lateral jerk).
"""

import numpy as np
import scipy.sparse as sp
import osqp


def _tri_matrices(dt, N):
    """Triple-integrator prediction matrices (constant, precomputed)."""
    k = np.arange(1, N + 1)
    Phi_p = np.column_stack([np.ones(N), k * dt, 0.5 * (k * dt) ** 2])
    Phi_v = np.column_stack([np.zeros(N), np.ones(N), k * dt])
    Phi_a = np.column_stack([np.zeros(N), np.zeros(N), np.ones(N)])
    Gp = np.zeros((N, N)); Gv = np.zeros((N, N)); Ga = np.zeros((N, N))
    for kk in range(1, N + 1):
        for j in range(kk):
            m = kk - 1 - j
            Gp[kk - 1, j] = dt ** 3 * (1 / 6 + m / 2 + m * m / 2)
            Gv[kk - 1, j] = dt ** 2 * (m + 0.5)
            Ga[kk - 1, j] = dt
    return Phi_p, Phi_v, Phi_a, Gp, Gv, Ga


def plan_lateral_jerk(wps, V, dt=0.05, N=30,
                      a_lat_max=4.0, j_max=8.0, vy_max=6.0,
                      wq=20.0, wa=0.2, wr=0.02, gamma=10.0,
                      switch_dx=4.0, t_max=30.0):
    """Plan lateral offset y(t) at constant forward speed V through (x,y) wps.

    Bounds: |y_ddot| <= a_lat_max (lateral accel), |y_dddot| <= j_max (lateral
    jerk -> steering rate), |y_dot| <= vy_max.  Returns V,A arrays (vx=V, ax=0)
    so steering() in benchmark_av.py applies unchanged.
    """
    wps = np.asarray(wps, float)
    xs, ys = wps[:, 0], wps[:, 1]
    Phi_p, Phi_v, Phi_a, Gp, Gv, Ga = _tri_matrices(dt, N)

    Wq = np.full(N, wq); Wq[-1] *= gamma
    H = (Gp.T @ (Wq[:, None] * Gp) + wa * Ga.T @ Ga + wr * np.eye(N))
    H = 0.5 * (H + H.T)
    P = sp.csc_matrix(H)
    # rows: lateral velocity (Gv), lateral accel (Ga), jerk (I)
    A = sp.csc_matrix(np.vstack([Gv, Ga, np.eye(N)]))
    prob = osqp.OSQP()
    prob.setup(P, np.zeros(N), A, -np.inf * np.ones(3 * N),
               np.inf * np.ones(3 * N), verbose=False, warm_starting=True,
               eps_abs=1e-6, eps_rel=1e-6)

    z = np.array([ys[0], 0.0, 0.0])
    x = xs[0]
    Y, VY, AY, J, st = [], [], [], [], []
    steps = int(t_max / dt)
    for _ in range(steps):
        # running lateral goal: offset of the next waypoint ahead in x
        idx = int(np.searchsorted(xs, x + 1e-9))
        idx = min(idx, len(ys) - 1)
        y_goal = ys[idx]

        ep = Phi_p @ z - y_goal
        q = Gp.T @ (Wq * ep)
        v_free = Phi_v @ z; a_free = Phi_a @ z
        l = np.concatenate([-vy_max - v_free, -a_lat_max - a_free,
                            -j_max * np.ones(N)])
        u = np.concatenate([vy_max - v_free, a_lat_max - a_free,
                            j_max * np.ones(N)])
        prob.update(q=q, l=l, u=u)
        r = prob.solve()
        j0 = r.x[0] if r.info.status_val in (1, 2) else 0.0
        st.append(r.info.solve_time)

        # exact triple-integrator step
        y, vy, ay = z
        z = np.array([
            y + dt * vy + 0.5 * dt ** 2 * ay + dt ** 3 / 6 * j0,
            vy + dt * ay + 0.5 * dt ** 2 * j0,
            ay + dt * j0,
        ])
        x += V * dt
        Y.append(z[0]); VY.append(z[1]); AY.append(z[2]); J.append(j0)
        if x >= xs[-1] - 1e-6 and abs(z[0] - ys[-1]) < 0.05:
            break

    n = len(Y)
    V_arr = np.column_stack([np.full(n, V), np.array(VY)])
    A_arr = np.column_stack([np.zeros(n), np.array(AY)])
    return dict(V=V_arr, A=A_arr, dt=dt, t=np.arange(n) * dt,
                solve_t=np.array(st), Y=np.array(Y), J=np.array(J))


if __name__ == "__main__":
    # moose-test offsets at 13 m/s
    wps = np.array([[0, 0], [15, 0], [30, 3.5], [45, 3.5], [60, 0], [80, 0]],
                   float)
    r = plan_lateral_jerk(wps, V=13.0)
    print(f"steps={len(r['t'])}  T={r['t'][-1]:.2f}s")
    print(f"peak |y_ddot| (lat accel) = {np.abs(r['A'][:,1]).max():.2f}")
    print(f"peak |jerk| = {np.abs(r['J']).max():.2f}")
    print(f"mean solve = {1e3*r['solve_t'].mean():.3f} ms")
