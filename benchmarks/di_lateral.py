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
                      switch_dx=4.0, t_max=30.0,
                      goal_tol=0.05, strict=False, feas_tol=1e-4):
    """Plan lateral offset y(t) at constant forward speed V through (x,y) wps.

    Bounds: |y_ddot| <= a_lat_max (lateral accel), |y_dddot| <= j_max (lateral
    jerk -> steering rate), |y_dot| <= vy_max.  Returns V,A arrays (vx=V, ax=0)
    so steering() in benchmark_av.py applies unchanged.

    NOTE ON WHAT IS AND IS NOT CONSTRAINED. The QP rows are lateral velocity,
    lateral acceleration, and lateral jerk. There is NO steering or
    steering-rate row. Bounding jerk bounds steering rate only through the
    bicycle relation delta = arctan(L (vx ay - vy ax)/|v|^3), whose full
    dependence on lateral velocity and acceleration must be accounted for
    before a given j_max implies a particular delta-dot cap. Treat the
    reported steering rate as measured compliance at the tested cruise speed,
    not as a constraint enforced by construction. `steering_rate_report()`
    below measures it from the produced trajectory.

    Failure reporting (an external review found infeasible solves were
    silently replaced by zero jerk, with no way for a caller to tell):
      strict    -- if True, raise RuntimeError on the first non-optimal solve
                   or primal-residual violation instead of holding jerk at 0.
      feas_tol  -- primal-residual tolerance for accepting a solve; the box
                   rows are checked explicitly rather than trusting the status
                   string, since OSQP's "solved inaccurate" does not guarantee
                   A U stays inside [l, u].
      goal_tol  -- lateral tolerance for declaring the maneuver complete.

    The returned dict carries `status`, `n_qp_failures`, `qp_statuses`,
    `max_primal_residual`, `completed`, `terminal_error`, `terminal_vy` and
    `terminal_ay`, so a stalled or incomplete plan is never indistinguishable
    from a converged one.
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
    A_dense = np.vstack([Gv, Ga, np.eye(N)])
    qp_statuses, n_fail, max_resid = {}, 0, 0.0
    completed = False
    steps = int(t_max / dt)
    for _ in range(steps):
        # running lateral goal: offset of the next waypoint ahead in x
        idx = int(np.searchsorted(xs, x + 1e-9))
        idx = min(idx, len(ys) - 1)
        y_goal = ys[idx]

        ep = Phi_p @ z - y_goal
        v_free = Phi_v @ z; a_free = Phi_a @ z
        # acceleration term penalizes total predicted accel a_free + Ga@U, not
        # just the control-induced increment -- its free-response part must
        # appear here to match the wa*Ga.T@Ga block already in H.
        q = Gp.T @ (Wq * ep) + wa * (Ga.T @ a_free)
        l = np.concatenate([-vy_max - v_free, -a_lat_max - a_free,
                            -j_max * np.ones(N)])
        u = np.concatenate([vy_max - v_free, a_lat_max - a_free,
                            j_max * np.ones(N)])
        prob.update(q=q, l=l, u=u)
        r = prob.solve()
        st.append(r.info.solve_time)
        qp_statuses[r.info.status] = qp_statuses.get(r.info.status, 0) + 1

        # Accept a solve only if it is optimal AND the returned input actually
        # satisfies the box rows. Status alone is not enough: "solved
        # inaccurate" (status_val 2) can still violate them.
        ok = r.info.status_val in (1, 2) and r.x is not None \
            and np.all(np.isfinite(r.x))
        resid = np.inf
        if ok:
            au = A_dense @ r.x
            resid = float(np.max(np.maximum(np.maximum(l - au, au - u), 0.0)))
            max_resid = max(max_resid, resid)
            ok = resid <= feas_tol
        if ok:
            j0 = float(r.x[0])
        else:
            n_fail += 1
            if strict:
                raise RuntimeError(
                    f"lateral QP failed at t={len(Y)*dt:.3f}s: "
                    f"status={r.info.status!r}, primal residual={resid:.3e} "
                    f"(tol {feas_tol:.1e}). Refusing to substitute zero jerk.")
            # Documented safe fallback: hold jerk at zero for this step. It is
            # COUNTED and surfaced in the returned status, never silent.
            j0 = 0.0

        # exact triple-integrator step
        y, vy, ay = z
        z = np.array([
            y + dt * vy + 0.5 * dt ** 2 * ay + dt ** 3 / 6 * j0,
            vy + dt * ay + 0.5 * dt ** 2 * j0,
            ay + dt * j0,
        ])
        x += V * dt
        Y.append(z[0]); VY.append(z[1]); AY.append(z[2]); J.append(j0)
        if x >= xs[-1] - 1e-6 and abs(z[0] - ys[-1]) < goal_tol:
            completed = True
            break

    n = len(Y)
    V_arr = np.column_stack([np.full(n, V), np.array(VY)])
    A_arr = np.column_stack([np.zeros(n), np.array(AY)])
    status = ("ok" if (completed and n_fail == 0) else
              "incomplete" if n_fail == 0 else
              "qp_failure")
    return dict(V=V_arr, A=A_arr, dt=dt, t=np.arange(n) * dt,
                solve_t=np.array(st), Y=np.array(Y), J=np.array(J),
                # --- explicit outcome reporting (see docstring) ---
                status=status,
                completed=bool(completed),
                n_qp_failures=int(n_fail),
                qp_statuses=dict(qp_statuses),
                max_primal_residual=float(max_resid),
                feas_tol=float(feas_tol),
                goal_tol=float(goal_tol),
                terminal_error=float(Y[-1] - ys[-1]) if n else float("nan"),
                terminal_vy=float(VY[-1]) if n else float("nan"),
                terminal_ay=float(AY[-1]) if n else float("nan"))


def steering_rate_report(res, L=2.7, V=None):
    """Measure steering angle and rate from a planned trajectory.

    Uses the full bicycle relation rather than the small-angle / constant-speed
    shortcut, so the number can be compared against a real actuator limit:

        delta = arctan( L * (vx*ay - vy*ax) / |v|^3 ),   |v|^2 = vx^2 + vy^2

    with delta-dot obtained by finite differences. The planner does NOT
    constrain delta or delta-dot (see plan_lateral_jerk), so this reports
    measured compliance, not an enforced bound. Returns max/RMS of both.
    """
    Vm, Am, dt = res["V"], res["A"], res["dt"]
    vx, vy = Vm[:, 0], Vm[:, 1]
    ax, ay = Am[:, 0], Am[:, 1]
    speed = np.sqrt(vx ** 2 + vy ** 2)
    with np.errstate(divide="ignore", invalid="ignore"):
        kappa = (vx * ay - vy * ax) / speed ** 3
    kappa = np.nan_to_num(kappa, nan=0.0, posinf=0.0, neginf=0.0)
    delta = np.arctan(L * kappa)
    ddelta = np.gradient(delta, dt) if len(delta) > 1 else np.zeros_like(delta)
    return {
        "max_abs_delta_rad": float(np.max(np.abs(delta))),
        "max_abs_delta_rate_rad_s": float(np.max(np.abs(ddelta))),
        "rms_delta_rate_rad_s": float(np.sqrt(np.mean(ddelta ** 2))),
        "wheelbase_m": float(L),
    }


if __name__ == "__main__":
    # moose-test offsets at 13 m/s
    wps = np.array([[0, 0], [15, 0], [30, 3.5], [45, 3.5], [60, 0], [80, 0]],
                   float)
    r = plan_lateral_jerk(wps, V=13.0)
    print(f"steps={len(r['t'])}  T={r['t'][-1]:.2f}s")
    print(f"peak |y_ddot| (lat accel) = {np.abs(r['A'][:,1]).max():.2f}")
    print(f"peak |jerk| = {np.abs(r['J']).max():.2f}")
    print(f"mean solve = {1e3*r['solve_t'].mean():.3f} ms")
    print(f"status={r['status']}  completed={r['completed']}  "
          f"qp_failures={r['n_qp_failures']}  "
          f"max_primal_residual={r['max_primal_residual']:.2e}")
    print(f"terminal: err={r['terminal_error']:+.4f} m  "
          f"vy={r['terminal_vy']:+.4f} m/s  ay={r['terminal_ay']:+.4f} m/s^2")
    sr = steering_rate_report(r)
    print(f"measured |delta|max={sr['max_abs_delta_rad']:.4f} rad  "
          f"|delta_dot|max={sr['max_abs_delta_rate_rad_s']:.4f} rad/s "
          f"(measured, NOT constrained by the QP)")
