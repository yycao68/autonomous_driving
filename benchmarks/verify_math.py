"""
Numerical verification of every governing equation in the benchmark suite and in
motion_planning_double_integrator.md.  Each check prints PASS/FAIL with the residual.

Run:  python3 verify_math.py
"""

import numpy as np
from scipy.linalg import expm
from di_planner import di_matrices, plan_di, FR3_VMAX, FR3_AMAX
from di_totg import BlendPath, topp, plan_totg
from di_lateral import _tri_matrices, plan_lateral_jerk
from benchmark_av import steering

np.set_printoptions(precision=4, suppress=True)
results = []


def check(name, ok, resid):
    results.append((name, ok, resid))
    tag = "PASS" if ok else "FAIL"
    print(f"[{tag}] {name:<52} residual={resid:.2e}")


# E1. ZOH exactness for the double integrator: Ad=expm(Ac dt), Bd=int_0^dt e^{Ac t}B
def check_zoh_double():
    dt = 0.013
    Ac = np.array([[0., 1.], [0., 0.]]); B = np.array([[0.], [1.]])
    Ad = expm(Ac * dt)
    Ad_claim = np.array([[1, dt], [0, 1]])
    # Bd by fine quadrature of int_0^dt expm(Ac t) B dt
    ts = np.linspace(0, dt, 20001)
    integ = np.trapezoid([expm(Ac * t) @ B for t in ts], ts, axis=0).ravel()
    Bd_claim = np.array([dt * dt / 2, dt])
    r = max(np.abs(Ad - Ad_claim).max(), np.abs(integ - Bd_claim).max())
    check("E1  double-integrator ZOH  Ad,Bd", r < 1e-7, r)


# E2. Triple-integrator ZOH: Ad=expm, Bd=[dt^3/6, dt^2/2, dt]
def check_zoh_triple():
    dt = 0.013
    Ac = np.array([[0, 1, 0], [0, 0, 1], [0, 0, 0]], float); B = np.array([[0.], [0.], [1.]])
    Ad = expm(Ac * dt)
    Ad_claim = np.array([[1, dt, dt * dt / 2], [0, 1, dt], [0, 0, 1]])
    ts = np.linspace(0, dt, 20001)
    integ = np.trapezoid([expm(Ac * t) @ B for t in ts], ts, axis=0).ravel()
    Bd_claim = np.array([dt ** 3 / 6, dt ** 2 / 2, dt])
    r = max(np.abs(Ad - Ad_claim).max(), np.abs(integ - Bd_claim).max())
    check("E2  triple-integrator ZOH  Ad,Bd", r < 1e-7, r)


# E3. DI prediction matrices Phi,Gamma vs brute-force state rollout
def check_di_prediction():
    dt, N = 0.02, 12
    Phi_p, Phi_v, Gam_p, Gam_v = di_matrices(dt, N)
    Ad = np.array([[1, dt], [0, 1]]); Bd = np.array([dt * dt / 2, dt])
    rng = np.random.default_rng(0)
    x0 = rng.standard_normal(2); U = rng.standard_normal(N)
    pos = np.zeros(N); vel = np.zeros(N); x = x0.copy()
    for k in range(N):
        x = Ad @ x + Bd * U[k]
        pos[k], vel[k] = x
    pos_pred = Phi_p @ x0 + Gam_p @ U
    vel_pred = Phi_v @ x0 + Gam_v @ U
    r = max(np.abs(pos - pos_pred).max(), np.abs(vel - vel_pred).max())
    check("E3  DI prediction Phi,Gamma vs rollout", r < 1e-12, r)


# E4. Triple-integrator prediction matrices vs brute-force rollout
def check_tri_prediction():
    dt, N = 0.02, 12
    Phi_p, Phi_v, Phi_a, Gp, Gv, Ga = _tri_matrices(dt, N)
    Ad = np.array([[1, dt, dt * dt / 2], [0, 1, dt], [0, 0, 1]])
    Bd = np.array([dt ** 3 / 6, dt ** 2 / 2, dt])
    rng = np.random.default_rng(1)
    z0 = rng.standard_normal(3); U = rng.standard_normal(N)
    P = np.zeros(N); V = np.zeros(N); A = np.zeros(N); z = z0.copy()
    for k in range(N):
        z = Ad @ z + Bd * U[k]
        P[k], V[k], A[k] = z
    r = max(np.abs(P - (Phi_p @ z0 + Gp @ U)).max(),
            np.abs(V - (Phi_v @ z0 + Gv @ U)).max(),
            np.abs(A - (Phi_a @ z0 + Ga @ U)).max())
    check("E4  triple-int prediction vs rollout", r < 1e-12, r)


# E5. QP cost algebra: H,h reproduce the explicit weighted least-squares objective
def check_cost_algebra():
    dt, N = 0.02, 10
    Phi_p, Phi_v, Gam_p, Gam_v = di_matrices(dt, N)
    wq, wv, wr, gamma = 50., 3., 0.05, 8.
    Wq = np.full(N, wq); Wq[-1] *= gamma
    Wv = np.full(N, wv); Wv[-1] *= gamma
    H = Gam_p.T @ (Wq[:, None] * Gam_p) + Gam_v.T @ (Wv[:, None] * Gam_v) + wr * np.eye(N)
    rng = np.random.default_rng(2)
    x0 = rng.standard_normal(2); qg = 0.7; vg = 0.0
    h = Gam_p.T @ (Wq * (Phi_p @ x0 - qg)) + Gam_v.T @ (Wv * (Phi_v @ x0 - vg))
    U = rng.standard_normal(N)
    # explicit objective J(U) = sum Wq(pos-qg)^2 + Wv(vel-vg)^2 + wr u^2
    pos = Phi_p @ x0 + Gam_p @ U; vel = Phi_v @ x0 + Gam_v @ U
    J = (Wq * (pos - qg) ** 2).sum() + (Wv * (vel - vg) ** 2).sum() + wr * (U @ U)
    # quadratic form value: U'HU + 2 h'U + const  (const independent of U)
    quad = U @ H @ U + 2 * h @ U
    # compare gradients (const drops out): dJ/dU vs 2HU + 2h
    gradJ = 2 * (Gam_p.T @ (Wq * (pos - qg)) + Gam_v.T @ (Wv * (vel - vg))) + 2 * wr * U
    grad_form = 2 * H @ U + 2 * h
    r = np.abs(gradJ - grad_form).max()
    check("E5  QP cost: gradient of H,h vs explicit J", r < 1e-10, r)


# E6. Blend path interpolates endpoints exactly and |q'(s)|=1 (arc-length param)
def check_path_endpoints_unit_tangent():
    W = np.array([[0, 0, 0], [1, 0, 0.2], [0.4, 0.9, 0.2]], float)
    p = BlendPath(W, max_dev=0.1)
    q0, _, _ = p.eval(0.0); qL, _, _ = p.eval(p.L)
    ss = np.linspace(0, p.L, 500)
    tang_norm = np.array([np.linalg.norm(p.eval(s)[1]) for s in ss])
    r = max(np.abs(q0 - W[0]).max(), np.abs(qL - W[-1]).max(),
            np.abs(tang_norm - 1.0).max())
    check("E6  path endpoints + unit tangent |q'|=1", r < 1e-6, r)


# E7. Blend path is C^1 (tangent continuous) but q'' jumps (0 on line, 1/r on arc)
def check_path_C1_and_curvature_jump():
    W = np.array([[0, 0, 0], [1, 0, 0.2], [0.4, 0.9, 0.2]], float)
    p = BlendPath(W, max_dev=0.1)
    ss = np.linspace(1e-4, p.L - 1e-4, 4000)
    tang = np.array([p.eval(s)[1] for s in ss])
    # tangent continuity: consecutive tangents nearly aligned (no flip)
    dots = np.einsum('ij,ij->i', tang[:-1], tang[1:])
    c1_resid = 1.0 - dots.min()                     # ~0 if C^1
    # curvature magnitude: should be ~0 (line) or ~1/r (arc); find both regimes
    curv = np.array([np.linalg.norm(p.eval(s)[2]) for s in ss])
    arc_r = p.min_blend_r
    has_line = (curv < 1e-6).any()
    has_arc = np.any(np.abs(curv - 1.0 / arc_r) < 1e-3 * (1.0 / arc_r))
    ok = (c1_resid < 1e-4) and has_line and has_arc
    check("E7  path C^1 + curvature jump 0 -> 1/r", ok, c1_resid)


# E8. Blend deviation <= max_dev (path stays within tolerance of each corner)
def check_blend_deviation():
    W = np.array([[0, 0, 0], [1, 0, 0.2], [0.4, 0.9, 0.2]], float)
    max_dev = 0.1
    p = BlendPath(W, max_dev=max_dev)
    ss = np.linspace(0, p.L, 6000)
    pts = np.array([p.eval(s)[0] for s in ss])
    d_corner = np.linalg.norm(pts - W[1], axis=1).min()   # closest approach
    check("E8  blend deviation <= max_dev", d_corner <= max_dev + 1e-3, d_corner)


# E9. TOPP path-domain feasibility: along the time-optimal profile,
#     |q_dot_j| <= vmax_j  AND  an admissible s_ddot interval exists at every grid pt
def check_topp_feasibility():
    from di_totg import _u_interval
    W = np.array([[0, 0, 0], [1, 0, 0.2], [0.4, 0.9, 0.2]], float)
    vmax, amax = FR3_VMAX[:3], FR3_AMAX[:3]
    p = BlendPath(W, max_dev=0.1)
    res = topp(p, vmax, amax, K=1200)
    s, x, QP, QPP = res["s"], res["x"], res["QP"], res["QPP"]
    sdot = np.sqrt(np.maximum(x, 0))
    qdot = QP * sdot[:, None]                      # joint velocity along path
    vel_ok = (np.abs(qdot) <= vmax + 1e-6).all()
    vel_resid = (np.abs(qdot) - vmax).max()
    feas = True
    for k in range(len(s)):
        _, _, ok = _u_interval(QP[k], QPP[k], x[k], vmax, amax)
        feas = feas and ok
    check("E9  TOPP path-domain |q_dot|<=vmax", vel_ok, vel_resid)
    check("E9b TOPP accel interval feasible all s", feas, 0.0 if feas else 1.0)


# E10. Time-domain integrator consistency of the DI planner output
#      V[k]=V[k-1]+dt A[k];  Q[k]=Q[k-1]+dt V[k-1]+0.5 dt^2 A[k]
def check_di_time_consistency():
    W = np.array([[0, 0, 0], [1, -0.5, 0.3], [1.5, 0.4, -0.4]], float)
    dt = 0.01
    r = plan_di(W, dt=dt, vmax=FR3_VMAX[:3], amax=FR3_AMAX[:3])
    Q, V, A = r["Q"], r["V"], r["A"]
    rv = np.abs(V[1:] - (V[:-1] + dt * A[1:])).max()
    rq = np.abs(Q[1:] - (Q[:-1] + dt * V[:-1] + 0.5 * dt * dt * A[1:])).max()
    check("E10 DI time-domain integrator consistency", max(rv, rq) < 1e-9, max(rv, rq))


# E11. Triple-integrator (lateral) time-domain consistency
#      a[k]=a[k-1]+dt j[k]; v=...; y=...
def check_tri_time_consistency():
    W = np.array([[0, 0], [15, 0], [30, 3.5], [45, 3.5], [60, 0], [80, 0]], float)
    dt = 0.05
    r = plan_lateral_jerk(W, V=13.0, dt=dt)
    Y, V, A, J = r["Y"], r["V"][:, 1], r["A"][:, 1], r["J"]
    ra = np.abs(A[1:] - (A[:-1] + dt * J[1:])).max()
    rv = np.abs(V[1:] - (V[:-1] + dt * A[:-1] + 0.5 * dt * dt * J[1:])).max()
    ry = np.abs(Y[1:] - (Y[:-1] + dt * V[:-1] + 0.5 * dt * dt * A[:-1]
                         + dt ** 3 / 6 * J[1:])).max()
    check("E11 triple-int time-domain consistency", max(ra, rv, ry) < 1e-9,
          max(ra, rv, ry))


# E12. Steering / curvature formula on a synthetic constant-curvature circle
#      delta = atan(L*kappa), kappa = 1/R for a circle of radius R
def check_steering_formula():
    L, R, V = 2.7, 25.0, 13.0
    dt = 0.02
    t = np.arange(0, 2.0, dt)
    w = V / R                                  # angular rate (constant speed V)
    # analytic velocity/acceleration of a circle of radius R traversed at speed V
    vx = V * np.cos(w * t); vy = V * np.sin(w * t)
    ax = -V * w * np.sin(w * t); ay = V * w * np.cos(w * t)
    res = dict(V=np.column_stack([vx, vy]), A=np.column_stack([ax, ay]), dt=dt)
    sd = steering(res, L=L)
    m = sd["mask"]
    kappa_err = np.abs(sd["kappa"][m] - 1.0 / R).max()
    delta_err = np.abs(sd["delta"][m] - np.arctan(L / R)).max()
    r = max(kappa_err, delta_err)
    check("E12 steering: kappa=1/R, delta=atan(L/R)", r < 1e-2, r)


# E13. s_ddot = 0.5 d(s_dot^2)/ds  used in the s->t reconstruction
def check_sddot_relation():
    s = np.linspace(0, 3, 4000)
    sdot = 0.5 + 0.4 * np.sin(s)               # arbitrary smooth profile
    x = sdot ** 2
    sdd_numeric = 0.5 * np.gradient(x, s)
    # analytic: s_ddot = s_dot d(s_dot)/ds
    sdd_analytic = sdot * np.gradient(sdot, s)
    r = np.abs(sdd_numeric - sdd_analytic).max()
    check("E13 s_ddot = 0.5 d(s_dot^2)/ds", r < 1e-3, r)


if __name__ == "__main__":
    check_zoh_double()
    check_zoh_triple()
    check_di_prediction()
    check_tri_prediction()
    check_cost_algebra()
    check_path_endpoints_unit_tangent()
    check_path_C1_and_curvature_jump()
    check_blend_deviation()
    check_topp_feasibility()
    check_di_time_consistency()
    check_tri_time_consistency()
    check_steering_formula()
    check_sddot_relation()
    npass = sum(1 for _, ok, _ in results if ok)
    print(f"\n{npass}/{len(results)} checks passed.")
