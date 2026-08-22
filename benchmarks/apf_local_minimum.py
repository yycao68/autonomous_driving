"""
APF local-minimum demonstration and waypoint-injection escape.

A planar double-integrator point robot must move from START to GOAL while a disk
obstacle lies exactly on the line of sight.  With a purely radial artificial
potential field (attraction to the goal + repulsion from the obstacle), symmetry
creates a stable equilibrium in front of the obstacle.  Waypoint injection breaks
that symmetry by temporarily targeting a lateral via point.

Run:  python3 apf_local_minimum.py
"""

import numpy as np

from di_planner import JointQP


START = np.array([0.0, 0.0])
GOAL = np.array([10.0, 0.0])
OBS = np.array([5.0, 0.0])
R_OBS = 0.55
R_SAFE = 0.85
RHO0 = 3.2
DT = 0.01
T_MAX = 18.0
VMAX = np.array([1.5, 1.5])
AMAX = np.array([3.0, 3.0])


def _apf_velocity(p, goal, eta=11.0, k_att=0.95):
    """Radial APF velocity reference. No tangential/swirl term."""
    dg = goal - p
    v_att = k_att * dg
    d = p - OBS
    dn = np.linalg.norm(d)
    v_rep = np.zeros(2)
    if dn < RHO0:
        rad = d / max(dn, 1e-9)
        mag = eta * (1.0 / dn - 1.0 / RHO0) / (dn * dn)
        v_rep = mag * rad
    return np.clip(v_att + v_rep, -VMAX, VMAX)


def run(use_waypoint=False):
    qpx = JointQP(DT, 20, VMAX[0], AMAX[0], wq=2.0, wv=20.0, wr=0.05, gamma=6.0)
    qpy = JointQP(DT, 20, VMAX[1], AMAX[1], wq=2.0, wv=20.0, wr=0.05, gamma=6.0)
    x = np.array([[START[0], 0.0], [START[1], 0.0]])
    P, V, A, solve_t = [], [], [], []
    solver_failures = 0
    min_clr = np.inf
    injected = False
    wp_index = 0
    stall_count = 0
    prev_goal_dist = np.linalg.norm(GOAL - START)
    waypoints = [
        OBS + np.array([-1.0, R_SAFE + 1.35]),
        OBS + np.array([1.5, R_SAFE + 1.35]),
        GOAL,
    ]

    for k in range(int(T_MAX / DT)):
        p = x[:, 0].copy()
        min_clr = min(min_clr, np.linalg.norm(p - OBS) - R_OBS)
        goal_dist = np.linalg.norm(GOAL - p)

        if goal_dist > prev_goal_dist - 1e-4 and np.linalg.norm(x[:, 1]) < 0.05:
            stall_count += 1
        else:
            stall_count = 0
        prev_goal_dist = goal_dist

        target = GOAL
        if use_waypoint:
            if (not injected) and stall_count * DT > 0.6:
                injected = True
            if injected:
                target = waypoints[wp_index]
                if np.linalg.norm(p - target) < 0.35 and wp_index < len(waypoints) - 1:
                    wp_index += 1
                    target = waypoints[wp_index]

        v_des = _apf_velocity(p, target)
        Ux, tx, okx = qpx.solve(x[0], target[0], v_des[0])
        Uy, ty, oky = qpy.solve(x[1], target[1], v_des[1])
        solver_failures += (0 if okx else 1) + (0 if oky else 1)
        u = np.array([Ux[0], Uy[0]])
        solve_t.append(tx + ty)
        for i in range(2):
            q0, v0 = x[i]
            x[i, 0] = q0 + DT * v0 + 0.5 * DT * DT * u[i]
            x[i, 1] = v0 + DT * u[i]
        P.append(x[:, 0].copy())
        V.append(x[:, 1].copy())
        A.append(u.copy())
        if np.linalg.norm(x[:, 0] - GOAL) < 0.12:
            break

    P, V, A = np.array(P), np.array(V), np.array(A)
    jerk = np.diff(A, axis=0) / DT
    return dict(
        P=P, V=V, A=A, t=np.arange(len(P)) * DT, solve_t=np.array(solve_t),
        reached=np.linalg.norm(P[-1] - GOAL) < 0.15,
        final_dist=float(np.linalg.norm(P[-1] - GOAL)),
        min_clr=float(min_clr),
        injected=injected,
        max_speed=float(np.abs(V).max()),
        max_accel=float(np.abs(A).max()),
        jerk_rms=float(np.sqrt((jerk ** 2).mean())) if jerk.size else 0.0,
        solver_failures=solver_failures,
    )


def main():
    pure = run(use_waypoint=False)
    escape = run(use_waypoint=True)
    print("=" * 72)
    print("APF LOCAL MINIMUM AND WAYPOINT-INJECTION ESCAPE")
    print("=" * 72)
    print(f"{'metric':<28}{'pure APF':>18}{'waypoint injection':>22}")
    print("-" * 72)
    rows = [
        ("reached goal", str(pure["reached"]), str(escape["reached"])),
        ("completion/final time [s]", f"{pure['t'][-1]:.2f}", f"{escape['t'][-1]:.2f}"),
        ("final distance to goal [m]", f"{pure['final_dist']:.2f}", f"{escape['final_dist']:.2f}"),
        ("min obstacle clearance [m]", f"{pure['min_clr']:.2f}", f"{escape['min_clr']:.2f}"),
        ("waypoint injected", str(pure["injected"]), str(escape["injected"])),
        ("max speed [m/s]", f"{pure['max_speed']:.2f}", f"{escape['max_speed']:.2f}"),
        ("max accel [m/s^2]", f"{pure['max_accel']:.2f}", f"{escape['max_accel']:.2f}"),
        ("jerk RMS [m/s^3]", f"{pure['jerk_rms']:.1f}", f"{escape['jerk_rms']:.1f}"),
        ("mean solve [ms]", f"{1e3*pure['solve_t'].mean():.3f}", f"{1e3*escape['solve_t'].mean():.3f}"),
    ]
    for label, a, b in rows:
        print(f"{label:<28}{a:>18}{b:>22}")


if __name__ == "__main__":
    main()
