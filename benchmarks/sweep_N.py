"""Sweep the DI-QP horizon N and report whether a longer horizon helps.

Metrics per scenario: time-to-goal, accel violations (must stay 0), jerk RMS,
mean/peak QP solve time (cost of a longer horizon).  A second sweep uses the
FR3 dynamic-obstacle benchmark, adding end-effector clearance.
"""
import numpy as np
from di_planner import plan_di, FR3_VMAX, FR3_AMAX
from benchmark_fr3 import (metrics, b1_point_to_point, b2_acute_corner,
                           b4_dense_noisy)
import fr3_dynamic_obstacle as fdo

NS = [10, 20, 30, 40]
SCN = [b1_point_to_point(), b2_acute_corner(0.05), b4_dense_noisy()]


def _jerk_rms(A, dt):
    J = np.diff(A, axis=0) / dt
    return float(np.sqrt((J ** 2).mean())) if J.size else 0.0


def static_sweep():
    for name, W, n, kw in SCN:
        vmax, amax = FR3_VMAX[:n], FR3_AMAX[:n]
        print(f"\n=== {name} ===")
        print(f"{'N':>4}{'T [s]':>10}{'a-viol':>8}{'jerkRMS':>10}"
              f"{'solve_mean[ms]':>16}{'solve_max[ms]':>15}")
        for N in NS:
            r = plan_di(W, dt=0.01, N=N, vmax=vmax, amax=amax)
            m = metrics(r, vmax, amax, 0.01)
            print(f"{N:>4}{m['T']:>10.3f}{m['a_viol']:>8}{m['jerk_rms']:>10.1f}"
                  f"{1e3*r['solve_t'].mean():>16.3f}{1e3*r['solve_t'].max():>15.3f}")


def dynamic_obstacle_sweep():
    old_N = fdo.N
    print("\n=== FR3 dynamic obstacle, fixed-sparsity coupled QP ===")
    print(f"{'N':>4}{'T [s]':>10}{'min clr[m]':>12}{'jerkRMS':>10}"
          f"{'solve_mean[ms]':>16}{'solve_p95[ms]':>15}{'cycle_p95[ms]':>15}")
    try:
        for N in NS:
            fdo.N = N
            r = fdo.run_qp_fixed_sparsity()
            T = r["t"][-1]
            jrms = _jerk_rms(r["A"], fdo.DT)
            print(f"{N:>4}{T:>10.3f}{r['min_clr']:>12.3f}{jrms:>10.1f}"
                  f"{1e3*r['solve_t'].mean():>16.3f}"
                  f"{1e3*np.percentile(r['solve_t'], 95):>15.3f}"
                  f"{1e3*np.percentile(r['cycle_t'], 95):>15.3f}")
    finally:
        fdo.N = old_N


if __name__ == "__main__":
    static_sweep()
    dynamic_obstacle_sweep()
