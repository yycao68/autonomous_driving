"""
Timing comparison for the coupled FR3 obstacle QP.

Compares:
  1. Rebuild path: constructs a new OSQP object and sparse matrices every cycle.
  2. Fixed-sparsity path: sets up OSQP once and updates q/l/u/Ax in place.

Run:  python3 fixed_sparsity_timing.py
"""

import numpy as np

import fr3_dynamic_obstacle as fdo


def _stats(times):
    return 1e3 * np.array([
        np.mean(times),
        np.percentile(times, 95),
        np.max(times),
    ])


def main():
    rebuild = fdo.run_qp()
    fixed = fdo.run_qp_fixed_sparsity()

    rs = _stats(rebuild["solve_t"])
    fs = _stats(fixed["solve_t"])
    fc = _stats(fixed["cycle_t"])

    print("=" * 78)
    print("COUPLED FR3 OBSTACLE QP TIMING")
    print("=" * 78)
    print(f"{'implementation':<34}{'mean [ms]':>12}{'p95 [ms]':>12}{'max [ms]':>12}")
    print("-" * 78)
    print(f"{'rebuild OSQP each cycle (solve)':<34}{rs[0]:>12.2f}{rs[1]:>12.2f}{rs[2]:>12.2f}")
    print(f"{'fixed sparsity OSQP update (solve)':<34}{fs[0]:>12.2f}{fs[1]:>12.2f}{fs[2]:>12.2f}")
    print(f"{'fixed sparsity full Python cycle':<34}{fc[0]:>12.2f}{fc[1]:>12.2f}{fc[2]:>12.2f}")
    print()
    print("Fixed-sparsity trajectory check:")
    print(f"  reached={fixed['reached']}")
    print(f"  min_clearance={fixed['min_clr']:.3f} m")
    print(f"  joint_velocity_violations={fixed['v_viol']}")
    print(f"  joint_acceleration_violations={fixed['a_viol']}")
    print(f"  max_slack={fixed['max_slack']:.6f} m")


if __name__ == "__main__":
    main()
