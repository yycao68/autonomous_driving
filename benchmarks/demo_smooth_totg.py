"""Demonstrate that TOTG limitations (a) seam discontinuity and (b) limit
violations are FIXABLE by replacing circular blends with a C^2 smooth path,
while (c) reactivity and (d) path-prerequisite remain unchanged.

Run: python3 demo_smooth_totg.py
"""
import numpy as np
from di_planner import FR3_VMAX, FR3_AMAX
from di_totg import plan_totg
from benchmark_fr3 import metrics, b2_acute_corner
from benchmark_av import steering, av_double_lane_change, av_intersection_turn, V_MAX, A_MAX


print("=" * 70)
print("(a)/(b)  FR3 acute corner B2  --  circular blend vs C^2 spline")
print("=" * 70)
_, W, n, _ = b2_acute_corner(0.05)
vmax, amax = FR3_VMAX[:n], FR3_AMAX[:n]
print(f"{'path':>16}{'accel ratio':>13}{'accel viol':>12}{'accel jump':>12}{'jerk RMS':>10}")
for label, sm in [("circular blend", False), ("C^2 spline", True)]:
    tg = plan_totg(W, dt=0.01, vmax=vmax, amax=amax, max_dev=0.05, smooth=sm)
    m = metrics(tg, vmax, amax, 0.01)
    print(f"{label:>16}{m['a_ratio']:>13.3f}{m['a_viol']:>12}"
          f"{m['a_jump']:>12.2f}{m['jerk_rms']:>10.1f}")

print("\n" + "=" * 70)
print("(a)/(b)  AV steering  --  circular blend vs C^2 spline")
print("=" * 70)
for name, W in [("moose test", av_double_lane_change()[1]),
                ("sharp 90-deg turn", av_intersection_turn()[1])]:
    print(f"\n{name}:")
    print(f"{'path':>16}{'peak steer rate':>17}{'steer jump[rad]':>17}"
          f"{'peak a_lat':>12}")
    for label, sm in [("circular blend", False), ("C^2 spline", True)]:
        tg = plan_totg(W, dt=0.05, vmax=V_MAX, amax=A_MAX, max_dev=0.3, smooth=sm)
        st = steering(tg)
        print(f"{label:>16}{st['peak_drate']:>17.3f}{st['delta_jump']:>17.4f}"
              f"{st['peak_alat']:>12.2f}")

print("\n" + "=" * 70)
print("(c)/(d) UNCHANGED: C^2 spline is still an offline batch parameterization")
print("of a precomputed path -- no reactive bandwidth, still needs the path.")
print("=" * 70)
