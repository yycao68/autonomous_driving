"""Is the DI advantage robust to the choice of time-parameterizer?

Compares the two path-first TOPP variants on the same blended paths:
  NI  = numerical-integration TOPP (forward/backward sweep)
  RA  = reachability-based TOPP-RA (controllable sets, Pham 2018 [7])

If the seam discontinuity / accel-limit violation / near-reversal singularity are
the SAME for NI and RA, they are properties of the path-first paradigm (path
curvature + s->t reconstruction), not of a weak parameterizer -- so the DI
comparison holds against the modern robust TOPP too.
"""
import numpy as np
from di_planner import FR3_VMAX, FR3_AMAX
from di_totg import plan_totg
from benchmark_fr3 import metrics, b2_acute_corner, b3_near_reversal, b4_dense_noisy

SCN = [("B2 acute corner", b2_acute_corner(0.05), 0.05),
       ("B3 near-reversal", b3_near_reversal(0.05), 0.05),
       ("B4 dense (24 wp)", b4_dense_noisy(), 0.08)]

print(f"{'scenario':<18}{'TOPP':>6}{'T[s]':>9}{'accel ratio':>13}"
      f"{'accel viol':>12}{'min s_dot':>11}{'stalled':>9}")
for name, (label, W, n, kw), dev in SCN:
    vmax, amax = FR3_VMAX[:n], FR3_AMAX[:n]
    for m in ("ni", "ra"):
        tg = plan_totg(W, dt=0.01, vmax=vmax, amax=amax, max_dev=dev, method=m)
        mm = metrics(tg, vmax, amax, 0.01)
        print(f"{name if m=='ni' else '':<18}{m.upper():>6}{mm['T']:>9.2f}"
              f"{mm['a_ratio']:>13.3f}{mm['a_viol']:>12}{tg['min_sdot']:>11.4f}"
              f"{str(tg.get('stalled', False)):>9}")
    print()
