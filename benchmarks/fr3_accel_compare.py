"""
Visualize the FR3 results (§VI.B) as curves: the peak-acceleration ratio
max_j |q_ddot_j|/q_ddot_{j,max} over time for the DI planner vs TOTG, with the
limit at 1.0.  Values above 1.0 are emitted-trajectory acceleration-limit
violations (TOTG); the DI planner is pinned at <=1.0 by its hard constraints.
-> fr3_accel_compare.png
"""
import numpy as np
from pathlib import Path
from di_planner import plan_di, FR3_VMAX, FR3_AMAX
from di_totg import plan_totg
from benchmark_fr3 import (b1_point_to_point, b2_acute_corner, b3_near_reversal,
                           b4_dense_noisy)


def ratio(res, amax):
    return (np.abs(res["A"]) / amax).max(axis=1)


def main():
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    scn = [("B1 point-to-point", b1_point_to_point()[1], 0.10),
           ("B2 acute corner", b2_acute_corner(0.05)[1], 0.05),
           ("B3 near-reversal", b3_near_reversal(0.05)[1], 0.05),
           ("B4 dense (24 wp)", b4_dense_noisy()[1], 0.08)]
    n = 7; vmax, amax = FR3_VMAX[:n], FR3_AMAX[:n]
    fig, axes = plt.subplots(2, 2, figsize=(12, 7.5))
    for ax, (name, W, dev) in zip(axes.ravel(), scn):
        di = plan_di(W, dt=0.01, vmax=vmax, amax=amax)
        tg = plan_totg(W, dt=0.01, vmax=vmax, amax=amax, max_dev=dev)
        rd, rt = ratio(di, amax), ratio(tg, amax)
        ax.plot(di["t"], rd, 'b-', lw=1.6, label=f'DI-QP (peak {rd.max():.2f})')
        ax.plot(tg["t"], rt, 'r-', lw=1.4, label=f'TOTG (peak {rt.max():.2f})')
        ax.axhline(1.0, ls='--', c='k', lw=1.0)
        ax.fill_between(tg["t"], 1.0, rt, where=(rt > 1.0), color='r', alpha=0.25)
        ax.set_title(name); ax.set_xlabel('t [s]')
        ax.set_ylabel(r'accel ratio  $\max_j |\ddot q_j|/\ddot q_{j,\max}$')
        ax.legend(fontsize=8); ax.set_ylim(0, max(1.6, rt.max() * 1.1))
        ax.text(0.02, 0.93, 'limit = 1.0', transform=ax.transAxes, fontsize=8)
    fig.suptitle('FR3: emitted-trajectory acceleration ratio — DI-QP stays within '
                 'the limit (hard constraint); TOTG exceeds it (red = violation)',
                 fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = Path(__file__).with_name("fr3_accel_compare.png")
    fig.savefig(out, dpi=120)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
