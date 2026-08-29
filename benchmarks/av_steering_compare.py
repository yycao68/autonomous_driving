"""
Visualize the AV steering results (§VI.C) as curves.

Top: moose test (double lane change) at cruise -- steering angle, steering rate,
and curvature for the jerk-bounded DI backbone vs TOTG.  The jerk-bounded planner
is smooth and bounded; TOTG steps at every blend seam.
Bottom-right: the sharp 90-deg turn, peak TOTG steering rate vs sampling dt --
it diverges as 1/dt, i.e. it samples a genuine steering discontinuity.
-> av_steering_compare.png
"""
import numpy as np
from pathlib import Path
from di_lateral import plan_lateral_jerk
from di_totg import plan_totg
from benchmark_av import (steering, av_double_lane_change, av_intersection_turn,
                          V_MAX, A_MAX, V_CRUISE)

RESULTS_DIR = Path(__file__).parent.parent / "simulationResults"


def main():
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    wps = av_double_lane_change()[1]
    di = plan_lateral_jerk(wps, V=V_CRUISE, dt=0.05)
    tg = plan_totg(wps, dt=0.05, vmax=V_MAX, amax=A_MAX, max_dev=0.3)
    sd, st = steering(di), steering(tg)

    fig, ax = plt.subplots(2, 2, figsize=(12, 7.5))
    # steering angle
    ax[0, 0].plot(di["t"], np.degrees(sd["delta"]), 'b', lw=1.6, label='jerk-DI')
    ax[0, 0].plot(tg["t"], np.degrees(st["delta"]), 'r', lw=1.3, label='TOTG')
    ax[0, 0].set_title('steering angle $\\delta$ (moose test)'); ax[0,0].set_ylabel('deg')
    ax[0, 0].set_xlabel('t [s]'); ax[0, 0].legend(fontsize=8)
    # steering rate
    ax[0, 1].plot(di["t"], sd["drate"], 'b', lw=1.6, label=f"jerk-DI (peak {np.abs(sd['drate'][sd['mask']]).max():.2f})")
    ax[0, 1].plot(tg["t"], st["drate"], 'r', lw=1.3, label=f"TOTG (peak {np.abs(st['drate'][st['mask']]).max():.2f})")
    for lim in (0.5, -0.5):
        ax[0, 1].axhline(lim, ls='--', c='k', lw=0.9)
    ax[0, 1].set_title('steering rate $\\dot\\delta$ (dashed = 0.5 rad/s actuator cap)')
    ax[0, 1].set_ylabel('rad/s'); ax[0, 1].set_xlabel('t [s]'); ax[0, 1].legend(fontsize=8)
    # curvature
    ax[1, 0].plot(di["t"], sd["kappa"], 'b', lw=1.6, label='jerk-DI')
    ax[1, 0].plot(tg["t"], st["kappa"], 'r', lw=1.3, label='TOTG (stepped)')
    ax[1, 0].set_title('path curvature $\\kappa$'); ax[1,0].set_ylabel('1/m')
    ax[1, 0].set_xlabel('t [s]'); ax[1, 0].legend(fontsize=8)
    # sharp-turn steering-rate divergence vs dt
    Wt = av_intersection_turn()[1]
    dts = np.array([0.1, 0.05, 0.02, 0.01, 0.005])
    peaks = []
    for dt in dts:
        s = steering(plan_totg(Wt, dt=dt, vmax=V_MAX, amax=A_MAX, max_dev=0.3))
        peaks.append(s["peak_drate"])
    ax[1, 1].loglog(dts, peaks, 'rs-', label='TOTG peak $\\dot\\delta$')
    ax[1, 1].loglog(dts, peaks[0] * dts[0] / dts, 'k--', lw=0.8, label='$\\propto 1/\\Delta t$')
    ax[1, 1].set_title('sharp 90$^\\circ$ turn: TOTG steering rate diverges as $1/\\Delta t$')
    ax[1, 1].set_xlabel('sampling $\\Delta t$ [s]'); ax[1, 1].set_ylabel('peak $\\dot\\delta$ [rad/s]')
    ax[1, 1].legend(fontsize=8); ax[1, 1].grid(True, which='both', alpha=0.3)
    fig.suptitle('AV steering: jerk-bounded DI is smooth & bounded; TOTG steps at '
                 'blend seams (a sampled discontinuity)', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = RESULTS_DIR / "av_steering_compare.png"
    fig.savefig(out, dpi=120)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
