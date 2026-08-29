"""
TOPP-RA-style phase-plane figure (cf. Pham & Pham, arXiv:1707.07239).

The (s, s_dot) phase plane is the canonical way to visualize a time-optimal path
parameterization: the Maximum-Velocity Curve (MVC), the reachability/controllable
set, and the optimal speed profile that rides the limit curve in a bang-bang
fashion.  We reproduce it for the path-first baseline on the acute-corner and
near-reversal scenarios, which also exposes *where* the s->t conversion of the
main paper becomes ill-conditioned: the MVC (hence s_dot) dips toward zero at the
tight blend, and the time map t(s)=∫ds/s_dot blows up there.

-> topp_phaseplane.png
"""
import numpy as np
from pathlib import Path
from di_planner import FR3_VMAX, FR3_AMAX
from di_totg import BlendPath, topp, topp_ra
from benchmark_fr3 import b2_acute_corner, b3_near_reversal

RESULTS_DIR = Path(__file__).parent.parent / "simulationResults"


def panels(ax_pp, ax_jv, title, W, max_dev, n=7):
    vmax, amax = FR3_VMAX[:n], FR3_AMAX[:n]
    path = BlendPath(W, max_dev=max_dev)
    ni = topp(path, vmax, amax, K=800)
    ra = topp_ra(path, vmax, amax, K=800)
    s = ni["s"]
    mvc = np.sqrt(np.maximum(ni["MVC"], 0))
    sd_ni = np.sqrt(np.maximum(ni["x"], 0))
    sd_ra = np.sqrt(np.maximum(ra["x"], 0))
    ctrl = np.sqrt(np.maximum(ra["xbar"], 0))           # controllable-set boundary

    # phase plane (s, s_dot)
    ax_pp.plot(s, mvc, 'k-', lw=1.6, label='MVC (velocity limit)')
    ax_pp.plot(s, ctrl, color='orange', lw=1.2, ls='--', label='controllable set (TOPP-RA)')
    ax_pp.plot(s, sd_ra, 'b-', lw=1.8, label='optimal profile $\\dot s^*(s)$')
    ax_pp.fill_between(s, 0, np.minimum(mvc, ctrl), color='b', alpha=0.06)
    imin = int(np.argmin(mvc))
    ax_pp.annotate('MVC dip at blend\n($\\dot s\\!\\to$ small $\\Rightarrow t(s)$ ill-cond.)',
                   xy=(s[imin], mvc[imin]), xytext=(s[imin], mvc.max()*0.55),
                   fontsize=8, ha='center', arrowprops=dict(arrowstyle='->', lw=0.8))
    ax_pp.set_title(title); ax_pp.set_xlabel('path parameter $s$ [rad]')
    ax_pp.set_ylabel('$\\dot s$ [1/s]'); ax_pp.legend(fontsize=8); ax_pp.set_ylim(bottom=0)

    # normalized joint velocities q_dot_j / q_dot_j,max  (saturation reads as +/-1)
    QP = ni["QP"]
    for j in range(n):
        ax_jv.plot(s, QP[:, j] * sd_ra / vmax[j], lw=1.0)
    ax_jv.axhline(1.0, ls='--', c='r', lw=0.8); ax_jv.axhline(-1.0, ls='--', c='r', lw=0.8)
    ax_jv.set_title('normalized joint velocity $\\dot q_j/\\dot q_{j,\\max}$ '
                    '(a joint rides $\\pm1$ = MVC active)')
    ax_jv.set_xlabel('path parameter $s$ [rad]'); ax_jv.set_ylabel('$\\dot q_j/\\dot q_{j,\\max}$')
    ax_jv.set_ylim(-1.2, 1.2)
    return ni, ra


def main():
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    panels(ax[0, 0], ax[1, 0], 'B2 acute corner', b2_acute_corner(0.10)[1], 0.10)
    panels(ax[0, 1], ax[1, 1], 'B3 near-reversal', b3_near_reversal(0.10)[1], 0.10)
    fig.suptitle('Time-optimal path parameterization in the $(s,\\dot s)$ phase plane '
                 '(TOPP-RA style)', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out = RESULTS_DIR / "topp_phaseplane.png"
    fig.savefig(out, dpi=120)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
