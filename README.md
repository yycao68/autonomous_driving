# Reactive motion planning with a configuration-independent time-domain backbone

Paper source in `paper/` (`mp_main.tex` includes `body.tex`); benchmarks in
`benchmarks/`; figures and trajectory artifacts in `simulationResults/`.

## Reproducing the benchmarks — read this first

**A clone of this repository alone cannot run most of the benchmarks.**
`benchmarks/di_planner.py`, `di_totg.py`, `benchmark_fr3.py`,
`fr3_kinematics.py`, `fr3_hessian_norm.py`, `improve_test.py`,
`benchmark_dynamic.py`, `mujoco_compare.py` and `fr3_dynamic_obstacle.py` are
*importlib shims*. They re-export the real modules from a **sibling**
`avoidance_obstacle/sim/` repository, which is the shared single source of
truth for the code behind both papers. This is deliberate for day-to-day
development — it prevents the two papers from forking the planner — but it
means the shims dangle in a standalone clone, and `benchmark_av.py` and
`verify_math.py` fail at import with a missing `avoidance_obstacle/sim/
di_planner.py`. An external review hit exactly this.

There are two supported ways to run everything:

**1. Use the frozen standalone snapshot (no second repo needed).**
`benchmarks_standalone.zip` is tracked in this repository and contains
`benchmarks/` with every shim replaced by the real module's actual content,
plus a `PROVENANCE.txt` recording the exact commit of both repositories the
snapshot was built from.

```bash
unzip benchmarks_standalone.zip -d /tmp/mp_bench
cd /tmp/mp_bench/benchmarks
python3 verify_math.py                # 14 equation checks
python3 benchmark_av.py               # AV steering comparison
python3 fr3_dynamic_randomized.py     # 20-trial randomized FR3 table (seed 7)
```

Each script prints its paper table to stdout. Raw per-trial dumps are not
committed; `simulationResults/` holds the rendered figures and the AV
trajectory export, and the reproduction path is to rerun the cited script.

**2. Clone both repositories as siblings** (the development layout):

```
parent/
  autonomous_driving/     # this repo
  avoidance_obstacle/     # sibling providing sim/
```

Then run the scripts in `benchmarks/` directly. Rebuild the snapshot after
changing anything with `python3 benchmarks/make_release_zip.py`; it refuses to
build if the sibling is missing, and stamps `PROVENANCE.txt` with
`(dirty working tree!)` if either repository has uncommitted changes.

## Python dependencies

`numpy`, `scipy`, `osqp`, `matplotlib`; `mujoco` additionally for
`mujoco_compare.py` and the FR3 rendering scripts. No lockfile is pinned; the
committed results were produced with the versions in the snapshot's
`PROVENANCE.txt` commit.

## What the AV planner does and does not constrain

`benchmarks/di_lateral.py` constrains **lateral velocity, lateral acceleration,
and lateral jerk**. It contains no steering or steering-rate row. Bounding jerk
bounds steering rate only through the bicycle relation

```
delta = arctan( L (vx*ay - vy*ax) / (vx^2 + vy^2)^(3/2) )
```

whose dependence on lateral velocity and acceleration must be accounted for
before a given `j_max` implies a particular steering-rate cap. Use
`steering_rate_report()` to *measure* the resulting angle and rate from a
planned trajectory; the paper reports this as measured compliance at the tested
cruise speed, not as a constraint enforced by the QP.

`plan_lateral_jerk()` returns `status`, `completed`, `n_qp_failures`,
`qp_statuses`, `max_primal_residual`, `terminal_error`, `terminal_vy` and
`terminal_ay`. Check them: a plan that stalls or fails to reach the goal is
reported as `status='incomplete'`, and a solver failure as `status='qp_failure'`
(pass `strict=True` to raise instead). Completion uses `goal_tol` (default
0.05 m) on lateral position only and does not require the terminal lateral
velocity or acceleration to be near zero — read `terminal_vy`/`terminal_ay` if
that matters for your use.

## Building the paper

```bash
cd paper
latexmk -pdf mp_main.tex
```

Needs a LaTeX install providing `IEEEtran`, `cite`, `amsmath`, `amssymb`,
`graphicx`, `booktabs` and `hyperref` (TeX Live `collection-latexrecommended`;
a minimal install may need `tlmgr install cite IEEEtran booktabs`). Verified
building clean — 10 pages, 0 undefined references — on TeX Live 2026.
