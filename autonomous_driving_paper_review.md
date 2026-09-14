# Autonomous-Driving Paper Review

Review date: 2026-09-12

## Scope

The maintained manuscripts are:

- `paper/mp_main.tex` with `paper/body.tex` (full IEEE version)
- `paper/mp_main_ICRA.tex` with `paper/body_icra.tex` (English ICRA version)
- `paper/mp_main_ICRA_zh.tex` with `paper/body_icra_zh.tex` (Chinese ICRA version)

`paper/body_raw.tex` is not included by any maintained wrapper and was not edited.
The review checked mathematical consistency, implementation-to-paper settings,
benchmark reproducibility, timing claims, English/Chinese synchronization, and
self-critical or adversarial phrasing.

## Findings and Corrections

### 1. Standalone reproducibility was overstated

The frozen archive inlines all shared Python modules, but it does not contain the
third-party Franka FR3 XML and associated MuJoCo assets. Consequently, all
non-MuJoCo tables can run from the archive alone, while the computed-torque
execution table requires `FR3_XML` to point to a compatible external model.

Corrections:

- Narrowed the standalone claim in the paper and root README.
- Added `benchmarks/README.md` with a script-to-result map.
- Added `benchmarks/requirements.txt` with the verified Python environment.
- Updated `make_release_zip.py` to record the asset boundary in provenance and
  to reuse provenance-recorded frozen modules if the sibling source repository
  is unavailable.
- Rebuilt `benchmarks_standalone.zip`; it contains 33 entries, including the
  provenance, README, requirements, and all ten inlined modules.

### 2. Hard real-time language exceeded the timing evidence

A fixed QP structure gives predictable problem dimensions, not a hard
worst-case execution-time bound. The tested Python implementation has sub-10 ms
p95 timing, but observed full-cycle and randomized maxima exceed a 10 ms period.

Corrections:

- Replaced “bounded-latency” claims in active prose with “fixed-size update” or
  measured p95/max language.
- Retained internal label names to avoid breaking cross-references.
- Stated explicitly that constant structure does not establish a hard execution
  bound.
- Renamed “Physical Execution in MuJoCo” to “Closed-Loop Dynamics Simulation in
  MuJoCo” and identified the evidence as simulation rather than hardware.

### 3. Single-run timing snapshots were not stable

Wall-clock values changed across reruns on the same workstation. The manuscripts
now report repeated ranges:

| Benchmark | Updated timing |
|---|---|
| Planar DI mean / worst | 0.118--0.130 / 1.92--2.22 ms |
| Planar MPPI mean / worst | 1.886--1.979 / 8.31--9.97 ms |
| Coupled QP solve mean / p95 / max | 1.87--2.07 / 3.87--4.49 / 8.34--9.19 ms |
| Coupled full cycle mean / p95 / max | 4.74--5.07 / 6.60--7.70 / 12.26--18.48 ms |
| Randomized FR3 solve p95 / max | 6.15--6.66 / 25.96--33.10 ms |

Deterministic completion, clearance, feasibility, and violation results were
left unchanged.

### 4. Weight provenance did not match the scripts

The paper variants now distinguish the actual experiment settings:

- Static FR3 and Hessian study: `K_q=50`, `D_q=0`, `R=0.05 I`, `gamma=8`
- Dynamic FR3: `K_q=6`, `D_q=8`, `R=0.05 I`, `gamma=8`
- Cartesian-DI vehicle benchmark: `K_q=12`, `D_q=4`, `R=0.3 I`, `gamma=6`
- Jerk-bounded lateral planner: `w_q=20`, `w_a=0.2`, `w_r=0.02`, `gamma=10`

### 5. Tone obscured otherwise useful scope statements

Phrases such as “pays for smoothness,” “bounded by four limitations,”
“intrinsic and unrepairable,” and “guarantees those methods lack natively” were
replaced with symmetric tradeoff language. “Scope and Limitations” is now
“Operating Envelope and Validation Scope.” The revised section still states the
actual boundaries: local planning, the time-optimality/smoothness choice, local
linearization accuracy, APF local equilibria, finite lookahead, and
simulation-only validation.

## Reproduction Evidence

The rebuilt standalone archive was extracted into a temporary directory and run
with the selected Python environment.

- `verify_math.py`: 14/14 checks passed.
- `fr3_dynamic_randomized.py`: both methods reached the goal in all 20 trials.
- DI-QP violations: 0 velocity, 0 acceleration.
- Reactive baseline violations: 618 velocity, 6889 acceleration.
- Three completed randomized timing runs produced p95 6.15--6.66 ms and maximum
  25.96--33.10 ms.
- Earlier targeted runs reproduced the static FR3, TOPP/TOPP-RA, smooth-path,
  velocity-law, vehicle, dynamic-obstacle, APF, horizon, Hessian, and
  fixed-sparsity conclusions used by the paper.

## Remaining Validation Boundaries

- `mujoco_compare.py` cannot be rerun in this workspace because no compatible
  FR3 XML/assets are installed. The stored table was generated with MuJoCo
  3.8.1; this review used MuJoCo 3.13.0 for the available environment checks.
- Evidence remains simulation-based. Physical FR3 moving-obstacle experiments,
  broader randomized coverage, and coupled 7-DOF comparisons with online
  nonlinear MPC or sampling-based MPC would strengthen the empirical scope.
- No `latexmk`, `pdflatex`, or `xelatex` executable is available on `PATH`.
  Structural TeX checks pass, but the existing PDFs dated 2026-09-10 are stale
  relative to the reviewed sources and were not rebuilt.

## Structural Checks

All maintained English and Chinese sources pass escaped-brace balance,
`begin`/`end` environment matching, duplicate-label checks, and cross-reference
resolution checks. The two English abstracts are identical and approximately
209 words, below the 250-word IEEE limit.
