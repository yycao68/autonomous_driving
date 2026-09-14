# Benchmark Reproduction Map

Run scripts from this directory with the dependencies in `requirements.txt`.
In a standalone clone, use the inlined modules in `benchmarks_standalone.zip`.
Wall-clock timings are machine dependent; constraint, completion, and geometry
metrics are the portable comparisons.

| Paper result | Script |
|---|---|
| Equation checks (14 checks) | `verify_math.py` |
| FR3 B1--B4 comparison | `benchmark_fr3.py` |
| TOPP-RA comparison | `compare_topp.py` |
| C2-path comparison | `demo_smooth_totg.py` |
| Velocity-law recovery | `improve_test.py` |
| AV steering table | `benchmark_av.py` |
| Planar dynamic obstacle, latency sweep, MPPI | `benchmark_dynamic.py` |
| APF local minimum and waypoint recovery | `apf_local_minimum.py` |
| Coupled FR3 dynamic obstacle | `fr3_dynamic_obstacle.py` |
| Fixed-sparsity timing | `fixed_sparsity_timing.py` |
| Horizon sweep | `sweep_N.py` |
| Randomized FR3 robustness | `fr3_dynamic_randomized.py` |
| FR3 Hessian and linearization bound | `fr3_hessian_norm.py` |
| TOTG build and steering analysis | `analysis_totg.py` |
| MuJoCo computed-torque execution | `mujoco_compare.py` |

`mujoco_compare.py` additionally requires a licensed/third-party Franka FR3
MuJoCo model and assets that are not included in the archive. Set `FR3_XML` to
the model XML before running it. The paper's stored execution table was generated
with MuJoCo 3.8.1; the pinned environment above is the environment used for the
2026-09-12 non-MuJoCo reproduction pass.

Some plotting scripts write to `../simulationResults`; create that directory when
running from a temporary extraction. Set `PYTHONIOENCODING=utf-8` on Windows for
scripts that print Greek symbols.
