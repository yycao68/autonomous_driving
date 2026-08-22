"""Drill-down on TOTG (1) compute/planning rate and (2) steering rate."""
import time
import numpy as np
from di_planner import plan_di, FR3_VMAX, FR3_AMAX
from di_totg import BlendPath, topp, plan_totg
from benchmark_fr3 import b4_dense_noisy
from benchmark_av import (steering, av_double_lane_change, av_intersection_turn,
                          V_MAX, A_MAX)


def time_stages(W, vmax, amax, dt=0.01, max_dev=0.08, K=1000, reps=3):
    """Break TOTG build into path / TOPP / s->t-resample sub-times (ms)."""
    tp = tt = tr = 0.0
    for _ in range(reps):
        t0 = time.perf_counter(); path = BlendPath(W, max_dev=max_dev)
        t1 = time.perf_counter(); res = topp(path, vmax, amax, K=K)
        t2 = time.perf_counter()
        # s->t resample stage (mirror plan_totg tail)
        s, ds, x = res["s"], res["ds"], res["x"]
        sdot = np.sqrt(np.maximum(x, 0)); sdd = 0.5 * np.gradient(x, s)
        denom = sdot[1:] + sdot[:-1]
        t_of_s = np.concatenate([[0], np.cumsum(2 * ds / np.maximum(denom, 1e-9))])
        T = t_of_s[-1]; nT = max(min(int(np.ceil(T / dt)) + 1, 200000), 2)
        tt_ = np.linspace(0, T, nT); ss = np.interp(tt_, t_of_s, s)
        for k in range(nT):
            path.eval(ss[k])
        t3 = time.perf_counter()
        tp += t1 - t0; tt += t2 - t1; tr += t3 - t2
    return 1e3 * tp / reps, 1e3 * tt / reps, 1e3 * tr / reps, T


print("=" * 64)
print("(1) TOTG COMPUTE / PLANNING RATE")
print("=" * 64)

# 1a. stage breakdown on a representative dense case
_, W, n, _ = b4_dense_noisy()
vmax, amax = FR3_VMAX[:n], FR3_AMAX[:n]
tp, tt, tr, T = time_stages(W, vmax, amax)
print(f"\nStage breakdown (B4 dense, 24 wp, K=1000):")
print(f"  path construction : {tp:6.2f} ms")
print(f"  TOPP fwd/bwd sweep: {tt:6.2f} ms")
print(f"  s->t resample     : {tr:6.2f} ms")
print(f"  TOTAL build       : {tp+tt+tr:6.2f} ms  for a {T:.2f}s trajectory")
print(f"  -> batch throughput: {T/((tp+tt+tr)/1e3):.0f}x real-time (one-shot)")
print(f"  -> as a replanner  : {1000/(tp+tt+tr):.0f} Hz max, recomputes ALL of it")

# 1b. scaling with number of waypoints
print(f"\nBuild time vs #waypoints (K=1000):")
print(f"{'#wp':>5}{'build[ms]':>11}{'traj T[s]':>11}")
for M in (2, 4, 8, 16, 32):
    rng = np.random.default_rng(0)
    t = np.linspace(0, 1, M)
    Wm = np.column_stack([1.2*np.sin(2*np.pi*t), 0.9*np.cos(2*np.pi*t),
                          0.6*np.sin(np.pi*t), -0.5*t, 0.4*np.cos(3*np.pi*t),
                          0.3*np.sin(2*t), -0.4*np.cos(t)])
    Wm[0] = 0
    a, b, c, Tt = time_stages(Wm, vmax, amax)
    print(f"{M:>5}{a+b+c:>11.2f}{Tt:>11.2f}")

# 1c. scaling with TOPP grid resolution K
print(f"\nBuild time vs TOPP grid K (B4 dense):")
print(f"{'K':>6}{'build[ms]':>11}")
for K in (500, 1000, 2000, 4000):
    a, b, c, _ = time_stages(W, vmax, amax, K=K)
    print(f"{K:>6}{a+b+c:>11.2f}")

# 1d. contrast: DI as a true online loop
di = plan_di(W, dt=0.01, vmax=vmax, amax=amax)
print(f"\nDI (online receding horizon): {1e3*di['solve_t'].mean():.3f} ms/cycle "
      f"mean, {1e3*di['solve_t'].max():.3f} ms worst  -> fixed 100 Hz+ loop, "
      f"reacts within ONE cycle.")

print("\n" + "=" * 64)
print("(2) TOTG STEERING RATE")
print("=" * 64)

# 2a. steering rate vs blend tightness (moose test)
_, Wm = av_double_lane_change()[0], av_double_lane_change()[1]
print(f"\nMoose test: peak steering rate vs blend tightness max_dev (dt=0.05):")
print(f"{'max_dev':>8}{'min r[m]':>9}{'peak |dδ| jump[rad]':>20}{'peak rate[rad/s]':>18}")
for dev in (0.6, 0.3, 0.15, 0.08):
    tg = plan_totg(Wm, dt=0.05, vmax=V_MAX, amax=A_MAX, max_dev=dev)
    st = steering(tg)
    print(f"{dev:>8}{tg['min_blend_r']:>9.2f}{st['delta_jump']:>20.4f}"
          f"{st['peak_drate']:>18.3f}")

# 2b. the divergence test: same path, finer dt -> rate blows up (true discontinuity)
print(f"\nSharp 90-deg turn: peak steering rate as dt -> 0 (same geometric path):")
print(f"{'dt[s]':>8}{'peak rate[rad/s]':>18}{'jump[rad]':>12}")
Wt = av_intersection_turn()[1]
for dt in (0.1, 0.05, 0.02, 0.01, 0.005):
    tg = plan_totg(Wt, dt=dt, vmax=V_MAX, amax=A_MAX, max_dev=0.3)
    st = steering(tg)
    print(f"{dt:>8}{st['peak_drate']:>18.2f}{st['delta_jump']:>12.4f}")
print("  (rate grows without bound as dt->0  =>  a genuine discontinuity,")
print("   not a finite physical steering rate)")
