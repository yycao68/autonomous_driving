"""
FR3 falling-ball cyclic-manipulation benchmark.

The arm repeatedly tracks A -> B -> A while a ball is released from 5 m above
the nominal swept volume.  The receding-horizon QP uses the same virtual
double-integrator backbone as the paper, but the obstacle rows are explicitly
time-indexed with a gravity-predicted ball trajectory:

    p_ball(tau) = p_release + 0.5 g (tau - t_release)^2.

Outputs:
  * fr3_falling_ball_cyclic.mp4  -- MuJoCo render of the avoidance process
  * fr3_falling_ball_cyclic.png  -- clearance / solve-time / tracking summary
  * fr3_falling_ball_cyclic_trajectory.png -- nominal vs avoided trajectory plot

Run:
  python3 autonomous_driving/benchmarks/fr3_falling_ball_cyclic.py
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import imageio
import mujoco
import numpy as np
import osqp
import scipy.sparse as sp

from di_planner import FR3_AMAX, FR3_QMAX, FR3_QMIN, FR3_VMAX, di_matrices
from fr3_kinematics import fk, jacobian_v


ROOT = Path(__file__).resolve().parents[2]
XML = ROOT / "pHRI/simulation/models/franka_fr3/fr3_darkscene.xml"

DT = 0.01
N = 20
CYCLES = 3
HALF_PERIOD = 1.35
CYCLE_PERIOD = 2.0 * HALF_PERIOD
T_MAX = CYCLES * CYCLE_PERIOD
FPS = 50
VIDEO_W = 960
VIDEO_H = 720

Q_A = np.array([0.0, -0.40, 0.0, -1.80, 0.0, 1.40, 0.0])
Q_B = np.array([1.4, 0.50, 0.2, -1.20, 0.1, 1.60, 0.3])

R_BALL = 0.070
R_LINK = 0.060
R_MARGIN = 0.030
R_SAFE = R_BALL + R_LINK + R_MARGIN
G = np.array([0.0, 0.0, -9.81])
RELEASE_HEIGHT = 5.0


def smoothstep01(s: float) -> float:
    s = float(np.clip(s, 0.0, 1.0))
    return s * s * (3.0 - 2.0 * s)


P_A = fk(Q_A)
P_B = fk(Q_B)


def nominal_cycle_q(t: float, half_period: float = HALF_PERIOD) -> np.ndarray:
    """Smooth A-B-A nominal reference used for video and tracking metrics."""
    phase = (t % (2.0 * half_period)) / half_period
    if phase <= 1.0:
        s = smoothstep01(phase)
        return (1.0 - s) * Q_A + s * Q_B
    s = smoothstep01(phase - 1.0)
    return (1.0 - s) * Q_B + s * Q_A


BALL_CROSS_TIME = CYCLE_PERIOD + 0.5 * HALF_PERIOD
P_CENTER = fk(nominal_cycle_q(BALL_CROSS_TIME))
BALL_RELEASE_POS = np.array([P_CENTER[0], P_CENTER[1], RELEASE_HEIGHT])
FALL_TIME_TO_CENTER = np.sqrt(2.0 * (RELEASE_HEIGHT - P_CENTER[2]) / abs(G[2]))
RELEASE_TIME = BALL_CROSS_TIME - FALL_TIME_TO_CENTER


def ball_pos(t: float) -> np.ndarray:
    """Predicted falling-ball position under gravity."""
    if t < RELEASE_TIME:
        return BALL_RELEASE_POS.copy()
    tau = t - RELEASE_TIME
    return BALL_RELEASE_POS + 0.5 * G * tau * tau


def ball_active(t: float) -> bool:
    p = ball_pos(t)
    return -0.10 <= p[2] <= RELEASE_HEIGHT + 0.05


def _build_qp_matrices():
    phi_p, phi_v, gam_p, gam_v = di_matrices(DT, N)
    wq, wv, wr, gamma = 45.0, 7.0, 0.08, 8.0
    Wq = np.full(N, wq)
    Wv = np.full(N, wv)
    Wq[-1] *= gamma
    Wv[-1] *= gamma
    blocks = [
        gam_p.T @ (Wq[:, None] * gam_p)
        + gam_v.T @ (Wv[:, None] * gam_v)
        + wr * np.eye(N)
        for _ in range(7)
    ]
    P = sp.block_diag([sp.block_diag(blocks), sp.csc_matrix([[1.0e7]])]).tocsc()
    return phi_p, phi_v, gam_p, gam_v, Wq, Wv, P


def _velocity_law(q: np.ndarray, goal: np.ndarray) -> np.ndarray:
    dq = goal - q
    return np.clip(np.sign(dq) * np.sqrt(2.0 * FR3_AMAX * np.abs(dq)),
                   -FR3_VMAX, FR3_VMAX)


def _avoidance_velocity(q: np.ndarray, t: float) -> np.ndarray:
    """Task-space preview bias that nudges the nominal cycle around the ball."""
    p = fk(q)
    look = np.linspace(0.0, 1.10, 24)
    balls = np.asarray([ball_pos(t + tau) for tau in look])
    dists = np.linalg.norm(balls - p[None, :], axis=1)
    idx = int(np.argmin(dists))
    dist = float(dists[idx])
    rho = 1.05
    if dist >= rho or not ball_active(t + look[idx]):
        return np.zeros(7)

    # Fixed lateral/upward escape direction makes the center-point collision
    # visually legible and avoids dithering when the ball lies exactly on the
    # nominal path.
    side = np.array([1.0, -0.20, 0.25])
    side /= np.linalg.norm(side)
    gain = 1.80 * smoothstep01((rho - dist) / rho)
    v_task = gain * side
    return np.linalg.pinv(jacobian_v(q)) @ v_task


def plan_falling_ball():
    phi_p, phi_v, gam_p, gam_v, Wq, Wv, P = _build_qp_matrices()
    x = np.zeros((7, 2))
    x[:, 0] = Q_A

    Q, V, A, T, clear, ball_log, nominal_log = [], [], [], [], [], [], []
    pred_ee_log, pred_ball_log = [], []
    solve_t, cycle_t = [], []
    failures = 0
    max_slack = 0.0

    nsteps = int(T_MAX / DT)
    for k in range(nsteps):
        tic = time.perf_counter()
        t = k * DT
        q = x[:, 0].copy()
        v = x[:, 1].copy()
        q_nom = nominal_cycle_q(t)
        q_ref = np.array([nominal_cycle_q(t + (i + 1) * DT) for i in range(N)])
        v_ref_now = np.clip(_velocity_law(q, q_ref[-1])
                            + _avoidance_velocity(q, t),
                            -FR3_VMAX, FR3_VMAX)

        p_ee = fk(q)
        Jv = jacobian_v(q)

        qvec = np.zeros(7 * N + 1)
        for j in range(7):
            ep = phi_p @ x[j] - q_ref[:, j]
            ev = phi_v @ x[j] - v_ref_now[j]
            qvec[j * N:(j + 1) * N] = (
                gam_p.T @ (Wq * ep) + gam_v.T @ (Wv * ev)
            )

        rows, lo, up = [], [], []

        # Velocity rows.
        for j in range(7):
            blk = np.zeros((N, 7 * N + 1))
            blk[:, j * N:(j + 1) * N] = gam_v
            rows.append(blk)
            vf = phi_v @ x[j]
            lo.append(-FR3_VMAX[j] - vf)
            up.append(FR3_VMAX[j] - vf)

        # Acceleration rows.
        for j in range(7):
            blk = np.zeros((N, 7 * N + 1))
            blk[:, j * N:(j + 1) * N] = np.eye(N)
            rows.append(blk)
            lo.append(-FR3_AMAX[j] * np.ones(N))
            up.append(FR3_AMAX[j] * np.ones(N))

        # Joint-position rows to keep the cyclic deflection physically admissible.
        for j in range(7):
            blk = np.zeros((N, 7 * N + 1))
            blk[:, j * N:(j + 1) * N] = gam_p
            rows.append(blk)
            pf = phi_p @ x[j]
            lo.append(FR3_QMIN[j] - pf)
            up.append(FR3_QMAX[j] - pf)

        # Penalized nonnegative obstacle slack.
        srow = np.zeros((1, 7 * N + 1))
        srow[0, -1] = 1.0
        rows.append(srow)
        lo.append([0.0])
        up.append([np.inf])

        # Predictive ball rows: n_k^T(p_ee + J dq_k - p_ball(t_k)) >= R_SAFE.
        obs = np.zeros((N, 7 * N + 1))
        obs_l = np.full(N, -np.inf)
        obs_u = np.full(N, np.inf)
        for i in range(N):
            tau = t + (i + 1) * DT
            pb = ball_pos(tau)
            if not ball_active(tau):
                continue
            sep = p_ee - pb
            dist = np.linalg.norm(sep)
            if dist < 1e-5:
                sep = p_ee - P_CENTER + np.array([0.0, 0.0, 0.05])
                dist = max(np.linalg.norm(sep), 1e-5)
            n = sep / dist
            nJ = n @ Jv
            for j in range(7):
                obs[i, j * N:(j + 1) * N] = nJ[j] * gam_p[i]
            obs[i, -1] = 1.0
            free = 0.0
            for j in range(7):
                free += nJ[j] * ((i + 1) * DT * v[j])
            obs_l[i] = R_SAFE - float(n @ (p_ee - pb)) - free
        rows.append(obs)
        lo.append(obs_l)
        up.append(obs_u)

        Acon = sp.csc_matrix(np.vstack(rows))
        l = np.concatenate([np.asarray(a, float) for a in lo])
        u = np.concatenate([np.asarray(a, float) for a in up])
        prob = osqp.OSQP()
        prob.setup(P, qvec, Acon, l, u, verbose=False, warm_starting=True,
                   eps_abs=1e-6, eps_rel=1e-6, max_iter=8000)
        res = prob.solve()
        solve_t.append(res.info.solve_time)
        ok = res.info.status_val in (1, 2)
        failures += 0 if ok else 1
        sol = res.x if ok else np.zeros(7 * N + 1)
        max_slack = max(max_slack, float(abs(sol[-1])))

        q_pred = np.zeros((N, 7))
        for j in range(7):
            Uj = sol[j * N:(j + 1) * N]
            q_pred[:, j] = phi_p @ x[j] + gam_p @ Uj
        pred_ee_log.append(np.asarray([fk(qi) for qi in q_pred]))
        pred_ball_log.append(np.asarray([ball_pos(t + (i + 1) * DT)
                                         for i in range(N)]))

        u0 = np.array([sol[j * N] for j in range(7)])
        for j in range(7):
            q0, v0 = x[j]
            x[j, 0] = q0 + DT * v0 + 0.5 * DT * DT * u0[j]
            x[j, 1] = v0 + DT * u0[j]

        p_ball = ball_pos(t + DT)
        p_ee_next = fk(x[:, 0])
        Q.append(x[:, 0].copy())
        V.append(x[:, 1].copy())
        A.append(u0.copy())
        T.append(t)
        clear.append(np.linalg.norm(p_ee_next - p_ball) - R_SAFE)
        ball_log.append(p_ball)
        nominal_log.append(q_nom)
        cycle_t.append(time.perf_counter() - tic)

    Q = np.asarray(Q)
    V = np.asarray(V)
    A = np.asarray(A)
    T = np.asarray(T)
    ball_log = np.asarray(ball_log)
    nominal_log = np.asarray(nominal_log)
    ee = np.asarray([fk(q) for q in Q])
    ee_nom = np.asarray([fk(q) for q in nominal_log])
    return {
        "Q": Q,
        "V": V,
        "A": A,
        "t": T,
        "ball": ball_log,
        "ee": ee,
        "ee_nom": ee_nom,
        "pred_ee": np.asarray(pred_ee_log),
        "pred_ball": np.asarray(pred_ball_log),
        "clearance": np.asarray(clear),
        "solve_t": np.asarray(solve_t),
        "cycle_t": np.asarray(cycle_t),
        "tracking_rmse": float(np.sqrt(np.mean((Q - nominal_log) ** 2))),
        "max_tracking": float(np.max(np.linalg.norm(Q - nominal_log, axis=1))),
        "min_clearance": float(np.min(clear)),
        "max_slack": max_slack,
        "failures": failures,
        "v_viol": int((np.abs(V) - FR3_VMAX > 1e-4).sum()),
        "a_viol": int((np.abs(A) - FR3_AMAX > 1e-4).sum()),
    }


def _add_sphere(scene, pos, radius, rgba):
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([radius, 0.0, 0.0], float),
        np.asarray(pos, float),
        np.eye(3).flatten(),
        np.asarray(rgba, np.float32),
    )
    scene.ngeom += 1


def _add_capsule(scene, a, b, radius, rgba):
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        np.array([radius, 0.0, 0.0], float),
        np.zeros(3),
        np.eye(3).flatten(),
        np.asarray(rgba, np.float32),
    )
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, radius,
                         np.asarray(a, float), np.asarray(b, float))
    scene.ngeom += 1


def _overlay_trajectory_inset(frame: np.ndarray, res: dict, k: int) -> np.ndarray:
    """Draw a live top-view trajectory plot into the rendered video frame."""
    try:
        from PIL import Image, ImageDraw, ImageFont
    except Exception:
        return frame

    img = Image.fromarray(frame).convert("RGBA")
    overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)

    panel_w, panel_h = 330, 235
    margin = 18
    x0 = img.size[0] - panel_w - margin
    y0 = margin
    x1 = x0 + panel_w
    y1 = y0 + panel_h
    draw.rounded_rectangle([x0, y0, x1, y1], radius=8,
                           fill=(255, 255, 255, 218),
                           outline=(30, 30, 30, 210), width=2)

    plot = (x0 + 28, y0 + 38, x1 - 18, y1 - 34)
    px0, py0, px1, py1 = plot
    draw.rectangle(plot, outline=(90, 90, 90, 210), width=1)

    ee = res["ee"]
    ee_nom = res["ee_nom"]
    ball = res["ball"]
    pred_ee = res["pred_ee"]
    pts = np.vstack([ee[:, :2], ee_nom[:, :2], ball[:, :2], pred_ee.reshape(-1, 3)[:, :2]])
    lo = pts.min(axis=0) - 0.05
    hi = pts.max(axis=0) + 0.05
    span = np.maximum(hi - lo, 1e-6)

    def xy(p):
        u = (p[0] - lo[0]) / span[0]
        v = (p[1] - lo[1]) / span[1]
        return (int(px0 + u * (px1 - px0)), int(py1 - v * (py1 - py0)))

    def polyline(points, color, width=2, step=1):
        if len(points) < 2:
            return
        pix = [xy(p) for p in points[::step]]
        if len(pix) >= 2:
            draw.line(pix, fill=color, width=width, joint="curve")

    # Cycle-2 band in the time plot would be hard to read in video; top view keeps
    # only the spatial essentials: nominal path, executed path, current horizon.
    polyline(ee_nom[:, :2], (115, 115, 115, 220), width=2, step=3)
    polyline(ee[:k + 1, :2], (20, 80, 235, 255), width=4, step=2)
    polyline(pred_ee[k, :, :2], (245, 170, 0, 255), width=4, step=1)

    bx, by = xy(ball[k, :2])
    ex, ey = xy(ee[k, :2])
    cx, cy = xy(P_CENTER[:2])
    draw.ellipse([bx - 6, by - 6, bx + 6, by + 6], fill=(220, 20, 20, 255))
    draw.ellipse([ex - 5, ey - 5, ex + 5, ey + 5], fill=(0, 150, 40, 255))
    draw.ellipse([cx - 4, cy - 4, cx + 4, cy + 4],
                 outline=(220, 20, 20, 255), width=2)

    try:
        font = ImageFont.truetype("Arial.ttf", 14)
        small = ImageFont.truetype("Arial.ttf", 12)
    except Exception:
        font = ImageFont.load_default()
        small = font
    draw.text((x0 + 14, y0 + 12), "Live top-view trajectory", fill=(20, 20, 20, 255), font=font)
    draw.text((x0 + 16, y1 - 25),
              "gray nominal  blue executed  yellow projected  red ball",
              fill=(20, 20, 20, 255), font=small)
    draw.text((x0 + 15, y0 + 38), "y", fill=(60, 60, 60, 255), font=small)
    draw.text((x1 - 31, y1 - 50), "x", fill=(60, 60, 60, 255), font=small)

    out = Image.alpha_composite(img, overlay).convert("RGB")
    return np.asarray(out)


def render_video(res, out_path: Path):
    model = mujoco.MjModel.from_xml_path(str(XML))
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, VIDEO_W)
    model.vis.global_.offheight = max(model.vis.global_.offheight, VIDEO_H)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, VIDEO_H, VIDEO_W)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = np.array([P_CENTER[0], P_CENTER[1], 0.65])
    cam.distance = 2.3
    cam.azimuth = 125
    cam.elevation = -18

    Q, t, ball, ee, ee_nom = res["Q"], res["t"], res["ball"], res["ee"], res["ee_nom"]
    pred_ee, pred_ball = res["pred_ee"], res["pred_ball"]
    stride = max(1, int(round((1.0 / FPS) / DT)))
    with imageio.get_writer(out_path, fps=FPS, codec="libx264",
                            quality=8, macro_block_size=8) as writer:
        for frame_i, k in enumerate(range(0, len(Q), stride)):
            data.qpos[:7] = Q[k]
            data.qvel[:7] = 0.0
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=cam)
            scene = renderer.scene

            _add_sphere(scene, ball[k], R_BALL, [0.92, 0.08, 0.04, 0.98])
            _add_sphere(scene, ee[k], 0.030, [0.0, 0.75, 0.1, 1.0])
            for p0, p1 in zip(pred_ee[k, :-1:2], pred_ee[k, 1::2]):
                _add_capsule(scene, p0, p1, 0.012, [1.0, 0.78, 0.02, 0.95])
            for p in pred_ee[k, ::4]:
                _add_sphere(scene, p, 0.018, [1.0, 0.95, 0.0, 0.95])
            for p in pred_ball[k, ::4]:
                if ball_active(float(t[k] + DT)):
                    _add_sphere(scene, p, R_BALL * 0.35, [1.0, 0.05, 0.02, 0.28])
            for p in ee[:k + 1:8]:                          # executed EE trail (white)
                _add_sphere(scene, p, 0.010, [0.96, 0.96, 0.99, 0.90])
            for p in ee_nom[:k + 1:12]:                     # nominal path (dim grey)
                _add_sphere(scene, p, 0.008, [0.55, 0.57, 0.62, 0.45])
            _add_capsule(scene, BALL_RELEASE_POS, np.array([BALL_RELEASE_POS[0],
                         BALL_RELEASE_POS[1], 0.0]), 0.004,
                         [0.85, 0.05, 0.05, 0.35])
            _add_sphere(scene, P_A, 0.035, [0.0, 0.45, 1.0, 0.95])
            _add_sphere(scene, P_B, 0.035, [1.0, 0.65, 0.0, 0.95])

            frame = renderer.render()
            frame = _overlay_trajectory_inset(frame, res, k)
            writer.append_data(frame)
            if frame_i % 50 == 0:
                print(f"  rendered frame {frame_i}")
    print(f"saved video: {out_path}")


def save_plot(res, out_path: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = res["t"]
    fig, ax = plt.subplots(3, 1, figsize=(8.0, 7.0), sharex=True)
    ax[0].plot(t, res["clearance"], "b", lw=1.5)
    ax[0].axhline(0.0, color="k", ls="--", lw=0.9)
    ax[0].set_ylabel("clearance [m]")
    ax[0].set_title("Falling-ball cyclic manipulation")
    ax[0].grid(True, alpha=0.25)

    ax[1].plot(t, 1e3 * res["solve_t"], "tab:green", label="OSQP solve")
    ax[1].plot(t, 1e3 * res["cycle_t"], "tab:orange", alpha=0.8, label="full cycle")
    ax[1].axhline(10.0, color="k", ls="--", lw=0.9, label="100 Hz budget")
    ax[1].set_ylabel("time [ms]")
    ax[1].legend(loc="upper right", fontsize=8)
    ax[1].grid(True, alpha=0.25)

    track = np.linalg.norm(res["Q"] - np.asarray([nominal_cycle_q(x) for x in t]), axis=1)
    ax[2].plot(t, track, "tab:purple")
    ax[2].set_ylabel("joint deflection [rad]")
    ax[2].set_xlabel("time [s]")
    ax[2].grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    print(f"saved plot: {out_path}")


def save_trajectory_plot(res, out_path: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = res["t"]
    ee = res["ee"]
    ee_nom = res["ee_nom"]
    ball = res["ball"]
    dev = np.linalg.norm(ee - ee_nom, axis=1)
    second_start = CYCLE_PERIOD
    second_end = 2.0 * CYCLE_PERIOD
    k_cross = int(np.argmin(np.abs(t - BALL_CROSS_TIME)))
    k_dev = int(np.argmax(dev))

    fig = plt.figure(figsize=(11.0, 8.0))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.2, 1.0])
    ax3 = fig.add_subplot(gs[:, 0], projection="3d")
    axd = fig.add_subplot(gs[0, 1])
    axz = fig.add_subplot(gs[1, 1])

    ax3.plot(ee_nom[:, 0], ee_nom[:, 1], ee_nom[:, 2],
             color="0.55", lw=2.0, ls="--", label="nominal A-B-A cycle")
    ax3.plot(ee[:, 0], ee[:, 1], ee[:, 2],
             color="tab:blue", lw=2.4, label="QP avoided trajectory")
    mask2 = (t >= second_start) & (t <= second_end)
    ax3.plot(ee[mask2, 0], ee[mask2, 1], ee[mask2, 2],
             color="tab:orange", lw=4.0, label="cycle 2 with falling ball")
    active = np.array([ball_active(float(tt)) for tt in t])
    ax3.plot(ball[active, 0], ball[active, 1], ball[active, 2],
             color="tab:red", lw=2.0, label="falling ball path")
    ax3.scatter(*P_A, s=45, color="tab:green", label="A")
    ax3.scatter(*P_B, s=45, color="gold", edgecolor="k", label="B")
    ax3.scatter(*ball[k_cross], s=90, color="tab:red", edgecolor="k",
                label="ball at nominal center point")
    ax3.scatter(*ee[k_dev], s=75, color="tab:purple", edgecolor="k",
                label="max deviation")
    ax3.set_xlabel("x [m]")
    ax3.set_ylabel("y [m]")
    ax3.set_zlabel("z [m]")
    ax3.set_title("Workspace trajectory change")
    ax3.view_init(elev=22, azim=-62)
    ax3.legend(loc="upper left", fontsize=8)

    axd.plot(t, dev, color="tab:purple", lw=1.8)
    axd.axvspan(second_start, second_end, color="tab:orange", alpha=0.12,
                label="cycle 2")
    axd.axvline(BALL_CROSS_TIME, color="tab:red", ls="--", lw=1.2,
                label="ball touches nominal center point")
    axd.set_ylabel("EE deviation [m]")
    axd.set_title(f"max deviation = {dev[k_dev]:.3f} m")
    axd.grid(True, alpha=0.25)
    axd.legend(loc="upper right", fontsize=8)

    labels = ["x", "y", "z"]
    colors = ["tab:blue", "tab:green", "tab:brown"]
    for i, (lab, color) in enumerate(zip(labels, colors)):
        axz.plot(t, ee_nom[:, i], color=color, ls="--", alpha=0.55,
                 label=f"nominal {lab}")
        axz.plot(t, ee[:, i], color=color, lw=1.5, label=f"QP {lab}")
    axz.axvspan(second_start, second_end, color="tab:orange", alpha=0.12)
    axz.axvline(BALL_CROSS_TIME, color="tab:red", ls="--", lw=1.2)
    axz.set_xlabel("time [s]")
    axz.set_ylabel("EE coordinate [m]")
    axz.grid(True, alpha=0.25)
    axz.legend(ncol=2, fontsize=7, loc="upper right")

    fig.suptitle(
        "Three-cycle falling-ball benchmark: ball touches nominal center point in cycle 2",
        y=0.98,
    )
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"saved trajectory plot: {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-video", action="store_true", help="only compute metrics and plot")
    args = parser.parse_args()

    res = plan_falling_ball()
    print("FR3 falling-ball cyclic benchmark")
    print(f"  cycles: {CYCLES}, cycle period: {CYCLE_PERIOD:.2f} s")
    print(f"  ball release/cross time: {RELEASE_TIME:.2f}/{BALL_CROSS_TIME:.2f} s")
    print(f"  ball-to-nominal-center error at crossing: "
          f"{np.linalg.norm(ball_pos(BALL_CROSS_TIME) - P_CENTER):.3e} m")
    print(f"  min clearance over safety boundary: {res['min_clearance']:.3f} m")
    print(f"  max slack: {res['max_slack']:.3e}")
    print(f"  velocity violations: {res['v_viol']}")
    print(f"  acceleration violations: {res['a_viol']}")
    print(f"  solver failures: {res['failures']}")
    print(f"  solve mean/p95/max: "
          f"{1e3*np.mean(res['solve_t']):.2f}/"
          f"{1e3*np.percentile(res['solve_t'], 95):.2f}/"
          f"{1e3*np.max(res['solve_t']):.2f} ms")
    print(f"  full-cycle mean/p95/max: "
          f"{1e3*np.mean(res['cycle_t']):.2f}/"
          f"{1e3*np.percentile(res['cycle_t'], 95):.2f}/"
          f"{1e3*np.max(res['cycle_t']):.2f} ms")
    print(f"  cyclic tracking RMSE: {res['tracking_rmse']:.3f} rad")

    out_plot = Path(__file__).with_name("fr3_falling_ball_cyclic.png")
    save_plot(res, out_plot)
    out_traj = Path(__file__).with_name("fr3_falling_ball_cyclic_trajectory.png")
    save_trajectory_plot(res, out_traj)
    if not args.no_video:
        out_video = Path(__file__).with_name("fr3_falling_ball_cyclic.mp4")
        render_video(res, out_video)


if __name__ == "__main__":
    main()
