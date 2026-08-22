"""
Render the FR3 avoiding the moving obstacle (the §VI.E scenario) as a strip of
MuJoCo frames.  The arm follows the DI-QP joint trajectory; the moving spherical
obstacle and the end-effector trail are drawn into the scene.  -> fr3_motion.png
"""
import numpy as np
import mujoco
from pathlib import Path
from fr3_dynamic_obstacle import run_qp, obstacle_pos, R_OBS
from fr3_kinematics import fk

XML = "/Users/yycao/Documents/git/ai_learn/pHRI/simulation/models/franka_fr3/fr3_lightscene.xml"
NFRAMES = 6
H, W = 460, 600


def add_sphere(scn, pos, radius, rgba):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([radius, 0, 0], float), np.asarray(pos, float),
                        np.eye(3).flatten(), np.asarray(rgba, np.float32))
    scn.ngeom += 1


def main():
    res = run_qp()
    Q, tt = res["Q"], res["t"]
    print(f"trajectory: {len(Q)} steps, T={tt[-1]:.2f}s, reached={res['reached']}")

    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    renderer = mujoco.Renderer(m, H, W)

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    p0, pg = fk(Q[0]), fk(Q[-1])
    cam.lookat[:] = 0.5 * (p0 + pg) + np.array([0, 0, 0.0])
    cam.distance = 1.9; cam.azimuth = 135; cam.elevation = -20

    idxs = np.linspace(0, len(Q) - 1, NFRAMES).astype(int)
    frames = []
    ee_trail = [fk(Q[i]) for i in range(0, len(Q), max(1, len(Q)//40))]
    for fi in idxs:
        d.qpos[:7] = Q[fi]; d.qvel[:7] = 0.0
        mujoco.mj_forward(m, d)
        renderer.update_scene(d, camera=cam)
        scn = renderer.scene
        add_sphere(scn, obstacle_pos(tt[fi]), R_OBS, [0.85, 0.15, 0.15, 0.95])   # obstacle
        for p in ee_trail:                                                       # EE trail
            add_sphere(scn, p, 0.016, [0.05, 0.25, 0.9, 0.9])                     # darker blue, opaque
        add_sphere(scn, fk(Q[fi]), 0.032, [0.0, 0.75, 0.1, 1.0])                 # current EE (green)
        frames.append(renderer.render().copy())
        print(f"  frame t={tt[fi]:.2f}s")

    strip = np.concatenate(frames, axis=1)
    try:
        from PIL import Image
        Image.fromarray(strip).save("fr3_motion.png")
    except Exception:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        plt.figure(figsize=(NFRAMES * 2.4, 2.4)); plt.imshow(strip); plt.axis("off")
        out = Path(__file__).with_name("fr3_motion.png")
        plt.tight_layout(); plt.savefig(out, dpi=130)
        print(f"saved {out}")


if __name__ == "__main__":
    main()
