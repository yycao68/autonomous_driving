"""
Render the FR3 obstacle-avoidance rollout to an MP4 video.

Plays the DI-QP joint trajectory; draws the moving spherical obstacle (red), the
current end-effector (green), and an accumulating end-effector trail (white) on a
dark background so the light-coloured robot and all markers stay visible.
-> fr3_motion.mp4
"""
import numpy as np
import mujoco
import imageio
from pathlib import Path
from fr3_dynamic_obstacle import run_qp, obstacle_pos, R_OBS
from fr3_kinematics import fk

XML = "/Users/yycao/ai_learn/pHRI/simulation/models/franka_fr3/fr3_darkscene.xml"
H, W = 600, 800
FPS = 50          # 100 Hz trajectory rendered every step -> ~0.5x slow-motion


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
    print(f"trajectory: {len(Q)} steps, T={tt[-1]:.2f}s")

    m = mujoco.MjModel.from_xml_path(XML)
    m.vis.global_.offwidth = max(m.vis.global_.offwidth, W)
    m.vis.global_.offheight = max(m.vis.global_.offheight, H)
    d = mujoco.MjData(m)
    renderer = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    p0, pg = fk(Q[0]), fk(Q[-1])
    cam.lookat[:] = 0.5 * (p0 + pg)
    cam.distance = 1.9; cam.azimuth = 135; cam.elevation = -20

    ee_all = np.array([fk(Q[i]) for i in range(len(Q))])
    out = Path(__file__).with_name("fr3_motion.mp4")
    with imageio.get_writer(out, fps=FPS, codec="libx264",
                            quality=8, macro_block_size=8) as wr:
        for k in range(len(Q)):
            d.qpos[:7] = Q[k]; d.qvel[:7] = 0.0
            mujoco.mj_forward(m, d)
            renderer.update_scene(d, camera=cam)
            scn = renderer.scene
            add_sphere(scn, obstacle_pos(tt[k]), R_OBS, [0.85, 0.15, 0.15, 0.95])
            for p in ee_all[:k+1:3]:                       # accumulating trail (white)
                add_sphere(scn, p, 0.014, [0.95, 0.95, 0.98, 0.95])
            add_sphere(scn, ee_all[k], 0.030, [0.0, 0.75, 0.1, 1.0])
            wr.append_data(renderer.render())
            if k % 40 == 0:
                print(f"  frame {k}/{len(Q)}")
    print(f"saved {out}")


if __name__ == "__main__":
    main()
