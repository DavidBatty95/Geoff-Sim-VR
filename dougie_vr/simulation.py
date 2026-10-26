import numpy as np
import mujoco
from .model import MODEL, ARM_PARTS, build

class Simulation:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL if MODEL.exists() else build()))
        self.data = mujoco.MjData(self.model)
        self.ik = mujoco.MjData(self.model)
        self.arms = {}
        for side in ('left', 'right'):
            ids = [self.model.joint(f'{side}_{part}_joint').id for part in ARM_PARTS]
            q = self.model.jnt_qposadr[ids]
            v = self.model.jnt_dofadr[ids]
            a = [self.model.actuator(f'{side}_{part}_joint').id for part in ARM_PARTS]
            self.arms[side] = (q, v, a, self.model.jnt_range[ids], self.model.site(f'{side}_tool').id)
            self.data.qpos[q] = [-0.25, 0.15 if side == 'left' else -0.15, 0., 1., 0., 0., 0.]
            self.data.ctrl[a] = self.data.qpos[q]
        mujoco.mj_forward(self.model, self.data)
        self.renderer = None

    def pose(self, side):
        site = self.arms[side][4]
        p = np.eye(4)
        p[:3, 3] = self.data.site_xpos[site]
        p[:3, :3] = self.data.site_xmat[site].reshape(3, 3)
        return p

    def solve(self, side, target):
        q, v, _, limits, site = self.arms[side]
        self.ik.qpos[:] = self.data.qpos
        jp = np.zeros((3, self.model.nv))
        jr = np.zeros_like(jp)
        for _ in range(12):
            mujoco.mj_forward(self.model, self.ik)
            rotation = self.ik.site_xmat[site].reshape(3, 3)
            # World-frame SO(3) logarithm; works at large wrist rotations too.
            quat = np.zeros(4)
            mujoco.mju_mat2Quat(quat, (target[:3, :3] @ rotation.T).reshape(-1))
            if quat[0] < 0:
                quat *= -1
            rotvec = np.zeros(3)
            mujoco.mju_quat2Vel(rotvec, quat, 1.)
            error = np.r_[target[:3, 3] - self.ik.site_xpos[site], 0.2 * rotvec]
            mujoco.mj_jacSite(self.model, self.ik, jp, jr, site)
            jac = np.vstack((jp[:, v], 0.2 * jr[:, v]))
            dq = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(6) * 0.0004, error)
            self.ik.qpos[q] = np.clip(self.ik.qpos[q] + np.clip(dq, -0.15, 0.15),
                                       limits[:, 0] + 0.01, limits[:, 1] - 0.01)
        return self.ik.qpos[q].copy()

    def step(self, targets, dt=0.02):
        for side, target in targets.items():
            q, _, a, _, _ = self.arms[side]
            if target is None:
                # Hold measured pose immediately on release/stale tracking.
                self.data.ctrl[a] = self.data.qpos[q]
            else:
                desired = self.solve(side, target)
                self.data.ctrl[a] += np.clip(desired - self.data.ctrl[a], -1.2 * dt, 1.2 * dt)
        for _ in range(round(dt / self.model.opt.timestep)):
            self.data.qfrc_applied[:] = self.data.qfrc_bias
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        if not np.isfinite(self.data.qpos).all():
            raise RuntimeError('Non-finite physics state')

    def render(self, stereo=True):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=360, width=480)
        images = []
        for side in ('left', 'right') if stereo else ('left',):
            self.renderer.update_scene(self.data, camera=f'eye_{side}')
            self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
            images.append(self.renderer.render().copy())
        return np.concatenate(images, axis=1)

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
