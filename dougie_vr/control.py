"""Relative, independently clutched wrist targets. No DDS/hardware connection."""
from dataclasses import dataclass
import numpy as np

XR_TO_ROBOT = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])

def valid_pose(pose):
    p = np.asarray(pose, dtype=float)
    return (p.shape == (4, 4) and np.isfinite(p).all()
            and np.allclose(p[3], [0, 0, 0, 1], atol=1e-4)
            and np.allclose(p[:3, :3].T @ p[:3, :3], np.eye(3), atol=0.02)
            and abs(np.linalg.det(p[:3, :3]) - 1) < 0.02)

@dataclass
class Clutch:
    timeout: float = 0.25
    scale: float = 0.75
    reach: float = 0.35
    engaged: bool = False
    xr_origin: object = None
    robot_origin: object = None

    def update(self, pose, timestamp, pressed, now, robot_pose):
        valid = (np.isfinite(timestamp) and 0 <= now - timestamp <= self.timeout
                 and valid_pose(pose))
        if not valid or not pressed:
            self.engaged = False
            return None
        pose = np.asarray(pose)
        if not self.engaged:
            self.xr_origin = pose.copy()
            self.robot_origin = robot_pose.copy()
            self.engaged = True
        delta = XR_TO_ROBOT @ (pose[:3, 3] - self.xr_origin[:3, 3]) * self.scale
        norm = np.linalg.norm(delta)
        if norm > self.reach:
            delta *= self.reach / norm
        target = self.robot_origin.copy()
        target[:3, 3] += delta
        target[:3, :3] = (XR_TO_ROBOT @ pose[:3, :3] @ self.xr_origin[:3, :3].T
                          @ XR_TO_ROBOT.T @ self.robot_origin[:3, :3])
        return target
