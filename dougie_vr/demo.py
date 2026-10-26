"""Deterministic synthetic wrist replay through the real clutch, IK and physics."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from .control import Clutch
from .simulation import Simulation

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('results'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    sim = Simulation()
    clutches = {s: Clutch() for s in sim.arms}
    origins = {s: sim.pose(s).copy() for s in sim.arms}
    max_motion = {s: 0. for s in sim.arms}
    frames, errors = [], []
    try:
        for i in range(400):
            t = i * 0.02
            targets = {}
            for side in sim.arms:
                p = np.eye(4)
                p[:3, 3] = [(-0.1 if side == 'left' else 0.1) * (1-np.cos(t)),
                            0.08 * np.sin(t), -0.12 * np.sin(t)]
                targets[side] = clutches[side].update(p, t, t < 6., t, sim.pose(side))
            sim.step(targets)
            for side in sim.arms:
                max_motion[side] = max(max_motion[side], float(np.linalg.norm(sim.pose(side)[:3, 3] - origins[side][:3, 3])))
                if targets[side] is not None:
                    errors.append(float(np.linalg.norm(sim.pose(side)[:3, 3] - targets[side][:3, 3])))
            if i % 10 == 0:
                frame = sim.render(stereo=False)
                cv2.putText(frame, f'SYNTHETIC REPLAY | {t:.1f}s | '+('FOLLOW' if t < 6 else 'HOLD'),
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 230, 255), 1)
                frames.append(frame)
        from PIL import Image
        images = [Image.fromarray(f) for f in frames]
        images[len(images) // 2].save(args.output / 'preview.png')
        images[0].save(args.output / 'synthetic-replay.gif', save_all=True,
                       append_images=images[1:], duration=200, loop=0)
        result = {'source': 'synthetic wrist poses; not a physical Quest session',
                  'physics_seconds': sim.data.time, 'frames': len(frames),
                  'max_wrist_motion_m': max_motion, 'tracking_error_mean_m': float(np.mean(errors)),
                  'tracking_error_max_m': float(np.max(errors)),
                  'finite_state': bool(np.isfinite(sim.data.qpos).all())}
        (args.output / 'verification.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
        assert min(max_motion.values()) > 0.05
        assert result['tracking_error_mean_m'] < 0.06
    finally:
        sim.close()

if __name__ == '__main__':
    main()
