"""Isolate rendering so headset video never blocks tracking/watchdog/physics."""
import multiprocessing as mp
import time
import cv2
import mujoco
import numpy as np
from .simulation import Simulation

def render_worker(qpos, flags, pixels, sequence, shutdown, fps, mode):
    sim = Simulation()
    try:
        while not shutdown.is_set():
            start = time.monotonic()
            with qpos.get_lock():
                sim.data.qpos[:] = qpos[:]
            mujoco.mj_forward(sim.model, sim.data)
            frame = sim.render()
            with flags.get_lock():
                states = ['FOLLOW' if flag else 'HOLD' for flag in flags[:]]
            for x in (0, 480):
                cv2.putText(frame, f'DOUGIE-VR  L:{states[0]} R:{states[1]}',
                            (x + 12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 230, 255), 1)
                cv2.putText(frame, 'PINCH to move' if mode == 'hand' else 'HOLD GRIP to move',
                            (x + 12, 345), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (235, 235, 235), 1)
            with pixels.get_lock():
                np.frombuffer(pixels.get_obj(), dtype=np.uint8)[:] = frame.reshape(-1)
                with sequence.get_lock():
                    sequence.value += 1
            shutdown.wait(max(0, 1/fps - (time.monotonic() - start)))
    finally:
        sim.close()

class Video:
    def __init__(self, sim, fps, mode):
        ctx = mp.get_context('spawn')
        self.qpos = ctx.Array('d', sim.data.qpos)
        self.flags = ctx.Array('b', [False, False])
        self.pixels = ctx.Array('B', 360 * 960 * 3)
        self.sequence = ctx.Value('i', 0)
        self.shutdown = ctx.Event()
        self.process = ctx.Process(target=render_worker,
            args=(self.qpos, self.flags, self.pixels, self.sequence, self.shutdown, fps, mode), daemon=True)
        self.process.start()
        self.seen = 0

    def update(self, sim, clutches):
        with self.qpos.get_lock():
            self.qpos[:] = sim.data.qpos
        with self.flags.get_lock():
            self.flags[:] = [clutches[s].engaged for s in ('left', 'right')]
        if self.sequence.value == self.seen:
            return None
        with self.pixels.get_lock():
            image = np.frombuffer(self.pixels.get_obj(), dtype=np.uint8).reshape(360, 960, 3).copy()
            self.seen = self.sequence.value
        return image

    def close(self):
        self.shutdown.set()
        self.process.join(timeout=5)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=2)
