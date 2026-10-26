"""TeleVuer ↔ MuJoCo runtime, plus a read-only desktop status/preview endpoint."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time
import cv2
from .control import Clutch
from .simulation import Simulation
from .rendering import Video

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-mode', choices=['hand', 'controller'], default=os.getenv('INPUT_MODE', 'controller'))
    parser.add_argument('--display-mode', choices=['immersive', 'ego'], default=os.getenv('DISPLAY_MODE', 'immersive'))
    parser.add_argument('--fps', type=float, default=float(os.getenv('DISPLAY_FPS', '15')))
    args = parser.parse_args()
    if not 1 <= args.fps <= 30:
        parser.error('fps must be between 1 and 30')
    for var in ('XR_TELEOP_CERT', 'XR_TELEOP_KEY'):
        if not Path(os.environ.get(var, '/missing')).is_file():
            parser.error(f'{var} missing: run ./dougie-vr certs HOST_IP')
    # Vuer has its own CLI parser; keep application flags out of its argv.
    sys.argv = sys.argv[:1]
    from televuer import TeleVuer
    tv = TeleVuer(use_hand_tracking=args.input_mode == 'hand', binocular=True,
                  img_shape=(360, 960), display_fps=args.fps, display_mode=args.display_mode,
                  zmq=True, image_quality=65, queue_len=1)
    sim = Simulation()
    video = Video(sim, args.fps, args.input_mode)
    clutches = {side: Clutch() for side in ('left', 'right')}
    stop = threading.Event()
    state = {'updated': 0., 'video_updated': 0., 'status': {}, 'jpeg': b''}

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            healthy = (time.monotonic() - state['updated'] < 5 and tv.process.is_alive()
                       and video.process.is_alive() and bool(state['jpeg']))
            healthy = healthy and time.monotonic() - state['video_updated'] < 5
            if self.path == '/health':
                content, kind = json.dumps({'ok': healthy, **state['status']}).encode(), 'application/json'
                code = 200 if healthy else 503
            elif self.path.split('?')[0] == '/frame.jpg':
                content, kind, code = state['jpeg'], 'image/jpeg', 200
            elif self.path == '/':
                content = (b'<!doctype html><title>Dougie-VR monitor</title><body style="background:#101b29;color:white;font:18px system-ui">'
                    b'<h1>Dougie-VR | Live MuJoCo stereo monitor</h1><img id="view" width="960"><pre id="status"></pre>'
                    b'<p>Open HTTPS port 8012 in Quest Browser for VR. Hold grip (controllers) or pinch (hands) to move.</p>'
                    b'<script>setInterval(()=>{document.getElementById("view").src="/frame.jpg?t="+Date.now();fetch("/health").then(r=>r.json()).then(s=>document.getElementById("status").textContent=JSON.stringify(s,null,2))},200)</script>')
                kind, code = 'text/html', 200
            else:
                content, kind, code = b'Not found', 'text/plain', 404
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(content)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(content)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('0.0.0.0', 8080), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.set())
    print('Dougie-VR ready: Quest HTTPS :8012, desktop monitor HTTP :8080', flush=True)
    last_log = 0.
    started = time.monotonic()
    try:
        while not stop.is_set():
            tick = time.monotonic()
            if not tv.process.is_alive():
                raise RuntimeError('XR server exited')
            if not video.process.is_alive():
                raise RuntimeError('Renderer exited')
            targets = {}
            now = time.time()
            for side, clutch in clutches.items():
                pressed = getattr(tv, f'{side}_hand_pinch' if args.input_mode == 'hand' else f'{side}_ctrl_squeeze')
                targets[side] = clutch.update(getattr(tv, f'{side}_arm_pose'),
                    getattr(tv, f'{side}_arm_update_time'), pressed, now, sim.pose(side))
            sim.step(targets)
            status = {'simulation_time': round(sim.data.time, 2), 'input_mode': args.input_mode,
                      'left': 'FOLLOW' if clutches['left'].engaged else 'HOLD',
                      'right': 'FOLLOW' if clutches['right'].engaged else 'HOLD',
                      'video_frames': video.seen,
                      'wall_seconds': round(tick - started, 2),
                      'wrist_positions': {s: sim.pose(s)[:3, 3].tolist() for s in sim.arms},
                      'camera_events': tv.camera_move_count,
                      'tracking_events': tv.hand_move_count if args.input_mode == 'hand' else tv.controller_move_count}
            frame = video.update(sim, clutches)
            if frame is not None:
                tv.render_to_xr(frame)
                state['jpeg'] = cv2.imencode('.jpg', cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))[1].tobytes()
                state['video_updated'] = time.monotonic()
            state.update(updated=time.monotonic(), status=status)
            if tick - last_log >= 5:
                print(json.dumps(status), flush=True)
                last_log = tick
            stop.wait(max(0, 0.02 - (time.monotonic() - tick)))
    finally:
        server.shutdown()
        server.server_close()
        tv.close()
        video.close()
        sim.close()

if __name__ == '__main__':
    main()
