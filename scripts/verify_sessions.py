#!/usr/bin/env python3
"""Restart this project's container and verify all four XR input/display profiles."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gpu', action='store_true')
    parser.add_argument('--output', type=Path, default=ROOT / 'results/live-sessions.json')
    args = parser.parse_args()
    compose = ['docker', 'compose', '-p', 'dougie-vr', '-f', str(ROOT / 'compose.yaml')]
    if args.gpu:
        compose += ['-f', str(ROOT / 'compose.gpu.yaml')]
    results = []
    try:
        for mode in ('controller', 'hand'):
            for display in ('immersive', 'ego'):
                env = dict(os.environ, INPUT_MODE=mode, DISPLAY_MODE=display)
                subprocess.run(compose + ['up', '-d', '--force-recreate'], cwd=ROOT, env=env, check=True)
                for _ in range(60):
                    time.sleep(1)
                    try:
                        with urllib.request.urlopen('http://127.0.0.1:8080/health', timeout=2) as r:
                            status = json.load(r)
                        if status['ok'] and status['input_mode'] == mode:
                            break
                    except (OSError, ValueError):
                        pass
                else:
                    raise RuntimeError('Container did not become healthy')
                completed = subprocess.run(compose + ['exec', '-T', 'vr', 'python', '-m', 'dougie_vr.probe'],
                                           capture_output=True, text=True, check=True)
                result = json.loads(completed.stdout)
                result['display_mode'] = display
                results.append(result)
                print(json.dumps(result), flush=True)
    finally:
        subprocess.run(compose + ['down'], cwd=ROOT, check=True)
    out = ROOT / 'results'
    out.mkdir(exist_ok=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({
        'timestamp_utc': datetime.now(timezone.utc).isoformat(),
        'renderer': 'EGL/NVIDIA' if args.gpu else 'OSMesa/CPU',
        'physical_headset_tested': False, 'sessions': results}, indent=2) + '\n')

if __name__ == '__main__':
    main()
