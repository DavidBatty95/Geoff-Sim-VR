#!/usr/bin/env python3
"""Publish successful local replay/profile evidence and source identifiers."""
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]

def main():
    result = json.loads((ROOT / 'results/verification.json').read_text())
    live = json.loads((ROOT / 'results/live-sessions.json').read_text())
    assert result['finite_state'] and min(result['max_wrist_motion_m'].values()) > 0.05
    assert len(live['sessions']) == 4 and all(s['disconnect_hold'] and s['tracking_component_received'] for s in live['sessions'])
    for directory in ('docs/media', 'docs/results'):
        (ROOT / directory).mkdir(parents=True, exist_ok=True)
    for name in ('synthetic-replay.gif', 'preview.png'):
        shutil.copy2(ROOT / 'results' / name, ROOT / 'docs/media' / name)
    for name in ('verification.json', 'live-sessions.json'):
        shutil.copy2(ROOT / 'results' / name, ROOT / 'docs/results' / name)
    if (ROOT / 'results/live-sessions-cpu.json').exists():
        shutil.copy2(ROOT / 'results/live-sessions-cpu.json', ROOT / 'docs/results/live-sessions-cpu.json')
    files = list((ROOT / 'dougie_vr').glob('*.py')) + list((ROOT / 'tests').glob('*.py'))
    files += list((ROOT / 'vendor/televuer/televuer').glob('*.py'))
    files += [ROOT / p for p in ('Dockerfile', 'requirements.txt', 'constraints.txt',
                                'compose.yaml', 'compose.gpu.yaml', 'assets/g1/original.xml',
                                'vendor/source-manifest.json')]
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
    (ROOT / 'docs/results/source-hashes.json').write_text(json.dumps(hashes, indent=2) + '\n')

if __name__ == '__main__':
    main()
