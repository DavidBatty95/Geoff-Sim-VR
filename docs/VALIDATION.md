# Verification and measured outcomes

[Home](../README.md) · [Docs](INDEX.md) · [Setup](QUEST_SETUP.md) · [Architecture](ARCHITECTURE.md)

## What passed

On 26 October 2026, the Linux Docker image was built and exercised with MuJoCo 3.3.6,
Python 3.11, Vuer 0.0.60, the vendored TeleVuer fork and the original G1 meshes.
Python versions are constrained in `constraints.txt`; the base image is digest-pinned.

| Check | Result |
| --- | --- |
| Docker image build and dependency resolution | Pass |
| Physics/IK/frame validation suite | **10 tests passed** |
| Eight-second wrist replay | Pass; both wrists move 0.14267 m |
| Replay mean/max tool position error | 0.03422 m / 0.06707 m |
| Controller + immersive live profile | Pass |
| Controller + ego live profile | Pass |
| Hand + immersive live profile | Pass |
| Hand + ego live profile | Pass |
| HTTPS page and read-only health/preview endpoints | Responded successfully |
| Physical Meta Quest session | **Not tested** |

The unit/integration suite exercises frame conversion, malformed/nonfinite/reflected
poses, stale/future timestamps, reach bounding, engagement calibration, wrist rotation,
both-arm physical reaching, joint limits, hold drift, stereo rendering, and the actual
vendored hand/controller event handlers.

## Live protocol probes

[`live-sessions.json`](results/live-sessions.json) records the complete four-profile
matrix. Each profile starts a fresh container, connects to its real secure Vuer
WebSocket, sends 150 synthetic input events, checks measured tool movement, receives
image messages, and verifies that both arms hold after the socket closes.
The probe also checks that the browser receives the initial tracking component.

The same four-profile matrix also passed with the CPU/OSMesa fallback:
[`live-sessions-cpu.json`](results/live-sessions-cpu.json). Image message counts
include republication of the latest frame and are not unique render-frame rates.

The recorded probes received 39–42 image messages, advanced simulation at
0.992–0.995 of wall time, and moved both wrists forward by at least 3 cm. Input
packets include synthetic 25-joint hand matrices in hand mode. This validates the
server contract, not optical tracking quality or browser gesture recognition.

```bash
./dougie-vr build
./dougie-vr test
./dougie-vr demo
./dougie-vr certs YOUR_LAN_IP  # once, or provide your trusted cert/key
python3 scripts/verify_sessions.py --gpu
```

The matrix starts/recreates and finally stops only the `dougie-vr` Compose project.
Run it without a headset connected. `./dougie-vr probe` is a single-profile check
for an already-running, freshly started scene. Repeated probes move the robot
farther into its workspace; use the matrix for reproducible results.

## Reproducible media

The README GIF contains all 40 frames of the eight-second replay at 200 ms/frame.
`dougie_vr.demo` generates the image and metrics from the same run. No prerecorded
robot movements or static stock images are substituted. The input is clearly
marked synthetic in the footage and metrics.

[`verification.json`](results/verification.json) records the replay measurements.
[`source-hashes.json`](results/source-hashes.json) identifies the code/assets used
when publishing the evidence. `python3 scripts/publish_evidence.py` copies a new
successful run into the documentation; normal runs write only ignored `results/`.

## Performance scope

The software-rendered stereo benchmark on this laptop was about 1.5 fps before
scene simplification and process separation. The NVIDIA/EGL benchmark measured
about 56 stereo renders/s over 20 renders, including initialization, without a
headset attached. Those are renderer measurements, **not end-to-end VR latency**.
The live application caps publication at the configured 15 fps by default.

## What remains to verify on hardware

The Quest's certificate trust, actual WebXR entry, stereo eye assignment, controller
button/hand pinch delivery, tracking-loss experience, passthrough panel placement,
and motion-to-photon latency need a real headset session. Use the
[on-device acceptance check](QUEST_SETUP.md#final-headset-acceptance-check) and record
the headset/browser/network versions with the outcome.

This release does not qualify physical robot operation, collision-aware reaching,
grasping, finger retargeting, locomotion or full-body balance.
