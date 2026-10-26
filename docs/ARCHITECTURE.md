# Architecture

[Home](../README.md) · [Docs](INDEX.md) · [Setup](QUEST_SETUP.md) · [Results](VALIDATION.md)

## Components

```text
Quest Browser (WebXR)
  │ wrist transforms + grip/pinch, TLS/WebSocket :8012
  ▼
TeleVuer process ── shared tracked poses/timestamps/buttons ──► control process
  ▲                                                              │
  │ stereo JPEG image updates                                    │ 50 Hz target
  │                                                              ▼
  └── latest rendered frame ◄── renderer process ◄── measured MuJoCo qpos
                                    │                     500 Hz physics substeps
                                    └── desktop preview/status :8080
```

The main process owns physics and the per-arm clutch states. TeleVuer owns its
async Vuer server and shared tracking state. A spawned render process owns a
separate MuJoCo model/data/rendering context. The renderer receives only joint
positions and engagement flags; it never writes physics state or targets.
The read-only monitor is served by a thread in the main process.

The nominal loop is 50 Hz with ten 2 ms physics substeps per iteration. It sleeps
for the unused portion of 20 ms; overload slows simulation rather than skipping
unbounded steps. Slow video does not block control. Latest-frame buffers bound
latency instead of retaining an image backlog. Health requires a live server,
live renderer, recent control update and a video frame less than five seconds old.

## Relative pose mapping

Raw WebXR poses are 4×4 homogeneous matrices in column-major serialization:
right is +X, up is +Y, backward is +Z. Robot/world coordinates are forward +X,
left +Y, up +Z. The basis conversion is:

```text
R = [ 0  0 -1
     -1  0  0
      0  1  0 ]
```

On grip/pinch engagement, save the XR wrist reference `(p0, Q0)` and measured
robot tool pose `(r0, S0)`. While engaged:

```text
r_target = r0 + clamp_norm(0.75 * R * (p - p0), 0.35 m)
S_target = R * Q * transpose(Q0) * transpose(R) * S0
```

No absolute headset-floor height is imposed on the robot. Both arms have independent
references. The application rejects nonfinite, malformed, reflected or non-orthogonal
poses. Release, missing poses, future timestamps or a timestamp older than 250 ms
disengage the corresponding clutch. Subsequent engagement recalibrates.

## IK and physics

`simulation.py` uses MuJoCo's tool-site Jacobian. Twelve bounded damped-least-squares
iterations solve a six-dimensional wrist task against seven arm joints, with
rotational error weighted by 0.2. Each iteration clamps the joints inside their
original limits. Desired actuator positions slew at no more than 1.2 rad/s.
On hold, the measured joint positions become the actuator setpoints.

Native position actuators use `kp=80`, `kv=8`, force limits of ±25 and the original
joint-level limits. The engine applies bias-force compensation before each physics
substep. This is an arm rig with modeled inertias and real integration, not a mock
interface that directly assigns the simulated joint positions on every step.

The scene builder preserves the original MJCF and derives `scene.xml` during the
image build. It fixes the base, legs, waist and finger joints, replaces the hardware
motor bank with 14 position actuators, removes duplicate visual/collision meshes,
and adds a workbench, target blocks, tool sites and fixed stereo cameras.
Robot collisions are disabled; grasp/contact and balance claims cannot be made.

## Rendering

The eye cameras have a 64 mm baseline and each render 480×360 pixels at 75° vertical
FOV, looking down at the workbench. Combined images are 960×360 RGB. The two halves
are delivered on the left/right image layers through TeleVuer. The overlay is drawn
into each eye's image. Shadows/reflections are disabled to reduce rendering cost.

Default rendering uses CPU/OSMesa. `compose.gpu.yaml` switches to EGL with NVIDIA
GPU access. Neither requires host X11 or a desktop MuJoCo window. Browser image
messages use JPEG, not a WebRTC video stream, and 15 fps is a requested publish
rate rather than a guaranteed headset frame rate or latency measurement.

## Source integration choices

- Imported the **modified TeleVuer fork** from `geoff_remote_ws`; retained the local
  HUD/overlay implementation, pose freshness fields and lifecycle improvements.
- Added an awaited queue-drain opportunity after the initial tracking schema in
  both binocular display paths. Without it, queue length one could discard the
  Hands/MotionControllers message when the first image arrived.
- Used the **G1 MJCF and referenced meshes from GeoffMain**, including its Dex3-style
  hand geometry. BrainCo hardware services and its separate ROS arm publisher are
  not required for simulated wrist control.
- Implemented native MuJoCo IK instead of bringing in hardware DDS, Pinocchio,
  CasADi and robot SDK dependencies. There is no robot-network command path.
- `unitree_api`, `unitree_go` and `unitree_hg` message packages remain in the source
  workspace; the simulation-only container does not require a ROS build.

See [source provenance](SOURCES.md) for exact revisions and original import hashes.
