# Project status

[Home](../README.md) · [Docs](INDEX.md) · [Validation](VALIDATION.md)

## Implemented

- Self-contained Docker build with original G1 assets; no private dependency fetch
  is needed during the image build after cloning this repository.
- Fixed-base G1 arm physics and native six-dimensional wrist IK.
- Independent relative clutches for tracked controllers and hands.
- Immersive stereo camera view and smaller ego/HUD display.
- Renderer isolated from tracking/watchdog/physics.
- NVIDIA/EGL runtime and CPU/OSMesa fallback.
- Browser status/preview, certificate helper, daily launcher and shutdown cleanup.
- Real protocol tests, four-profile live probe, synthetic replay and evidence media.

## Next: on-device acceptance

Connect a physical Quest and complete the documented acceptance check. Adjust camera
panel ergonomics, motion scaling and orientation weighting using recorded headset
feedback. Do not interpret successful synthetic packet probes as completed pairing.

## Future integration

1. Add validated finger/BrainCo retargeting and a contact-enabled grasp scene.
2. Add optional head-orientation camera steering and camera/pose latency metrics.
3. Introduce collision-aware IK and task-specific workspace constraints.
4. Connect to Geoff-Sim's mobile platform through an explicit command interface,
   keeping one arm-command owner and separately qualifying balance and locomotion.

The current project is intentionally an arm-control commissioning rig. None of
these future features is reported as working in the present evidence.
