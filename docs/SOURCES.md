# Sources and provenance

[Home](../README.md) · [Docs](INDEX.md) · [Architecture](ARCHITECTURE.md)

## GeoffMain

- Repository: `git@github.com:DavidBatty95/GeoffMain.git`
- Revision: `01b690f2dbe670d37a763bbf2846e8c3b4b0bf09`
- Imported `src/xr_teleoperate/assets/g1/g1_body29_hand14.xml` as
  `assets/g1/original.xml`, its 49 referenced STL files, upstream model README,
  and XR license notice.
- Reviewed the custom upper-body controllers and the file named
  `sim/mujoco_smoke_test.py`. That file exercises a mock controller; it does not
  run MuJoCo physics. Dougie-VR's physics implementation is new.
- `model.py` deterministically derives the arm-rig scene during the build; it
  does not modify the imported original MJCF.

## geoff_remote_ws / TeleVuer

- Source: `/home/david/geoff_remote_ws/src/televuer` on the development laptop.
- Upstream: `https://github.com/unitreerobotics/televuer.git`, revision `b6ed5db`.
- The local working tree contained substantial uncommitted additions to
  `televuer.py`: arm/hand overlays, camera-plane configuration, image-quality
  controls, low-latency buffering, tracking timestamps/counters, malformed-event
  handling, and process teardown. Those local changes were imported, not replaced
  with the clean upstream package.
- Imported the three Python package files and MIT license. ROS build/install/log
  artifacts, private HTTPS keys and Python caches were not imported.
- Dougie-VR adds a queue-drain yield in the binocular immersive and ego methods,
  preventing initial tracking configuration being overwritten by image updates.

[`vendor/source-manifest.json`](../vendor/source-manifest.json) contains the hashes
of files **at import time**, before the documented queue fix. Current application
and vendor hashes associated with the evidence are in
[`docs/results/source-hashes.json`](results/source-hashes.json).

## Licenses

Original application, test and documentation additions are covered by the root
MIT license. TeleVuer retains its own MIT notice. G1 assets retain the Unitree
XR Apache-2.0 notice and upstream model documentation. MuJoCo, Vuer, OpenCV and
other installed dependencies retain their respective licenses.

The `geoff_remote_ws` source workspace and the excluded Geoff-Sim/Dougie-Sim
workspaces were not modified by this integration.
