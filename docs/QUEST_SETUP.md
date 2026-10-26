# Meta Quest setup and operation

[Home](../README.md) · [Docs](INDEX.md) · [Architecture](ARCHITECTURE.md) · [Validation](VALIDATION.md)

## 1. Prepare the host

Use Linux x86-64 with Docker Engine, Compose v2 or newer, Python 3 and OpenSSL.
The current host was tested with Compose 5.6.0. Docker must work for your user.
For GPU mode, install NVIDIA Container Toolkit and confirm that Docker can access
your NVIDIA GPU; otherwise select `./dougie-vr up` for software rendering.

```bash
./dougie-vr build
./dougie-vr test
hostname -I
```

Choose the LAN address used by your Wi-Fi/Ethernet connection, not a Docker
bridge address. A wired host and a Quest on the same router's 5/6 GHz Wi-Fi are
preferable to two Wi-Fi links. Disable router client isolation for this local
connection if it prevents the headset reaching the host.

## 2. HTTPS certificates

WebXR needs a secure browser context. The server also uses a secure WebSocket.
The launcher creates a development self-signed certificate with SANs for localhost,
127.0.0.1 and the supplied LAN address:

```bash
./dougie-vr certs YOUR_LAN_IP
```

Replace `YOUR_LAN_IP` with an IPv4/IPv6 address, without a scheme or port. The
certificate and private key are `certs/cert.pem` and `certs/key.pem`; they are
ignored by Git and excluded from the image. Existing files are never overwritten.
Certificates are mounted read-only. If the LAN address changes, intentionally
move the old files aside and generate a new matching certificate.

**The generated certificate does not establish device trust by itself.** Trust
must be established in Quest Browser/device settings. Some browser/firmware
versions allow an explicit exception; others still block WebXR after the page
loads. Accepting a warning on your PC does not configure the headset.

If your Quest cannot trust this self-signed certificate, use a certificate issued
by a CA trusted by the headset (for example, a valid certificate for a hostname
you own that resolves to the host on your LAN). Place the full chain in
`certs/cert.pem` and its key in `certs/key.pem`, and use that hostname in **both**
URL positions. A local CA such as mkcert only helps if its CA certificate is also
installed and trusted on the headset; host-only `mkcert -install` is insufficient.
Do not copy the private server key to the headset.

## 3. Start and connect

```bash
./dougie-vr up-gpu   # NVIDIA/EGL
# ./dougie-vr up     # CPU/OSMesa fallback
./dougie-vr status
```

Open `https://HOST:8012/?ws=wss://HOST:8012` in Quest Browser. Replace both `HOST`
values. The page is bundled with Vuer in the image. Grant the requested spatial
tracking permissions and choose **Virtual Reality** for immersive mode. The
button labels and AR availability depend on browser/firmware capabilities.

Only the HTTPS port 8012 needs to be reachable from the headset. If a firewall
is enabled, permit TCP 8012 from your local headset network. The desktop monitor
at `http://127.0.0.1:8080` is bound to host loopback by default. This local
development server has no operator login; keep it on your trusted local network.

Before engaging control, confirm that the HUD displays the two virtual hands
and `HOLD` for both arms. The physical controller/hand pose does not need to match
the robot's anatomy: each arm is relatively anchored on engagement.

## Profiles

| Setting | Options | Default |
| --- | --- | --- |
| `INPUT_MODE` | `controller`, `hand` | `controller` |
| `DISPLAY_MODE` | `immersive`, `ego` | `immersive` |
| `DISPLAY_FPS` | 1–30, target camera publish rate | 15 |
| `VR_PORT` | Host TCP port mapped to 8012 | 8012 |
| `DASHBOARD_PORT` | Host desktop-monitor port | 8080 |
| `DASHBOARD_BIND` | Host monitor bind address | 127.0.0.1 |

```bash
INPUT_MODE=hand DISPLAY_MODE=ego ./dougie-vr up-gpu
```

Compose recreates the service when its profile changes, resetting the simulated
robot. For a persistent choice, put the settings in an untracked `.env` beside
`compose.yaml`. If changing `VR_PORT`, update **both** port values in the Quest URL.
The verification script uses the default dashboard port.

### Controller mode

- Use Quest Touch controllers with tracked poses.
- Hold the **grip/squeeze**, not the index trigger, to move the corresponding arm.
- The rising grip edge establishes an independent relative wrist reference.
- Release to hold; reposition yourself; grip again to recalibrate.

### Hand mode

- Enable hand tracking on the Quest and put the controllers down.
- Keep the wrists and fingers visible to the headset tracking cameras.
- **Pinch thumb and index finger** to move that arm; release to hold.
- The fingers in MuJoCo are fixed. Pinch is an arm clutch, not a grasp command.

### Viewing modes

- **Immersive:** a large stereo workbench display.
- **Ego:** a smaller stereo image panel. Choose AR/pass-through in the browser
  where supported to view it over the room. Passthrough is browser-controlled.
- Both show per-arm `FOLLOW`/`HOLD` and the relevant engagement instruction.
- The simulated camera pair is fixed; turning your head does not steer it.

Release both clutches before browser recentering or moving your standing position.
Close the headset session to stop tracking. The 250 ms freshness gate changes
both arms to hold. `./dougie-vr down` stops the application completely.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Container exits immediately | Run `./dougie-vr logs`; confirm both certificate files exist |
| NVIDIA runtime error | Confirm Container Toolkit works, or use `./dougie-vr up` |
| Headset cannot load page | LAN IP, TCP 8012 mapping/firewall, same-network reachability |
| Page loads but VR is missing | Certificate trust, WebXR/spatial permissions, headset browser support |
| Images appear but tracking counter stays zero | Enter XR mode; choose the matching hand/controller profile; inspect browser permissions |
| Counter advances but arm stays HOLD | Grip/pinch must be held and pose timestamps must be fresh; inspect logs |
| Hand disappears from view | Release pinch, bring it back into tracking view, pinch again |
| Arm cannot reach a target exactly | Relative workspace is bounded; joint limits and orientation compete in weighted IK |
| Video stutters but arms update | CPU rendering is much slower; switch to `up-gpu`; control and video are separate processes |
| Dark or frozen video | Check container health/logs and renderer backend; health fails on a stale video stream |
| Probe fails after repeated runs | It moves the arms; use `scripts/verify_sessions.py` to start each profile from a fresh scene |

## Final headset acceptance check

The automated checks cannot complete this step. With a Quest connected, confirm:
both eyes receive the correct view; each grip/pinch affects only its intended arm;
forward/left/up motion maps correctly; wrist rotation is comfortable; release,
tracking occlusion and browser exit hold the arms; re-engagement does not jump;
and both immersive and ego viewing are usable. Record the Quest model, firmware,
browser version, GPU, network and observed latency alongside the result.
