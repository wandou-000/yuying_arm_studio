# Arm Teach Studio

[简体中文](README.zh-CN.md) · English

Browser-based teaching pendant and trajectory studio for a 6-axis serial robot arm driven by an
ESP32 controller over a UART text protocol.

Drag the arm by hand, record the motion, denoise it, and play it back at adjustable speed — or
build a waypoint path in the 3D view and let a jerk-limited S-curve planner drive it through
inverse kinematics.

No Flask, no Qt, no build step. The backend is Python standard library + `pyserial` + `numpy`;
the frontend is plain ES modules with a vendored copy of three.js, so it runs fully offline.

> **This repository does not ship a robot model.** You must supply your own URDF + STL mesh
> directory. See [Robot model](#robot-model).

## Features

- **Live pose** — 3D visualisation of the real arm at ~20 Hz via Server-Sent Events, plus a
  per-axis table of position / velocity / current / torque / temperature / online state.
- **Drag teaching** — put the motors into zero-torque mode, move the arm by hand, record the
  joint stream, then median-filter (spike removal) and moving-average (smoothing) it.
- **One-click replay** — replay the smoothed trajectory at 0.1×–3.0× with a progress bar and a
  stop button.
- **Waypoint planner** — line and Catmull-Rom curve waypoints solved through IK, with a
  closed-form seven-segment asymmetric S-curve velocity profile (continuous acceleration,
  bounded jerk) and junction-deviation cornering limits.
- **Jogging & calibration** — per-axis sliders, a zero-point calibration for URDF/motor offset,
  and per-axis sign flipping.
- **Safety** — an always-available E-stop button; enable/disable map to firmware `CLEAR`/`ESTOP`.
- **Offline analysis** — `analyze_plan.py` inspects an exported plan dump and reports Cartesian
  vs. joint-space velocity ripple, per-axis peak velocity/acceleration, and optional CSV export.

## Architecture

```
Browser frontend (three.js + URDF)          Python backend (server.py)
  ├─ live 3D pose / TCP path      <──SSE──   ├─ serial I/O (pyserial)
  ├─ per-axis telemetry table     ──HTTP──>  ├─ recording, numpy denoise
  ├─ waypoint planner + IK                   ├─ trajectory replay (speed-scaled)
  └─ control panel                           └─ static file + asset serving
```

## Requirements

- Python 3.9+
- `pyserial >= 3.5`, `numpy >= 1.20`
- A Chromium-based browser (Chrome / Edge) — ES modules and SSE are required
- The robot-side firmware speaking the `$ARM,...*XX` UART protocol (see [Protocol](#protocol))

```bash
git clone https://github.com/wandou-000/yuying_arm_studio.git
cd yuying_arm_studio
pip install -r requirements.txt
```

## Robot model

The 3D view needs a URDF and its STL meshes. They are **not** included here (they are specific to
one arm design and are large binaries). Point the server at your own model directory:

```
your_robot_model/
├── urdf/
│   └── robot.urdf        # any single .urdf file; auto-detected
└── meshes/
    └── Link1.STL ...     # referenced as package://<pkg>/meshes/... in the URDF
```

`package://<anything>/` prefixes in the URDF are rewritten to `/assets/`, so a mesh declared as
`package://myarm/meshes/Link1.STL` is served from `your_robot_model/meshes/Link1.STL`.

Then start with either:

```bash
python server.py --assets /path/to/your_robot_model
# or
ARM_TEACH_ASSETS=/path/to/your_robot_model python server.py
```

Without a model the server still starts and the serial/telemetry side works, but the 3D view will
report that the URDF failed to load.

## Running

```bash
python server.py                              # http://127.0.0.1:8000
python server.py --port 8080
python server.py --assets ../my_model --port 8080
```

Open the printed URL in your browser.

## Usage

1. **Connect** — pick the serial port (e.g. `COM3`, `/dev/ttyUSB0`) and baud rate (default
   `1000000`) in the top bar.
2. **Enable / disable** — enable sends `ARM,CLEAR` (clears E-stop, restores position-velocity
   mode); disable sends `ARM,ESTOP`.
3. **Teach** — *Enter teach mode* sends `ARM,TEACH,1` and drops the motors to zero torque so the
   arm can be moved by hand. *Start recording* streams joint angles into the backend;
   *Stop recording* draws the raw TCP path in orange.
4. **Smooth** — set the spike-removal and smoothing window sizes, then run smoothing; the result
   is overlaid in cyan.
5. **Replay** — choose a speed multiplier and play the smoothed trajectory back.
6. **Plan** — add line/curve waypoints in the 3D view; the planner solves IK and generates a
   jerk-limited joint trajectory, which is streamed as `ARM,ALL` frames.
7. **Jog** — per-axis sliders send `ARM,SET`.

### Joint calibration

Motors report absolute position (0–360°), which will not match the URDF zero pose. Move the arm
to a known reference pose and press **Calibrate zero** to define the visual zero. If an axis
rotates the wrong way in the 3D view, set `calib.sign[j] = -1` near the top of `web/app.js`.

## Protocol

Frames are `$...*XX` with an XOR checksum, matching the ESP32 firmware.

| Purpose | Frame |
| --- | --- |
| Query feedback | `$ARM,GET*..` |
| Single-axis move | `$ARM,SET,<axis>,<angle_cdeg>,<speed_dps>*..` |
| Six-axis move | `$ARM,ALL,...*..` |
| E-stop / disable | `$ARM,ESTOP*..` |
| Clear / enable | `$ARM,CLEAR*..` |
| Enter / exit teach | `$ARM,TEACH,1` / `$ARM,TEACH,0` |
| Feedback frame | `$ARM,FB,<estop>,<moving>,<8 fields × 6 axes>*..` |

Angles are centi-degrees; speeds are integer degrees per second.

## Layout

```
arm_teach_studio/
├── server.py          # serial link, HTTP + SSE, recording, denoise, replay
├── analyze_plan.py    # offline plan-dump analysis
├── requirements.txt
└── web/
    ├── index.html
    ├── style.css
    ├── app.js         # 3D scene, SSE client, UI wiring
    ├── robot.js       # URDF parsing, STL loading, forward kinematics
    ├── planner.js     # waypoints → S-curve joint trajectory
    ├── ik.js          # inverse kinematics
    └── vendor/        # vendored three.js, OrbitControls, STLLoader
```

## Safety

This software commands real motors that can move fast and pinch. Keep clear of the workspace,
keep the E-stop reachable, and test new trajectories at low speed. The software is provided
without warranty of any kind — see [LICENSE](LICENSE).

## Contributing

Issues and pull requests are welcome. See [CONTRIBUTING.md](CONTRIBUTING.md).

## License

MIT — see [LICENSE](LICENSE).

Bundled third-party code in `web/vendor/` (three.js, OrbitControls, STLLoader) is MIT licensed and
copyright its respective authors.
