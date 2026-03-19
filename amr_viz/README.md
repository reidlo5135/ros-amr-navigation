# amr_viz

`amr_viz` is the dedicated visualization package for this AMR stack.

It keeps the ROS-facing bridge and the React web client in one package so we can evolve the
visualization protocol alongside the navigation stack without depending on generic tooling.

## Requirements

- ROS 2 Humble workspace
- Python 3.10
- Node.js and npm
- Python `websockets` package version `10+`

### Python WebSocket Runtime

`amr_viz` bridge requires a Python `websockets` version compatible with Python 3.10.

Ubuntu `apt install python3-websockets` may install `websockets 9.1`, which fails during
WebSocket connection on Python 3.10.

Recommended installation:

```bash
sudo apt install python3-pip
python3 -m pip install --user --upgrade "websockets>=12,<13"
```

Check the active runtime:

```bash
python3 - <<'PY'
import websockets
print(websockets.__version__)
print(websockets.__file__)
PY
```

Expected result:

- version `12.x` or another `10+` release
- module path usually under `~/.local/lib/...`

## Layout

- `amr_viz/`
  - Python ROS 2 bridge package
- `scripts/amr_viz_bridge`
  - bridge entrypoint
- `launch/amr_viz.launch.py`
  - starts the WebSocket bridge
- `config/bridge.yaml`
  - default ROS topic and WebSocket settings
- `desktop/`
  - React + TypeScript web application scaffold

## Current Scope

- subscribes to:
  - `/amr/localization/pose`
  - `/amr/planner/global`
  - `/amr/planner/local`
  - `/amr/map/data`
  - `/amr/costmap/global`
  - `/amr/costmap/local`
  - `/amr/motion/status`
  - `/amr/obstacle/report`
- exposes a dedicated WebSocket protocol for:
  - live state updates
  - `navigate_to_pose`
  - `set_initial_pose`

## Web App

Inside `desktop/`:

- Vite powers the renderer build
- React + TypeScript drive the UI
- Three.js renders the scene
- the app is hosted as a web client and can be opened from another PC

Once Node.js and npm are available:

```bash
cd amr_viz/desktop
npm install
npm run dev
```

By default, the app derives the bridge URL from the current page host and connects to
`ws://<page-host>:8765`.

## Convenience Scripts

Use these from the repository root:

```bash
./amr_viz/scripts/install_desktop_deps.sh
./amr_viz/scripts/run_bridge.sh
./amr_viz/scripts/run_web_dev.sh
./amr_viz/scripts/run_desktop_dev.sh
./amr_viz/scripts/run_viz_dev.sh
```

- `run_bridge.sh`
  - launches only the ROS bridge
- `run_web_dev.sh`
  - starts only the web dev server on `0.0.0.0`
- `run_desktop_dev.sh`
  - compatibility alias for `run_web_dev.sh`
- `run_viz_dev.sh`
  - starts the ROS bridge in the background and then starts the web dev server

To use a different bridge port:

```bash
AMR_VIZ_BRIDGE_PORT=8877 ./amr_viz/scripts/run_bridge.sh
AMR_VIZ_BRIDGE_PORT=8877 ./amr_viz/scripts/run_web_dev.sh
AMR_VIZ_BRIDGE_PORT=8877 ./amr_viz/scripts/run_desktop_dev.sh
AMR_VIZ_BRIDGE_PORT=8877 ./amr_viz/scripts/run_viz_dev.sh

AMR_VIZ_WEB_PORT=5174 ./amr_viz/scripts/run_web_dev.sh
AMR_VIZ_WEB_PORT=5174 ./amr_viz/scripts/run_viz_dev.sh
```

## First Run

From the workspace root:

```bash
colcon build --packages-select amr_viz
./src/ros-amr-navigation/amr_viz/scripts/install_desktop_deps.sh
```

Run bridge and web app separately:

Terminal 1:

```bash
./src/ros-amr-navigation/amr_viz/scripts/run_bridge.sh
```

Terminal 2:

```bash
./src/ros-amr-navigation/amr_viz/scripts/run_web_dev.sh
```

Or launch both together:

```bash
./src/ros-amr-navigation/amr_viz/scripts/run_viz_dev.sh
```

From a host PC on the same network, open:

```text
http://<ubuntu-server-ip>:5173
```

The page will use:

```text
ws://<ubuntu-server-ip>:8765
```

unless `VITE_AMR_VIZ_BRIDGE_URL` is explicitly provided.

## Troubleshooting

- If the web app shows `Bridge connection error`, confirm the bridge terminal stays open.
- If the bridge starts but WebSocket clients cannot connect, check the Python `websockets` version first.
- If a bridge is already running, do not start `run_viz_dev.sh` again. Use `run_web_dev.sh`.
- The scripts prefer the current workspace overlay such as `~/ws/install/setup.bash`.
