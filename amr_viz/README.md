# amr_viz

`amr_viz` is the visualization workspace for this project.

Right now it is a transitional package. The long-term direction is a React UI
that consumes MQTT-backed data, while transport ownership stays outside this
package.

## Current Position

Today:

- `amr_viz` contains the React web client workspace under `desktop/`
- legacy WebSocket bridge code still exists from the earlier prototype phase
- helper scripts remain useful for UI iteration

Direction:

- transport belongs in:
  - `amr_mqtt_bridge`
  - `amr_mqtt_robot_plugin`
- `amr_viz` should converge toward UI-only responsibility

## Layout

- `desktop/`
  - React + TypeScript web app workspace
- `amr_viz/bridge.py`
  - legacy Python WebSocket bridge from the pre-MQTT phase
- `launch/amr_viz.launch.py`
  - launch entry for the legacy bridge
- `scripts/`
  - helper scripts for frontend and legacy runs

## Requirements

For the current web client:

- Node.js
- npm

For the legacy bridge path:

- ROS 2 Humble
- Python 3.10
- Python `websockets` package compatible with Python 3.10

Recommended Python install for the legacy bridge:

```bash
sudo apt install python3-pip
python3 -m pip install --user --upgrade "websockets>=12,<13"
```

## Web Dev Run

Install frontend dependencies:

```bash
./amr_viz/scripts/install_desktop_deps.sh
```

Start the web app:

```bash
./amr_viz/scripts/run_web_dev.sh
```

Typical host-PC access:

```text
http://<ubuntu-server-ip>:5173
```

## Notes

- current bridge scripts are legacy/prototype paths
- the target architecture is React + MQTT, not Python WebSocket bridging
- this package should stay focused on visualization rather than transport
