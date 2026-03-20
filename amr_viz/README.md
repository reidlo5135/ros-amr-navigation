# amr_viz

`amr_viz` is the current visualization workspace for this project.

At the moment it contains:

- a React web client under `desktop/`
- the older Python WebSocket bridge path used during transition
- helper scripts for running the web client and the legacy bridge together

## Current Position In The Stack

Today:

- `amr_viz` can still be used as a web visualization package
- it can run against the existing WebSocket bridge path

Project direction:

- transport responsibilities move to MQTT packages
  - `amr_mqtt_bridge`
  - `amr_mqtt_robot_plugin`
- `amr_viz` remains focused on the UI side

So this package should be read as a transitional visualization package, not the
long-term transport layer.

## Layout

- `desktop/`
  - React + TypeScript web app
- `amr_viz/bridge.py`
  - existing Python WebSocket bridge
- `launch/amr_viz.launch.py`
  - launch entry for the legacy bridge
- `scripts/`
  - helper scripts for dev runs

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

Start only the web app:

```bash
./amr_viz/scripts/run_web_dev.sh
```

Start bridge plus web app together:

```bash
./amr_viz/scripts/run_viz_dev.sh
```

Host access:

- web app is served on `0.0.0.0`
- default dev port is `5173`

Typical host-PC access:

```text
http://<ubuntu-server-ip>:5173
```

## Legacy Bridge Notes

The current bridge scripts are still useful for quick visualization tests, but
they are not the preferred long-term transport architecture.

Preferred long-term direction:

- navigation and robot data move through MQTT
- web UI consumes modeled data from the MQTT transport layer

## Scripts

- `install_desktop_deps.sh`
  - installs frontend dependencies
- `run_web_dev.sh`
  - starts the web dev server
- `run_bridge.sh`
  - starts the legacy WebSocket bridge
- `run_viz_dev.sh`
  - starts the legacy bridge and the web app together
- `kill_all.sh`
  - if present in your local branch, use it to stop leftover viz processes
