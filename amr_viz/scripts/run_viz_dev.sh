#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PACKAGE_DIR}/.." && pwd)"
WEB_DIR="${PACKAGE_DIR}/desktop"
BRIDGE_HOST="${AMR_VIZ_BRIDGE_HOST:-0.0.0.0}"
BRIDGE_PORT="${AMR_VIZ_BRIDGE_PORT:-8765}"
WEB_PORT="${AMR_VIZ_WEB_PORT:-5173}"

detect_workspace_root() {
  local repo_root="$1"
  local parent_dir
  parent_dir="$(dirname "${repo_root}")"
  if [[ "$(basename "${parent_dir}")" == "src" ]]; then
    dirname "${parent_dir}"
    return
  fi
  echo "${repo_root}"
}

WORKSPACE_ROOT="$(detect_workspace_root "${REPO_ROOT}")"

source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  echo "Using ROS setup: ${WORKSPACE_ROOT}/install/setup.bash"
  source_setup "${WORKSPACE_ROOT}/install/setup.bash"
elif [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "Using ROS setup: ${REPO_ROOT}/install/setup.bash"
  source_setup "${REPO_ROOT}/install/setup.bash"
elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
  echo "Using ROS setup: /opt/ros/humble/setup.bash"
  source_setup "/opt/ros/humble/setup.bash"
else
  echo "ROS environment setup script was not found." >&2
  exit 1
fi

if [[ ! -f "${WEB_DIR}/package.json" ]]; then
  echo "amr_viz web package.json was not found at ${WEB_DIR}" >&2
  exit 1
fi

if ss -ltn "( sport = :${BRIDGE_PORT} )" | tail -n +2 | grep -q .; then
  echo "AMR Viz bridge port ${BRIDGE_PORT} is already in use." >&2
  echo "If another bridge is already running, use ./amr_viz/scripts/run_web_dev.sh instead." >&2
  echo "Or choose another port with AMR_VIZ_BRIDGE_PORT." >&2
  exit 1
fi

if ss -ltn "( sport = :${WEB_PORT} )" | tail -n +2 | grep -q .; then
  echo "AMR Viz web port ${WEB_PORT} is already in use." >&2
  echo "Stop the existing dev server or choose another port with AMR_VIZ_WEB_PORT." >&2
  exit 1
fi

cleanup() {
  if [[ -n "${BRIDGE_PID:-}" ]]; then
    kill "${BRIDGE_PID}" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT INT TERM

ros2 launch amr_viz amr_viz.launch.py websocket_host:="${BRIDGE_HOST}" websocket_port:="${BRIDGE_PORT}" "$@" &
BRIDGE_PID=$!

echo "Serving AMR Viz web app on 0.0.0.0:${WEB_PORT}"
echo "Bridge is available on ws://${BRIDGE_HOST}:${BRIDGE_PORT}"

cd "${WEB_DIR}"
VITE_AMR_VIZ_BRIDGE_URL="${VITE_AMR_VIZ_BRIDGE_URL:-}" \
  npm run dev -- --host 0.0.0.0 --port "${WEB_PORT}"
