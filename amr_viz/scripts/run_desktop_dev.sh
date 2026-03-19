#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WEB_DIR="${PACKAGE_DIR}/desktop"
WEB_PORT="${AMR_VIZ_WEB_PORT:-5173}"

if [[ ! -f "${WEB_DIR}/package.json" ]]; then
  echo "amr_viz web package.json was not found at ${WEB_DIR}" >&2
  exit 1
fi

echo "run_desktop_dev.sh is kept as a compatibility alias."
echo "Serving AMR Viz web app on 0.0.0.0:${WEB_PORT}"

cd "${WEB_DIR}"
VITE_AMR_VIZ_BRIDGE_URL="${VITE_AMR_VIZ_BRIDGE_URL:-}" \
  npm run dev -- --host 0.0.0.0 --port "${WEB_PORT}"
