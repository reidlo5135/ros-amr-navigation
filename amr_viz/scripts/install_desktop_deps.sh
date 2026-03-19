#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DESKTOP_DIR="${PACKAGE_DIR}/desktop"

if [[ ! -f "${DESKTOP_DIR}/package.json" ]]; then
  echo "amr_viz desktop package.json was not found at ${DESKTOP_DIR}" >&2
  exit 1
fi

cd "${DESKTOP_DIR}"
npm install
