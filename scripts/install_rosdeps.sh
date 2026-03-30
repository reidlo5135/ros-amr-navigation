#!/usr/bin/env bash
set -euo pipefail

sudo apt update -y && sudo apt upgrade -y

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[rosdep] workspace: ${ROOT_DIR}"
echo "[rosdep] updating index..."
rosdep update

echo "[rosdep] installing package dependencies..."
rosdep install \
  --from-paths "${ROOT_DIR}" \
  --ignore-src \
  --rosdistro humble \
  -r \
  -y

echo "[rosdep] done"
