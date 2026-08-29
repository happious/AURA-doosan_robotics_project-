#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
if [ -f "$HOME/aura_ws/install/setup.bash" ]; then
  source "$HOME/aura_ws/install/setup.bash"
fi

export AURA_DB_PATH="${AURA_DB_PATH:-$HOME/Downloads/data/amr_system.db}"
export AURA_ADMIN_CCTV_TOPIC="${AURA_ADMIN_CCTV_TOPIC:-/cctv/image_raw/compressed}"
export AURA_ADMIN_PORT="${AURA_ADMIN_PORT:-7000}"

cd "$(dirname "$0")"
exec python3 app.py
