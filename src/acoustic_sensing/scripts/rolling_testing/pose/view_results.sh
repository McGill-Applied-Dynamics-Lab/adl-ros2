#!/usr/bin/env bash
# Launch the rolling-result replay viewer with the project's pixi env.
#
# Usage:
#   ./view_results.sh                              # interactive: scan default results/ folder
#   ./view_results.sh path/to/folder               # interactive: scan that folder
#   ./view_results.sh path/to/file.pkl             # replay that file directly
#   ./view_results.sh --results-dir /elsewhere     # scan elsewhere
#   ROLLING_RESULTS_DIR=/elsewhere ./view_results.sh
#   ./view_results.sh --fps 30 …                   # change replay framerate
#
# Resolution order:
#   1. .pkl file given as argument
#   2. Directory given as argument (a trial folder, or a root of trial folders)
#   3. --results-dir <DIR>
#   4. $ROLLING_RESULTS_DIR
#   5. results/ folder next to read_rolling_results.py
#
# The pixi python is found by walking up from this script's location looking
# for a pixi.toml — so you can move the whole repo without breaking the path.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Walk upward to locate the project root (one containing pixi.toml).
find_pixi_root() {
  local d="$SCRIPT_DIR"
  while [[ "$d" != "/" ]]; do
    if [[ -f "$d/pixi.toml" ]]; then
      echo "$d"
      return 0
    fi
    d="$(dirname "$d")"
  done
  return 1
}

REPO_ROOT="$(find_pixi_root || true)"
if [[ -z "${REPO_ROOT:-}" ]]; then
  echo "Error: could not find pixi.toml anywhere above $SCRIPT_DIR." >&2
  echo "       Move view_results.sh back inside the repo, or set PIXI_PY=<path/to/python>." >&2
  exit 1
fi

# Prefer an explicit PIXI_PY if the user exported one; otherwise use the
# default-env path inside this repo.
if [[ -n "${PIXI_PY:-}" ]]; then
  PY="$PIXI_PY"
else
  PY="$REPO_ROOT/.pixi/envs/humble/bin/python"
fi

if [[ ! -x "$PY" ]]; then
  echo "Error: pixi python not found at $PY" >&2
  echo "       Run 'pixi install' from $REPO_ROOT first, or set PIXI_PY to a usable python." >&2
  exit 1
fi

exec "$PY" "$SCRIPT_DIR/read_rolling_results.py" "$@"
