#!/usr/bin/env bash
set -euo pipefail
OUTPUT_ROOT="${MECH_OUTPUT_ROOT:-/workspace}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set +u
source "$OUTPUT_ROOT/install/setup.bash"
set -u
python3 "$REPO_ROOT/tools/ci/run_stability.py" \
  --build "$OUTPUT_ROOT/build/mech_bringup" \
  --output "$OUTPUT_ROOT/ci-stability" \
  --repetitions "${MECH_STABILITY_REPETITIONS:-20}"
