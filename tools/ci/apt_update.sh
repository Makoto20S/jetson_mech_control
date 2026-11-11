#!/usr/bin/env bash
# A partial index refresh must never advance to dependency installation.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for attempt in 1 2 3; do
  printf 'APT index refresh attempt %s/3\n' "$attempt"
  if apt-get -o APT::Update::Error-Mode=any update; then
    exit 0
  fi
  printf 'APT index refresh failed; installation remains blocked.\n' >&2
  bash "$script_dir/build_network_diagnostics.sh" build-container
  if [[ "$attempt" -lt 3 ]]; then
    sleep "$((attempt * 5))"
  fi
done
printf 'ERROR: complete APT indexes unavailable after 3 attempts.\n' >&2
exit 100
