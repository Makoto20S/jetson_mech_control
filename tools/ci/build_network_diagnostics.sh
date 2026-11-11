#!/usr/bin/env bash
# Diagnostic only. No host networking changes; missing DNS remains visible.
set -uo pipefail
printf '=== CI DNS diagnostics: %s ===\n' "${1:-build-container}"
cat /etc/resolv.conf || true
for host in archive.ubuntu.com security.ubuntu.com ports.ubuntu.com packages.ros.org; do
  printf '\nResolving %s\n' "$host"
  if ! timeout 5 getent ahostsv4 "$host"; then
    printf 'DNS lookup failed or timed out: %s\n' "$host" >&2
  fi
done
# Diagnostics must not replace the apt/build command's actual exit status.
exit 0
