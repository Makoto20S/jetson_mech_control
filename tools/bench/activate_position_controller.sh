#!/usr/bin/env bash
# Arms a loaded-but-inactive controller once the hardware will accept the
# claim. This is an operator action on a real motor: activating a position
# controller is the moment the motor starts accepting position commands.
#
# WARNING: run this only with the owner present and per-run authorization, on
# a bench whose ADR-006 / G0-G3 gates have been reviewed for that run. It sends
# no command itself, but the controller it activates will.
#
# Why this exists. ADR-016 refuses the command claim until valid feedback has
# flowed, and controller_manager's spawner issues exactly one switch_controller
# with STRICT strictness and exits 1 when it is refused - its max_attempts
# retry only covers a response that never arrives, and a refusal is a perfectly
# valid response. Spawning the controller at launch therefore races the first
# feedback frame, which at motor1's configured 50 Hz is up to 20 ms after the
# hardware activates. The fix is to load it --inactive and run this afterwards.
#
# This script deliberately does NOT judge whether feedback is good. That
# judgement already exists in the hardware and is better than anything an
# operator or a script could do from outside: has_valid_sample() is recomputed
# every read() cycle from the current sample's quality and its 60 ms validity
# window, so a successful claim means feedback is usable right now. Watching
# /joint_states proves nothing by comparison - that topic is published by the
# 500 Hz control loop, not by the 50 Hz motor, so it keeps ticking after the
# device goes silent. All this script adds is not giving up on the first try.
#
# It uses typed ListControllers and SwitchController services. This avoids
# depending on ros2 control's human-readable, release-dependent table format.
# SwitchController is sent with STRICT strictness explicitly.
#
# Success is confirmed by reading the controller state back. A tool's own
# report of success is not evidence.
#
# Exit codes:
#   0  the controller is active, confirmed by read-back
#   2  usage error
#   3  precondition failed (controller not loaded, or not inactive)
#   4  activation was not confirmed before the total timeout
#   5  activation reported success that the read-back did not confirm
set -euo pipefail

usage() {
  cat >&2 <<USAGE
usage: $(basename "$0") <controller_name> [timeout_s] [controller_manager]

  controller_name     required; there is no default target for arming a motor
  timeout_s           whole seconds to keep retrying the claim (default 5)
  controller_manager  node name (default /controller_manager)
USAGE
}

controller="${1:-}"
timeout_s="${2:-5}"
manager="${3:-/controller_manager}"

if [[ -z "${controller}" ]]; then
  usage
  exit 2
fi
if [[ ! "${timeout_s}" =~ ^[0-9]+$ ]] || [[ "${timeout_s}" -lt 1 ]]; then
  echo "ERROR: timeout_s must be a whole number of seconds >= 1" >&2
  usage
  exit 2
fi

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "${script_dir}/activate_position_controller.py" \
  "${controller}" "${timeout_s}" "${manager}"
