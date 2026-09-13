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
# It uses `ros2 control set_controller_state`, not `switch_controllers
# --activate`. The latter's --strict is a flag that defaults to false
# (controller_manager_services.py maps a falsy strict to BEST_EFFORT), and
# SwitchController.srv states that the meaning of "ok" depends on strictness -
# so a BEST_EFFORT activation that achieved nothing may still report success.
# NOTE: that last consequence is read off the service definition and has NOT
# been measured here. set_controller_state passes strict=True unconditionally,
# which sidesteps the question.
#
# Success is confirmed by reading the controller state back. A tool's own
# report of success is not evidence.
#
# Exit codes:
#   0  the controller is active, confirmed by read-back
#   2  usage error
#   3  precondition failed (controller not loaded, or not inactive)
#   4  the claim was still refused when the timeout expired; the controller is
#      left exactly as it was found
#   5  activation reported success that the read-back did not confirm
set -euo pipefail

readonly RETRY_INTERVAL_S=0.2

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

# Prints the controller's state, or nothing when it is not loaded.
# `ros2 control list_controllers` prints one controller per line as
# "name[type] state"; matching on "name[" keeps a longer name that merely
# starts with the same text from matching.
controller_state() {
  local listing
  if ! listing="$(ros2 control list_controllers -c "${manager}" 2>/dev/null)"; then
    return 1
  fi
  awk -v want="${controller}[" 'index($1, want) == 1 { print $NF }' <<<"${listing}"
}

initial_state="$(controller_state)" || {
  echo "ERROR: could not read controllers from ${manager}" >&2
  exit 3
}

if [[ -z "${initial_state}" ]]; then
  echo "ERROR: ${controller} is not loaded on ${manager}." >&2
  echo "       Load it first with the spawner's --inactive argument." >&2
  exit 3
fi
if [[ "${initial_state}" != "inactive" ]]; then
  echo "ERROR: ${controller} is '${initial_state}', not 'inactive'." >&2
  echo "       Refusing rather than reporting a success this run did not cause." >&2
  exit 3
fi

echo "Activating ${controller} (up to ${timeout_s}s; refusals are expected"
echo "until feedback is flowing)..."

attempts=0
activated=0
start="${SECONDS}"
while true; do
  attempts=$(( attempts + 1 ))
  if ros2 control set_controller_state "${controller}" active -c "${manager}" \
       >/dev/null 2>&1; then
    activated=1
    break
  fi
  if (( SECONDS - start >= timeout_s )); then
    break
  fi
  sleep "${RETRY_INTERVAL_S}"
done

final_state="$(controller_state || true)"

if [[ "${activated}" -ne 1 ]]; then
  echo "ERROR: ${controller} was still refused after ${timeout_s}s" \
       "(${attempts} attempts); it is '${final_state:-not loaded}'." >&2
  echo "       The claim gate is fail-closed, so this most likely means usable" >&2
  echo "       feedback never arrived. Check the controller_manager log and" >&2
  echo "       whether the device is reporting at all. Do not retry with" >&2
  echo "       'switch_controllers --activate': it defaults to BEST_EFFORT." >&2
  exit 4
fi

if [[ "${final_state}" != "active" ]]; then
  echo "ERROR: activation reported success but ${controller} reads back as" \
       "'${final_state:-not loaded}'." >&2
  exit 5
fi

echo "${controller} is active (confirmed by read-back, ${attempts} attempts)."
