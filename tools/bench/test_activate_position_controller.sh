#!/usr/bin/env bash
# Offline tests for activate_position_controller.sh.
#
# The script under test is the one operator-facing action that arms a motor,
# so the thing it must never do is report a success it did not achieve. These
# tests drive it against a stub `ros2` on PATH: no ROS, no controller_manager
# and no device is involved, which is why they run in the portable CI job.
#
# The stub is configured through the environment, reset before every case:
#   STUB_LISTED         1 = list_controllers reports the controller at all
#   STUB_STATE          state list_controllers reports (inactive/active/...)
#   STUB_REFUSALS       how many set_controller_state calls fail before one
#                       succeeds; negative refuses forever
#   STUB_ACTIVATE_LIES  1 = set_controller_state exits 0 without activating
#
# No case runs in a subshell: a count incremented in one would not survive.
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
UNDER_TEST="${SCRIPT_DIR}/activate_position_controller.sh"
CONTROLLER=motor1_position_controller
BASE_PATH="${PATH}"

failures=0
checks=0
stub_dir=""

setup_stub() {
  stub_dir="$(mktemp -d)"
  mkdir -p "${stub_dir}/bin"
  cat >"${stub_dir}/bin/ros2" <<'STUB'
#!/usr/bin/env bash
set -uo pipefail
verb="${2:-}"
case "${verb}" in
  list_controllers)
    echo "joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active"
    if [[ "${STUB_LISTED}" == "1" ]]; then
      echo "motor1_position_controller[mech_controllers/DemoController] $(cat "${STUB_DIR}/state")"
    fi
    ;;
  set_controller_state)
    attempts=$(( $(cat "${STUB_DIR}/attempts") + 1 ))
    echo "${attempts}" >"${STUB_DIR}/attempts"
    if [[ "${STUB_REFUSALS}" -lt 0 || "${attempts}" -le "${STUB_REFUSALS}" ]]; then
      echo "Error activating controller, check controller_manager logs" >&2
      exit 1
    fi
    if [[ "${STUB_ACTIVATE_LIES}" != "1" ]]; then
      echo active >"${STUB_DIR}/state"
    fi
    echo "Successfully activated motor1_position_controller"
    ;;
  *)
    echo "stub ros2: unexpected call: $*" >&2
    exit 97
    ;;
esac
STUB
  chmod +x "${stub_dir}/bin/ros2"
  echo 0 >"${stub_dir}/attempts"
  echo "${STUB_STATE:-inactive}" >"${stub_dir}/state"
  export STUB_DIR="${stub_dir}"
  export STUB_LISTED="${STUB_LISTED:-1}"
  export STUB_REFUSALS="${STUB_REFUSALS:-0}"
  export STUB_ACTIVATE_LIES="${STUB_ACTIVATE_LIES:-0}"
  export PATH="${stub_dir}/bin:${BASE_PATH}"
}

teardown_stub() {
  rm -rf "${stub_dir}"
  export PATH="${BASE_PATH}"
  unset STUB_DIR STUB_LISTED STUB_STATE STUB_REFUSALS STUB_ACTIVATE_LIES
}

check() {
  local description="$1" expected="$2" actual="$3"
  checks=$(( checks + 1 ))
  if [[ "${expected}" == "${actual}" ]]; then
    echo "ok   - ${description}"
  else
    echo "FAIL - ${description}: expected '${expected}', got '${actual}'"
    echo "       script output:"
    sed 's/^/       | /' "${stub_dir}/out" 2>/dev/null || true
    failures=$(( failures + 1 ))
  fi
}

# Reports the script's exit status without letting a non-zero status abort this
# harness. The status is asserted exactly, not merely as "non-zero": the script
# documents its exit codes as a contract, and checking only the sign lets a
# case pass through the wrong refusal path. Several of these cases refuse for
# more than one reason, so the sign alone proves very little.
run_under_test() {
  local status=0
  "${UNDER_TEST}" "$@" >"${stub_dir}/out" 2>&1 || status=$?
  echo "${status}"
}

attempts_made() { cat "${stub_dir}/attempts"; }
reported_state() { cat "${stub_dir}/state"; }

# Reports whether the script's output contained a string. Used sparingly: for
# an operator tool the refusal message is the product, and "not loaded" and
# "not inactive" both exit 3, so without this the two are indistinguishable
# and one could silently stop telling the operator what to do about it.
output_contains() {
  grep -qF -- "$1" "${stub_dir}/out" && echo yes || echo no
}

# --- No controller name -------------------------------------------------
# Arming has no sensible default target, so the name must be given.
setup_stub
check "refuses to run without a controller name" 2 "$(run_under_test)"
check "asks the controller manager for nothing" 0 "$(attempts_made)"
teardown_stub

# --- Controller not loaded ----------------------------------------------
# Without the --inactive spawner the controller is not there at all. Guessing
# on the operator's behalf is the behaviour this script exists to remove.
STUB_LISTED=0 setup_stub
check "refuses when the controller is not loaded" 3 \
      "$(run_under_test "${CONTROLLER}" 1)"
check "does not try to activate a controller it cannot see" 0 "$(attempts_made)"
check "tells the operator to load it --inactive first" yes \
      "$(output_contains "--inactive")"
teardown_stub

# --- Controller already active ------------------------------------------
# The world is not what the operator thinks it is. Say so rather than report a
# success this run did not cause.
STUB_STATE=active setup_stub
check "refuses when the controller is already active" 3 \
      "$(run_under_test "${CONTROLLER}" 1)"
check "does not re-activate an active controller" 0 "$(attempts_made)"
teardown_stub

# --- The gate never opens -----------------------------------------------
# ADR-016 refuses the claim while feedback is not usable.
#
# The companion assertion is on the attempt count, not on the controller state
# afterwards: with the gate refusing forever the stub cannot activate anything,
# so "it is still inactive" would hold no matter what the script did. Retrying
# more than once is the property that actually distinguishes this script from
# the spawner it exists to replace.
STUB_REFUSALS=-1 setup_stub
check "fails closed when the gate never opens" 4 \
      "$(run_under_test "${CONTROLLER}" 1)"
check "keeps retrying instead of giving up on the first refusal" yes \
      "$([[ "$(attempts_made)" -gt 1 ]] && echo yes || echo no)"
teardown_stub

# --- The gate opens on a later attempt ----------------------------------
# This is the whole reason the script exists: the spawner issues exactly one
# STRICT switch_controller and gives up when it is refused.
STUB_REFUSALS=2 setup_stub
check "succeeds once the gate opens on a later attempt" 0 \
      "$(run_under_test "${CONTROLLER}" 5)"
check "activates the controller" active "$(reported_state)"
teardown_stub

# --- The command reports success it did not achieve ---------------------
# `ros2 control switch_controllers --activate` defaults to BEST_EFFORT, and
# the service definition says the meaning of ok depends on strictness. This
# script uses the strict verb instead, but still reads the state back: a
# tool's own report of success is not evidence.
STUB_ACTIVATE_LIES=1 setup_stub
check "rejects a reported success the read-back does not confirm" 5 \
      "$(run_under_test "${CONTROLLER}" 1)"
teardown_stub

echo
if [[ "${failures}" -ne 0 ]]; then
  echo "${failures} failed / ${checks} checks"
  exit 1
fi
echo "all ${checks} checks passed"
