#!/usr/bin/env bash
# Integration tests for activate_position_controller.sh using real ROS services.
set -uo pipefail

if [[ "${MECH_ACTIVATION_TEST_OUTER_TIMEOUT:-0}" != "1" ]]; then
  export MECH_ACTIVATION_TEST_OUTER_TIMEOUT=1
  exec timeout --signal=TERM --kill-after=2s 30s "$0" "$@"
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
UNDER_TEST="${SCRIPT_DIR}/activate_position_controller.sh"
SERVER="${SCRIPT_DIR}/test_activation_service_server.py"
CONTROLLER=motor1_position_controller
failures=0
checks=0
case_dir=""
server_pid=""

cleanup_case() {
  if [[ -n "${server_pid}" ]]; then
    kill "${server_pid}" 2>/dev/null || true
    local count=0
    while kill -0 "${server_pid}" 2>/dev/null && [[ "${count}" -lt 50 ]]; do
      sleep 0.02
      count=$(( count + 1 ))
    done
    if kill -0 "${server_pid}" 2>/dev/null; then
      kill -KILL "${server_pid}" 2>/dev/null || true
    fi
    wait "${server_pid}" 2>/dev/null || true
  fi
  [[ -z "${case_dir}" ]] || rm -rf "${case_dir}"
  server_pid=""
  case_dir=""
}
trap cleanup_case EXIT

check() {
  local description="$1" expected="$2" actual="$3"
  checks=$(( checks + 1 ))
  if [[ "${expected}" == "${actual}" ]]; then
    echo "ok   - ${description}"
  else
    echo "FAIL - ${description}: expected '${expected}', got '${actual}'"
    [[ ! -f "${case_dir}/out" ]] || sed 's/^/       | /' "${case_dir}/out"
    failures=$(( failures + 1 ))
  fi
}

output_contains() { grep -qF -- "$1" "${case_dir}/out" && echo yes || echo no; }

run_under_test() {
  local status=0
  timeout --signal=TERM --kill-after=1s 8s \
    "${UNDER_TEST}" "$@" >"${case_dir}/out" 2>&1 || status=$?
  echo "${status}"
}

start_server() {
  local scenario="$1"
  cleanup_case
  case_dir="$(mktemp -d /tmp/mech-activation-test.XXXXXX)"
  export ROS_DOMAIN_ID=$(( 80 + (RANDOM % 20) ))
  export ROS_LOG_DIR="${case_dir}/ros-log"
  mkdir -p "${ROS_LOG_DIR}"
  /usr/bin/python3 "${SERVER}" --manager /test_controller_manager \
    --controller "${CONTROLLER}" --scenario "${scenario}" \
    --record "${case_dir}/requests.jsonl" --ready "${case_dir}/ready" \
    >"${case_dir}/server.log" 2>&1 &
  server_pid=$!
  local count=0
  while [[ ! -f "${case_dir}/ready" && "${count}" -lt 100 ]]; do
    sleep 0.02
    count=$(( count + 1 ))
  done
  if [[ ! -f "${case_dir}/ready" ]]; then
    echo "test service did not start" >&2
    sed -n '1,120p' "${case_dir}/server.log" >&2
    exit 2
  fi
}

json_field() {
  /usr/bin/python3 - "$case_dir/requests.jsonl" "$1" <<'PY'
import json
import sys
rows = [json.loads(line) for line in open(sys.argv[1], encoding="utf-8")]
switches = [row for row in rows if row["service"] == "switch"]
key = sys.argv[2]
print(len(switches) if key == "switch_count" else (switches[-1][key] if switches else ""))
PY
}

case_dir="$(mktemp -d /tmp/mech-activation-test.XXXXXX)"
check "refuses to run without a controller name" 2 "$(run_under_test)"
cleanup_case

case_dir="$(mktemp -d /tmp/mech-activation-test.XXXXXX)"
export ROS_DOMAIN_ID=$(( 80 + (RANDOM % 20) ))
export ROS_LOG_DIR="${case_dir}/ros-log"
mkdir -p "${ROS_LOG_DIR}"
started_ns="$(date +%s%N)"
check "bounds controller-manager discovery" 3 \
  "$(run_under_test "${CONTROLLER}" 1 /missing_controller_manager)"
ended_ns="$(date +%s%N)"
elapsed_ms=$(( (ended_ns - started_ns) / 1000000 ))
check "applies the total deadline to discovery" yes \
  "$([[ "${elapsed_ms}" -ge 700 && "${elapsed_ms}" -lt 1800 ]] && echo yes || echo no)"
cleanup_case

start_server not_loaded
check "refuses when the controller is not loaded" 3 "$(run_under_test "${CONTROLLER}" 3 /test_controller_manager)"
check "does not switch an unknown controller" 0 "$(json_field switch_count)"
check "tells the operator to load it inactive" yes "$(output_contains --inactive)"

start_server active
check "refuses when the controller is already active" 3 "$(run_under_test "${CONTROLLER}" 3 /test_controller_manager)"
check "does not re-activate an active controller" 0 "$(json_field switch_count)"

start_server refuse_twice
check "retries STRICT switching until the claim opens" 0 "$(run_under_test "${CONTROLLER}" 3 /test_controller_manager)"
check "makes three switch requests" 3 "$(json_field switch_count)"
check "uses STRICT switch semantics" 2 "$(json_field strictness)"
check "activates only the requested controller" "['motor1_position_controller']" "$(json_field activate_controllers)"

start_server lie
check "rejects success without active readback" 5 "$(run_under_test "${CONTROLLER}" 1 /test_controller_manager)"

start_server refuse_forever
check "reports a bounded definitive refusal" 4 "$(run_under_test "${CONTROLLER}" 1 /test_controller_manager)"
check "reports confirmed inactive state after refusal" yes "$(output_contains "confirmed 'inactive'")"

start_server hang
started_ns="$(date +%s%N)"
check "bounds an unanswered switch request" 4 "$(run_under_test "${CONTROLLER}" 1 /test_controller_manager)"
ended_ns="$(date +%s%N)"
elapsed_ms=$(( (ended_ns - started_ns) / 1000000 ))
check "uses one bounded total deadline" yes "$([[ "${elapsed_ms}" -ge 700 && "${elapsed_ms}" -lt 1800 ]] && echo yes || echo no)"
check "describes an unanswered request as uncertain" yes "$(output_contains "outcome is uncertain")"

cleanup_case
echo
if [[ "${failures}" -ne 0 ]]; then
  echo "${failures} failed / ${checks} checks"
  exit 1
fi
echo "all ${checks} checks passed"
