# Foundation RC validation

This document is the executable evidence checklist for FND-014/FND-015. All
commands are software-only unless an operator separately authorizes a vcan
interface. They never enable a physical CAN interface or open `/dev/ttyACM*`.

## Required checks

1. Portable repository checks:

   ```bash
   python3 tools/ci/context_check.py
   python3 tools/ci/check_adrs.py
   git diff --check
   ```

2. Clean Humble build and tests:

   ```bash
   MECH_OUTPUT_ROOT=/tmp/mech-foundation-rc tools/ci/build_workspace.sh
   ```

3. Address/undefined behavior sanitizers:

   ```bash
   tools/ci/run_sanitizers.sh /tmp/mech-foundation-sanitizers
   ```

   LeakSanitizer is optional because it cannot run under ptrace-managed
   executors. Enable it on a standalone runner with
   `MECH_ASAN_DETECT_LEAKS=1`; ASan and UBSan remain mandatory in all cases.

4. Deterministic lifecycle/performance test:

   ```bash
   ctest --test-dir /tmp/mech-foundation-rc/build/mech_bringup \
     --output-on-failure
   ```

   `RunsFiveHundredHertzAndExpiresCommandToNeutral` executes 1000 virtual
   2 ms cycles, checks command TTL returns to neutral, records cycle latency,
   and fails if a host cycle exceeds 2 ms. This is a host benchmark, not a
   real-time or device-performance claim.

5. Linux vcan round-trip (software-only, requires network administration):

   ```bash
   sudo modprobe vcan
   sudo ip link add dev vcan0 type vcan
   sudo ip link set up vcan0
   MECH_RUN_VCAN_TESTS=1 ctest \
     --test-dir /tmp/mech-foundation-rc/build/mech_control_core \
     --output-on-failure -R mech_control_core_test
   sudo ip link delete vcan0
   ```

   The test opens only `vcan0`, sends one Classic standard frame, and verifies
   filter matching, payload, host/source timestamps, and RX/TX counters.

6. ARM64 clean build and 30-minute simulated stability run must be executed on
   the target Jetson for the exact RC commit:

   ```bash
   MECH_STABILITY_SECONDS=1800 tools/ci/run_foundation_stability.sh
   ```

## RC acceptance

- FND-010 through FND-015, RSP-002, and INT-001 are merged through protected
  CI from one task PR.
- Five packages build and all tests pass on Jammy/Humble amd64 and ARM64.
- Sanitizers report no finding.
- The 30-minute simulated stability command completes with zero failed cycles.
- SocketCAN/vcan and USB-CDC results are not presented as physical-device
  compatibility evidence.
- The release candidate tag is `v0.1.0-foundation-rc1`; create it only on the
  exact commit that has all required evidence.

## Known limitations

- HighTorque nominal/data bitrate ownership, device timestamps, full error API,
  firmware matrix, and reusable-source licensing remain unresolved.
- No CubeMars or HI12 device adapter is part of Foundation RC.
- Real CAN and device activation remain gated by G0-G3 and ADR-006.
