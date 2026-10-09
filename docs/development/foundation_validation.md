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

   CI executes this build/test path on both native x86_64 and ARM64 hosted
   runners, with the pinned Jammy/Humble Docker image. Runner/image architecture
   guards must pass, caches/artifacts are separated by architecture, and both
   platform results are retained even if one fails. This ARM64 runner is not
   the physical Jetson and supplies no device or real-time acceptance evidence.

3. Address/undefined behavior sanitizers:

   ```bash
   tools/ci/run_sanitizers.sh /tmp/mech-foundation-sanitizers
   ```

   LeakSanitizer is optional because it cannot run under ptrace-managed
   executors. Enable it on a standalone runner with
   `MECH_ASAN_DETECT_LEAKS=1`; ASan and UBSan remain mandatory in all cases.

   The CI sanitizer job is independent of the normal build/test job and uses
   the same pinned Humble dependencies. Before tests, the script verifies all
   package CMake/compilation flags and writes `sanitizer-build.json`. CI retains
   this proof and failure logs in its separate sanitizer artifact. The existing
   upstream Humble allocator mismatch exception is scoped to manager integration
   processes; other ASan checks and UBSan remain enabled. See the
   [workspace instructions](../../ros2_ws/README.md#address-and-undefined-behavior-sanitizers).

   Inspect GTest XML `result="skipped"` as well as the colcon summary; the latter
   can report zero skips despite conditional vcan/position-only cases.

   Synthetic servo JTC functional tests share a logical hardware clock that
   advances 2 ms per read/update/write cycle. ROS trajectory/action clocks and
   host pacing still use real time. A host pause beyond 6 ms must not become a
   synthetic hardware fault; dedicated runtime/plugin tests independently
   enforce the unchanged 3 ms soft and 6 ms hard deadlines in logical time.
   These functional tests do not establish a real-time scheduling guarantee.

   The FakeTransport force-control JTC fixture likewise advances its runtime
   clock and 50 Hz feedback arrival stamps in logical time. Its JTC update
   timestamp also advances by the declared 2 ms period, so host pauses cannot
   change the per-cycle trajectory step under test. ROS node clocks, callbacks,
   discovery and host pacing retain real time; `use_sim_time` is not enabled.
   Both position and position+velocity deployments must survive a 20 ms host
   pause during hold and trajectory interpolation. A separate regression
   advances hardware time past the unchanged 4/6 ms command lease with fresh
   feedback and requires command failure with no further transmission.

   E5 force-control process tests use the same 2 ms logical hardware cadence,
   while the controller's 100/106 ms target policy and manager update periods
   keep real time. They retain the first transmitted frame after each trace
   reset, checking its slew against the actual update period and CAN payload
   quantization; a later telemetry snapshot is not the first command. An
   additional scenario injects a 20 ms host pause and requires the functional
   chain to remain usable, including target expiry and inactive restart checks.
   The 4/6 ms hardware lease and feedback-loss protections remain enforced by
   separate runtime tests; production code and parameters are unchanged.

4. Deterministic lifecycle/performance test:

   ```bash
   ctest --test-dir /tmp/mech-foundation-rc/build/mech_bringup \
     --output-on-failure
   ```

   `RunsFiveHundredHertzWhileTargetKeepsBeingRefreshed` executes 1000 virtual
   2 ms cycles and records host cycle latency without a scheduling threshold.
   Separate expiry tests check hold followed by fault, never movement to zero.
   The opt-in `mech_bringup_performance_test` enforces the 2 ms host bound with
   `-DMECH_ENABLE_HOST_PERFORMANCE_TESTS=ON`. This is a host benchmark, not a
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

## CI clock and acceptance contract

The current six-package CI separates functional correctness, deadline semantics
and host performance. A green hosted run supplies no real-time guarantee.

| Test family | Hardware time | Host time and acceptance |
|---|---|---|
| Force/servo runtime and plugin unit tests | Explicit injected timestamps | Exact soft/hard expiry, missing feedback and no post-expiry transmission remain mandatory |
| Force/servo JTC and E5 integration | 2 ms logical hardware cycles | Bounded ROS discovery/action waits; injected pauses must not replace hardware time |
| Single/two-bus PTY operator workflows | Test-only composition of the production servo plugin, 2 ms per read | Real manager/JTC, PTY codecs, lifecycle, operator timers and 31-second hold; 20/50/100 ms pauses with a pending command |
| Foundation functional endurance | 1000 logical 2 ms cycles | Position, refresh, lifecycle and protection assertions; host maximum recorded in XML |
| Foundation host performance | Logical harness stimulus, actual elapsed cycle measurements | Explicit opt-in, unsanitized qualified host; maximum must remain below 2 ms |

`ServoPtyTestSystem` exists only under `BUILD_TESTING`, in a separate plugin
manifest. It composes the unchanged final `Ak30ServoSystem`, forwards its
lifecycle and interfaces, and accepts only `/dev/pts/<number>`. Production
operator launch/configuration never selects it. The helper
`tools/servo/pty_clock_fixture.py` selects it only for synthetic process tests.
The production servo 3/6 ms and force 4/6 ms deadlines remain unchanged.
Before each logical read, a bounded Unix-socket handshake asks the gateway
thread to queue a feedback frame. During intentional feedback loss it acknowledges
the cycle without sending a frame; the runtime still reaches expiry. This keeps
manager catch-up after a host pause from outrunning the synthetic peer. Within
one cycle, clock reads advance by 1 ns so multiple queued feedback arrivals
retain the strict ordering required by the production session.

PTY raw command/serial traces retain host time; runtime deadlines use the test
hardware clock. A separate per-process CSV records the mapping and injected
pauses. The test auditor converts a copy to hardware time for deadline checks,
then restores host claim timestamps for operator/service correlation. It requires
the mapping and pause evidence, preserves the raw files, and rejects expired
logical commands. These synthetic captures must be audited with that helper,
not presented as production host timing evidence. Missing feedback still advances
toward logical expiry and the two-bus loss test must stop and disable both buses.

ROS discovery, process readiness and trajectory completion use bounded host
waits. Do not replace them with arbitrary sleeps, weaken production leases,
remove lifecycle assertions, disable sanitizer checks or retry failures to green.

### Stability validation and evidence

For CI timing changes, validate the final source with 20 independent rounds of
the single/two-bus PTY, servo/force JTC and E5 host-pause suites. The three PTY
pause durations must be observed. Run the full portable/native x86/native ARM64/
ASan matrix three times, including one without restored build layers. Preserve
every attempt and failure; these are empirical acceptance criteria, not a proof
that future runs cannot fail.

The Foundation workflow has manual inputs `cold_cache` and
`stability_repetitions` (0 or 20). The normal PR/main path runs the full suite
once; manual stability runs add repetitions after it. Both paths use the same
test targets and still require all ordinary tests. From a built Linux workspace:

```bash
MECH_OUTPUT_ROOT=/tmp/mech-foundation-rc MECH_STABILITY_REPETITIONS=20 \
  bash tools/ci/run_stability.sh
```

Each round has a console log, XML copy and exit code under `ci-stability`.
Any failed round fails the command even if later rounds pass. CI retains these
alongside colcon/ROS logs, sanitizer instrumentation evidence and
`SERVO_TEST_ARTIFACT_DIR=/workspace/ci-test-artifacts`. PTY diagnostics include
terminal output, gateway TX, command/serial captures, config, first-fault logs
and clock mappings. The test copier allows only diagnostic extensions, does not
follow symlinks, compresses unsealed snapshots, and limits payload to 256 MiB per
test capture. `artifact-manifest.json` explicitly reports partial retention;
partial/missing evidence must never be described as complete.

Branch protection is a separate repository setting: verify that all four matrix
checks are required before relying on protection as the merge gate. A successful
PR run does not replace checking the post-merge main run.
