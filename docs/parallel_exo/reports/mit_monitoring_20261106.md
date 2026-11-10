# MIT monitoring software verification
Date: 2026-11-06 Asia/Shanghai

Changed tools/project/mit_communication.py, its test file and usage document.
Reused servo supervision semantics: 200 ms asynchronous hardware checks, 1 s request deadline, selected effort interface availability/claim checks, and permanent first-fault latch. The event stream records the first monitor failure. Enabled existing native feedback telemetry including host receipt age/sequence and raw speed/failure reason. Existing native feedback TTL and overspeed protection remain unchanged.

Windows: 11 portable unit tests passed; --check reported zero physical ports opened and zero motor command frames; git diff --check passed (existing README line-ending warning).
Orin: same 11 portable tests and --check passed. Native CTest targets mech_bringup_test, mech_bringup_effort_launch and mech_bringup_servo_motion all passed (3/3). C++ sources were unchanged in this update; existing native binaries were used.
Sync: 716 managed files, 3 updated, no deletions; stamp 29346c2c985e8b542dd48baebc50654ab9ed5f1df343041cdba491465a9e24e9.

No physical device was opened or commanded. No hardware timing/physical-motion acceptance claimed. The new supervisor has unit coverage on both hosts; a full ROS fault-injection run of the MIT runner remains unperformed because its execute path opens real ports. Existing native runtime/plugin tests cover simulated feedback/fault handling. Runtime telemetry is diagnostic evidence, not a new ROS sensor-state validity topic. Reports remain local and are excluded from SSH sync.
