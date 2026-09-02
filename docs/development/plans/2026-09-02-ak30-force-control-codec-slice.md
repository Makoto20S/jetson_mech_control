# AK3.0 Force-Control Codec Slice Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a new `mech_protocol_cubemars` package implementing the AK3.0 force-control wire codec, evidence-gated mapping, `DeviceCodec` and `DeviceSession`, validated entirely offline.

**Architecture:** Four layers under `RawCanFrame` — `ak30_force_wire` (bytes ↔ device-native units), `ak30_mapping` (device-native ↔ canonical SI, holds tri-state evidence), `ak30_force_codec` (implements `DeviceCodec` by composing the two), `ak30_force_session` (implements `DeviceSession`). Golden vectors are asserted against the wire layer so every test number traces to the manual with no interpretation in between.

**Tech Stack:** C++17, ament_cmake, gtest, ROS 2 Humble. No ROS headers in this package — it sits below the ros2_control adapter.

**Spec:** `docs/development/ak30_force_control_adapter_design.md` (merged under ADR-013). Read it before Task 1.

**Branch / PR:** one branch `feat/ak30-force-control-codec`, one PR. `main` is protected: PR required, two required checks, merge-commit only.

## Global Constraints

- **No new ADR is required.** This package is additive; it changes no core type. Per `adapter_contract_v1.md` item 7, any change to `CanonicalDeviceCommand`, `CanonicalDeviceState`, `DeviceCodec`, `DeviceSession` or `ProtocolProfile` would need an ADR *before* implementation — so do not touch them. If a task seems to require it, stop and report instead.
- **No CAN, serial, `/dev/ttyACM*`, vcan, or real device access.** Offline tests only. `mech_simulation`'s fake transport is the only transport used in tests.
- **Quantization uses `((1 << bits) - 1)`, never `(1 << bits)`.** See "Verified manual findings" below. This is the single most important number in the package.
- **Encoding rejects out-of-range input; it never clamps.** The vendor's reference code clamps; silently clamping a command is the degradation ADR-012 forbids.
- C++17, `-Wall -Wextra -Wpedantic -Werror`, `[[nodiscard]]` on all query functions, `noexcept` on the active path, no allocation on the active path, no ROS headers.
- Package name `mech_protocol_cubemars`; namespace `mech::mech_protocol_cubemars`.
- Internal dependencies are exactly `{mech_control_core, mech_simulation}` — `mech_simulation` is a `test_depend`, and `context_check.py` counts `test_depend`, so it must be declared in `EXPECTED_PACKAGES`.
- Commit after every task. Conventional Commits, matching existing history (`feat(protocol): …`, `test(protocol): …`, `chore(ci): …`).

## Verified manual findings (established 2026-09-02, before this plan)

These were re-derived from `company/ak-series-prodcut-manual-v3-2-0-for-ak-3-0-robotic-actuator-cn.pdf` this session and are the basis for the golden vectors. Do not re-derive them; do not "fix" code to match the manual's printed source.

1. **L07 §4.4's worked example code is explicitly `参数以 AK10-9 为例`** — AK10-9 constants are position `±12.56 rad`, velocity `±28.0 rad/s`, torque `±54.0 N·m`. Decoding the §4.4.1 example bytes with AKE60-8's `±40 / ±15` gives wrong answers. Any test that cites a manual example must use `ak10_9_ranges()`.

2. **The manual's printed `float_to_uint()` reproduces none of the manual's own examples.** It computes `(x - x_min) * ((1<<bits)/span)`. The example table was generated with `((1<<bits)-1)/span`. Three examples discriminate between them and all three favour `-1`:

   | example | manual bytes | `(1<<bits)-1` | printed code `(1<<bits)` |
   |---|---|---|---|
   | position `0 rad` | `7F FF` | `7F FF` ✅ | `80 00` ❌ |
   | velocity `0 rad/s` | `7F` + nibble `F` | `0x7FF` ✅ | `0x800` ❌ |
   | velocity `-6 rad/s` | `64 87` | `64 87` ✅ | `64 98` ❌ |
   | torque `2 N·m` | `…F8 4B` | `…F8 4B` ✅ | `…08 4B` ❌ |
   | torque `4 N·m` | `…F8 97` | `…F8 97` ✅ | `…08 97` ❌ |

3. **The printed code has a safety-relevant overflow.** With `(1<<16)`, `p_des = P_MAX = +12.56 rad` yields `p_int = 65536`, which does not fit 16 bits: `buffer[3] = 0x00, buffer[4] = 0x00`, decoding as **−12.56 rad**. Commanding maximum position produces minimum position. Task 2 pins this as a regression test so nobody "restores" the vendor formula.

4. **One single-hex-digit transcription error in the example table, as previously recorded.** §4.4.1's `速度设置为 6rad/s` row reads `00 06 66 7F FF 98 67 FF`; correct is `9B`. `0x98` decodes to `5.3402 rad/s` under AK10-9. The negative row (`-6 rad/s`) is correct, which is what proves the packing right and this one cell wrong.

5. **Feedback `0x29` scaling confirmed verbatim from §4.3.1:** position `int16 × 0.1°`, speed `int16 × 10` ERPM, `Iq` `int16 × 0.01 A`, temperature `Data[6]` `int8` °C, `Data[7]` status. Fault codes `0`–`7` are `无故障 / 电机过温度 / 过电流 / 过压 / 欠压 / 编码器 / mos 管过温度 / 电机堵转`. **`0x77` in `Data[7]` is the disable-succeeded acknowledgement** returned once after control mode `15` (§4.1.8), not a fault.

---

## File Structure

| File | Responsibility |
|---|---|
| `ros2_ws/src/mech_protocol_cubemars/package.xml` | Package manifest, format 3, deps `mech_control_core` + test deps |
| `ros2_ws/src/mech_protocol_cubemars/CMakeLists.txt` | Library + single gtest target |
| `ros2_ws/src/mech_protocol_cubemars/README.md` | Package scope, the two manual defects, what is deliberately not implemented |
| `include/mech_protocol_cubemars/package_marker.hpp` | Skeleton marker required by `context_check.py` |
| `include/mech_protocol_cubemars/ak30_force_wire.hpp` | Ranges, quantize/dequantize, command encode, `0x29` decode, status classification. No core types, no evidence, no state |
| `include/mech_protocol_cubemars/ak30_mapping.hpp` | `EvidencedValue`, `Ak30Mapping`, sub-mode → required-parameter gate, device-native ↔ canonical conversions |
| `include/mech_protocol_cubemars/ak30_force_codec.hpp` | `Ak30ForceControlCodec : DeviceCodec` |
| `include/mech_protocol_cubemars/ak30_force_session.hpp` | `Ak30SessionConfig`, `CommandStage`, `Ak30ForceControlSession : DeviceSession` |
| `src/*.cpp` | Implementations, one per header |
| `test/test_package_marker.cpp` | Marker test required by `context_check.py` |
| `test/test_ak30_force_wire.cpp` | Manual cross-check, AKE60-8 golden vectors, boundaries, the two defect regressions |
| `test/test_ak30_feedback_decode.cpp` | `0x29` decode, sign handling, status byte classification |
| `test/test_ak30_mapping.cpp` | Evidence gate per sub-mode, conversions |
| `test/test_ak30_force_codec.cpp` | Identifier composition, encode/decode via core types, negative frames |
| `test/test_ak30_force_session.cpp` | configure validation matrix, lifecycle, watchdog, fault latch, cross-profile rejection |
| `test/test_ak30_simulation_integration.cpp` | Round-trip against `mech_simulation` fake transport |
| `tools/ci/context_check.py:14-25,53-57` | Register the new package and its `package.xml` |

---

## Task 1: Package skeleton and CI registration

Nothing else can compile until the package exists and `context_check.py` accepts it. `context_check.py` fails hard when the discovered package set differs from `EXPECTED_PACKAGES`, so the registration and the package must land in the same commit.

**Files:**
- Create: `ros2_ws/src/mech_protocol_cubemars/package.xml`
- Create: `ros2_ws/src/mech_protocol_cubemars/CMakeLists.txt`
- Create: `ros2_ws/src/mech_protocol_cubemars/README.md`
- Create: `ros2_ws/src/mech_protocol_cubemars/include/mech_protocol_cubemars/package_marker.hpp`
- Create: `ros2_ws/src/mech_protocol_cubemars/src/package_marker.cpp`
- Test: `ros2_ws/src/mech_protocol_cubemars/test/test_package_marker.cpp`
- Modify: `tools/ci/context_check.py:14-25` (`EXPECTED_PACKAGES`), `tools/ci/context_check.py:53-57` (`REQUIRED_FILES`)

**Interfaces:**
- Consumes: nothing.
- Produces: the package target `mech_protocol_cubemars`, the test target `mech_protocol_cubemars_test`, and `mech::mech_protocol_cubemars::package_name()`.

- [ ] **Step 1: Register the package in CI first, and watch it fail**

In `tools/ci/context_check.py`, add to `EXPECTED_PACKAGES` after the `mech_bringup` entry:

```python
    "mech_protocol_cubemars": {"mech_control_core", "mech_simulation"},
```

and add to `REQUIRED_FILES`:

```python
    "ros2_ws/src/mech_protocol_cubemars/package.xml",
```

- [ ] **Step 2: Run the check to verify it fails**

Run: `python3 tools/ci/context_check.py`
Expected: FAIL with `missing required file: ros2_ws/src/mech_protocol_cubemars/package.xml`

- [ ] **Step 3: Write the marker test**

`test/test_package_marker.cpp`:

```cpp
#include "mech_protocol_cubemars/package_marker.hpp"

#include <gtest/gtest.h>

TEST(MechProtocolCubemars, PackageMarker) {
  EXPECT_EQ(mech::mech_protocol_cubemars::package_name(),
            "mech_protocol_cubemars");
}
```

- [ ] **Step 4: Create the package files**

`include/mech_protocol_cubemars/package_marker.hpp`:

```cpp
#pragma once

#include <string_view>

namespace mech::mech_protocol_cubemars {

[[nodiscard]] constexpr std::string_view package_name() noexcept {
  return "mech_protocol_cubemars";
}

}  // namespace mech::mech_protocol_cubemars
```

`src/package_marker.cpp`:

```cpp
#include "mech_protocol_cubemars/package_marker.hpp"

namespace mech::mech_protocol_cubemars {

// Translation unit exists so the package produces a library target.

}  // namespace mech::mech_protocol_cubemars
```

`package.xml` — note `mech_simulation` is a `test_depend`; `context_check.py` counts `test_depend` in the internal dependency set, which is why Step 1 declared both:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>mech_protocol_cubemars</name>
  <version>0.1.0</version>
  <description>AK3.0 force-control codec and device session for CubeMars actuators.</description>
  <maintainer email="project-owners@example.invalid">Project Owners</maintainer>
  <license>Internal Research Use - All Rights Reserved</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>mech_control_core</depend>
  <test_depend>mech_simulation</test_depend>
  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_cmake_lint_cmake</test_depend>
  <test_depend>ament_cmake_xmllint</test_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`CMakeLists.txt` — the test source list grows in later tasks; only `test_package_marker.cpp` exists now:

```cmake
cmake_minimum_required(VERSION 3.16)
project(mech_protocol_cubemars LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(mech_control_core REQUIRED)

add_library(${PROJECT_NAME}
  src/package_marker.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} mech_control_core)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mech_control_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(mech_simulation REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_package_marker.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  ament_target_dependencies(${PROJECT_NAME}_test mech_simulation)
  ament_lint_cmake()
  ament_xmllint()
endif()

ament_package()
```

`README.md`:

```markdown
# mech_protocol_cubemars

AK3.0 force-control codec and device session for CubeMars AKE60-8 actuators,
per `docs/development/ak30_force_control_adapter_design.md` and ADR-013.

## Scope

Force control is control mode ID `8`, a 29-bit extended Classic CAN frame.
Its three sub-modes (position, velocity, torque) share that one ID and are
distinguished by payload content, so the sub-mode is explicit configuration
and is never inferred from received data.

Not implemented here: the AK3.0 servo profile, any AK2.0/L02 profile, `0x2A`,
single-turn mode, ros2_control interface export, and any real device access.

## Two defects in the vendor manual that this package deliberately does not copy

1. L07's printed `float_to_uint()` scales by `(1 << bits) / span`. The manual's
   own worked-example table was generated with `((1 << bits) - 1) / span`, and
   the printed formula reproduces none of the manual's examples. This package
   uses `((1 << bits) - 1)`. With the printed formula, commanding `P_MAX`
   (+12.56 rad) produces `p_int = 65536`, which overflows 16 bits to `0x0000`
   and decodes as **-12.56 rad** — maximum position commands minimum position.
   `EncodingMaxPositionDoesNotWrapToMinimum` pins this.
2. L07 §4.4.1's `速度设置为 6rad/s` row reads `... 98 67 FF`; the correct byte
   is `0x9B`. The `-6 rad/s` row in the same table is correct, which is what
   proves the packing right and this one cell wrong.

The §4.4 example code is explicitly `参数以 AK10-9 为例` — velocity ±28 rad/s,
torque ±54 N·m. Tests citing manual examples use `ak10_9_ranges()`; AKE60-8 is
±40 / ±15.

## Evidence gate

`configure()` fails closed unless every mapping parameter the configured
sub-mode consumes is verified. With motor1's present evidence only
`pole_pairs` and `torque_constant` are verified, so **every sub-mode still
refuses to configure**; `direction_sign` (vendor question B9) is the single
parameter that unblocks the torque sub-mode.
```

- [ ] **Step 5: Verify the check now passes and the package builds**

Run: `python3 tools/ci/context_check.py`
Expected: `PASS: ... and 6 ROS package skeletons validated`

Run: `MECH_OUTPUT_ROOT=/tmp/t1 MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh`
Expected: 6 packages, all tests pass (118 existing + 1 new = 119)

- [ ] **Step 6: Commit**

```bash
git add ros2_ws/src/mech_protocol_cubemars tools/ci/context_check.py
git commit -m "feat(protocol): add the mech_protocol_cubemars package skeleton

Registers the package in context_check.py's EXPECTED_PACKAGES with its exact
internal dependency set. mech_simulation is a test_depend and the checker
counts test_depend, so it is declared as an internal dependency.

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 2: Wire layer — command encoding

The one task where a wrong constant produces plausible bytes that no offline test elsewhere would catch. Every assertion here is a literal from the manual or a value back-solved against the documented ranges.

**Files:**
- Create: `ros2_ws/src/mech_protocol_cubemars/include/mech_protocol_cubemars/ak30_force_wire.hpp`
- Create: `ros2_ws/src/mech_protocol_cubemars/src/ak30_force_wire.cpp`
- Test: `ros2_ws/src/mech_protocol_cubemars/test/test_ak30_force_wire.cpp`
- Modify: `ros2_ws/src/mech_protocol_cubemars/CMakeLists.txt` (add source + test to the existing lists)

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: `ForceControlRanges`, `ake60_8_ranges()`, `ak10_9_ranges()`, `ForceControlCommand`, `kForceControlPayloadBytes`, `kForceControlModeId`, `kFeedbackFunctionId`, `force_control_can_id(std::uint8_t)`, `feedback_can_id(std::uint8_t)`, `quantize(double,double,double,unsigned) -> std::uint32_t`, `dequantize(std::uint32_t,double,double,unsigned) -> double`, `encode_force_control(const ForceControlCommand&, const ForceControlRanges&, std::array<std::uint8_t,8>&) -> bool`.

- [ ] **Step 1: Write the failing tests**

`test/test_ak30_force_wire.cpp`:

```cpp
#include "mech_protocol_cubemars/ak30_force_wire.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>

#include <gtest/gtest.h>

namespace {

using mech::mech_protocol_cubemars::ak10_9_ranges;
using mech::mech_protocol_cubemars::ake60_8_ranges;
using mech::mech_protocol_cubemars::encode_force_control;
using mech::mech_protocol_cubemars::ForceControlCommand;
using mech::mech_protocol_cubemars::ForceControlRanges;
using mech::mech_protocol_cubemars::quantize;

using Payload = std::array<std::uint8_t, 8>;

// Rendering the payload as hex makes a failure legible as a wire frame rather
// than as eight unrelated integer mismatches.
[[nodiscard]] std::string hex(const Payload& payload) {
  std::string out;
  for (std::size_t index = 0; index < payload.size(); ++index) {
    char buffer[4];
    std::snprintf(buffer, sizeof(buffer), "%02X", payload[index]);
    if (index != 0) {
      out.push_back(' ');
    }
    out.append(buffer);
  }
  return out;
}

[[nodiscard]] std::string encoded(const ForceControlCommand& command,
                                  const ForceControlRanges& ranges) {
  Payload payload{};
  EXPECT_TRUE(encode_force_control(command, ranges, payload));
  return hex(payload);
}

// L07 section 4.4's example code states 参数以 AK10-9 为例. Decoding those
// example rows with AKE60-8's ranges gives wrong answers, so every
// manual-derived assertion below uses ak10_9_ranges().
TEST(Ak30ForceWire, ReproducesManualExampleRowsForAk10_9) {
  const auto ranges = ak10_9_ranges();

  // 扭矩 2N, section 4.4.1.
  EXPECT_EQ(encoded(ForceControlCommand{0.0, 0.0, 0.0, 0.0, 2.0}, ranges),
            "00 00 00 7F FF 7F F8 4B");
  // 扭矩 4N, section 4.4.1.
  EXPECT_EQ(encoded(ForceControlCommand{0.0, 0.0, 0.0, 0.0, 4.0}, ranges),
            "00 00 00 7F FF 7F F8 97");
  // 速度设置为 -6rad/s with Kd 2, section 4.4.1.
  EXPECT_EQ(encoded(ForceControlCommand{0.0, 2.0, 0.0, -6.0, 0.0}, ranges),
            "00 06 66 7F FF 64 87 FF");
}

// The 速度设置为 6rad/s row reads "00 06 66 7F FF 98 67 FF". DATA[5] should be
// 0x9B: 0x98 decodes to 5.3402 rad/s under AK10-9's +/-28 range. The -6 row
// above is correct, which is what proves the packing right and this cell wrong.
// Copying the manual verbatim would bake in a systematic ~11% velocity error
// that no offline test could otherwise catch.
TEST(Ak30ForceWire, CorrectsTheTranscriptionErrorInThePositiveVelocityRow) {
  const auto ranges = ak10_9_ranges();
  EXPECT_EQ(encoded(ForceControlCommand{0.0, 2.0, 0.0, 6.0, 0.0}, ranges),
            "00 06 66 7F FF 9B 67 FF");
}

// AKE60-8 (L07 p.37): position +/-12.56 rad, velocity +/-40.0 rad/s,
// torque +/-15.0 N.m, Kp 0-500, Kd 0-5. Back-solved, not copied.
TEST(Ak30ForceWire, EncodesAke60_8GoldenVectors) {
  const auto ranges = ake60_8_ranges();

  // Torque sub-mode: gains zero, position and velocity centred.
  EXPECT_EQ(encoded(ForceControlCommand{0.0, 0.0, 0.0, 0.0, 2.0}, ranges),
            "00 00 00 7F FF 7F F9 10");
  // Velocity sub-mode: Kd only.
  EXPECT_EQ(encoded(ForceControlCommand{0.0, 2.0, 0.0, 6.0, 0.0}, ranges),
            "00 06 66 7F FF 93 27 FF");
  // Position sub-mode: Kp and Kd both set.
  EXPECT_EQ(encoded(ForceControlCommand{100.0, 2.0, 6.0, 0.0, 0.0}, ranges),
            "33 36 66 BD 24 7F F7 FF");
}

TEST(Ak30ForceWire, EncodesRangeBoundariesWithoutOverflow) {
  const auto ranges = ake60_8_ranges();

  EXPECT_EQ(encoded(ForceControlCommand{0.0, 0.0, 12.56, 0.0, 0.0}, ranges),
            "00 00 00 FF FF 7F F7 FF");
  EXPECT_EQ(encoded(ForceControlCommand{0.0, 0.0, -12.56, 0.0, 0.0}, ranges),
            "00 00 00 00 00 7F F7 FF");
  EXPECT_EQ(encoded(ForceControlCommand{0.0, 0.0, 0.0, 40.0, 0.0}, ranges),
            "00 00 00 7F FF FF F7 FF");
  EXPECT_EQ(encoded(ForceControlCommand{0.0, 0.0, 0.0, -40.0, 0.0}, ranges),
            "00 00 00 7F FF 00 07 FF");
  EXPECT_EQ(encoded(ForceControlCommand{0.0, 0.0, 0.0, 0.0, 15.0}, ranges),
            "00 00 00 7F FF 7F FF FF");
  EXPECT_EQ(encoded(ForceControlCommand{0.0, 0.0, 0.0, 0.0, -15.0}, ranges),
            "00 00 00 7F FF 7F F0 00");
}

// Kp at its maximum quantizes to 4094, not 4095, because the manual's table is
// generated by truncation and 500 * (4095/500) evaluates just below 4095 in
// binary floating point. Kd at its maximum does reach 4095. This asymmetry is
// real and matches the manual; do not "fix" it by rounding, which would break
// the position-zero and velocity-zero rows that pin the truncation behaviour.
TEST(Ak30ForceWire, QuantizesGainMaximaByTruncationLikeTheManualTable) {
  EXPECT_EQ(quantize(500.0, 0.0, 500.0, 12U), 4094U);
  EXPECT_EQ(quantize(5.0, 0.0, 5.0, 12U), 4095U);
  EXPECT_EQ(encoded(ForceControlCommand{500.0, 5.0, 0.0, 0.0, 0.0},
                    ake60_8_ranges()),
            "FF EF FF 7F FF 7F F7 FF");
}

// L07's printed float_to_uint() scales by (1<<bits)/span rather than
// ((1<<bits)-1)/span. Under that formula p_des = P_MAX yields p_int = 65536,
// which does not fit 16 bits: the payload becomes 00 00 and decodes as
// -12.56 rad. Maximum position would command minimum position. This test fails
// loudly if anyone restores the vendor formula.
TEST(Ak30ForceWire, EncodingMaxPositionDoesNotWrapToMinimum) {
  const auto ranges = ake60_8_ranges();
  Payload at_max{};
  Payload at_min{};
  ASSERT_TRUE(encode_force_control(ForceControlCommand{0.0, 0.0, 12.56, 0.0, 0.0},
                                   ranges, at_max));
  ASSERT_TRUE(encode_force_control(ForceControlCommand{0.0, 0.0, -12.56, 0.0, 0.0},
                                   ranges, at_min));
  EXPECT_NE(hex(at_max), hex(at_min));
  EXPECT_EQ(at_max[3], 0xFFU);
  EXPECT_EQ(at_max[4], 0xFFU);
}

// The vendor code clamps out-of-range input. Clamping silently converts an
// invalid command into a valid-looking one, which is exactly the degradation
// ADR-012 forbids. Reject instead.
TEST(Ak30ForceWire, RejectsOutOfRangeInputInsteadOfClamping) {
  const auto ranges = ake60_8_ranges();
  Payload payload{};

  EXPECT_FALSE(encode_force_control(
      ForceControlCommand{0.0, 0.0, 12.57, 0.0, 0.0}, ranges, payload));
  EXPECT_FALSE(encode_force_control(
      ForceControlCommand{0.0, 0.0, 0.0, 40.1, 0.0}, ranges, payload));
  EXPECT_FALSE(encode_force_control(
      ForceControlCommand{0.0, 0.0, 0.0, 0.0, -15.1}, ranges, payload));
  EXPECT_FALSE(encode_force_control(
      ForceControlCommand{500.1, 0.0, 0.0, 0.0, 0.0}, ranges, payload));
  EXPECT_FALSE(encode_force_control(
      ForceControlCommand{-0.1, 0.0, 0.0, 0.0, 0.0}, ranges, payload));
  EXPECT_FALSE(encode_force_control(
      ForceControlCommand{0.0, 5.1, 0.0, 0.0, 0.0}, ranges, payload));
}

TEST(Ak30ForceWire, RejectsNonFiniteInput) {
  const auto ranges = ake60_8_ranges();
  Payload payload{};
  const double nan_value = std::numeric_limits<double>::quiet_NaN();
  const double inf_value = std::numeric_limits<double>::infinity();

  EXPECT_FALSE(encode_force_control(
      ForceControlCommand{0.0, 0.0, nan_value, 0.0, 0.0}, ranges, payload));
  EXPECT_FALSE(encode_force_control(
      ForceControlCommand{0.0, 0.0, 0.0, inf_value, 0.0}, ranges, payload));
  EXPECT_FALSE(encode_force_control(
      ForceControlCommand{0.0, 0.0, 0.0, 0.0, -inf_value}, ranges, payload));
  EXPECT_FALSE(encode_force_control(
      ForceControlCommand{nan_value, 0.0, 0.0, 0.0, 0.0}, ranges, payload));
}

TEST(Ak30ForceWire, RejectsDegenerateRanges) {
  Payload payload{};
  const ForceControlRanges zero_position{0.0, 40.0, 15.0, 500.0, 5.0};
  EXPECT_FALSE(zero_position.is_valid());
  EXPECT_FALSE(encode_force_control(ForceControlCommand{}, zero_position,
                                    payload));
}

TEST(Ak30ForceWire, RoundTripsThroughDequantizeWithinOneLeastSignificantBit) {
  using mech::mech_protocol_cubemars::dequantize;
  const auto ranges = ake60_8_ranges();
  const double torque_lsb = (2.0 * ranges.torque_max_nm) / 4095.0;
  const std::uint32_t raw =
      quantize(2.0, -ranges.torque_max_nm, ranges.torque_max_nm, 12U);
  EXPECT_NEAR(dequantize(raw, -ranges.torque_max_nm, ranges.torque_max_nm, 12U),
              2.0, torque_lsb);
}

TEST(Ak30ForceWire, ComposesIdentifiersFromModeAndDriveId) {
  using mech::mech_protocol_cubemars::feedback_can_id;
  using mech::mech_protocol_cubemars::force_control_can_id;
  // motor1: drive id 104 decimal = 0x68.
  EXPECT_EQ(force_control_can_id(104U), 0x0868U);
  EXPECT_EQ(feedback_can_id(104U), 0x2968U);
  EXPECT_EQ(force_control_can_id(0U), 0x0800U);
  EXPECT_EQ(feedback_can_id(255U), 0x29FFU);
}

}  // namespace
```

- [ ] **Step 2: Run the tests to verify they fail**

Add `src/ak30_force_wire.cpp` to the `add_library` source list and `test/test_ak30_force_wire.cpp` to the `ament_add_gtest` list in `CMakeLists.txt`, then:

Run: `MECH_OUTPUT_ROOT=/tmp/t2 MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh`
Expected: FAIL — `ak30_force_wire.hpp: No such file or directory`

- [ ] **Step 3: Write the header**

`include/mech_protocol_cubemars/ak30_force_wire.hpp`:

```cpp
#pragma once

#include <array>
#include <cstdint>

namespace mech::mech_protocol_cubemars {

// Force control is control mode ID 8 (L07 section 4.2). Its three sub-modes
// share this one ID and are distinguished by payload content.
inline constexpr std::uint32_t kForceControlModeId = 8U;
// Periodic feedback function ID (L07 section 4.3.1).
inline constexpr std::uint32_t kFeedbackFunctionId = 0x29U;
inline constexpr std::size_t kForceControlPayloadBytes = 8U;

using ForceControlPayload = std::array<std::uint8_t, kForceControlPayloadBytes>;

// Per-model normalization limits from the force-control parameter table,
// L07 p.37. All signed quantities are symmetric about zero in that table.
struct ForceControlRanges final {
  double position_max_rad{12.56};
  double velocity_max_rad_s{40.0};
  double torque_max_nm{15.0};
  double kp_max{500.0};
  double kd_max{5.0};

  [[nodiscard]] constexpr bool is_valid() const noexcept {
    return position_max_rad > 0.0 && velocity_max_rad_s > 0.0 &&
           torque_max_nm > 0.0 && kp_max > 0.0 && kd_max > 0.0;
  }
};

// The project's motor. Position, Kp and Kd are merged cells spanning every
// model in the source table.
[[nodiscard]] constexpr ForceControlRanges ake60_8_ranges() noexcept {
  return ForceControlRanges{12.56, 40.0, 15.0, 500.0, 5.0};
}

// Not a motor we own. L07 section 4.4's worked examples are stated to use
// AK10-9 constants, so tests that cite the manual need these to decode its
// example rows correctly. Never use these for a real device.
[[nodiscard]] constexpr ForceControlRanges ak10_9_ranges() noexcept {
  return ForceControlRanges{12.56, 28.0, 54.0, 500.0, 5.0};
}

// A force-control command in device-native units. All five fields ride every
// frame; a sub-mode is expressed by which of them are non-zero, never by a
// different frame or identifier.
struct ForceControlCommand final {
  double kp{0.0};
  double kd{0.0};
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  double torque_nm{0.0};
};

[[nodiscard]] constexpr std::uint32_t force_control_can_id(
    std::uint8_t drive_id) noexcept {
  return (kForceControlModeId << 8U) | static_cast<std::uint32_t>(drive_id);
}

[[nodiscard]] constexpr std::uint32_t feedback_can_id(
    std::uint8_t drive_id) noexcept {
  return (kFeedbackFunctionId << 8U) | static_cast<std::uint32_t>(drive_id);
}

// Maps a value onto an unsigned field of `bits` width by truncation.
//
// The divisor is (1 << bits) - 1, NOT (1 << bits). L07 prints a
// float_to_uint() that uses (1 << bits); that formula reproduces none of the
// manual's own worked examples, and at position maximum it yields 65536, which
// overflows the 16-bit field to zero and decodes as minimum position. See the
// package README.
[[nodiscard]] std::uint32_t quantize(double value, double minimum,
                                     double maximum, unsigned bits) noexcept;

[[nodiscard]] double dequantize(std::uint32_t raw, double minimum,
                                double maximum, unsigned bits) noexcept;

// Packs a command into the 8-byte payload. Returns false, leaving `payload`
// untouched, when `ranges` is degenerate or any field is non-finite or outside
// its range. Deliberately does not clamp: the vendor reference clamps, which
// turns an invalid command into a valid-looking one.
[[nodiscard]] bool encode_force_control(const ForceControlCommand& command,
                                        const ForceControlRanges& ranges,
                                        ForceControlPayload& payload) noexcept;

}  // namespace mech::mech_protocol_cubemars
```

- [ ] **Step 4: Write the implementation**

`src/ak30_force_wire.cpp`:

```cpp
#include "mech_protocol_cubemars/ak30_force_wire.hpp"

#include <cmath>

namespace mech::mech_protocol_cubemars {
namespace {

[[nodiscard]] bool within(double value, double low, double high) noexcept {
  return std::isfinite(value) && value >= low && value <= high;
}

}  // namespace

std::uint32_t quantize(double value, double minimum, double maximum,
                       unsigned bits) noexcept {
  if (bits == 0U || bits > 31U) {
    return 0U;
  }
  const double span = maximum - minimum;
  if (!std::isfinite(span) || !(span > 0.0) || !std::isfinite(value)) {
    return 0U;
  }
  const std::uint32_t max_raw = (1U << bits) - 1U;
  if (value < minimum) {
    value = minimum;
  } else if (value > maximum) {
    value = maximum;
  }
  const double scaled = (value - minimum) * (static_cast<double>(max_raw) / span);
  if (scaled <= 0.0) {
    return 0U;
  }
  if (scaled >= static_cast<double>(max_raw)) {
    return max_raw;
  }
  // Truncation, not rounding: it is what reproduces the manual's example table.
  return static_cast<std::uint32_t>(scaled);
}

double dequantize(std::uint32_t raw, double minimum, double maximum,
                  unsigned bits) noexcept {
  if (bits == 0U || bits > 31U) {
    return minimum;
  }
  const std::uint32_t max_raw = (1U << bits) - 1U;
  if (raw > max_raw) {
    raw = max_raw;
  }
  return minimum + (static_cast<double>(raw) / static_cast<double>(max_raw)) *
                       (maximum - minimum);
}

bool encode_force_control(const ForceControlCommand& command,
                          const ForceControlRanges& ranges,
                          ForceControlPayload& payload) noexcept {
  if (!ranges.is_valid()) {
    return false;
  }
  if (!within(command.position_rad, -ranges.position_max_rad,
              ranges.position_max_rad) ||
      !within(command.velocity_rad_s, -ranges.velocity_max_rad_s,
              ranges.velocity_max_rad_s) ||
      !within(command.torque_nm, -ranges.torque_max_nm, ranges.torque_max_nm) ||
      !within(command.kp, 0.0, ranges.kp_max) ||
      !within(command.kd, 0.0, ranges.kd_max)) {
    return false;
  }

  const std::uint32_t kp = quantize(command.kp, 0.0, ranges.kp_max, 12U);
  const std::uint32_t kd = quantize(command.kd, 0.0, ranges.kd_max, 12U);
  const std::uint32_t position =
      quantize(command.position_rad, -ranges.position_max_rad,
               ranges.position_max_rad, 16U);
  const std::uint32_t velocity =
      quantize(command.velocity_rad_s, -ranges.velocity_max_rad_s,
               ranges.velocity_max_rad_s, 12U);
  const std::uint32_t torque = quantize(
      command.torque_nm, -ranges.torque_max_nm, ranges.torque_max_nm, 12U);

  // Field order is KP KD POS VEL TRQ. AK2.0 MIT and the widely copied Arduino
  // demos use POS VEL KP KD TRQ on an 11-bit standard frame; a packing helper
  // must never be shared between the two.
  payload[0] = static_cast<std::uint8_t>(kp >> 4U);
  payload[1] = static_cast<std::uint8_t>(((kp & 0x0FU) << 4U) | (kd >> 8U));
  payload[2] = static_cast<std::uint8_t>(kd & 0xFFU);
  payload[3] = static_cast<std::uint8_t>(position >> 8U);
  payload[4] = static_cast<std::uint8_t>(position & 0xFFU);
  payload[5] = static_cast<std::uint8_t>(velocity >> 4U);
  payload[6] =
      static_cast<std::uint8_t>(((velocity & 0x0FU) << 4U) | (torque >> 8U));
  payload[7] = static_cast<std::uint8_t>(torque & 0xFFU);
  return true;
}

}  // namespace mech::mech_protocol_cubemars
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `MECH_OUTPUT_ROOT=/tmp/t2 MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh`
Expected: PASS, 0 failures.

If `RejectsOutOfRangeInputInsteadOfClamping` fails on `12.57`, check that `within()` is not comparing against a clamped value. If a golden vector is off by one in the last nibble, the divisor is wrong — recheck `(1U << bits) - 1U`.

- [ ] **Step 6: Commit**

```bash
git add ros2_ws/src/mech_protocol_cubemars
git commit -m "feat(protocol): add the AK3.0 force-control wire encoder

Golden vectors are back-solved against L07's documented ranges, never copied.
Two manual defects are pinned by regression tests: the printed float_to_uint()
scales by (1<<bits) and overflows position maximum to minimum, and the
6 rad/s example row has 0x98 where 0x9B is correct.

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 3: Wire layer — feedback decoding

**Files:**
- Modify: `include/mech_protocol_cubemars/ak30_force_wire.hpp` (append), `src/ak30_force_wire.cpp` (append)
- Test: `ros2_ws/src/mech_protocol_cubemars/test/test_ak30_feedback_decode.cpp`
- Modify: `CMakeLists.txt` (add the test)

**Interfaces:**
- Consumes: `ForceControlPayload`, `kForceControlPayloadBytes` from Task 2.
- Produces: `ForceControlFeedback` (fields `position_deg`, `electrical_speed_erpm`, `current_iq_a`, `board_temperature_c`, `raw_status`), `StatusMeaning` (`NoFault`, `Fault`, `DisableAcknowledged`, `Unknown`), `classify_status(std::uint8_t) -> StatusMeaning`, `decode_feedback(const ForceControlPayload&, ForceControlFeedback&) -> void`.

- [ ] **Step 1: Write the failing tests**

`test/test_ak30_feedback_decode.cpp`:

```cpp
#include "mech_protocol_cubemars/ak30_force_wire.hpp"

#include <cstdint>

#include <gtest/gtest.h>

namespace {

using mech::mech_protocol_cubemars::classify_status;
using mech::mech_protocol_cubemars::decode_feedback;
using mech::mech_protocol_cubemars::ForceControlFeedback;
using mech::mech_protocol_cubemars::ForceControlPayload;
using mech::mech_protocol_cubemars::StatusMeaning;

// L07 section 4.3.1: position int16 x 0.1 deg, speed int16 x 10 ERPM,
// Iq int16 x 0.01 A, temperature Data[6] int8 deg C, Data[7] status.
TEST(Ak30Feedback, DecodesBigEndianFieldsWithDocumentedScaling) {
  // position 0x0BB8 = 3000 -> 300.0 deg
  // speed    0x03E8 = 1000 -> 10000 ERPM
  // Iq       0x0064 = 100  -> 1.0 A
  // temp     0x2D   = 45   -> 45 deg C
  const ForceControlPayload payload{0x0B, 0xB8, 0x03, 0xE8,
                                    0x00, 0x64, 0x2D, 0x00};
  ForceControlFeedback feedback{};
  decode_feedback(payload, feedback);

  EXPECT_DOUBLE_EQ(feedback.position_deg, 300.0);
  EXPECT_DOUBLE_EQ(feedback.electrical_speed_erpm, 10000.0);
  EXPECT_DOUBLE_EQ(feedback.current_iq_a, 1.0);
  EXPECT_DOUBLE_EQ(feedback.board_temperature_c, 45.0);
  EXPECT_EQ(feedback.raw_status, 0x00U);
}

TEST(Ak30Feedback, DecodesNegativeSignedFields) {
  // position 0xF448 = -3000 -> -300.0 deg
  // speed    0xFC18 = -1000 -> -10000 ERPM
  // Iq       0xFF9C = -100  -> -1.0 A
  // temp     0xEC   = -20   -> -20 deg C, the documented minimum
  const ForceControlPayload payload{0xF4, 0x48, 0xFC, 0x18,
                                    0xFF, 0x9C, 0xEC, 0x00};
  ForceControlFeedback feedback{};
  decode_feedback(payload, feedback);

  EXPECT_DOUBLE_EQ(feedback.position_deg, -300.0);
  EXPECT_DOUBLE_EQ(feedback.electrical_speed_erpm, -10000.0);
  EXPECT_DOUBLE_EQ(feedback.current_iq_a, -1.0);
  EXPECT_DOUBLE_EQ(feedback.board_temperature_c, -20.0);
}

TEST(Ak30Feedback, DecodesDocumentedRangeExtremes) {
  // The manual states position -32000..32000 maps to -3200..3200 degrees,
  // speed -32000..32000 maps to -320000..320000 ERPM, and Iq -6000..6000
  // maps to -60..60 A.
  ForceControlFeedback feedback{};
  decode_feedback(ForceControlPayload{0x7D, 0x00, 0x7D, 0x00,
                                      0x17, 0x70, 0x7F, 0x00},
                  feedback);
  EXPECT_DOUBLE_EQ(feedback.position_deg, 3200.0);
  EXPECT_DOUBLE_EQ(feedback.electrical_speed_erpm, 320000.0);
  EXPECT_DOUBLE_EQ(feedback.current_iq_a, 60.0);
  EXPECT_DOUBLE_EQ(feedback.board_temperature_c, 127.0);
}

// Data[7] carries two disjoint meanings. Fault codes are 0-7. 0x77 is the
// disable-succeeded acknowledgement returned once after control mode 15
// (L07 section 4.1.8). Decoding 0x77 as a fault would misreport the safety
// path as a failure. Anything else is unknown and must surface as unknown
// rather than being silently mapped onto a fault.
TEST(Ak30Feedback, ClassifiesTheStatusByteIntoItsThreeDisjointMeanings) {
  EXPECT_EQ(classify_status(0x00U), StatusMeaning::NoFault);
  for (std::uint8_t code = 1U; code <= 7U; ++code) {
    EXPECT_EQ(classify_status(code), StatusMeaning::Fault)
        << "fault code " << static_cast<int>(code);
  }
  EXPECT_EQ(classify_status(0x77U), StatusMeaning::DisableAcknowledged);
  EXPECT_EQ(classify_status(0x08U), StatusMeaning::Unknown);
  EXPECT_EQ(classify_status(0x76U), StatusMeaning::Unknown);
  EXPECT_EQ(classify_status(0x78U), StatusMeaning::Unknown);
  EXPECT_EQ(classify_status(0xFFU), StatusMeaning::Unknown);
}

TEST(Ak30Feedback, PreservesTheRawStatusByteAlongsideItsClassification) {
  ForceControlFeedback feedback{};
  decode_feedback(ForceControlPayload{0x00, 0x00, 0x00, 0x00,
                                      0x00, 0x00, 0x00, 0x77},
                  feedback);
  EXPECT_EQ(feedback.raw_status, 0x77U);
  EXPECT_EQ(classify_status(feedback.raw_status),
            StatusMeaning::DisableAcknowledged);
}

}  // namespace
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `MECH_OUTPUT_ROOT=/tmp/t3 MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh`
Expected: FAIL — `classify_status` / `decode_feedback` not declared.

- [ ] **Step 3: Append to the header**

Add to `ak30_force_wire.hpp`, before the closing namespace:

```cpp
// Fault codes occupy 0-7 (L07 section 4.3.1). 0x77 is the disable-succeeded
// acknowledgement (section 4.1.8), which shares the same byte.
inline constexpr std::uint8_t kMaxFaultCode = 7U;
inline constexpr std::uint8_t kDisableAcknowledgedStatus = 0x77U;

// Decoded 0x29 feedback in device-native units. Nothing here is canonical SI:
// the position's shaft source is undetermined, so converting it is the mapping
// layer's job and is gated on evidence.
struct ForceControlFeedback final {
  double position_deg{0.0};
  double electrical_speed_erpm{0.0};
  double current_iq_a{0.0};
  double board_temperature_c{0.0};
  std::uint8_t raw_status{0U};
};

enum class StatusMeaning : std::uint8_t {
  NoFault,
  Fault,
  DisableAcknowledged,
  Unknown,
};

[[nodiscard]] constexpr StatusMeaning classify_status(
    std::uint8_t raw) noexcept {
  if (raw == 0U) {
    return StatusMeaning::NoFault;
  }
  if (raw <= kMaxFaultCode) {
    return StatusMeaning::Fault;
  }
  if (raw == kDisableAcknowledgedStatus) {
    return StatusMeaning::DisableAcknowledged;
  }
  return StatusMeaning::Unknown;
}

// Total over all 8-byte payloads, so there is no failure mode to report. The
// caller is responsible for having checked identifier, DLC and frame format.
void decode_feedback(const ForceControlPayload& payload,
                     ForceControlFeedback& output) noexcept;
```

- [ ] **Step 4: Append the implementation**

Add to `src/ak30_force_wire.cpp`, inside the namespace:

```cpp
namespace {

[[nodiscard]] std::int16_t big_endian_int16(std::uint8_t high,
                                            std::uint8_t low) noexcept {
  const auto raw = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(high) << 8U) | static_cast<std::uint16_t>(low));
  return static_cast<std::int16_t>(raw);
}

}  // namespace

void decode_feedback(const ForceControlPayload& payload,
                     ForceControlFeedback& output) noexcept {
  output.position_deg =
      static_cast<double>(big_endian_int16(payload[0], payload[1])) * 0.1;
  output.electrical_speed_erpm =
      static_cast<double>(big_endian_int16(payload[2], payload[3])) * 10.0;
  output.current_iq_a =
      static_cast<double>(big_endian_int16(payload[4], payload[5])) * 0.01;
  output.board_temperature_c =
      static_cast<double>(static_cast<std::int8_t>(payload[6]));
  output.raw_status = payload[7];
}
```

Add `#include <cstdint>` to the `.cpp` includes if it is not already pulled in by the header.

- [ ] **Step 5: Run the tests to verify they pass**

Run: `MECH_OUTPUT_ROOT=/tmp/t3 MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh`
Expected: PASS, 0 failures.

Note `EXPECT_DOUBLE_EQ(feedback.position_deg, 300.0)`: `3000 * 0.1` is exact enough here because 3000 is exactly representable and 0.1's rounding lands on 300.0. If a scaling assertion fails by an ulp, switch that one line to `EXPECT_NEAR(..., 1e-9)` rather than changing the scaling.

- [ ] **Step 6: Commit**

```bash
git add ros2_ws/src/mech_protocol_cubemars
git commit -m "feat(protocol): decode the AK3.0 0x29 feedback frame

Data[7] carries two disjoint meanings: fault codes 0-7 and the 0x77
disable-succeeded acknowledgement. Classifying 0x77 as a fault would
misreport the safety path, so it is a distinct StatusMeaning and the raw
byte is preserved regardless.

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 4: Mapping layer — evidence gate and canonical conversion

Everything unevidenced lives here. The gate is the whole point of the task: with motor1's current evidence it must refuse all three sub-modes, and it must refuse them for the *right* reason so that resolving `direction_sign` alone visibly unblocks torque.

**Files:**
- Create: `include/mech_protocol_cubemars/ak30_mapping.hpp`, `src/ak30_mapping.cpp`
- Test: `test/test_ak30_mapping.cpp`
- Modify: `CMakeLists.txt` (add source and test to the existing lists)

**Interfaces:**
- Consumes: `ForceControlRanges`, `ake60_8_ranges()`, `ForceControlCommand`, `ForceControlFeedback` from Tasks 2–3.
- Produces: `EvidencedValue{double value; bool verified;}`, `ForceControlSubMode{Position,Velocity,Torque}`, `EvidencedStateFields{bool position; bool velocity; bool effort;}`, `Ak30Mapping`, `ForceControlGains{double kp; double kd;}`, `evidenced_state_fields(ForceControlSubMode) -> EvidencedStateFields`, `mapping_is_sufficient(const Ak30Mapping&, ForceControlSubMode) -> bool`, `to_device_command(...) -> void`, `to_canonical_state(...) -> void`.

> **Unverified assumption introduced here, flag it in the PR body.** The wire *command* velocity field is documented only as `电机速度 (rad/s)`, while *feedback* is ERPM requiring `÷ pole_pairs ÷ gear_ratio`. Whether the command's rad/s is output-side or motor-side is not stated anywhere in L07. This task assumes **output-side**, matching the torque field, which the manual does state is 输出端. The assumption is invisible until a real motor spins, so add it to `company/vendor_questions_2026-08-31.md` as **B15** in Task 9. It is gated behind the same `gear_ratio` evidence requirement, so it cannot reach a device before someone answers.

- [ ] **Step 1: Write the failing tests**

`test/test_ak30_mapping.cpp`:

```cpp
#include "mech_protocol_cubemars/ak30_mapping.hpp"

#include <gtest/gtest.h>

namespace {

using mech::mech_control_core::CanonicalDeviceCommand;
using mech::mech_control_core::CanonicalDeviceState;
using mech::mech_protocol_cubemars::Ak30Mapping;
using mech::mech_protocol_cubemars::evidenced_state_fields;
using mech::mech_protocol_cubemars::ForceControlCommand;
using mech::mech_protocol_cubemars::ForceControlFeedback;
using mech::mech_protocol_cubemars::ForceControlGains;
using mech::mech_protocol_cubemars::ForceControlSubMode;
using mech::mech_protocol_cubemars::mapping_is_sufficient;
using mech::mech_protocol_cubemars::to_canonical_state;
using mech::mech_protocol_cubemars::to_device_command;

// Everything verified. Individual tests knock out one parameter at a time so a
// rejection can only be attributed to that parameter.
[[nodiscard]] Ak30Mapping fully_verified() {
  Ak30Mapping mapping{};
  mapping.pole_pairs = {14.0, true};
  mapping.gear_ratio = {8.0, true};
  mapping.zero_offset_rad = {0.0, true};
  mapping.direction_sign = {1.0, true};
  mapping.torque_constant_nm_per_a = {0.7382, true};
  mapping.position_source_known = true;
  mapping.position_is_output_shaft = true;
  return mapping;
}

// motor1 as the repository actually evidences it today: only pole_pairs and
// torque_constant are verified. This is the state the evidence gate must
// refuse, and it is why no sub-mode configures yet.
[[nodiscard]] Ak30Mapping motor1_as_evidenced() {
  Ak30Mapping mapping{};
  mapping.pole_pairs = {14.0, true};
  mapping.torque_constant_nm_per_a = {0.7382, true};
  mapping.gear_ratio = {8.0, false};
  mapping.zero_offset_rad = {0.0, false};
  mapping.direction_sign = {1.0, false};
  mapping.position_source_known = false;
  return mapping;
}

TEST(Ak30Mapping, RefusesEverySubModeWithMotor1sCurrentEvidence) {
  const auto mapping = motor1_as_evidenced();
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Velocity));
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Position));
}

// The design's central claim: direction_sign alone stands between the project
// and a torque sub-mode that configures. If this test ever needs changing,
// the shortest path to a usable adapter has moved and the design doc is stale.
TEST(Ak30Mapping, VerifyingDirectionSignAloneUnblocksTorqueAndNothingElse) {
  auto mapping = motor1_as_evidenced();
  mapping.direction_sign = {1.0, true};

  EXPECT_TRUE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Velocity));
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Position));
}

TEST(Ak30Mapping, TorqueSubModeConsumesOnlyDirectionSignAndTorqueConstant) {
  auto mapping = fully_verified();
  mapping.gear_ratio.verified = false;
  mapping.zero_offset_rad.verified = false;
  mapping.position_source_known = false;
  mapping.pole_pairs.verified = false;
  EXPECT_TRUE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));

  mapping.direction_sign.verified = false;
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));

  mapping = fully_verified();
  mapping.torque_constant_nm_per_a.verified = false;
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));
}

TEST(Ak30Mapping, VelocitySubModeAlsoConsumesPolePairsAndGearRatio) {
  auto mapping = fully_verified();
  EXPECT_TRUE(mapping_is_sufficient(mapping, ForceControlSubMode::Velocity));

  mapping.pole_pairs.verified = false;
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Velocity));

  mapping = fully_verified();
  mapping.gear_ratio.verified = false;
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Velocity));

  // Velocity does not need the position chain.
  mapping = fully_verified();
  mapping.zero_offset_rad.verified = false;
  mapping.position_source_known = false;
  EXPECT_TRUE(mapping_is_sufficient(mapping, ForceControlSubMode::Velocity));
}

TEST(Ak30Mapping, PositionSubModeAlsoConsumesZeroOffsetAndShaftSource) {
  auto mapping = fully_verified();
  EXPECT_TRUE(mapping_is_sufficient(mapping, ForceControlSubMode::Position));

  mapping.position_source_known = false;
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Position));

  mapping = fully_verified();
  mapping.zero_offset_rad.verified = false;
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Position));
}

// A verified-but-nonsensical value is still a rejection. "Verified" means an
// evidence source was read, not that whatever number is present is usable.
TEST(Ak30Mapping, RejectsVerifiedButPhysicallyImpossibleValues) {
  auto mapping = fully_verified();
  mapping.direction_sign = {0.0, true};
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));

  mapping = fully_verified();
  mapping.direction_sign = {2.0, true};
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));

  mapping = fully_verified();
  mapping.direction_sign = {-1.0, true};
  EXPECT_TRUE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));

  mapping = fully_verified();
  mapping.torque_constant_nm_per_a = {0.0, true};
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));

  mapping = fully_verified();
  mapping.gear_ratio = {0.0, true};
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Velocity));

  mapping = fully_verified();
  mapping.pole_pairs = {0.0, true};
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Velocity));
}

// Fields a sub-mode cannot evidence stay at 0.0 and must not be exported as
// ros2_control interfaces by the later slice. This table is that contract.
TEST(Ak30Mapping, ReportsWhichCanonicalFieldsEachSubModeCanEvidence) {
  const auto torque = evidenced_state_fields(ForceControlSubMode::Torque);
  EXPECT_FALSE(torque.position);
  EXPECT_FALSE(torque.velocity);
  EXPECT_TRUE(torque.effort);

  const auto velocity = evidenced_state_fields(ForceControlSubMode::Velocity);
  EXPECT_FALSE(velocity.position);
  EXPECT_TRUE(velocity.velocity);
  EXPECT_TRUE(velocity.effort);

  const auto position = evidenced_state_fields(ForceControlSubMode::Position);
  EXPECT_TRUE(position.position);
  EXPECT_TRUE(position.velocity);
  EXPECT_TRUE(position.effort);
}

TEST(Ak30Mapping, ConvertsFeedbackToCanonicalSiForTheOutputShaft) {
  // position 900  -> 90.0 deg -> pi/2 rad
  // speed    1000 -> 10000 ERPM -> 10000/14/8 rpm -> 9.3499781... rad/s
  // Iq        200 -> 2.0 A -> 2.0 * 0.7382 = 1.4764 N.m
  const ForceControlFeedback feedback{90.0, 10000.0, 2.0, 40.0, 0x00U};
  CanonicalDeviceState state{};
  to_canonical_state(fully_verified(), ForceControlSubMode::Position, feedback,
                     state);

  EXPECT_NEAR(state.position, 1.5707963267948966, 1e-12);
  EXPECT_NEAR(state.velocity, 9.349978135683909, 1e-12);
  EXPECT_NEAR(state.effort, 1.4764, 1e-12);
}

TEST(Ak30Mapping, LeavesUnevidencedCanonicalFieldsUntouched) {
  const ForceControlFeedback feedback{90.0, 10000.0, 2.0, 40.0, 0x00U};
  CanonicalDeviceState state{};
  to_canonical_state(fully_verified(), ForceControlSubMode::Torque, feedback,
                     state);

  EXPECT_DOUBLE_EQ(state.position, 0.0);
  EXPECT_DOUBLE_EQ(state.velocity, 0.0);
  EXPECT_NEAR(state.effort, 1.4764, 1e-12);
}

TEST(Ak30Mapping, AppliesDirectionSignAndZeroOffsetToDecodedPosition) {
  const ForceControlFeedback feedback{90.0, 10000.0, 2.0, 40.0, 0x00U};

  auto inverted = fully_verified();
  inverted.direction_sign = {-1.0, true};
  CanonicalDeviceState state{};
  to_canonical_state(inverted, ForceControlSubMode::Position, feedback, state);
  EXPECT_NEAR(state.position, -1.5707963267948966, 1e-12);
  EXPECT_NEAR(state.velocity, -9.349978135683909, 1e-12);
  EXPECT_NEAR(state.effort, -1.4764, 1e-12);

  auto offset = fully_verified();
  offset.zero_offset_rad = {0.5, true};
  CanonicalDeviceState offset_state{};
  to_canonical_state(offset, ForceControlSubMode::Position, feedback,
                     offset_state);
  EXPECT_NEAR(offset_state.position, 1.5707963267948966 - 0.5, 1e-12);
}

// When the encoder reports on the motor side, the reduction must be divided
// out. Which side it actually reports is vendor question B4, which is why
// position_source_known gates the whole sub-mode.
TEST(Ak30Mapping, DividesOutTheReductionWhenPositionIsMotorSide) {
  auto mapping = fully_verified();
  mapping.position_is_output_shaft = false;
  const ForceControlFeedback feedback{90.0, 0.0, 0.0, 40.0, 0x00U};
  CanonicalDeviceState state{};
  to_canonical_state(mapping, ForceControlSubMode::Position, feedback, state);
  EXPECT_NEAR(state.position, 0.19634954084936207, 1e-12);
}

TEST(Ak30Mapping, BuildsTorqueSubModeCommandsFromEffortAlone) {
  ForceControlCommand wire{};
  CanonicalDeviceCommand command{};
  command.effort = 2.0;
  command.position = 1.0;   // ignored by this sub-mode, see below
  command.velocity = 3.0;   // ignored by this sub-mode, see below
  to_device_command(fully_verified(), ForceControlSubMode::Torque,
                    ForceControlGains{}, command, wire);

  EXPECT_DOUBLE_EQ(wire.kp, 0.0);
  EXPECT_DOUBLE_EQ(wire.kd, 0.0);
  EXPECT_DOUBLE_EQ(wire.torque_nm, 2.0);
}

// Deliberate and documented: in torque sub-mode the canonical command's
// position and velocity are ignored rather than rejected, because the later
// ros2_control slice claims only the effort interface for a torque-mode
// device, so no consumer can send a position it expects to be honoured.
// Named so it reads as intent, not as a bug someone should "fix".
TEST(Ak30Mapping, TorqueSubModeIgnoresPositionAndVelocityByDesign) {
  ForceControlCommand wire{};
  CanonicalDeviceCommand command{};
  command.effort = 2.0;
  command.position = 5.0;
  command.velocity = 7.0;
  to_device_command(fully_verified(), ForceControlSubMode::Torque,
                    ForceControlGains{}, command, wire);

  EXPECT_DOUBLE_EQ(wire.position_rad, 0.0);
  EXPECT_DOUBLE_EQ(wire.velocity_rad_s, 0.0);
}

TEST(Ak30Mapping, BuildsPositionSubModeCommandsWithGainsSignAndOffset) {
  auto mapping = fully_verified();
  mapping.zero_offset_rad = {0.5, true};
  ForceControlCommand wire{};
  CanonicalDeviceCommand command{};
  command.position = 1.0;
  command.velocity = 2.0;
  command.effort = 0.25;  // rides along as feedforward torque, the field's role
  to_device_command(mapping, ForceControlSubMode::Position,
                    ForceControlGains{100.0, 2.0}, command, wire);

  EXPECT_DOUBLE_EQ(wire.kp, 100.0);
  EXPECT_DOUBLE_EQ(wire.kd, 2.0);
  EXPECT_DOUBLE_EQ(wire.position_rad, 1.5);
  EXPECT_DOUBLE_EQ(wire.velocity_rad_s, 2.0);
  EXPECT_DOUBLE_EQ(wire.torque_nm, 0.25);
}

TEST(Ak30Mapping, VelocitySubModeCarriesKdButNotKp) {
  ForceControlCommand wire{};
  CanonicalDeviceCommand command{};
  command.velocity = 6.0;
  to_device_command(fully_verified(), ForceControlSubMode::Velocity,
                    ForceControlGains{100.0, 2.0}, command, wire);

  EXPECT_DOUBLE_EQ(wire.kp, 0.0);
  EXPECT_DOUBLE_EQ(wire.kd, 2.0);
  EXPECT_DOUBLE_EQ(wire.velocity_rad_s, 6.0);
  EXPECT_DOUBLE_EQ(wire.position_rad, 0.0);
}

// Encode and decode must be exact inverses, otherwise a commanded position and
// the position reported back would disagree by a constant nobody notices.
TEST(Ak30Mapping, PositionEncodeAndDecodeAreInverses) {
  auto mapping = fully_verified();
  mapping.direction_sign = {-1.0, true};
  mapping.zero_offset_rad = {0.25, true};

  CanonicalDeviceCommand command{};
  command.position = 1.0;
  ForceControlCommand wire{};
  to_device_command(mapping, ForceControlSubMode::Position, ForceControlGains{},
                    command, wire);

  // Feed the encoded shaft angle back through the decoder as degrees.
  ForceControlFeedback feedback{};
  feedback.position_deg = wire.position_rad * 180.0 / 3.14159265358979323846;
  CanonicalDeviceState state{};
  to_canonical_state(mapping, ForceControlSubMode::Position, feedback, state);

  EXPECT_NEAR(state.position, command.position, 1e-12);
}

}  // namespace
```

- [ ] **Step 2: Run the tests to verify they fail**

Add `src/ak30_mapping.cpp` and `test/test_ak30_mapping.cpp` to `CMakeLists.txt`, then:

Run: `MECH_OUTPUT_ROOT=/tmp/t4 MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh`
Expected: FAIL — `ak30_mapping.hpp: No such file or directory`

- [ ] **Step 3: Write the header**

`include/mech_protocol_cubemars/ak30_mapping.hpp`:

```cpp
#pragma once

#include <cstdint>

#include "mech_control_core/adapter_template.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"

namespace mech::mech_protocol_cubemars {

inline constexpr double kPi = 3.14159265358979323846;

// Mirrors the *_verified tri-state that ADR-012 established for
// TransportCapabilities: verified means an evidence source was read,
// unverified with a value means somebody asserted it, unverified with zero
// means genuinely unknown and deliberately not claimed.
struct EvidencedValue final {
  double value{0.0};
  bool verified{false};
};

// The three sub-modes share control mode ID 8 and are distinguished by payload
// content, so this is explicit configuration and is never inferred from a
// received frame.
enum class ForceControlSubMode : std::uint8_t { Position, Velocity, Torque };

// Which CanonicalDeviceState fields the configured sub-mode can populate from
// evidence. Anything false is left at 0.0 and must not be exported as a
// ros2_control interface by a later slice.
struct EvidencedStateFields final {
  bool position{false};
  bool velocity{false};
  bool effort{false};
};

// Gains are configuration, not canonical command fields: CanonicalDeviceCommand
// has no place for them and adding one would be a contract change needing an
// ADR.
struct ForceControlGains final {
  double kp{0.0};
  double kd{0.0};
};

// Defaults are motor1's evidence state as the repository records it today:
// only pole_pairs and torque_constant are verified, so no sub-mode configures.
struct Ak30Mapping final {
  EvidencedValue pole_pairs{14.0, true};
  EvidencedValue gear_ratio{8.0, false};
  EvidencedValue zero_offset_rad{0.0, false};
  EvidencedValue direction_sign{1.0, false};
  // L07 p.37 for AKE60-8, owner-guaranteed for this variant (ADR-013 section 4).
  EvidencedValue torque_constant_nm_per_a{0.7382, true};
  // Vendor question B4: L07 writes 输出端 explicitly for torque and speed but
  // not for position, so the source is undetermined.
  bool position_source_known{false};
  bool position_is_output_shaft{false};
  ForceControlRanges ranges{ake60_8_ranges()};
};

[[nodiscard]] constexpr EvidencedStateFields evidenced_state_fields(
    ForceControlSubMode sub_mode) noexcept {
  switch (sub_mode) {
    case ForceControlSubMode::Torque:
      return EvidencedStateFields{false, false, true};
    case ForceControlSubMode::Velocity:
      return EvidencedStateFields{false, true, true};
    case ForceControlSubMode::Position:
      return EvidencedStateFields{true, true, true};
  }
  return EvidencedStateFields{};
}

// Fails closed. The same mapping converts both directions, so suppressing only
// decode would still let a session emit a command computed from an unverified
// mapping - worse, because it moves the motor. ADR-009 Decision 2 prescribes
// rejecting at configure rather than downgrading.
[[nodiscard]] bool mapping_is_sufficient(const Ak30Mapping& mapping,
                                         ForceControlSubMode sub_mode) noexcept;

// Precondition: mapping_is_sufficient(mapping, sub_mode). Range and finiteness
// checking belongs to encode_force_control, which runs on the result.
void to_device_command(
    const Ak30Mapping& mapping, ForceControlSubMode sub_mode,
    const ForceControlGains& gains,
    const mech_control_core::CanonicalDeviceCommand& command,
    ForceControlCommand& output) noexcept;

// Populates only the fields evidenced_state_fields() reports for the sub-mode;
// the rest are left as the caller set them. Does not touch output.status.
void to_canonical_state(
    const Ak30Mapping& mapping, ForceControlSubMode sub_mode,
    const ForceControlFeedback& feedback,
    mech_control_core::CanonicalDeviceState& output) noexcept;

}  // namespace mech::mech_protocol_cubemars
```

- [ ] **Step 4: Write the implementation**

`src/ak30_mapping.cpp`:

```cpp
#include "mech_protocol_cubemars/ak30_mapping.hpp"

#include <cmath>

namespace mech::mech_protocol_cubemars {
namespace {

[[nodiscard]] bool positive_and_verified(const EvidencedValue& value) noexcept {
  return value.verified && std::isfinite(value.value) && value.value > 0.0;
}

// Shaft angle in radians on whichever side the encoder reports, converted to
// the output shaft.
[[nodiscard]] double feedback_position_rad(const Ak30Mapping& mapping,
                                           double position_deg) noexcept {
  const double raw_rad = position_deg * kPi / 180.0;
  return mapping.position_is_output_shaft ? raw_rad
                                          : raw_rad / mapping.gear_ratio.value;
}

}  // namespace

bool mapping_is_sufficient(const Ak30Mapping& mapping,
                           ForceControlSubMode sub_mode) noexcept {
  if (!mapping.ranges.is_valid()) {
    return false;
  }
  // direction_sign and torque_constant are consumed by every sub-mode: the
  // former in both directions, the latter because effort is always exported.
  if (!mapping.direction_sign.verified ||
      (mapping.direction_sign.value != 1.0 &&
       mapping.direction_sign.value != -1.0)) {
    return false;
  }
  if (!positive_and_verified(mapping.torque_constant_nm_per_a)) {
    return false;
  }
  if (sub_mode == ForceControlSubMode::Torque) {
    return true;
  }
  // Velocity decode is ERPM / pole_pairs / gear_ratio.
  if (!positive_and_verified(mapping.pole_pairs) ||
      !positive_and_verified(mapping.gear_ratio)) {
    return false;
  }
  if (sub_mode == ForceControlSubMode::Velocity) {
    return true;
  }
  // Position additionally needs the zero reference and the encoder's shaft.
  return mapping.position_source_known && mapping.zero_offset_rad.verified &&
         std::isfinite(mapping.zero_offset_rad.value);
}

void to_device_command(const Ak30Mapping& mapping, ForceControlSubMode sub_mode,
                       const ForceControlGains& gains,
                       const mech_control_core::CanonicalDeviceCommand& command,
                       ForceControlCommand& output) noexcept {
  const double sign = mapping.direction_sign.value;
  output = ForceControlCommand{};
  // The torque field is a feedforward term in every sub-mode - the vendor's own
  // pack_cmd() names it t_ff - so effort rides along rather than being dropped.
  output.torque_nm = sign * command.effort;

  if (sub_mode == ForceControlSubMode::Torque) {
    return;
  }

  // Assumed output-side rad/s, matching the torque field. L07 does not state
  // which side the command velocity refers to; vendor question B15.
  output.kd = gains.kd;
  output.velocity_rad_s = sign * command.velocity;

  if (sub_mode == ForceControlSubMode::Velocity) {
    return;
  }

  output.kp = gains.kp;
  const double shaft_rad = sign * (command.position + mapping.zero_offset_rad.value);
  output.position_rad = mapping.position_is_output_shaft
                            ? shaft_rad
                            : shaft_rad * mapping.gear_ratio.value;
}

void to_canonical_state(const Ak30Mapping& mapping, ForceControlSubMode sub_mode,
                        const ForceControlFeedback& feedback,
                        mech_control_core::CanonicalDeviceState& output) noexcept {
  const double sign = mapping.direction_sign.value;
  const auto fields = evidenced_state_fields(sub_mode);

  if (fields.effort) {
    // T = Kt * Iq, and L07 states T is output-shaft torque.
    output.effort = sign * mapping.torque_constant_nm_per_a.value *
                    feedback.current_iq_a;
  }
  if (fields.velocity) {
    // 输出端转速 = ERPM / 极对数 / 减速比, then rpm to rad/s.
    const double output_rpm = feedback.electrical_speed_erpm /
                              mapping.pole_pairs.value /
                              mapping.gear_ratio.value;
    output.velocity = sign * output_rpm * (2.0 * kPi / 60.0);
  }
  if (fields.position) {
    output.position = sign * feedback_position_rad(mapping, feedback.position_deg) -
                      mapping.zero_offset_rad.value;
  }
}

}  // namespace mech::mech_protocol_cubemars
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `MECH_OUTPUT_ROOT=/tmp/t4 MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh`
Expected: PASS, 0 failures.

If `PositionEncodeAndDecodeAreInverses` fails, the sign/offset order disagrees between the two functions. The convention is `canonical = sign * shaft - offset`, hence `shaft = sign * (canonical + offset)`; both must use exactly that.

- [ ] **Step 6: Commit**

```bash
git add ros2_ws/src/mech_protocol_cubemars
git commit -m "feat(protocol): add the evidence-gated AK3.0 mapping layer

mapping_is_sufficient() fails closed per ADR-009 Decision 2. With motor1's
current evidence every sub-mode is refused; verifying direction_sign alone
unblocks torque and nothing else, which is pinned by a named test.

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 5: `DeviceCodec` implementation

**Files:**
- Create: `include/mech_protocol_cubemars/ak30_force_codec.hpp`, `src/ak30_force_codec.cpp`
- Test: `test/test_ak30_force_codec.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: everything from Tasks 2–4; `mech_control_core::{DeviceCodec, AdapterResult, CanonicalDeviceCommand, CanonicalDeviceState, RawCanFrame, CanId, StatusSnapshot, ProtocolProfile}`.
- Produces: `class Ak30ForceControlCodec final : public mech_control_core::DeviceCodec` with constructor `(std::uint8_t drive_id, ForceControlSubMode, Ak30Mapping, ForceControlGains)` and the three overrides.

- [ ] **Step 1: Write the failing tests**

`test/test_ak30_force_codec.cpp`:

```cpp
#include "mech_protocol_cubemars/ak30_force_codec.hpp"

#include <gtest/gtest.h>

namespace {

using mech::mech_control_core::AdapterResult;
using mech::mech_control_core::CanFrameFormat;
using mech::mech_control_core::CanFrameType;
using mech::mech_control_core::CanId;
using mech::mech_control_core::CanonicalDeviceCommand;
using mech::mech_control_core::CanonicalDeviceState;
using mech::mech_control_core::DeviceState;
using mech::mech_control_core::FrameDirection;
using mech::mech_control_core::MonotonicTime;
using mech::mech_control_core::RawCanFrame;
using mech::mech_control_core::SampleQuality;
using mech::mech_protocol_cubemars::Ak30ForceControlCodec;
using mech::mech_protocol_cubemars::Ak30Mapping;
using mech::mech_protocol_cubemars::ForceControlGains;
using mech::mech_protocol_cubemars::ForceControlSubMode;

constexpr std::uint8_t kDriveId = 104U;  // motor1, decimal 104 = 0x68
constexpr std::uint16_t kLogicalBus = 0U;

[[nodiscard]] Ak30Mapping fully_verified() {
  Ak30Mapping mapping{};
  mapping.pole_pairs = {14.0, true};
  mapping.gear_ratio = {8.0, true};
  mapping.zero_offset_rad = {0.0, true};
  mapping.direction_sign = {1.0, true};
  mapping.torque_constant_nm_per_a = {0.7382, true};
  mapping.position_source_known = true;
  mapping.position_is_output_shaft = true;
  return mapping;
}

[[nodiscard]] Ak30ForceControlCodec torque_codec() {
  return Ak30ForceControlCodec{kDriveId, ForceControlSubMode::Torque,
                               fully_verified(), ForceControlGains{}};
}

[[nodiscard]] MonotonicTime at(std::int64_t nanoseconds) {
  return MonotonicTime::from_nanoseconds(nanoseconds).value();
}

// Builds a well-formed 0x29 feedback frame, which individual tests then break
// one field at a time.
[[nodiscard]] RawCanFrame feedback_frame() {
  std::array<std::uint8_t, 64U> payload{};
  payload[0] = 0x03U;  // position 900 -> 90.0 deg
  payload[1] = 0x84U;
  payload[2] = 0x03U;  // speed 1000 -> 10000 ERPM
  payload[3] = 0xE8U;
  payload[4] = 0x00U;  // Iq 200 -> 2.0 A
  payload[5] = 0xC8U;
  payload[6] = 0x28U;  // 40 deg C
  payload[7] = 0x00U;  // no fault
  return RawCanFrame::create(kLogicalBus,
                             CanId::create(0x2968U, CanFrameFormat::Extended).value(),
                             CanFrameType::Classic, FrameDirection::Rx, 8U,
                             payload, at(1000))
      .value();
}

TEST(Ak30Codec, DeclaresTheForceControlExtendedProfile) {
  EXPECT_EQ(torque_codec().profile(),
            mech::mech_control_core::ProtocolProfile::Ak30ForceControlExtended);
}

TEST(Ak30Codec, EncodesToAnExtendedClassicFrameAtTheCommandIdentifier) {
  CanonicalDeviceCommand command{};
  command.effort = 2.0;
  RawCanFrame frame{};
  ASSERT_EQ(torque_codec().encode(command, kLogicalBus, at(500), frame),
            AdapterResult::Ok);

  EXPECT_EQ(frame.id.value, 0x0868U);
  EXPECT_EQ(frame.id.format, CanFrameFormat::Extended);
  EXPECT_EQ(frame.type, CanFrameType::Classic);
  EXPECT_EQ(frame.direction, FrameDirection::Tx);
  EXPECT_EQ(frame.payload_size, 8U);
  EXPECT_FALSE(frame.bitrate_switch);
  EXPECT_FALSE(frame.remote_request);
  EXPECT_EQ(frame.logical_bus, kLogicalBus);
  // Same bytes as the wire-layer AKE60-8 torque golden vector.
  EXPECT_EQ(frame.payload[6], 0xF9U);
  EXPECT_EQ(frame.payload[7], 0x10U);
}

TEST(Ak30Codec, RejectsCommandsTheWireLayerCannotRepresent) {
  CanonicalDeviceCommand command{};
  command.effort = 15.1;  // beyond AKE60-8's +/-15 N.m
  RawCanFrame frame{};
  EXPECT_EQ(torque_codec().encode(command, kLogicalBus, at(500), frame),
            AdapterResult::InvalidCommand);

  command.effort = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(torque_codec().encode(command, kLogicalBus, at(500), frame),
            AdapterResult::InvalidCommand);
}

TEST(Ak30Codec, DecodesAWellFormedFeedbackFrame) {
  CanonicalDeviceState state{};
  ASSERT_EQ(torque_codec().decode(feedback_frame(), state), AdapterResult::Ok);

  EXPECT_NEAR(state.effort, 1.4764, 1e-12);
  EXPECT_EQ(state.status.quality, SampleQuality::Valid);
  EXPECT_EQ(state.status.raw_fault_code, 0U);
  ASSERT_TRUE(state.status.host_rx_time.has_value());
  EXPECT_EQ(state.status.host_rx_time.value(), at(1000));
}

TEST(Ak30Codec, RejectsFramesThatAreNotThisDevicesFeedback) {
  CanonicalDeviceState state{};

  // Wrong function ID: the servo start frame 0x2C, not 0x29.
  auto wrong_function = feedback_frame();
  wrong_function.id = CanId::create(0x2C68U, CanFrameFormat::Extended).value();
  EXPECT_EQ(torque_codec().decode(wrong_function, state),
            AdapterResult::InvalidCommand);

  // Right function, different drive.
  auto other_drive = feedback_frame();
  other_drive.id = CanId::create(0x2969U, CanFrameFormat::Extended).value();
  EXPECT_EQ(torque_codec().decode(other_drive, state),
            AdapterResult::InvalidCommand);

  // A standard-format frame carrying the same numeric value. This is the AK2.0
  // MIT shape; accepting it would be the exact defect ADR-013 removed.
  auto standard = feedback_frame();
  standard.id = CanId::create(0x268U, CanFrameFormat::Standard).value();
  EXPECT_EQ(torque_codec().decode(standard, state),
            AdapterResult::InvalidCommand);
}

TEST(Ak30Codec, RejectsMalformedFeedbackFrames) {
  CanonicalDeviceState state{};

  auto short_dlc = feedback_frame();
  short_dlc.payload_size = 7U;
  EXPECT_EQ(torque_codec().decode(short_dlc, state), AdapterResult::InvalidCommand);

  auto flexible = feedback_frame();
  flexible.type = CanFrameType::FlexibleDataRate;
  EXPECT_EQ(torque_codec().decode(flexible, state), AdapterResult::InvalidCommand);

  auto brs = feedback_frame();
  brs.bitrate_switch = true;
  EXPECT_EQ(torque_codec().decode(brs, state), AdapterResult::InvalidCommand);

  auto remote = feedback_frame();
  remote.remote_request = true;
  EXPECT_EQ(torque_codec().decode(remote, state), AdapterResult::InvalidCommand);

  auto errored = feedback_frame();
  errored.error_frame = true;
  EXPECT_EQ(torque_codec().decode(errored, state), AdapterResult::InvalidCommand);
}

TEST(Ak30Codec, SurfacesAFaultCodeWithoutOverwritingTheRawByte) {
  auto frame = feedback_frame();
  frame.payload[7] = 0x05U;  // encoder fault
  CanonicalDeviceState state{};
  ASSERT_EQ(torque_codec().decode(frame, state), AdapterResult::Ok);

  EXPECT_EQ(state.status.raw_fault_code, 0x05U);
  EXPECT_EQ(state.status.device_state, DeviceState::Fault);
  EXPECT_EQ(state.status.quality, SampleQuality::Valid);
}

// 0x77 is the disable-succeeded acknowledgement, not fault code 0x77. Treating
// it as a fault would misreport the safety path as a failure.
TEST(Ak30Codec, DoesNotTreatTheDisableAcknowledgementAsAFault) {
  auto frame = feedback_frame();
  frame.payload[7] = 0x77U;
  CanonicalDeviceState state{};
  ASSERT_EQ(torque_codec().decode(frame, state), AdapterResult::Ok);

  EXPECT_EQ(state.status.raw_fault_code, 0x77U);
  EXPECT_NE(state.status.device_state, DeviceState::Fault);
}

// An uninterpretable status byte is a genuine per-sample condition, so the
// sample is Degraded rather than silently mapped onto "no fault".
TEST(Ak30Codec, MarksAnUnknownStatusByteDegradedRatherThanGuessing) {
  auto frame = feedback_frame();
  frame.payload[7] = 0x42U;
  CanonicalDeviceState state{};
  ASSERT_EQ(torque_codec().decode(frame, state), AdapterResult::Ok);

  EXPECT_EQ(state.status.raw_fault_code, 0x42U);
  EXPECT_EQ(state.status.quality, SampleQuality::Degraded);
  EXPECT_NE(state.status.device_state, DeviceState::Fault);
}

TEST(Ak30Codec, PreservesASourceTimestampWhenTheBackendSuppliedOne) {
  auto frame = feedback_frame();
  frame.source_timestamp = mech::mech_control_core::SourceTimestamp{
      mech::mech_control_core::SourceClockDomain::Transport, 4242U};
  CanonicalDeviceState state{};
  ASSERT_EQ(torque_codec().decode(frame, state), AdapterResult::Ok);

  ASSERT_TRUE(state.status.source_timestamp.has_value());
  EXPECT_EQ(state.status.source_timestamp.value().ticks, 4242U);
}

}  // namespace
```

Add `#include <array>`, `#include <cstdint>` and `#include <limits>` to the test's includes.

- [ ] **Step 2: Run the tests to verify they fail**

Run: `MECH_OUTPUT_ROOT=/tmp/t5 MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh`
Expected: FAIL — `ak30_force_codec.hpp: No such file or directory`

- [ ] **Step 3: Write the header**

`include/mech_protocol_cubemars/ak30_force_codec.hpp`:

```cpp
#pragma once

#include <cstdint>

#include "mech_control_core/adapter_template.hpp"
#include "mech_protocol_cubemars/ak30_mapping.hpp"

namespace mech::mech_protocol_cubemars {

// Pure codec: no I/O, no session state, no allocation, no device probing. The
// sub-mode is fixed at construction because force control's three sub-modes
// share control mode ID 8 and inferring one from payload content would be a
// silent-failure path.
class Ak30ForceControlCodec final : public mech_control_core::DeviceCodec {
 public:
  Ak30ForceControlCodec(std::uint8_t drive_id, ForceControlSubMode sub_mode,
                        Ak30Mapping mapping, ForceControlGains gains) noexcept;

  [[nodiscard]] mech_control_core::ProtocolProfile profile() const noexcept override;

  [[nodiscard]] mech_control_core::AdapterResult encode(
      const mech_control_core::CanonicalDeviceCommand& command,
      std::uint16_t logical_bus, mech_control_core::MonotonicTime now,
      mech_control_core::RawCanFrame& output) const noexcept override;

  [[nodiscard]] mech_control_core::AdapterResult decode(
      const mech_control_core::RawCanFrame& frame,
      mech_control_core::CanonicalDeviceState& output) const noexcept override;

  [[nodiscard]] ForceControlSubMode sub_mode() const noexcept { return sub_mode_; }
  [[nodiscard]] const Ak30Mapping& mapping() const noexcept { return mapping_; }

 private:
  std::uint8_t drive_id_;
  ForceControlSubMode sub_mode_;
  Ak30Mapping mapping_;
  ForceControlGains gains_;
};

}  // namespace mech::mech_protocol_cubemars
```

- [ ] **Step 4: Write the implementation**

`src/ak30_force_codec.cpp`:

```cpp
#include "mech_protocol_cubemars/ak30_force_codec.hpp"

#include <array>

namespace mech::mech_protocol_cubemars {

using mech_control_core::AdapterResult;
using mech_control_core::CanFrameFormat;
using mech_control_core::CanFrameType;
using mech_control_core::CanId;
using mech_control_core::DeviceState;
using mech_control_core::FrameDirection;
using mech_control_core::ProtocolProfile;
using mech_control_core::SampleQuality;
using mech_control_core::StatusSnapshot;

Ak30ForceControlCodec::Ak30ForceControlCodec(std::uint8_t drive_id,
                                             ForceControlSubMode sub_mode,
                                             Ak30Mapping mapping,
                                             ForceControlGains gains) noexcept
    : drive_id_(drive_id),
      sub_mode_(sub_mode),
      mapping_(mapping),
      gains_(gains) {}

ProtocolProfile Ak30ForceControlCodec::profile() const noexcept {
  return ProtocolProfile::Ak30ForceControlExtended;
}

AdapterResult Ak30ForceControlCodec::encode(
    const mech_control_core::CanonicalDeviceCommand& command,
    std::uint16_t logical_bus, mech_control_core::MonotonicTime now,
    mech_control_core::RawCanFrame& output) const noexcept {
  ForceControlCommand wire{};
  to_device_command(mapping_, sub_mode_, gains_, command, wire);

  ForceControlPayload packed{};
  if (!encode_force_control(wire, mapping_.ranges, packed)) {
    return AdapterResult::InvalidCommand;
  }

  const auto id = CanId::create(force_control_can_id(drive_id_),
                                CanFrameFormat::Extended);
  if (!id.has_value()) {
    return AdapterResult::InvalidConfiguration;
  }

  std::array<std::uint8_t, mech_control_core::kMaxCanPayloadBytes> payload{};
  for (std::size_t index = 0; index < packed.size(); ++index) {
    payload[index] = packed[index];
  }

  const auto frame = mech_control_core::RawCanFrame::create(
      logical_bus, id.value(), CanFrameType::Classic, FrameDirection::Tx,
      static_cast<std::uint8_t>(packed.size()), payload, now);
  if (!frame.has_value()) {
    return AdapterResult::InvalidCommand;
  }
  output = frame.value();
  return AdapterResult::Ok;
}

AdapterResult Ak30ForceControlCodec::decode(
    const mech_control_core::RawCanFrame& frame,
    mech_control_core::CanonicalDeviceState& output) const noexcept {
  // Reject rather than degrade: a frame that is not exactly this device's
  // Classic extended 8-byte feedback is not this device's feedback.
  if (frame.id.format != CanFrameFormat::Extended ||
      frame.id.value != feedback_can_id(drive_id_) ||
      frame.type != CanFrameType::Classic || frame.error_frame ||
      frame.remote_request || frame.bitrate_switch ||
      frame.payload_size != kForceControlPayloadBytes) {
    return AdapterResult::InvalidCommand;
  }

  ForceControlPayload packed{};
  for (std::size_t index = 0; index < packed.size(); ++index) {
    packed[index] = frame.payload[index];
  }

  ForceControlFeedback feedback{};
  decode_feedback(packed, feedback);
  to_canonical_state(mapping_, sub_mode_, feedback, output);

  const StatusMeaning meaning = classify_status(feedback.raw_status);
  // The codec reports only what the frame itself proves. The session owns the
  // device state machine and overwrites device_state with its own view.
  const DeviceState device_state = meaning == StatusMeaning::Fault
                                       ? DeviceState::Fault
                                       : DeviceState::Unknown;
  const SampleQuality quality = meaning == StatusMeaning::Unknown
                                    ? SampleQuality::Degraded
                                    : SampleQuality::Valid;

  const auto status = StatusSnapshot::create(
      quality, device_state, static_cast<std::uint32_t>(feedback.raw_status),
      0U, frame.host_arrival, frame.source_timestamp);
  if (!status.has_value()) {
    return AdapterResult::Fault;
  }
  output.status = status.value();
  return AdapterResult::Ok;
}

}  // namespace mech::mech_protocol_cubemars
```

Note `sequence` is `0U`: the codec is stateless and `const`, so sequencing is the session's job per `02:82`.

- [ ] **Step 5: Run the tests to verify they pass**

Run: `MECH_OUTPUT_ROOT=/tmp/t5 MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh`
Expected: PASS, 0 failures.

If `DecodesAWellFormedFeedbackFrame` fails inside `StatusSnapshot::create`, check its precondition: a non-`Unknown` quality requires `host_rx_time` to be present, which `frame.host_arrival` always is.

- [ ] **Step 6: Commit**

```bash
git add ros2_ws/src/mech_protocol_cubemars
git commit -m "feat(protocol): implement DeviceCodec for AK3.0 force control

Rejects anything that is not this device's Classic extended 8-byte 0x29
feedback, including a standard-format frame carrying the same numeric ID -
the AK2.0 MIT shape whose acceptance was the defect ADR-013 removed.

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 6: Session configuration and the validation matrix

`configure()` is where the evidence gate actually bites. Every rejection reason from design §6 gets its own test, each knocking out exactly one field of an otherwise-valid pair, so a failure names the cause.

**Files:**
- Create: `include/mech_protocol_cubemars/ak30_force_session.hpp`, `src/ak30_force_session.cpp`
- Create: `test/ak30_test_fixtures.hpp` (shared by Tasks 6, 7, 8)
- Test: `test/test_ak30_force_session.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: everything from Tasks 2–5; `mech_control_core::{DeviceSession, Transport, TransportResult, TransportCapabilities, DeviceConfig, profile_requirements}`.
- Produces: `kMaxHardTtlNanoseconds`, `CommandStage{Following,Holding,Expired}`, `Ak30SessionConfig`, `class Ak30ForceControlSession final : public mech_control_core::DeviceSession` with constructor `(Transport&, Ak30SessionConfig)`. Task 7 fills in the runtime overrides this task stubs.
- Produces (test fixtures): `RecordingTransport`, `classic_extended_capabilities()`, `valid_device_config()`, `valid_session_config()`, `fully_verified_mapping()`, `at(std::int64_t)`, `feedback_payload(...)`, `feedback_frame(...)`.

- [ ] **Step 1: Write the shared test fixtures**

`test/ak30_test_fixtures.hpp`:

```cpp
#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "mech_control_core/transport.hpp"
#include "mech_protocol_cubemars/ak30_force_session.hpp"

namespace mech::mech_protocol_cubemars::testing {

// motor1: drive id 104 decimal = 0x68, so command 0x0868 and feedback 0x2968.
inline constexpr std::uint16_t kDriveId = 104U;
inline constexpr std::uint32_t kCommandId = 0x0868U;
inline constexpr std::uint32_t kFeedbackId = 0x2968U;
inline constexpr std::uint16_t kLogicalBus = 0U;

// A transport double with an injectable send result. A double that reproduces
// only the success path is how the RC shipped a defect that permanently
// faulted the bus on a transient WouldBlock: no fake ever produced it.
class RecordingTransport final : public mech_control_core::Transport {
 public:
  explicit RecordingTransport(
      mech_control_core::TransportCapabilities capabilities) noexcept
      : capabilities_(capabilities) {}

  [[nodiscard]] mech_control_core::TransportKind kind() const noexcept override {
    return mech_control_core::TransportKind::Fake;
  }
  [[nodiscard]] const mech_control_core::TransportCapabilities& capabilities()
      const noexcept override {
    return capabilities_;
  }
  [[nodiscard]] bool is_open() const noexcept override { return open_; }
  bool open() noexcept override {
    open_ = true;
    return true;
  }
  void close() noexcept override { open_ = false; }
  [[nodiscard]] mech_control_core::TransportResult try_receive(
      mech_control_core::RawCanFrame&) noexcept override {
    return mech_control_core::TransportResult::WouldBlock;
  }
  [[nodiscard]] mech_control_core::TransportResult try_send(
      const mech_control_core::RawCanFrame& frame) noexcept override {
    if (next_send_result_ != mech_control_core::TransportResult::Ok) {
      return next_send_result_;
    }
    sent_.push_back(frame);
    return mech_control_core::TransportResult::Ok;
  }
  [[nodiscard]] mech_control_core::TransportStats stats() const noexcept override {
    return mech_control_core::TransportStats{};
  }

  void inject_send_result(mech_control_core::TransportResult result) noexcept {
    next_send_result_ = result;
  }
  [[nodiscard]] const std::vector<mech_control_core::RawCanFrame>& sent()
      const noexcept {
    return sent_;
  }

 private:
  mech_control_core::TransportCapabilities capabilities_;
  std::vector<mech_control_core::RawCanFrame> sent_;
  mech_control_core::TransportResult next_send_result_{
      mech_control_core::TransportResult::Ok};
  bool open_{false};
};

[[nodiscard]] inline mech_control_core::TransportCapabilities
classic_extended_capabilities() {
  mech_control_core::TransportCapabilities capabilities{};
  capabilities.supports_classic_can = true;
  capabilities.supports_extended_frames = true;
  capabilities.supports_non_blocking_io = true;
  capabilities.max_payload_bytes = 8U;
  capabilities.queue_capacity = 16U;
  // Bitrate stays unverified with value zero: this is a fake channel and has
  // no bitrate to read back. ADR-012 makes that a legal state.
  return capabilities;
}

[[nodiscard]] inline Ak30Mapping fully_verified_mapping() {
  Ak30Mapping mapping{};
  mapping.pole_pairs = {14.0, true};
  mapping.gear_ratio = {8.0, true};
  mapping.zero_offset_rad = {0.0, true};
  mapping.direction_sign = {1.0, true};
  mapping.torque_constant_nm_per_a = {0.7382, true};
  mapping.position_source_known = true;
  mapping.position_is_output_shaft = true;
  return mapping;
}

[[nodiscard]] inline mech_control_core::DeviceConfig valid_device_config() {
  mech_control_core::DeviceConfig config{};
  config.device_id = 1U;
  config.name = "motor1";
  config.logical_bus = kLogicalBus;
  config.profile = mech_control_core::ProtocolProfile::Ak30ForceControlExtended;
  config.frame_type = mech_control_core::CanFrameType::Classic;
  config.frame_format = mech_control_core::CanFrameFormat::Extended;
  config.command_id = mech_control_core::CanId::create(
      kCommandId, mech_control_core::CanFrameFormat::Extended);
  config.feedback_id = mech_control_core::CanId::create(
      kFeedbackId, mech_control_core::CanFrameFormat::Extended);
  config.command_payload_bytes = 8U;
  config.feedback_payload_bytes = 8U;
  config.writable = true;
  return config;
}

[[nodiscard]] inline Ak30SessionConfig valid_session_config() {
  Ak30SessionConfig config{};
  config.drive_id = kDriveId;
  config.sub_mode = ForceControlSubMode::Torque;
  config.mapping = fully_verified_mapping();
  config.gains = ForceControlGains{};
  // Operator-asserted encoded firmware version; motor1 displays AKE60_8_DE_V3.4.
  config.firmware_id = 0x0304U;
  config.firmware_id_min = 0x0300U;
  config.firmware_id_max = 0x03FFU;
  config.command_ttl_nanoseconds = 4000000;
  config.command_hard_ttl_nanoseconds = 6000000;
  config.feedback_ttl_nanoseconds = 6000000;
  return config;
}

[[nodiscard]] inline mech_control_core::MonotonicTime at(
    std::int64_t nanoseconds) {
  return mech_control_core::MonotonicTime::from_nanoseconds(nanoseconds).value();
}

// position 900 -> 90.0 deg, speed 1000 -> 10000 ERPM, Iq 200 -> 2.0 A, 40 C.
[[nodiscard]] inline std::array<std::uint8_t, 64U> feedback_payload(
    std::uint8_t status) {
  std::array<std::uint8_t, 64U> payload{};
  payload[0] = 0x03U;
  payload[1] = 0x84U;
  payload[2] = 0x03U;
  payload[3] = 0xE8U;
  payload[4] = 0x00U;
  payload[5] = 0xC8U;
  payload[6] = 0x28U;
  payload[7] = status;
  return payload;
}

[[nodiscard]] inline mech_control_core::RawCanFrame feedback_frame(
    std::uint8_t status, mech_control_core::MonotonicTime arrival) {
  return mech_control_core::RawCanFrame::create(
             kLogicalBus,
             mech_control_core::CanId::create(
                 kFeedbackId, mech_control_core::CanFrameFormat::Extended)
                 .value(),
             mech_control_core::CanFrameType::Classic,
             mech_control_core::FrameDirection::Rx, 8U,
             feedback_payload(status), arrival)
      .value();
}

}  // namespace mech::mech_protocol_cubemars::testing
```

- [ ] **Step 2: Write the failing configure tests**

`test/test_ak30_force_session.cpp` (Task 7 appends to this same file):

```cpp
#include "mech_protocol_cubemars/ak30_force_session.hpp"

#include <gtest/gtest.h>

#include "ak30_test_fixtures.hpp"

namespace {

using mech::mech_control_core::AdapterResult;
using mech::mech_control_core::CanFrameFormat;
using mech::mech_control_core::CanFrameType;
using mech::mech_control_core::CanId;
using mech::mech_control_core::ProtocolProfile;
using mech::mech_protocol_cubemars::Ak30ForceControlSession;
using mech::mech_protocol_cubemars::ForceControlSubMode;
namespace fixtures = mech::mech_protocol_cubemars::testing;

TEST(Ak30SessionConfigure, AcceptsAFullyEvidencedConfiguration) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};
  EXPECT_EQ(session.configure(fixtures::valid_device_config(),
                              fixtures::classic_extended_capabilities()),
            AdapterResult::Ok);
}

// The state the repository is actually in today. If this test ever passes,
// either evidence landed or the gate broke; both need a human to look.
TEST(Ak30SessionConfigure, RefusesMotor1sCurrentEvidenceForEverySubMode) {
  for (const auto sub_mode :
       {ForceControlSubMode::Torque, ForceControlSubMode::Velocity,
        ForceControlSubMode::Position}) {
    fixtures::RecordingTransport transport{
        fixtures::classic_extended_capabilities()};
    auto config = fixtures::valid_session_config();
    config.sub_mode = sub_mode;
    config.mapping = mech::mech_protocol_cubemars::Ak30Mapping{};  // defaults
    Ak30ForceControlSession session{transport, config};
    EXPECT_EQ(session.configure(fixtures::valid_device_config(),
                                fixtures::classic_extended_capabilities()),
              AdapterResult::InvalidConfiguration);
  }
}

TEST(Ak30SessionConfigure, RejectsAnyProfileOtherThanForceControlExtended) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};

  for (const auto profile :
       {ProtocolProfile::Unknown, ProtocolProfile::LoopbackV1,
        ProtocolProfile::Ak30ServoExtended, ProtocolProfile::Hi12J1939,
        ProtocolProfile::Hi12Canopen}) {
    auto config = fixtures::valid_device_config();
    config.profile = profile;
    EXPECT_EQ(session.configure(config, fixtures::classic_extended_capabilities()),
              AdapterResult::InvalidConfiguration);
  }
}

// The superseded AK2.0 MIT enum required Standard here. Accepting a Standard
// frame format is the exact inversion ADR-013 removed, so it gets its own test.
TEST(Ak30SessionConfigure, RejectsStandardFormatAndNonClassicFrameTypes) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};

  auto standard = fixtures::valid_device_config();
  standard.frame_format = CanFrameFormat::Standard;
  EXPECT_EQ(session.configure(standard, fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  auto flexible = fixtures::valid_device_config();
  flexible.frame_type = CanFrameType::FlexibleDataRate;
  EXPECT_EQ(session.configure(flexible, fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

// The extended identifier reserves bits [7:0] for the drive ID, so 255 is the
// hard ceiling and a CubeMarsTool reading of "104" can only ever be decimal.
TEST(Ak30SessionConfigure, RejectsADriveIdThatDoesNotFitEightBits) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  auto config = fixtures::valid_session_config();
  config.drive_id = 256U;
  Ak30ForceControlSession session{transport, config};
  EXPECT_EQ(session.configure(fixtures::valid_device_config(),
                              fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

TEST(Ak30SessionConfigure, RejectsIdentifiersThatDoNotMatchTheDriveId) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};

  auto missing_command = fixtures::valid_device_config();
  missing_command.command_id.reset();
  EXPECT_EQ(session.configure(missing_command,
                              fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  auto missing_feedback = fixtures::valid_device_config();
  missing_feedback.feedback_id.reset();
  EXPECT_EQ(session.configure(missing_feedback,
                              fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  // Control mode 9 instead of force control's 8.
  auto wrong_mode = fixtures::valid_device_config();
  wrong_mode.command_id = CanId::create(0x0968U, CanFrameFormat::Extended);
  EXPECT_EQ(session.configure(wrong_mode, fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  // Feedback pointing at a different drive.
  auto wrong_drive = fixtures::valid_device_config();
  wrong_drive.feedback_id = CanId::create(0x2969U, CanFrameFormat::Extended);
  EXPECT_EQ(session.configure(wrong_drive, fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

TEST(Ak30SessionConfigure, RejectsPayloadSizesThatAreNotEightBytes) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};

  auto short_command = fixtures::valid_device_config();
  short_command.command_payload_bytes = 4U;
  EXPECT_EQ(session.configure(short_command,
                              fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  auto long_feedback = fixtures::valid_device_config();
  long_feedback.feedback_payload_bytes = 16U;
  EXPECT_EQ(session.configure(long_feedback,
                              fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

TEST(Ak30SessionConfigure, RejectsAReadOnlyDeviceBecauseForceControlCommands) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};
  auto config = fixtures::valid_device_config();
  config.writable = false;
  EXPECT_EQ(session.configure(config, fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

// Reject unavailable capabilities rather than synthesizing them.
TEST(Ak30SessionConfigure, RejectsBackendsThatCannotCarryClassicExtendedFrames) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};

  auto no_extended = fixtures::classic_extended_capabilities();
  no_extended.supports_extended_frames = false;
  no_extended.supports_standard_frames = true;
  EXPECT_EQ(session.configure(fixtures::valid_device_config(), no_extended),
            AdapterResult::InvalidConfiguration);

  auto no_classic = fixtures::classic_extended_capabilities();
  no_classic.supports_classic_can = false;
  no_classic.supports_can_fd = true;
  EXPECT_EQ(session.configure(fixtures::valid_device_config(), no_classic),
            AdapterResult::InvalidConfiguration);

  auto too_small = fixtures::classic_extended_capabilities();
  too_small.max_payload_bytes = 4U;
  EXPECT_EQ(session.configure(fixtures::valid_device_config(), too_small),
            AdapterResult::InvalidConfiguration);

  auto invalid = fixtures::classic_extended_capabilities();
  invalid.queue_capacity = 0U;
  ASSERT_FALSE(invalid.is_valid());
  EXPECT_EQ(session.configure(fixtures::valid_device_config(), invalid),
            AdapterResult::InvalidConfiguration);
}

TEST(Ak30SessionConfigure, RejectsAFirmwareIdOutsideTheAcceptedRange) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};

  auto below = fixtures::valid_session_config();
  below.firmware_id = 0x02FFU;
  Ak30ForceControlSession below_session{transport, below};
  EXPECT_EQ(below_session.configure(fixtures::valid_device_config(),
                                    fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  auto above = fixtures::valid_session_config();
  above.firmware_id = 0x0400U;
  Ak30ForceControlSession above_session{transport, above};
  EXPECT_EQ(above_session.configure(fixtures::valid_device_config(),
                                    fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  auto inverted = fixtures::valid_session_config();
  inverted.firmware_id_min = 0x0400U;
  inverted.firmware_id_max = 0x0300U;
  Ak30ForceControlSession inverted_session{transport, inverted};
  EXPECT_EQ(inverted_session.configure(fixtures::valid_device_config(),
                                       fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

// ADR-012 caps the whole watchdog at <=3 control cycles, which is <=6 ms at
// 500 Hz per 03_mvp_delivery_plan.md:215. The 2026-08-27 review found a
// shipped TTL 16x over that budget and the first fix made it 33x; nothing in
// the build catches a number that contradicts a planning document, so it is
// checked here.
TEST(Ak30SessionConfigure, RejectsWatchdogTimingsOutsideTheDocumentedBudget) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};

  auto too_long = fixtures::valid_session_config();
  too_long.command_hard_ttl_nanoseconds = 6000001;
  Ak30ForceControlSession too_long_session{transport, too_long};
  EXPECT_EQ(too_long_session.configure(fixtures::valid_device_config(),
                                       fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  auto inverted = fixtures::valid_session_config();
  inverted.command_ttl_nanoseconds = 5000000;
  inverted.command_hard_ttl_nanoseconds = 4000000;
  Ak30ForceControlSession inverted_session{transport, inverted};
  EXPECT_EQ(inverted_session.configure(fixtures::valid_device_config(),
                                       fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  auto zero = fixtures::valid_session_config();
  zero.command_ttl_nanoseconds = 0;
  Ak30ForceControlSession zero_session{transport, zero};
  EXPECT_EQ(zero_session.configure(fixtures::valid_device_config(),
                                   fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

TEST(Ak30SessionConfigure, RejectsGainsOutsideTheModelsDocumentedRange) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};

  auto high_kp = fixtures::valid_session_config();
  high_kp.sub_mode = ForceControlSubMode::Position;
  high_kp.gains.kp = 500.1;
  Ak30ForceControlSession kp_session{transport, high_kp};
  EXPECT_EQ(kp_session.configure(fixtures::valid_device_config(),
                                 fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  auto high_kd = fixtures::valid_session_config();
  high_kd.gains.kd = 5.1;
  Ak30ForceControlSession kd_session{transport, high_kd};
  EXPECT_EQ(kd_session.configure(fixtures::valid_device_config(),
                                 fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

TEST(Ak30SessionConfigure, IsRepeatable) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};
  EXPECT_EQ(session.configure(fixtures::valid_device_config(),
                              fixtures::classic_extended_capabilities()),
            AdapterResult::Ok);
  EXPECT_EQ(session.configure(fixtures::valid_device_config(),
                              fixtures::classic_extended_capabilities()),
            AdapterResult::Ok);
}

}  // namespace
```

- [ ] **Step 3: Run to verify the tests fail**

Add `src/ak30_force_session.cpp` and `test/test_ak30_force_session.cpp` to `CMakeLists.txt`, and add `target_include_directories(${PROJECT_NAME}_test PRIVATE test)` inside the `BUILD_TESTING` block so `ak30_test_fixtures.hpp` resolves.

Run: `MECH_OUTPUT_ROOT=/tmp/t6 MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh`
Expected: FAIL — `ak30_force_session.hpp: No such file or directory`

- [ ] **Step 4: Write the header**

`include/mech_protocol_cubemars/ak30_force_session.hpp`:

```cpp
#pragma once

#include <cstdint>
#include <optional>

#include "mech_control_core/adapter_template.hpp"
#include "mech_control_core/transport.hpp"
#include "mech_protocol_cubemars/ak30_force_codec.hpp"

namespace mech::mech_protocol_cubemars {

// ADR-012 requires the whole watchdog to fit <=3 control cycles, which is
// <=6 ms at the documented 500 Hz (03_mvp_delivery_plan.md:215).
inline constexpr std::int64_t kMaxHardTtlNanoseconds = 6000000;

// Restates the staged watchdog that mech_controllers::WatchdogStage already
// implements. This package must not depend on mech_controllers - that would
// invert the dependency direction and break the EXPECTED_PACKAGES boundary -
// so the semantics are duplicated deliberately. ADR-012 already records
// unifying these into core's CommandSlot as an open review trigger.
enum class CommandStage : std::uint8_t { Following, Holding, Expired };

struct Ak30SessionConfig final {
  // Wider than the 8-bit wire field so an out-of-range value is representable
  // and therefore rejectable at configure rather than silently truncated.
  std::uint16_t drive_id{0U};
  ForceControlSubMode sub_mode{ForceControlSubMode::Torque};
  Ak30Mapping mapping{};
  ForceControlGains gains{};
  // 02:160 requires on_configure to validate a firmware range. DeviceConfig has
  // no firmware field, and adding one would be a canonical contract change
  // requiring an ADR first (adapter_contract_v1.md item 7), so it lives here.
  // AK3.0 has no firmware query on the wire, so this is operator-asserted.
  std::uint32_t firmware_id{0U};
  std::uint32_t firmware_id_min{0U};
  std::uint32_t firmware_id_max{0U};
  std::int64_t command_ttl_nanoseconds{4000000};
  std::int64_t command_hard_ttl_nanoseconds{6000000};
  std::int64_t feedback_ttl_nanoseconds{6000000};
};

// Owns device semantics; borrows the transport and never opens a channel.
class Ak30ForceControlSession final : public mech_control_core::DeviceSession {
 public:
  Ak30ForceControlSession(mech_control_core::Transport& transport,
                          Ak30SessionConfig config) noexcept;

  [[nodiscard]] mech_control_core::AdapterResult configure(
      const mech_control_core::DeviceConfig& config,
      const mech_control_core::TransportCapabilities& capabilities) noexcept override;
  [[nodiscard]] mech_control_core::AdapterResult activate() noexcept override;
  void deactivate() noexcept override;
  [[nodiscard]] mech_control_core::AdapterResult submit(
      const mech_control_core::CanonicalDeviceCommand& command,
      mech_control_core::MonotonicTime now) noexcept override;
  [[nodiscard]] mech_control_core::AdapterResult process(
      const mech_control_core::RawCanFrame& frame,
      mech_control_core::MonotonicTime now) noexcept override;
  [[nodiscard]] mech_control_core::CanonicalDeviceState snapshot(
      mech_control_core::MonotonicTime now) const noexcept override;

  [[nodiscard]] CommandStage command_stage(
      mech_control_core::MonotonicTime now) const noexcept;
  [[nodiscard]] bool fault_latched() const noexcept { return fault_latched_; }

 private:
  enum class Lifecycle : std::uint8_t { Unconfigured, Ready, Active };

  Ak30SessionConfig config_;
  std::optional<Ak30ForceControlCodec> codec_;
  Lifecycle lifecycle_{Lifecycle::Unconfigured};
  std::uint16_t logical_bus_{0U};
  bool fault_latched_{false};
  std::uint64_t sequence_{0U};
  mech_control_core::CanonicalDeviceState last_state_{};
  std::optional<mech_control_core::MonotonicTime> last_feedback_time_;
  std::optional<mech_control_core::MonotonicTime> last_command_time_;
};

}  // namespace mech::mech_protocol_cubemars
```

- [ ] **Step 5: Implement `configure` and stub the rest**

`src/ak30_force_session.cpp` — Task 7 replaces the stubs:

```cpp
#include "mech_protocol_cubemars/ak30_force_session.hpp"

namespace mech::mech_protocol_cubemars {

using mech_control_core::AdapterResult;
using mech_control_core::CanFrameFormat;
using mech_control_core::CanFrameType;
using mech_control_core::ProtocolProfile;

Ak30ForceControlSession::Ak30ForceControlSession(
    mech_control_core::Transport& transport, Ak30SessionConfig config) noexcept
    : mech_control_core::DeviceSession(transport), config_(config) {}

AdapterResult Ak30ForceControlSession::configure(
    const mech_control_core::DeviceConfig& config,
    const mech_control_core::TransportCapabilities& capabilities) noexcept {
  // ACTIVE never changes profile or re-binds configuration.
  if (lifecycle_ == Lifecycle::Active) {
    return AdapterResult::InvalidConfiguration;
  }
  if (config.profile != ProtocolProfile::Ak30ForceControlExtended) {
    return AdapterResult::InvalidConfiguration;
  }
  const auto requirements =
      mech_control_core::profile_requirements(config.profile);
  if (!requirements.has_value() ||
      requirements->frame_type != config.frame_type ||
      requirements->frame_format != config.frame_format) {
    return AdapterResult::InvalidConfiguration;
  }
  if (config_.drive_id > 255U) {
    return AdapterResult::InvalidConfiguration;
  }
  const auto drive_id = static_cast<std::uint8_t>(config_.drive_id);
  if (!config.command_id.has_value() || !config.feedback_id.has_value() ||
      config.command_id->format != CanFrameFormat::Extended ||
      config.feedback_id->format != CanFrameFormat::Extended ||
      config.command_id->value != force_control_can_id(drive_id) ||
      config.feedback_id->value != feedback_can_id(drive_id)) {
    return AdapterResult::InvalidConfiguration;
  }
  if (config.command_payload_bytes != kForceControlPayloadBytes ||
      config.feedback_payload_bytes != kForceControlPayloadBytes) {
    return AdapterResult::InvalidConfiguration;
  }
  if (!config.writable) {
    return AdapterResult::InvalidConfiguration;
  }
  if (!capabilities.is_valid() || !capabilities.supports_classic_can ||
      !capabilities.supports_extended_frames ||
      capabilities.max_payload_bytes < kForceControlPayloadBytes) {
    return AdapterResult::InvalidConfiguration;
  }
  if (config_.firmware_id_min > config_.firmware_id_max ||
      config_.firmware_id < config_.firmware_id_min ||
      config_.firmware_id > config_.firmware_id_max) {
    return AdapterResult::InvalidConfiguration;
  }
  if (!mapping_is_sufficient(config_.mapping, config_.sub_mode)) {
    return AdapterResult::InvalidConfiguration;
  }
  if (config_.gains.kp < 0.0 || config_.gains.kp > config_.mapping.ranges.kp_max ||
      config_.gains.kd < 0.0 || config_.gains.kd > config_.mapping.ranges.kd_max) {
    return AdapterResult::InvalidConfiguration;
  }
  if (config_.command_ttl_nanoseconds <= 0 ||
      config_.command_hard_ttl_nanoseconds <= config_.command_ttl_nanoseconds ||
      config_.command_hard_ttl_nanoseconds > kMaxHardTtlNanoseconds ||
      config_.feedback_ttl_nanoseconds <= 0) {
    return AdapterResult::InvalidConfiguration;
  }

  logical_bus_ = config.logical_bus;
  codec_.emplace(drive_id, config_.sub_mode, config_.mapping, config_.gains);
  lifecycle_ = Lifecycle::Ready;
  fault_latched_ = false;
  sequence_ = 0U;
  last_state_ = mech_control_core::CanonicalDeviceState{};
  last_feedback_time_.reset();
  last_command_time_.reset();
  return AdapterResult::Ok;
}

// Task 7 replaces these stubs.
AdapterResult Ak30ForceControlSession::activate() noexcept {
  return AdapterResult::InvalidConfiguration;
}
void Ak30ForceControlSession::deactivate() noexcept {}
AdapterResult Ak30ForceControlSession::submit(
    const mech_control_core::CanonicalDeviceCommand&,
    mech_control_core::MonotonicTime) noexcept {
  return AdapterResult::InvalidConfiguration;
}
AdapterResult Ak30ForceControlSession::process(
    const mech_control_core::RawCanFrame&,
    mech_control_core::MonotonicTime) noexcept {
  return AdapterResult::InvalidConfiguration;
}
mech_control_core::CanonicalDeviceState Ak30ForceControlSession::snapshot(
    mech_control_core::MonotonicTime) const noexcept {
  return last_state_;
}
CommandStage Ak30ForceControlSession::command_stage(
    mech_control_core::MonotonicTime) const noexcept {
  return CommandStage::Expired;
}

}  // namespace mech::mech_protocol_cubemars
```

- [ ] **Step 6: Run to verify the tests pass, then commit**

Run: `MECH_OUTPUT_ROOT=/tmp/t6 MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh`
Expected: PASS, 0 failures.

```bash
git add ros2_ws/src/mech_protocol_cubemars
git commit -m "feat(protocol): validate AK3.0 force-control session configuration

configure() fails closed on every reason in the design's section 6, including
the evidence gate. Firmware range lives in the package-local session config
because DeviceConfig has no firmware field and adding one would be a canonical
contract change requiring an ADR first.

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 7: Session runtime — lifecycle, watchdog, faults

**Files:**
- Modify: `src/ak30_force_session.cpp` (replace the Task 6 stubs)
- Test: append to `test/test_ak30_force_session.cpp`

**Interfaces:**
- Consumes: everything from Task 6.
- Produces: working `activate`/`deactivate`/`submit`/`process`/`snapshot`/`command_stage`.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_ak30_force_session.cpp`, inside the same anonymous namespace:

```cpp
using mech::mech_control_core::CanonicalDeviceCommand;
using mech::mech_control_core::DeviceState;
using mech::mech_control_core::SampleQuality;
using mech::mech_control_core::TransportResult;
using mech::mech_protocol_cubemars::CommandStage;

// Configures and activates, so runtime tests start from a known state.
class Ak30SessionRuntime : public ::testing::Test {
 protected:
  Ak30SessionRuntime()
      : transport_(fixtures::classic_extended_capabilities()),
        session_(transport_, fixtures::valid_session_config()) {}

  void SetUp() override {
    ASSERT_EQ(session_.configure(fixtures::valid_device_config(),
                                 fixtures::classic_extended_capabilities()),
              AdapterResult::Ok);
    ASSERT_EQ(session_.activate(), AdapterResult::Ok);
  }

  [[nodiscard]] static CanonicalDeviceCommand torque_command(double effort,
                                                             std::int64_t deadline) {
    CanonicalDeviceCommand command{};
    command.effort = effort;
    command.deadline = fixtures::at(deadline);
    return command;
  }

  fixtures::RecordingTransport transport_;
  Ak30ForceControlSession session_;
};

TEST_F(Ak30SessionRuntime, SendsOneFrameToTheBorrowedTransportPerCommand) {
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(1000)),
            AdapterResult::Ok);
  ASSERT_EQ(transport_.sent().size(), 1U);
  EXPECT_EQ(transport_.sent().front().id.value, fixtures::kCommandId);
  EXPECT_EQ(transport_.sent().front().payload[7], 0x10U);
}

TEST(Ak30SessionLifecycle, RefusesToActivateBeforeConfigure) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};
  EXPECT_EQ(session.activate(), AdapterResult::InvalidConfiguration);
  EXPECT_EQ(session.submit(CanonicalDeviceCommand{}, fixtures::at(1000)),
            AdapterResult::InvalidConfiguration);
}

TEST_F(Ak30SessionRuntime, RefusesToReconfigureWhileActive) {
  EXPECT_EQ(session_.configure(fixtures::valid_device_config(),
                               fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

TEST_F(Ak30SessionRuntime, IsRepeatableAcrossDeactivateAndActivate) {
  session_.deactivate();
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(1000)),
            AdapterResult::InvalidConfiguration);
  EXPECT_EQ(session_.activate(), AdapterResult::Ok);
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(1000)),
            AdapterResult::Ok);
}

TEST_F(Ak30SessionRuntime, ReportsAnEmptySnapshotBeforeAnyFeedbackArrives) {
  const auto state = session_.snapshot(fixtures::at(1000));
  EXPECT_EQ(state.status.quality, SampleQuality::Unknown);
  EXPECT_FALSE(state.status.host_rx_time.has_value());
  EXPECT_FALSE(state.status.has_sample());
}

TEST_F(Ak30SessionRuntime, AcceptsACommandBeforeAnyFeedbackHasArrived) {
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(1000)),
            AdapterResult::Ok);
}

TEST_F(Ak30SessionRuntime, SequencesAcceptedFeedbackFrames) {
  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x00U, fixtures::at(1000)),
                             fixtures::at(1000)),
            AdapterResult::Ok);
  EXPECT_EQ(session_.snapshot(fixtures::at(1000)).status.sequence, 1U);

  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x00U, fixtures::at(2000)),
                             fixtures::at(2000)),
            AdapterResult::Ok);
  EXPECT_EQ(session_.snapshot(fixtures::at(2000)).status.sequence, 2U);
}

TEST_F(Ak30SessionRuntime, IgnoresFramesThatAreNotThisDevicesFeedback) {
  auto foreign = fixtures::feedback_frame(0x00U, fixtures::at(1000));
  foreign.id = mech::mech_control_core::CanId::create(
                   0x2969U, CanFrameFormat::Extended)
                   .value();
  EXPECT_EQ(session_.process(foreign, fixtures::at(1000)),
            AdapterResult::InvalidCommand);
  EXPECT_EQ(session_.snapshot(fixtures::at(1000)).status.sequence, 0U);
}

TEST_F(Ak30SessionRuntime, MarksTheSampleStaleOnceFeedbackExceedsItsTtl) {
  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x00U, fixtures::at(1000)),
                             fixtures::at(1000)),
            AdapterResult::Ok);
  EXPECT_EQ(session_.snapshot(fixtures::at(1000)).status.quality,
            SampleQuality::Valid);
  // feedback_ttl is 6 ms.
  EXPECT_EQ(session_.snapshot(fixtures::at(6000999)).status.quality,
            SampleQuality::Valid);
  EXPECT_EQ(session_.snapshot(fixtures::at(7001000)).status.quality,
            SampleQuality::Stale);
}

// ADR-012's staged watchdog: follow, then freeze the last valid command, then
// an explicit error past the hard TTL. ttl 4 ms, hard_ttl 6 ms.
TEST_F(Ak30SessionRuntime, StagesTheCommandWatchdogFollowingHoldingExpired) {
  ASSERT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(1000)),
            AdapterResult::Ok);

  EXPECT_EQ(session_.command_stage(fixtures::at(1000)), CommandStage::Following);
  EXPECT_EQ(session_.command_stage(fixtures::at(4000999)), CommandStage::Following);
  EXPECT_EQ(session_.command_stage(fixtures::at(4001000)), CommandStage::Holding);
  EXPECT_EQ(session_.command_stage(fixtures::at(6000999)), CommandStage::Holding);
  EXPECT_EQ(session_.command_stage(fixtures::at(6001000)), CommandStage::Expired);
}

TEST_F(Ak30SessionRuntime, ReportsExpiredBeforeAnyCommandHasBeenSubmitted) {
  EXPECT_EQ(session_.command_stage(fixtures::at(1000)), CommandStage::Expired);
}

// The failure this guards is a position command resolving to 0.0, which on a
// position interface is a commanded move to the zero position. The session
// emits nothing it was not given, so an expired watchdog produces silence.
TEST_F(Ak30SessionRuntime, NeverSynthesizesACommandWhenTheWatchdogExpires) {
  ASSERT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(1000)),
            AdapterResult::Ok);
  ASSERT_EQ(transport_.sent().size(), 1U);

  ASSERT_EQ(session_.command_stage(fixtures::at(9000000)), CommandStage::Expired);
  (void)session_.snapshot(fixtures::at(9000000));
  (void)session_.command_stage(fixtures::at(9000000));

  EXPECT_EQ(transport_.sent().size(), 1U);
}

TEST_F(Ak30SessionRuntime, RejectsACommandWhoseDeadlineHasAlreadyPassed) {
  EXPECT_EQ(session_.submit(torque_command(2.0, 1000), fixtures::at(2000)),
            AdapterResult::InvalidCommand);
  EXPECT_TRUE(transport_.sent().empty());
}

TEST_F(Ak30SessionRuntime, RejectsAnUnrepresentableCommandWithoutSending) {
  EXPECT_EQ(session_.submit(torque_command(99.0, 10000000), fixtures::at(1000)),
            AdapterResult::InvalidCommand);
  EXPECT_TRUE(transport_.sent().empty());
}

// A transient backpressure result must not fault the bus. The RC shipped a
// defect where one WouldBlock permanently faulted it; the lease is retried
// instead.
TEST_F(Ak30SessionRuntime, TreatsTransientBackpressureAsRetryableNotFatal) {
  transport_.inject_send_result(TransportResult::WouldBlock);
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(1000)),
            AdapterResult::WouldBlock);
  EXPECT_FALSE(session_.fault_latched());

  transport_.inject_send_result(TransportResult::QueueFull);
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(2000)),
            AdapterResult::WouldBlock);
  EXPECT_FALSE(session_.fault_latched());

  transport_.inject_send_result(TransportResult::Ok);
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(3000)),
            AdapterResult::Ok);
  EXPECT_EQ(transport_.sent().size(), 1U);
}

TEST_F(Ak30SessionRuntime, SurfacesADisconnectedTransport) {
  transport_.inject_send_result(TransportResult::Disconnected);
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(1000)),
            AdapterResult::Disconnected);
}

TEST_F(Ak30SessionRuntime, LatchesAFaultAndRefusesFurtherCommands) {
  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x05U, fixtures::at(1000)),
                             fixtures::at(1000)),
            AdapterResult::Ok);
  EXPECT_TRUE(session_.fault_latched());
  EXPECT_EQ(session_.snapshot(fixtures::at(1000)).status.device_state,
            DeviceState::Fault);
  EXPECT_EQ(session_.snapshot(fixtures::at(1000)).status.raw_fault_code, 0x05U);
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(2000)),
            AdapterResult::Fault);
}

// Latched means latched: a subsequent clean frame is not a recovery signal,
// because the condition that caused the fault is not observable from one frame.
TEST_F(Ak30SessionRuntime, DoesNotClearALatchedFaultOnTheNextCleanFrame) {
  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x02U, fixtures::at(1000)),
                             fixtures::at(1000)),
            AdapterResult::Ok);
  ASSERT_TRUE(session_.fault_latched());

  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x00U, fixtures::at(2000)),
                             fixtures::at(2000)),
            AdapterResult::Ok);
  EXPECT_TRUE(session_.fault_latched());
}

TEST_F(Ak30SessionRuntime, ClearsALatchedFaultOnlyThroughTheLifecycle) {
  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x07U, fixtures::at(1000)),
                             fixtures::at(1000)),
            AdapterResult::Ok);
  ASSERT_TRUE(session_.fault_latched());

  session_.deactivate();
  ASSERT_EQ(session_.activate(), AdapterResult::Ok);
  EXPECT_FALSE(session_.fault_latched());
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(3000)),
            AdapterResult::Ok);
}

// 0x77 is the disable-succeeded acknowledgement. Latching a fault on it would
// turn a successful, deliberate disable into an error state requiring recovery.
TEST_F(Ak30SessionRuntime, DoesNotLatchAFaultOnTheDisableAcknowledgement) {
  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x77U, fixtures::at(1000)),
                             fixtures::at(1000)),
            AdapterResult::Ok);
  EXPECT_FALSE(session_.fault_latched());
  EXPECT_EQ(session_.snapshot(fixtures::at(1000)).status.raw_fault_code, 0x77U);
}

TEST_F(Ak30SessionRuntime, DoesNotLatchAFaultOnAnUnknownStatusByte) {
  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x42U, fixtures::at(1000)),
                             fixtures::at(1000)),
            AdapterResult::Ok);
  EXPECT_FALSE(session_.fault_latched());
  EXPECT_EQ(session_.snapshot(fixtures::at(1000)).status.quality,
            SampleQuality::Degraded);
}
```

- [ ] **Step 2: Run to verify the new tests fail**

Run: `MECH_OUTPUT_ROOT=/tmp/t7 MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh`
Expected: FAIL — the Task 6 stubs return `InvalidConfiguration`.

- [ ] **Step 3: Replace the stubs**

In `src/ak30_force_session.cpp`, replace the stub block with:

```cpp
AdapterResult Ak30ForceControlSession::activate() noexcept {
  if (lifecycle_ == Lifecycle::Unconfigured || !codec_.has_value()) {
    return AdapterResult::InvalidConfiguration;
  }
  fault_latched_ = false;
  last_command_time_.reset();
  lifecycle_ = Lifecycle::Active;
  return AdapterResult::Ok;
}

void Ak30ForceControlSession::deactivate() noexcept {
  if (lifecycle_ == Lifecycle::Active) {
    lifecycle_ = Lifecycle::Ready;
  }
  last_command_time_.reset();
}

AdapterResult Ak30ForceControlSession::submit(
    const mech_control_core::CanonicalDeviceCommand& command,
    mech_control_core::MonotonicTime now) noexcept {
  if (lifecycle_ != Lifecycle::Active || !codec_.has_value()) {
    return AdapterResult::InvalidConfiguration;
  }
  if (fault_latched_) {
    return AdapterResult::Fault;
  }
  // An already-expired deadline is an invalid command, not a reason to invent
  // one. Nothing here ever substitutes a default value for a stale input.
  if (command.deadline.nanoseconds() <= now.nanoseconds()) {
    return AdapterResult::InvalidCommand;
  }

  mech_control_core::RawCanFrame frame{};
  const AdapterResult encoded = codec_->encode(command, logical_bus_, now, frame);
  if (encoded != AdapterResult::Ok) {
    return encoded;
  }

  switch (transport().try_send(frame)) {
    case mech_control_core::TransportResult::Ok:
      last_command_time_ = now;
      return AdapterResult::Ok;
    // Backpressure is retryable. Faulting the bus on a transient WouldBlock is
    // the RC defect this branch exists to avoid.
    case mech_control_core::TransportResult::WouldBlock:
    case mech_control_core::TransportResult::QueueFull:
      return AdapterResult::WouldBlock;
    case mech_control_core::TransportResult::Disconnected:
      return AdapterResult::Disconnected;
    case mech_control_core::TransportResult::Invalid:
      return AdapterResult::InvalidCommand;
    case mech_control_core::TransportResult::Fault:
      fault_latched_ = true;
      return AdapterResult::Fault;
  }
  return AdapterResult::Fault;
}

AdapterResult Ak30ForceControlSession::process(
    const mech_control_core::RawCanFrame& frame,
    mech_control_core::MonotonicTime now) noexcept {
  if (lifecycle_ != Lifecycle::Active || !codec_.has_value()) {
    return AdapterResult::InvalidConfiguration;
  }
  mech_control_core::CanonicalDeviceState decoded{};
  const AdapterResult result = codec_->decode(frame, decoded);
  if (result != AdapterResult::Ok) {
    return result;
  }

  ++sequence_;
  last_state_ = decoded;
  last_feedback_time_ = now;
  if (classify_status(static_cast<std::uint8_t>(decoded.status.raw_fault_code)) ==
      StatusMeaning::Fault) {
    fault_latched_ = true;
  }
  return AdapterResult::Ok;
}

mech_control_core::CanonicalDeviceState Ak30ForceControlSession::snapshot(
    mech_control_core::MonotonicTime now) const noexcept {
  mech_control_core::CanonicalDeviceState state = last_state_;
  if (!last_feedback_time_.has_value()) {
    // No sample yet. StatusSnapshot::create() requires Unknown quality to carry
    // no host_rx_time, which a default-constructed snapshot already satisfies.
    state.status = mech_control_core::StatusSnapshot{};
    state.status.device_state = fault_latched_
                                    ? mech_control_core::DeviceState::Fault
                                    : (lifecycle_ == Lifecycle::Active
                                           ? mech_control_core::DeviceState::Active
                                           : mech_control_core::DeviceState::Ready);
    return state;
  }

  state.status.sequence = sequence_;
  const auto age = mech_control_core::elapsed_since(last_feedback_time_.value(), now);
  const bool stale =
      !age.has_value() || age->nanoseconds() > config_.feedback_ttl_nanoseconds;
  if (stale) {
    state.status.quality = mech_control_core::SampleQuality::Stale;
  }
  state.status.device_state = fault_latched_
                                  ? mech_control_core::DeviceState::Fault
                                  : (lifecycle_ == Lifecycle::Active
                                         ? mech_control_core::DeviceState::Active
                                         : mech_control_core::DeviceState::Ready);
  return state;
}

CommandStage Ak30ForceControlSession::command_stage(
    mech_control_core::MonotonicTime now) const noexcept {
  if (!last_command_time_.has_value()) {
    return CommandStage::Expired;
  }
  const auto age = mech_control_core::elapsed_since(last_command_time_.value(), now);
  if (!age.has_value()) {
    return CommandStage::Expired;
  }
  // Strict `<` on both bounds. The tests pin the transition instants exactly:
  // submitted at t=1000 ns with ttl 4 ms, age 3999999 is the last Following
  // sample and age 4000000 is the first Holding one. Using `<=` here shifts
  // every boundary by one nanosecond and fails those assertions.
  if (age->nanoseconds() < config_.command_ttl_nanoseconds) {
    return CommandStage::Following;
  }
  if (age->nanoseconds() < config_.command_hard_ttl_nanoseconds) {
    return CommandStage::Holding;
  }
  return CommandStage::Expired;
}
```

Add `#include "mech_control_core/time.hpp"` if `elapsed_since` is not already visible through the existing includes.

- [ ] **Step 4: Run to verify all tests pass**

Run: `MECH_OUTPUT_ROOT=/tmp/t7 MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh`
Expected: PASS, 0 failures.

Watch the boundary arithmetic: the watchdog test submits at `t=1000` ns, so `Following` holds until `1000 + 4000000 = 4001000` exclusive. If `StagesTheCommandWatchdogFollowingHoldingExpired` is off by one, the comparison is `<` where it should be `<=`, or vice versa.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/mech_protocol_cubemars
git commit -m "feat(protocol): implement the AK3.0 force-control session runtime

Staged watchdog per ADR-012, latching faults that only the lifecycle clears,
and transient transport backpressure mapped to a retryable result rather than
a bus fault. The session emits nothing it was not given, so an expired
watchdog produces silence rather than a synthesized zero command.

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 8: Integration against the simulator's fake transport

Tasks 6–7 used a purpose-built double with an injectable send result. This task runs the same session against `mech_simulation::FakeTransport`, the transport the rest of the repository already trusts, through its real queue-backed paths. `FakeTransport::try_send` returns `Disconnected` when the transport is not open, so the test must `open()` it — that is itself worth asserting.

**Files:**
- Test: `test/test_ak30_simulation_integration.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: everything from Tasks 2–7, plus `mech_simulation::FakeTransport` (`open`, `try_send`, `try_receive`, `inject_receive`, `take_transmit`, `pending_transmit`, `pending_receive`, `force_next_send_results`, `stats`, `default_capabilities`).
- Produces: nothing consumed by later tasks.

- [ ] **Step 1: Write the failing test**

`test/test_ak30_simulation_integration.cpp`:

```cpp
#include <gtest/gtest.h>

#include "ak30_test_fixtures.hpp"
#include "mech_simulation/fake_transport.hpp"

namespace {

using mech::mech_control_core::AdapterResult;
using mech::mech_control_core::CanonicalDeviceCommand;
using mech::mech_control_core::RawCanFrame;
using mech::mech_control_core::SampleQuality;
using mech::mech_control_core::TransportResult;
using mech::mech_protocol_cubemars::Ak30ForceControlSession;
using mech::mech_simulation::FakeTransport;
namespace fixtures = mech::mech_protocol_cubemars::testing;

// The simulator's capabilities advertise CAN FD and BRS as well as Classic
// extended. Our configure() must accept a superset, not demand an exact match.
TEST(Ak30Simulation, ConfiguresAgainstTheSimulatorsAdvertisedCapabilities) {
  FakeTransport transport;
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};
  EXPECT_EQ(session.configure(fixtures::valid_device_config(),
                              FakeTransport::default_capabilities()),
            AdapterResult::Ok);
}

TEST(Ak30Simulation, CompletesACommandFeedbackRoundTripWithNoRealHardware) {
  FakeTransport transport;
  ASSERT_TRUE(transport.open());
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};
  ASSERT_EQ(session.configure(fixtures::valid_device_config(),
                              FakeTransport::default_capabilities()),
            AdapterResult::Ok);
  ASSERT_EQ(session.activate(), AdapterResult::Ok);

  CanonicalDeviceCommand command{};
  command.effort = 2.0;
  command.deadline = fixtures::at(10000000);
  ASSERT_EQ(session.submit(command, fixtures::at(1000)), AdapterResult::Ok);

  // The frame really went through the simulator's transmit queue.
  ASSERT_EQ(transport.pending_transmit(), 1U);
  RawCanFrame transmitted{};
  ASSERT_TRUE(transport.take_transmit(transmitted));
  EXPECT_EQ(transmitted.id.value, fixtures::kCommandId);
  EXPECT_EQ(transmitted.payload_size, 8U);
  EXPECT_EQ(transmitted.payload[6], 0xF9U);
  EXPECT_EQ(transmitted.payload[7], 0x10U);
  EXPECT_EQ(transport.stats().tx_frames, 1U);

  // Feedback arrives the same way a real backend would deliver it.
  ASSERT_EQ(transport.inject_receive(
                fixtures::feedback_frame(0x00U, fixtures::at(2000))),
            TransportResult::Ok);
  RawCanFrame received{};
  ASSERT_EQ(transport.try_receive(received), TransportResult::Ok);
  ASSERT_EQ(session.process(received, fixtures::at(2000)), AdapterResult::Ok);

  const auto state = session.snapshot(fixtures::at(2000));
  EXPECT_NEAR(state.effort, 1.4764, 1e-12);
  EXPECT_EQ(state.status.quality, SampleQuality::Valid);
  EXPECT_EQ(state.status.sequence, 1U);
}

// A closed transport is a disconnect, not a silent no-op. If this ever returns
// Ok, the session is reporting a command as sent that never left the process.
TEST(Ak30Simulation, ReportsDisconnectedWhenTheTransportWasNeverOpened) {
  FakeTransport transport;
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};
  ASSERT_EQ(session.configure(fixtures::valid_device_config(),
                              FakeTransport::default_capabilities()),
            AdapterResult::Ok);
  ASSERT_EQ(session.activate(), AdapterResult::Ok);

  CanonicalDeviceCommand command{};
  command.effort = 2.0;
  command.deadline = fixtures::at(10000000);
  EXPECT_EQ(session.submit(command, fixtures::at(1000)),
            AdapterResult::Disconnected);
  EXPECT_FALSE(session.fault_latched());
}

TEST(Ak30Simulation, SurfacesQueueFullAsBackpressureAndRecovers) {
  FakeTransport transport;
  ASSERT_TRUE(transport.open());
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};
  ASSERT_EQ(session.configure(fixtures::valid_device_config(),
                              FakeTransport::default_capabilities()),
            AdapterResult::Ok);
  ASSERT_EQ(session.activate(), AdapterResult::Ok);

  CanonicalDeviceCommand command{};
  command.effort = 2.0;
  command.deadline = fixtures::at(10000000);

  transport.force_next_send_results({TransportResult::QueueFull});
  EXPECT_EQ(session.submit(command, fixtures::at(1000)), AdapterResult::WouldBlock);
  EXPECT_FALSE(session.fault_latched());
  EXPECT_EQ(transport.stats().queue_full, 1U);

  // Forced results are consumed once, so the next call takes the normal path.
  EXPECT_EQ(session.submit(command, fixtures::at(2000)), AdapterResult::Ok);
  EXPECT_EQ(transport.pending_transmit(), 1U);
}

// Filling the queue for real, rather than forcing a result, checks that the
// session survives genuine backpressure from the transport's own accounting.
TEST(Ak30Simulation, SurvivesGenuineQueueSaturation) {
  FakeTransport transport{2U};
  ASSERT_TRUE(transport.open());
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};
  ASSERT_EQ(session.configure(fixtures::valid_device_config(),
                              FakeTransport::default_capabilities()),
            AdapterResult::Ok);
  ASSERT_EQ(session.activate(), AdapterResult::Ok);

  CanonicalDeviceCommand command{};
  command.effort = 2.0;
  command.deadline = fixtures::at(10000000);

  EXPECT_EQ(session.submit(command, fixtures::at(1000)), AdapterResult::Ok);
  EXPECT_EQ(session.submit(command, fixtures::at(2000)), AdapterResult::Ok);
  EXPECT_EQ(session.submit(command, fixtures::at(3000)), AdapterResult::WouldBlock);
  EXPECT_FALSE(session.fault_latched());

  RawCanFrame drained{};
  ASSERT_TRUE(transport.take_transmit(drained));
  EXPECT_EQ(session.submit(command, fixtures::at(4000)), AdapterResult::Ok);
}

// A frame for a different drive on a shared bus must be ignored without
// disturbing this session's sample or sequence.
TEST(Ak30Simulation, IgnoresAnotherDrivesFeedbackOnTheSameBus) {
  FakeTransport transport;
  ASSERT_TRUE(transport.open());
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};
  ASSERT_EQ(session.configure(fixtures::valid_device_config(),
                              FakeTransport::default_capabilities()),
            AdapterResult::Ok);
  ASSERT_EQ(session.activate(), AdapterResult::Ok);

  auto foreign = fixtures::feedback_frame(0x00U, fixtures::at(1000));
  foreign.id = mech::mech_control_core::CanId::create(
                   0x2969U, mech::mech_control_core::CanFrameFormat::Extended)
                   .value();
  ASSERT_EQ(transport.inject_receive(foreign), TransportResult::Ok);
  RawCanFrame received{};
  ASSERT_EQ(transport.try_receive(received), TransportResult::Ok);

  EXPECT_EQ(session.process(received, fixtures::at(1000)),
            AdapterResult::InvalidCommand);
  EXPECT_EQ(session.snapshot(fixtures::at(1000)).status.sequence, 0U);
  EXPECT_FALSE(session.fault_latched());
}

}  // namespace
```

- [ ] **Step 2: Run to verify it fails, then passes**

Add `test/test_ak30_simulation_integration.cpp` to `CMakeLists.txt`.

Run: `MECH_OUTPUT_ROOT=/tmp/t8 MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh`
Expected first: FAIL to compile if `mech_simulation` is not linked into the test target — confirm `ament_target_dependencies(${PROJECT_NAME}_test mech_simulation)` from Task 1 is present. Then: PASS, 0 failures.

- [ ] **Step 3: Confirm no device was touched**

Run: `grep -rn "ttyACM\|socketcan\|can0\|vcan\|AF_CAN\|SIOCGIFINDEX" ros2_ws/src/mech_protocol_cubemars/`
Expected: no matches. The package must contain no transport implementation at all.

- [ ] **Step 4: Commit**

```bash
git add ros2_ws/src/mech_protocol_cubemars
git commit -m "test(protocol): run the AK3.0 session against the simulator transport

Exercises the real queue-backed FakeTransport paths, including genuine queue
saturation and a never-opened transport, with no CAN or serial access.

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 9: Documentation, vendor questions, and full verification

The code is worthless if the next reader restores the vendor formula. This task writes down why it is the way it is, in the places a reader will actually look.

**Files:**
- Modify: `docs/development/ak30_force_control_adapter_design.md` (status line, §4.3, §8, §10)
- Modify: `docs/planning/06_cubemars_material_review.md` (the AK3.0 force-control section)
- Modify: `company/vendor_questions_2026-08-31.md` (**local, git-ignored** — add B15)
- Modify: `company/motor1/motor1_parameters.md` (**local, git-ignored** — §9)
- Modify: `ros2_ws/src/mech_protocol_cubemars/README.md` if Task 2–7 changed any stated behaviour

- [ ] **Step 1: Update the design document**

Change the status line from `Draft for review` to:

```markdown
- **Status:** Implemented — `mech_protocol_cubemars`, offline only. Real activation remains gated by ADR-006 and G0–G3.
```

Add to §4.3, immediately after the normalization table:

```markdown
**Two defects in L07 that the implementation deliberately does not reproduce.**

1. §4.4's printed `float_to_uint()` scales by `(1 << bits) / span`. The manual's
   own example table was generated with `((1 << bits) - 1) / span`, and the
   printed formula reproduces none of the manual's examples: position `0 rad`
   becomes `80 00` instead of `7F FF`, velocity `0` becomes `0x800` instead of
   `0x7FF`, and velocity `-6 rad/s` becomes `64 98` instead of `64 87`. At
   `p_des = P_MAX` it yields `p_int = 65536`, which overflows the 16-bit field
   to `0x0000` and decodes as `-12.56 rad` — **maximum position commands
   minimum position**. `EncodingMaxPositionDoesNotWrapToMinimum` pins this.
2. §4.4's worked examples are stated to use **AK10-9** constants (`±12.56 rad`,
   `±28.0 rad/s`, `±54.0 N·m`), not AKE60-8's. Decoding an example row with
   AKE60-8's ranges gives a wrong answer, which is why the test suite carries
   `ak10_9_ranges()` purely for manual cross-checking.

Quantization therefore truncates with divisor `((1 << bits) - 1)`, and encoding
**rejects** out-of-range input rather than clamping as the vendor code does:
clamping converts an invalid command into a valid-looking one, which is the
degradation ADR-012 forbids.
```

Add to §10:

```markdown
- **Which shaft the command velocity refers to** — L07 documents the wire
  command velocity only as `电机速度 (rad/s)`, while feedback is ERPM requiring
  `÷ pole_pairs ÷ gear_ratio`. The implementation assumes output-side, matching
  the torque field, which the manual does state is 输出端. Vendor question B15.
  Gated behind the same `gear_ratio` evidence requirement, so it cannot reach a
  device unanswered.
```

- [ ] **Step 2: Record the defects in the planning document**

Add to `docs/planning/06_cubemars_material_review.md`'s AK3.0 force-control section a short subsection titled `L07 的两处缺陷（2026-09-02 数值验证）` stating both defects and pointing at the design doc §4.3 for detail. Keep it to a few lines — the design doc is the authority, and duplicating it invites drift.

- [ ] **Step 3: Add vendor question B15 to the local list**

In `company/vendor_questions_2026-08-31.md`, in the motor (B) section:

```markdown
- **B15 力控命令速度字段的参考轴。** 力控命令帧的速度字段在手册中只写作
  「电机速度（rad/s）」，而 `0x29` 反馈是电气转速 ERPM，需要 `÷极对数÷减速比`
  才得到输出端转速。请确认命令速度是**输出端**还是**电机端** rad/s。扭矩字段
  手册已明确为输出端；若速度字段与之不一致，按 8:1 减速比会产生 8 倍误差。
```

- [ ] **Step 4: Update the local motor1 record**

In `company/motor1/motor1_parameters.md` §9, note that the force-control codec is implemented and that `direction_sign` (B9) is the only parameter standing between motor1 and a torque sub-mode that configures. Do not restate the manual defects — link to the design doc.

- [ ] **Step 5: Run the full verification set**

```bash
python3 tools/ci/context_check.py
python3 tools/ci/check_adrs.py
git diff --check
MECH_OUTPUT_ROOT=/tmp/t9 MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh
bash tools/ci/run_sanitizers.sh /tmp/t9-sanitizers
```

Expected: `context_check.py` PASS with **6** package skeletons; `check_adrs.py` PASS with 9 decisions, 8 Accepted, 1 Proposed; `git diff --check` clean; 6 packages built; **118 pre-existing tests plus this package's, 0 failures**; no ASan/UBSan reports.

**Before claiming the sanitizer run is clean, confirm sanitizers were actually enabled** — grep the CMake cache for `-fsanitize=address,undefined`. A clean grep for ASan output proves nothing if the build never had the flag.

- [ ] **Step 6: Commit and open the PR**

```bash
git add docs ros2_ws/src/mech_protocol_cubemars
git commit -m "docs(adapter): record the L07 defects the codec does not reproduce

Co-Authored-By: Claude <noreply@anthropic.com>"

git -c http.proxy=http://127.0.0.1:12000 -c https.proxy=http://127.0.0.1:12000 push -u origin feat/ak30-force-control-codec
HTTPS_PROXY=http://127.0.0.1:12000 gh pr create --fill
```

The PR body must state, because none of it is visible in the diff:

1. The two L07 defects and that the golden vectors were back-solved, not copied.
2. That firmware validation lives in the package-local session config because adding a `DeviceConfig` field would be a canonical contract change requiring an ADR first.
3. That the staged watchdog is now duplicated a third time, and that ADR-012 already records unifying it as an open review trigger.
4. The new unverified assumption about the command velocity's shaft, filed as vendor question B15.
5. That **no sub-mode configures with motor1's current evidence** — this PR does not bring the project closer to moving a motor; it brings it closer to being *ready* to, once B9 answers.

---

## Self-Review notes

Run after all nine tasks were written, against the spec with fresh eyes.

**Spec coverage.** design §1 scope → Task 1; §2 constraints → enforced in Tasks 6–7; §3 layering → the file structure; §4.1 identifier → Task 2 `force_control_can_id`; §4.2 payload → Task 2; §4.3 normalization → Task 2; §4.4 feedback → Task 3; §5 mapping and hard rules → Task 4; §6 configure → Task 6; §7 session → Task 7; §8 test groups → Tasks 2–8 (golden, identifier, negative, command validation, status byte, mapping gate, session, cross-profile, integration all have a home); §9 registration → Task 1. §10's open items stay open by design — they are exactly what the evidence gate blocks on, and Task 9 adds B15 to that list rather than closing any of them.

**Defect found and fixed during this review.** Task 7's `command_stage` originally used `<=` on both bounds, which contradicts the transition instants asserted in `StagesTheCommandWatchdogFollowingHoldingExpired` by one nanosecond. Changed to strict `<` and a comment added explaining why, since the next reader's instinct will be to "fix" it back.

**Type consistency, checked pairwise across task boundaries.** `ForceControlPayload` (Task 2) is consumed by Tasks 3 and 5 under that exact name. `decode_feedback` returns `void`, and Task 5 calls it as a statement. `Ak30Mapping`'s seven fields are spelled identically in Tasks 4, 5, 6 and the fixtures. `Ak30SessionConfig`'s `command_ttl_nanoseconds` / `command_hard_ttl_nanoseconds` / `feedback_ttl_nanoseconds` are spelled out in full everywhere rather than abbreviated in one place and not another. The codec constructor's four parameters match `codec_.emplace(...)` in Task 6 in both order and type.

**Decisions that were open when Tasks 1–3 were written, now closed.**

1. *Torque sub-mode ignoring unused command fields* — resolved as ignore-and-document, pinned by `TorqueSubModeIgnoresPositionAndVelocityByDesign`. Rejecting them would fight ros2_control, which writes every claimed interface every cycle.
2. *Watchdog duplication* — accepted. `CommandStage` restates `mech_controllers::WatchdogStage` because depending on `mech_controllers` would invert the dependency direction and break the `EXPECTED_PACKAGES` boundary. This is the third copy; the PR body must say so, and ADR-012 already lists unification as an open review trigger.
3. *Firmware validation* — lives in the package-local `Ak30SessionConfig`, because adding a field to `DeviceConfig` is a canonical contract change requiring an ADR before implementation.

**New unverified assumption this plan introduces**, and the only one: the wire command velocity is treated as output-side rad/s. L07 never says. It is gated behind `gear_ratio` evidence, pinned nowhere in a way that could silently drift, and filed as vendor question B15 in Task 9.

**What this plan does not achieve.** With motor1's current evidence every sub-mode still fails to configure. The deliverable is an adapter that is *ready* to configure the moment `direction_sign` (vendor question B9) is answered — not one that can move a motor.
