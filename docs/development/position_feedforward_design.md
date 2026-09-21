# Position desired velocity and feedforward commands

- Scope: T10 follow-on; owner authorized software implementation and PR on 2026-09-21.
- Real hardware experiments, persistent deployment and merge require later approval.
- Revises ADR-014/017 command shape; retains ADR-015/016 authorization/freshness and
  ADR-019 position envelope. Kp/Kd remain adapter configuration.

## Purpose and alternatives

The drive already accepts position, desired velocity, gains and feedforward torque
in one force-control frame. The Position runtime previously discarded velocity and
effort; its only exported motion interface was position. The extension carries
optional desired velocity and effort through the generic hardware boundary without
moving vendor encoding into controllers.

Selected: explicitly configured bundles of standard command interfaces. A universal
always-exported superset was rejected because it obscures intent and permits partial
ownership. New vendor-specific command names and a new full impedance controller are
unnecessary for this slice. Existing position-only deployments remain unchanged.

## Interface contract

Allowed motion bundles per joint:

| Bundle | Intended use |
|---|---|
| position | Existing position controllers and JTC deployment |
| velocity | Existing velocity mode |
| effort | Existing torque mode |
| position + velocity | Position trajectory with desired speed, including Humble JTC |
| position + effort | Position with feedforward torque, compatible controller |
| position + velocity + effort | Full Position tuple, compatible controller |

Each also declares command_generation; claiming it is optional under ADR-017.
State interface shape remains position/velocity/effort. Unsupported feedback fields
retain their documented availability limits; export does not invent measurements.

Whole motion bundles must be acquired/released in one transaction. Partial starts,
partial stops, duplicate/unknown names and orphan generation claims are rejected
without altering live state. One controller must own the bundle. Controller-manager
callbacks contain aggregate interface names, so hardware cannot identify which
controller supplied each member of a simultaneous multi-controller switch; correct
single-controller deployment is a required contract, not a hardware-enforced proof.

One update writes the tuple before hardware write snapshots it. Strong-tier owners
write all fields then advance one generation for the tuple. Weak-tier owners inherit
the existing limitation that every active manager cycle counts as a refresh. This
interface does not detect a controller updating position while leaving an old speed
or feedforward value; coherent updates are the owning controller's responsibility.

## Lifecycle and mapping

On new Position claim, auxiliary command buffers are zeroed. Weak-tier position is
seeded from accepted feedback to cover the manager switch-cycle before JTC writes.
Strong-tier claims still wait for a changed generation. Release/deactivate/cleanup/
error clear auxiliary buffers and cancel pending commands per joint. No position-zero
command is synthesized; reactivation cannot revive an old auxiliary target.

Position runtime accepts all three canonical values and maps them into one AK3.0
frame. Velocity/Torque single-mode semantics remain unchanged. Kp/Kd are fixed runtime
configuration and cannot be changed by claiming an interface.

## Bounds

- position_min_rad, position_max_rad, position_max_error_rad: existing position gate.
- position_max_abs_velocity_rad_s: maximum absolute desired velocity, default zero.
- position_max_abs_feedforward_nm: maximum absolute feedforward torque, default zero.

Auxiliary limits must be finite, nonnegative and within the protocol ranges. Exporting
an auxiliary interface requires its explicit positive bound. Zero means disabled,
not unlimited. Nonfinite or out-of-bound Position auxiliary commands reject/latch the
entire tuple, cancel pending output and require lifecycle recovery. The appended
PositionTuple telemetry reason is 13; existing values remain stable. No clipping.
Existing feedback checks, absolute deadline and position envelope remain in force.

These bounds are on commanded values. The drive's nominal law is
`Kp*(p_des-p) + Kd*(v_des-v) + torque_ff`; neither desired-speed nor feedforward limits
nor the position-error envelope establish a total physical torque or speed bound.

## Standard controller compatibility and deployment

Humble JTC supports position+velocity but requires effort to be used alone. Therefore
it cannot drive the full position+velocity+effort bundle. Existing position-only YAML
remains unchanged. New examples must declare precisely the bundle their controller
claims, remain inactive by default and never suggest launching against an unverified
real device. Full-tuple integration is proven using a single controller-manager test
controller and FakeTransport; no production gravity/impedance controller is added.

Switching between different declared bundles requires an inactive reconfiguration;
this slice does not introduce active protocol/sub-mode switching. Pure position
controllers use the existing position-only deployment, not a partial claim of a
full-tuple deployment.

## Verification and acceptance

- Generic shape/claim transactions for all legal bundles, rejection atomicity,
  slash-containing joint names, weak takeover, strong generation and auxiliary reset.
- Runtime/parser tests for defaults, declared auxiliary limits, nonfinite/boundary
  values, no TX on rejection, retained latch, fresh-feedback and retry/TTL semantics.
- Decode actual FakeTransport frames for position+velocity+effort and fixed gains;
  verify a real JTC position+velocity path and old single-interface paths.
- Fresh local complete build/tests, sanitizer checks and independent review.
  Report native ARM64 as not run if unavailable; do not reuse historical counts.
- No real motion, serial device access or deployment. Physical tracking and stop
  behavior remain unqualified until a separately approved experiment.
