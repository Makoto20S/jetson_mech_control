#pragma once

namespace mech::mech_control_core {

// ADR-017: the per-joint command interface that carries command freshness.
// The hardware always exports it; whether a controller claims it decides which
// protection tier that joint gets. It is deliberately NOT one of the canonical
// motion kinds - it commands no motion. It expresses the one thing a bare
// double* cannot: that the controller spoke again.
//
// It lives in this package rather than beside CompositeSystem because it is a
// contract BETWEEN the hardware and this project's own controllers, and both
// sides have to spell it identically. Putting it in the hardware plugin package
// would force every controller that joins the strong tier to depend on a
// specific hardware plugin, which is the coupling this framework exists to
// avoid - a controller must stay swappable against any conforming hardware.
// Re-typing the literal in each package was the other option and was rejected:
// two copies of a contract string drift silently the first time one is renamed.
inline constexpr char kCommandGenerationInterface[] = "command_generation";

}  // namespace mech::mech_control_core
