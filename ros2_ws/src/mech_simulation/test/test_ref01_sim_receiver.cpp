#include "mech_simulation/ref01_sim_receiver.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

using mech::mech_simulation::Ref01SimReceiver;
void require(bool condition, const char* description) {
  if (!condition) throw std::runtime_error(description);
}

int main() {
  try {
    Ref01SimReceiver guard;
    require(!guard.accept({0., 0., 0., 0.}, 1), "startup is not an accepted command");
    require(!guard.started() && !guard.latched(), "startup state");
    require(guard.accept({.2, 1., -.2, 1.}, 100), "boundary effort accepted");
    require(guard.effort()[0] == .2 && guard.effort()[1] == -.2, "effort order");
    require(!guard.accept({0., 1., 0., 1.}, 1000000), "duplicate payload not applied");
    require(guard.last_accept_ns() == 100 && guard.effort()[0] == .2,
            "duplicate does not extend lease or change effort");
    require(guard.accept({.1, 2., -.1, 2.}, 2000000), "fresh pair accepted");
    require(!guard.accept({0., 2., 0., 2.}, 61999999), "lease alive until boundary");
    require(!guard.latched(), "not expired early");
    require(!guard.accept({0., 3., 0., 3.}, 62000000), "deadline checked before new generation");
    require(guard.latched() && guard.zero_ns() == 62000000 &&
            guard.effort()[0] == 0. && guard.effort()[1] == 0., "expiry latches zero");
    require(!guard.accept({.1, 4., .1, 4.}, 63000000), "recovery cannot unlatch");

    const std::array<std::array<double, 4>, 7> invalid{{
        {.21, 1., 0., 1.}, {0., 1., -.21, 1.}, {0., 1., 0., 2.},
        {0., .5, 0., .5}, {0., -1., 0., -1.},
        {std::numeric_limits<double>::quiet_NaN(), 1., 0., 1.},
        {0., std::numeric_limits<double>::infinity(), 0., 1.}}};
    for (const auto& command : invalid) {
      guard.reset();
      require(!guard.accept(command, 1) && guard.latched(), "malformed command fails closed");
    }
    guard.reset();
    require(guard.accept({0., 2., 0., 2.}, 100), "initial nonunit positive generation");
    require(!guard.accept({0., 1., 0., 1.}, 101) && guard.latched(), "generation rollback");
    guard.reset();
    require(guard.accept({0., 1., 0., 1.}, 100), "fresh after explicit reset");
    require(!guard.accept({0., 2., 0., 2.}, 99) && guard.latched(), "clock rollback");
    guard.reset();
    require(guard.accept({0., Ref01SimReceiver::kMaxExactGeneration,
                          0., Ref01SimReceiver::kMaxExactGeneration}, 1),
            "exact generation limit");
    require(!guard.accept({0., Ref01SimReceiver::kMaxExactGeneration + 1.,
                           0., Ref01SimReceiver::kMaxExactGeneration + 1.}, 2) &&
            guard.latched(), "inexact generation is rejected");
    std::cout << "Ref01SimReceiver checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
