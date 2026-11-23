// Numerical oracle process only. One retained model instance exercises seed reuse.
#include "ref01_model.hpp"
#include "ref01_impedance_core.hpp"

#include <array>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace core = mech::mech_controllers::ref01;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: ref01_native_probe relative_ascii_model.xml\n";
    return 2;
  }
  try {
    ph_exo::Ref01Model model(argv[1]);
    core::Plan plan{true, true, 1., 1., 1.,
      {0., 20. * core::pi / 180.}, {2.4, 2.}, {.18, .15}, {.2, .2}};
    std::cout << std::setprecision(17);
    std::string line;
    while (std::getline(std::cin, line)) {
      std::istringstream input(line);
      std::array<double, 4> values{};
      bool parsed = true;
      std::string token;
      for (double& value : values) {
        if (!(input >> token)) { parsed = false; break; }
        char* end = nullptr;
        value = std::strtod(token.c_str(), &end);  // NaN reaches the input validator.
        if (end == token.c_str() || *end != '\0') parsed = false;
      }
      plan.dynamic=false;plan.reference={0.,20.*core::pi/180.};plan.reference_velocity={};
      if(input >> token) {
        std::array<double,4> reference{};
        for(unsigned i=0;i<4;++i) {
          if(i>0 && !(input>>token)) {parsed=false;break;}
          char* end=nullptr;reference[i]=std::strtod(token.c_str(),&end);
          if(end==token.c_str() || *end!='\0')parsed=false;
        }
        if(input>>token)parsed=false;
        plan.dynamic=true;plan.reference={reference[0],reference[1]};
        plan.reference_velocity={reference[2],reference[3]};
      }
      ph_exo::ModelState native;
      core::Torque torque;
      if (!parsed || !model.evaluate({values[0], values[1]}, {values[2], values[3]}, native)) {
        std::cout << "0\n" << std::flush;
        continue;
      }
      const core::ModelState state{native.q, native.dq, native.gravity, native.A,
                                   native.residual, native.iterations};
      if (!core::fixed_pose_torque(state, plan, torque)) {
        std::cout << "0\n" << std::flush;
        continue;
      }
      std::cout << 1;
      for (const auto& pair : {state.q, state.dq, state.gravity})
        for (double value : pair) std::cout << ' ' << value;
      for (double value : state.A) std::cout << ' ' << value;
      for (double value : torque.alpha_nm) std::cout << ' ' << value;
      std::cout << ' ' << torque.scale << '\n' << std::flush;
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 3;
  }
}
