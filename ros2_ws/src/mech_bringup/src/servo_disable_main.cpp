// Shutdown-only device operation. The caller must first stop/join the framework
// and retain its operator lock. Never construct a controller or submit mode 6.
#include "mech_bringup/ak30_servo_runtime_params.hpp"
#include "mech_bringup/command_trace.hpp"
#include "mech_bringup/posix_cdc_serial_port.hpp"
#include "mech_protocol_cubemars/ak30_servo_wire.hpp"
#include "hardware_interface/component_parser.hpp"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <set>
#include <thread>

int main(int argc, char** argv) {
  using namespace mech::mech_control_core;
  using namespace mech::mech_bringup;
  using namespace mech::mech_protocol_cubemars;
  CommandTrace trace(8192);
  std::ofstream output;
  int result = 2;
  try {
    std::string path, trace_path;
    double timeout = 2.;
    bool check = false;
    for (int i = 1; i < argc; ++i) {
      const std::string arg(argv[i]);
      if (arg == "--urdf" && i+1 < argc) path = argv[++i];
      else if (arg == "--trace" && i+1 < argc) trace_path = argv[++i];
      else if (arg == "--check") check = true;
      else if (arg == "--timeout" && i+1 < argc) {
        std::size_t end = 0;
        const std::string value(argv[++i]);
        timeout = std::stod(value, &end);
        if (end != value.size() || !std::isfinite(timeout) || timeout <= 0 || timeout > 3)
          throw std::runtime_error("timeout must be within (0,3] seconds");
      } else throw std::runtime_error("usage: servo_disable --urdf FILE --trace FILE [--check] [--timeout SECONDS]");
    }
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot read URDF");
    const std::string xml((std::istreambuf_iterator<char>(file)), {});
    const auto resources = hardware_interface::parse_control_resources_from_urdf(xml);
    if (resources.size() != 1 || resources[0].hardware_class_type != "mech_bringup/Ak30ServoSystem")
      throw std::runtime_error("expected one Ak30ServoSystem");
    const auto parsed = Ak30ServoRuntimeParams::parse(resources[0]);
    if (!parsed) throw std::runtime_error("invalid servo configuration");
    std::set<int> missing;
    for (const auto& joint : parsed->config.joints) missing.insert(joint.drive_id);
    if (missing.empty()) throw std::runtime_error("no motors configured");
    if (check) { std::cout << "{\"event\":\"check\",\"configuration_valid\":true}\n"; return 0; }
    if (trace_path.empty()) throw std::runtime_error("--trace is required");
    output.open(trace_path);
    if (!output) std::cerr << "warning: cannot open disable trace; proceeding with shutdown\n";
    PosixCdcSerialPort serial(parsed->device_path);
    serial.set_command_trace(&trace);
    UsbCdcOptions options;
    options.logical_bus = parsed->config.logical_bus;
    options.verified_board_version = {4,8,8};
    options.receive_queue_capacity = 128;
    UsbCdcTransport transport(serial, options);
    if (!transport.open()) throw std::runtime_error("port unavailable or occupied");
    if (!serial.send_pass_through_init()) throw std::runtime_error("gateway initialization failed");
    bool all_sent = true;
    for (int id : missing) {
      RawCanFrame frame{};
      const auto now = *MonotonicTime::from_nanoseconds(CommandTrace::now());
      if (!encode_servo_disable(id, options.logical_bus, now, frame))
        throw std::runtime_error("invalid disable frame");
      trace.frame("disable_request", frame);
      const auto sent = transport.try_send(frame);
      trace.frame("disable_result", frame, static_cast<int>(sent));
      all_sent = all_sent && sent == TransportResult::Ok;
      std::cout << "{\"event\":\"disable_sent\",\"id\":" << id
                << ",\"write_ok\":" << (sent == TransportResult::Ok ? "true" : "false") << "}" << std::endl;
      // Do not retry a possibly partial USB packet, or skip the other motor.
    }
    const auto deadline = CommandTrace::now() + static_cast<std::int64_t>(timeout*1e9);
    while (!missing.empty() && CommandTrace::now() < deadline) {
      RawCanFrame frame{};
      const auto received = transport.try_receive(frame);
      if (received == TransportResult::Ok) {
        trace.frame("disable_feedback", frame);
        const int id = static_cast<int>(frame.id.value & 255U);
        ServoFeedback sample{};
        if (missing.count(id) && frame.logical_bus == options.logical_bus &&
            decode_servo_feedback(id, frame, sample) &&
            sample.status == ServoFeedbackStatus::DisableAcknowledged) {
          missing.erase(id);
          std::cout << "{\"event\":\"disable_ack\",\"id\":" << id
                    << ",\"status\":119,\"position_deg\":" << sample.position_deg
                    << ",\"erpm\":" << sample.electrical_speed_erpm << "}" << std::endl;
        }
      } else if (received == TransportResult::WouldBlock) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      } else break;
    }
    transport.close();
    const bool confirmed = all_sent && missing.empty();
    std::cout << "{\"event\":\"disable_complete\",\"confirmed\":"
              << (confirmed ? "true" : "false") << ",\"missing\":[";
    bool first = true;
    for (int id : missing) { if (!first) std::cout << ','; first = false; std::cout << id; }
    std::cout << "]}" << std::endl;
    result = confirmed ? 0 : 2;
  } catch (const std::exception& error) {
    std::cerr << "servo_disable: " << error.what() << '\n';
  }
  if (output.is_open()) {
    trace.dump(output); output.flush();
    if (!output) std::cerr << "warning: disable trace save failed; see acknowledgement output\n";
  }
  return result;
}
