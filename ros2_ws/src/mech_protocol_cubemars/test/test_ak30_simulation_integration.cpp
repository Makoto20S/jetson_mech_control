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
