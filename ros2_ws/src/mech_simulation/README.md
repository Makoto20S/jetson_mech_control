# mech_simulation

Fake-clock, fake-transport and reference-device package boundary. `FakeClock`
advances only through explicit virtual time, and `FakeTransport` exposes
bounded RX/TX queues with deterministic disconnect and queue-full results. This
package remains hardware-independent and contains no supplier defaults.
