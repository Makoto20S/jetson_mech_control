# mech_protocol_ctrboard

Receive-side C++ codec for the public STM32 CtrBoard sensor protocol over
Classic CAN. The package does not open a CAN interface, configure hardware, or
encode host commands.

## Wire contract

- STM32 to host standard CAN ID: `0x621`
- CAN mode: Classic CAN at 1 Mbit/s, maximum payload 8 bytes
- Application packet: `EB 90 | 0x05 | 196-byte payload | CRC16`
- CRC: CRC-16/CCITT-FALSE, initial value `0xFFFF`, stored little-endian
- Multi-frame rule: concatenate CAN payload bytes in receive order

The 196-byte sensor payload contains two IMU samples and two 20-point plantar
pressure samples. A complete 202-byte application packet occupies 26 Classic
CAN frames.

The stream has no fragment sequence number. A dropped or reordered CAN frame
invalidates the packet; the decoder rejects its CRC and searches for the next
`EB 90` header.

Wire structures intentionally match the little-endian, IEEE-754 STM32 layout.
The target Jetson platforms satisfy those assumptions. A big-endian target
would require explicit scalar conversion.
