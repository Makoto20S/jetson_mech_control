"""ROS-independent live state for the receive-only CtrBoard dashboard."""

from collections import deque
from dataclasses import dataclass, field
import math


@dataclass
class ImuState:
    stamp: int = 0
    received_at: float = 0.0
    orientation_valid: bool = False
    euler: tuple[float, float, float] | None = None
    euler_stamp: int = 0

    def fresh(self, now: float, timeout: float = 0.5) -> bool:
        return self.received_at > 0 and now - self.received_at < timeout

    def angles(self, now: float) -> tuple[float, float, float] | None:
        return (self.euler if self.orientation_valid and self.fresh(now)
                and self.euler_stamp == self.stamp else None)


@dataclass
class DashboardState:
    imus: list[ImuState] = field(default_factory=lambda: [ImuState(), ImuState()])
    pressures: list[list[int]] = field(default_factory=lambda: [[0] * 20, [0] * 20])
    pressure_at: list[float] = field(default_factory=lambda: [0.0, 0.0])
    phases: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    packet_at: float = 0.0
    device_time_ms: int = 0
    bridge_state: str = "waiting_for_data"
    bridge_diagnostic_at: float = 0.0
    knee_status: str = "未启动"
    knee_angle: float | None = None
    knee_at: float = 0.0
    history: deque = field(default_factory=lambda: deque(maxlen=600))

    def set_imu(self, index: int, stamp: int, valid: bool, now: float) -> None:
        imu = self.imus[index]
        if stamp < imu.stamp:
            return
        if not valid or imu.euler_stamp != stamp:
            imu.euler = None
        imu.stamp = stamp
        imu.received_at = now
        imu.orientation_valid = valid

    def set_euler(self, index: int, stamp: int, angles: tuple[float, float, float]) -> None:
        imu = self.imus[index]
        if stamp >= imu.stamp and all(map(math.isfinite, angles)):
            imu.euler = angles
            imu.euler_stamp = stamp

    def set_pressure(self, index: int, values: list[int], now: float) -> None:
        if len(values) == 20 and all(0 <= value <= 65535 for value in values):
            self.pressures[index] = values
            self.pressure_at[index] = now

    def set_packet(self, device_time_ms: int, now: float) -> None:
        self.device_time_ms = device_time_ms
        self.packet_at = now

    def packet_fresh(self, now: float) -> bool:
        return self.packet_at > 0 and now - self.packet_at < 0.5

    def bridge_status(self, now: float) -> str | None:
        if self.bridge_diagnostic_at > 0 and now - self.bridge_diagnostic_at < 1.0:
            return self.bridge_state
        return None

    def pressure_values(self, index: int, now: float) -> list[int] | None:
        if self.packet_fresh(now) and now - self.pressure_at[index] < 0.5:
            return self.pressures[index]
        return None

    def live_knee_angle(self, now: float) -> float | None:
        if (self.packet_fresh(now) and self.knee_at > 0 and now - self.knee_at < 0.5
                and all(imu.angles(now) is not None for imu in self.imus)
                and self.knee_status == "calibrated"):
            return self.knee_angle
        return None

    def append_history(self, now: float) -> None:
        first, second = (imu.angles(now) for imu in self.imus)
        self.history.append((now, first[1] if first else None,
                             second[1] if second else None, self.live_knee_angle(now)))
