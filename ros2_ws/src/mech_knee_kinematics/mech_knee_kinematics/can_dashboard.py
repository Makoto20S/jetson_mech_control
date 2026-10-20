"""Read-only Tk dashboard for CtrBoard CAN telemetry published through ROS 2."""

import csv
from datetime import datetime
import math
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, ttk

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, UInt16MultiArray, UInt32, UInt8MultiArray
from std_srvs.srv import Trigger

from .dashboard_state import DashboardState


COLORS = ("#147d9e", "#d66b32", "#268a58")
BACKGROUND = "#f7f9fa"
BRIDGE_LABELS = {
    "waiting_for_data": "诊断待更新",
    "healthy": "双 IMU 正常",
    "degraded": "传感器数据不完整",
    "transport_fault": "传输故障",
    "timed_out": "数据超时",
}
KNEE_LABELS = {
    "not calibrated": "未标定",
    "axis calibration required": "需屈伸轴标定",
    "calibrated": "标定完成",
    "IMU pair timed out": "双 IMU 数据超时",
    "calibrating static": "正在采集站立零位",
    "calibrating axis": "正在采集屈伸动作轴",
}


def stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class DashboardSubscriber(Node):
    def __init__(self, events):
        super().__init__("ctrboard_can_dashboard")
        self.events = events
        for index in range(2):
            name = f"imu{index + 1}"
            self.create_subscription(
                Imu, f"/imu/{name}/data",
                lambda msg, i=index: self.emit("imu", i, stamp_ns(msg),
                                               msg.orientation_covariance[0] >= 0),
                qos_profile_sensor_data)
            self.create_subscription(
                Vector3Stamped, f"/imu/{name}/euler_deg",
                lambda msg, i=index: self.emit("euler", i, stamp_ns(msg),
                                               (msg.vector.x, msg.vector.y, msg.vector.z)),
                qos_profile_sensor_data)
            side = "left" if index == 0 else "right"
            self.create_subscription(
                UInt16MultiArray, f"/fsr/{side}/raw",
                lambda msg, i=index: self.emit("pressure", i, list(msg.data)),
                qos_profile_sensor_data)
        self.create_subscription(UInt32, "/ctrboard/timestamp_ms",
                                 lambda msg: self.emit("packet", msg.data), qos_profile_sensor_data)
        self.create_subscription(UInt8MultiArray, "/ctrboard/sensor_status",
                                 lambda msg: self.emit("phase", list(msg.data)), qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, "/ctrboard/diagnostics",
                                 lambda msg: self.emit("diagnostic", "bridge", msg), 10)
        self.create_subscription(DiagnosticArray, "/knee/diagnostics",
                                 lambda msg: self.emit("diagnostic", "knee", msg), 10)
        self.create_subscription(Float64, "/knee/flexion_deg",
                                 lambda msg: self.emit("knee_angle", msg.data), 10)
        self.calibration_clients = {
            "zero": self.create_client(Trigger, "/knee/start_static_calibration"),
            "axis": self.create_client(Trigger, "/knee/start_axis_calibration"),
            "reset": self.create_client(Trigger, "/knee/reset_calibration"),
        }

    def emit(self, *event):
        try:
            self.events.put_nowait(event)
        except queue.Full:
            try:
                self.events.get_nowait()
                self.events.put_nowait(event)
            except queue.Empty:
                pass

    def calibrate(self, operation):
        client = self.calibration_clients[operation]
        if not client.service_is_ready():
            self.emit("result", "膝角节点未启动")
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda result: self.emit("result", result.result().message if result.result()
                                     else "标定服务失败"))


class Dashboard:
    def __init__(self, root, node, events):
        self.root = root
        self.node = node
        self.events = events
        self.state = DashboardState()
        self.record_file = None
        self.record_writer = None
        self.pending_packet = False
        self.last_chart_at = 0.0
        self.status_text = tk.StringVar(value="等待 CAN 数据")
        self.imu_text = [tk.StringVar(value="无数据"), tk.StringVar(value="无数据")]
        self.knee_text = tk.StringVar(value="--.-°")
        self.knee_status_text = tk.StringVar(value="未标定")
        self.record_text = tk.StringVar(value="开始记录 CSV")
        root.title("CtrBoard CAN 传感器上位机")
        root.geometry("1200x760")
        root.minsize(880, 580)
        self.build()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(50, self.refresh)

    def build(self):
        top = ttk.Frame(self.root, padding=(14, 12))
        top.pack(fill=tk.X)
        ttk.Label(top, text="CtrBoard · CAN 传感器", font=("Microsoft YaHei UI", 15, "bold")).pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.status_text).pack(side=tk.RIGHT)

        body = ttk.Frame(self.root, padding=(14, 2, 14, 14))
        body.pack(fill=tk.BOTH, expand=True)
        side = ttk.Frame(body, width=265)
        side.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 15))
        side.pack_propagate(False)
        ttk.Label(side, text="膝屈伸角", foreground="#52616b").pack(anchor=tk.W, pady=(8, 0))
        ttk.Label(side, textvariable=self.knee_text,
                  font=("Microsoft YaHei UI", 29, "bold")).pack(anchor=tk.W)
        ttk.Label(side, textvariable=self.knee_status_text, wraplength=255).pack(anchor=tk.W, pady=(0, 18))
        ttk.Separator(side).pack(fill=tk.X, pady=3)
        for index, name in enumerate(("I1 大腿", "I2 小腿")):
            ttk.Label(side, text=name, font=("Microsoft YaHei UI", 11, "bold")).pack(anchor=tk.W, pady=(13, 2))
            ttk.Label(side, textvariable=self.imu_text[index], wraplength=255).pack(anchor=tk.W)
        ttk.Separator(side).pack(fill=tk.X, pady=(17, 12))
        ttk.Label(side, text="双 IMU 标定", font=("Microsoft YaHei UI", 11, "bold")).pack(anchor=tk.W)
        for label, operation in (("站立零位", "zero"), ("屈伸动作轴", "axis"), ("重置标定", "reset")):
            ttk.Button(side, text=label,
                       command=lambda op=operation: self.node.calibrate(op)).pack(fill=tk.X, pady=(7, 0))
        ttk.Separator(side).pack(fill=tk.X, pady=(17, 12))
        ttk.Button(side, textvariable=self.record_text,
                   command=self.toggle_recording).pack(fill=tk.X)

        tabs = ttk.Notebook(body)
        tabs.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        chart_page = ttk.Frame(tabs)
        pressure_page = ttk.Frame(tabs)
        tabs.add(chart_page, text="角度曲线")
        tabs.add(pressure_page, text="足底压力")
        self.chart = tk.Canvas(chart_page, bg=BACKGROUND, highlightthickness=0)
        self.chart.pack(fill=tk.BOTH, expand=True)
        self.pressure = tk.Canvas(pressure_page, bg=BACKGROUND, highlightthickness=0)
        self.pressure.pack(fill=tk.BOTH, expand=True)
        if os.environ.get("CTRBOARD_DASHBOARD_SMOKE_TAB") == "pressure":
            tabs.select(pressure_page)

    def apply_event(self, event, now):
        kind = event[0]
        if kind == "imu":
            _, index, stamp, valid = event
            self.state.set_imu(index, stamp, valid, now)
        elif kind == "euler":
            _, index, stamp, angles = event
            self.state.set_euler(index, stamp, angles)
        elif kind == "pressure":
            _, index, values = event
            self.state.set_pressure(index, values, now)
        elif kind == "packet":
            self.state.set_packet(event[1], now)
            self.pending_packet = True
        elif kind == "phase":
            self.state.phases = (event[1] + [0] * 4)[:4]
        elif kind == "knee_angle":
            if math.isfinite(event[1]):
                self.state.knee_angle = event[1]
                self.state.knee_at = now
        elif kind == "diagnostic":
            source, message = event[1:]
            target = "ctrboard_sensor_safety" if source == "bridge" else "knee_kinematics"
            status = next((item for item in message.status if item.name == target), None)
            if status:
                if source == "bridge":
                    values = {item.key: item.value for item in status.values}
                    self.state.bridge_state = values.get("state", status.message)
                    self.state.bridge_diagnostic_at = now
                else:
                    self.state.knee_status = status.message
        elif kind == "result":
            self.knee_status_text.set(event[1])

    def refresh(self):
        now = time.monotonic()
        for _ in range(512):
            try:
                self.apply_event(self.events.get_nowait(), now)
            except queue.Empty:
                break
        state = self.state
        if state.packet_fresh(now):
            safety = state.bridge_status(now)
            label = BRIDGE_LABELS.get(safety, safety) if safety else "安全诊断未提供"
            self.status_text.set(f"CAN 已收包 · 设备 {state.device_time_ms} ms · {label}")
        else:
            self.status_text.set("CAN 数据中断或桥接未启动")
        for index, var in enumerate(self.imu_text):
            imu = state.imus[index]
            angles = imu.angles(now) if state.packet_fresh(now) else None
            if angles:
                var.set("R {:.1f}°   P {:.1f}°   Y {:.1f}°".format(*angles))
            else:
                var.set("无效" if imu.fresh(now) and not imu.orientation_valid else "无数据 / 过期")
        angle = state.live_knee_angle(now)
        self.knee_text.set(f"{angle:.1f}°" if angle is not None else "--.-°")
        self.knee_status_text.set(KNEE_LABELS.get(state.knee_status, state.knee_status))
        if now - self.last_chart_at >= 0.1:
            state.append_history(now)
            self.draw_chart(now)
            self.draw_pressure(now)
            self.last_chart_at = now
        if self.pending_packet:
            self.write_record(now)
            self.pending_packet = False
        self.root.after(50, self.refresh)

    def draw_chart(self, now):
        canvas = self.chart
        canvas.delete("all")
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width < 100 or height < 100:
            return
        left, right, top, bottom = 62, width - 22, 60, height - 42
        canvas.create_text(left, 24, text="角度 (°)", anchor="w", fill="#27343b",
                           font=("Microsoft YaHei UI", 12, "bold"))
        for index, label in enumerate(("I1 pitch", "I2 pitch", "膝屈伸")):
            x = left + 125 * index
            canvas.create_line(x, 46, x + 18, 46, fill=COLORS[index], width=3)
            canvas.create_text(x + 24, 46, text=label, anchor="w", fill="#27343b")
        samples = [row for row in self.state.history if now - row[0] <= 20]
        values = [value for row in samples for value in row[1:] if value is not None]
        lo, hi = (min(values), max(values)) if values else (-30.0, 30.0)
        margin = max(8.0, (hi - lo) * 0.12)
        lo -= margin
        hi += margin
        for step in range(5):
            y = top + (bottom - top) * step / 4
            value = hi - (hi - lo) * step / 4
            canvas.create_line(left, y, right, y, fill="#d8e0e3")
            canvas.create_text(left - 9, y, text=f"{value:.0f}", anchor="e", fill="#596770")
        for index, color in enumerate(COLORS, 1):
            segment = []
            for row in samples + [(now, None, None, None)]:
                value = row[index]
                if value is None:
                    if len(segment) >= 4:
                        canvas.create_line(*segment, fill=color, width=2, smooth=True)
                    segment = []
                else:
                    x = right - (now - row[0]) * (right - left) / 20
                    y = bottom - (value - lo) * (bottom - top) / (hi - lo)
                    segment.extend((x, y))
        canvas.create_text(left, bottom + 20, text="-20 s", anchor="w", fill="#596770")
        canvas.create_text(right, bottom + 20, text="现在", anchor="e", fill="#596770")

    def draw_pressure(self, now):
        canvas = self.pressure
        canvas.delete("all")
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width < 100 or height < 100:
            return
        cell = max(15, min((width - 100) / 9, (height - 120) / 5))
        gap = cell * 0.08
        for side in range(2):
            values = self.state.pressure_values(side, now)
            origin_x = width / 2 + (-4.35 if side == 0 else 0.35) * cell
            origin_y = (height - 5 * cell) / 2
            canvas.create_text(origin_x, origin_y - 20,
                               text=("左脚" if side == 0 else "右脚") +
                               (f"  合计 {sum(values)}" if values else "  无数据 / 过期"),
                               anchor="w", fill="#27343b", font=("Microsoft YaHei UI", 12, "bold"))
            for index in range(20):
                value = values[index] if values else 0
                intensity = min(value, 255) / 255
                color = "#{:02x}{:02x}{:02x}".format(
                    int(45 + 202 * intensity), int(181 - 130 * intensity),
                    int(108 - 58 * intensity)) if values else "#c5cdd0"
                x = origin_x + (index % 4) * cell
                y = origin_y + (index // 4) * cell
                canvas.create_rectangle(x + gap, y + gap, x + cell - gap,
                                        y + cell - gap, fill=color, outline="")
                canvas.create_text(x + cell / 2, y + cell / 2,
                                   text=f"{index}\n{value if values else '-'}", fill="#162629")

    def toggle_recording(self):
        if self.record_file:
            self.record_file.close()
            self.record_file = None
            self.record_writer = None
            self.record_text.set("开始记录 CSV")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv",
            initialfile=f"ctrboard_can_{datetime.now():%Y%m%d_%H%M%S}.csv",
            filetypes=[("CSV", "*.csv")])
        if not path:
            return
        self.record_file = open(path, "w", newline="", encoding="utf-8-sig")
        self.record_writer = csv.writer(self.record_file)
        self.record_writer.writerow(["pc_time", "device_ms", "bridge_state",
            *[f"imu{i}_{axis}" for i in (1, 2) for axis in ("roll", "pitch", "yaw")],
            "knee_flexion_deg", *[f"fsr_{side}_{i}" for side in ("left", "right") for i in range(20)]])
        self.record_text.set("停止记录 CSV")

    def write_record(self, now):
        if not self.record_writer:
            return
        state = self.state
        angles = [imu.angles(now) if state.packet_fresh(now) else None for imu in state.imus]
        pressures = [state.pressure_values(i, now) for i in range(2)]
        self.record_writer.writerow([datetime.now().isoformat(timespec="milliseconds"),
            state.device_time_ms, state.bridge_status(now) or "",
            *(value for group in angles for value in (group if group else ("", "", ""))),
            state.live_knee_angle(now) if state.live_knee_angle(now) is not None else "",
            *(value for group in pressures for value in (group if group else [""] * 20))])
        self.record_file.flush()

    def close(self):
        if self.record_file:
            self.record_file.close()
        self.root.quit()
        self.root.destroy()


def main():
    rclpy.init()
    events = queue.Queue(maxsize=512)
    node = DashboardSubscriber(events)
    def spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    worker = threading.Thread(target=spin, daemon=True)
    worker.start()
    root = tk.Tk()
    try:
        dashboard = Dashboard(root, node, events)
        smoke_seconds = os.environ.get("CTRBOARD_DASHBOARD_SMOKE_SECONDS")
        if smoke_seconds:
            root.after(int(float(smoke_seconds) * 1000), dashboard.close)
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        worker.join(timeout=2)
        node.destroy_node()


if __name__ == "__main__":
    main()
