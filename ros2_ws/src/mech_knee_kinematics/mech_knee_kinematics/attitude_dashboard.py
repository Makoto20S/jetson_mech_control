"""Read-only Tk posture viewer; --demo has no ROS or hardware I/O."""
import argparse
import math
import time
import tkinter as tk
from .attitude_view_state import AttitudeViewState
from .dual_imu_knee import quaternion_from_euler_degrees

BG = '#101b2a'
FG = '#e8eef7'
MUTED = '#8395ac'
CYAN = '#45cfdb'
ORANGE = '#ffb367'
LABELS = {
    'not calibrated': '未标定', 'axis calibration required': '请执行动作轴标定',
    'repeat standing zero required': '请重新站立归零',
    'calibrating static': '站立零位标定中', 'calibrating axis': '动作轴标定中',
    'IMU pair timed out': '双 IMU 超时', 'calibrated': '标定完成',
}


def make_subscriber(state):
    from rclpy.node import Node
    from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
    from sensor_msgs.msg import JointState
    from diagnostic_msgs.msg import DiagnosticArray

    class Subscriber(Node):
        def __init__(self):
            super().__init__('dual_imu_attitude_viewer')
            for name in ('thigh', 'shank'):
                self.create_subscription(
                    QuaternionStamped, (f'attitude/{name}/orientation' if state.raw else
                                        f'attitude/{name}_from_zero/orientation'),
                    lambda msg, n=name: self.orientation(n, msg), 10)
                self.create_subscription(
                    Vector3Stamped, f'attitude/{name}/rpy_rad',
                    lambda msg, n=name: state.rpy.update({n: (
                        tuple(math.degrees(v) for v in (msg.vector.x, msg.vector.y,
                                                       msg.vector.z)), time.monotonic())}), 10)
            self.create_subscription(JointState, 'knee/joint_state', self.joint, 10)
            self.create_subscription(DiagnosticArray, 'ctrboard/diagnostics' if state.raw
                                     else 'knee/diagnostics', self.diag, 10)

        @staticmethod
        def stamp(msg):
            return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

        def orientation(self, name, msg):
            q = msg.quaternion
            state.segment(name, self.stamp(msg), (q.w, q.x, q.y, q.z), time.monotonic())

        def joint(self, msg):
            if 'knee_joint' not in msg.name:
                return
            i = msg.name.index('knee_joint')
            if len(msg.position) > i and len(msg.velocity) > i:
                state.update_joint(self.stamp(msg), math.degrees(msg.position[i]),
                                   math.degrees(msg.velocity[i]), time.monotonic())

        def diag(self, msg):
            for status in msg.status:
                if status.name == ('ctrboard_sensor_safety' if state.raw else 'knee_kinematics'):
                    level = (int.from_bytes(status.level, 'little')
                             if isinstance(status.level, bytes) else int(status.level))
                    state.diagnostic = (level, status.message, time.monotonic())
    return Subscriber()


class AttitudeDashboard:
    def __init__(self, root, state, pump=None, demo=False):
        self.root, self.state, self.pump, self.demo = root, state, pump, demo
        self.start = time.monotonic()
        root.title('双 IMU · Jetson 实时姿态')
        root.geometry('1100x760')
        root.minsize(900, 650)
        self.canvas = tk.Canvas(root, bg=BG, highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.tick()

    def text(self, x, y, label, color=FG, size=13, anchor='nw'):
        self.canvas.create_text(x, y, text=label, fill=color, anchor=anchor,
                                font=('sans', size))

    def tick(self):
        if self.pump:
            self.pump()
        now = time.monotonic()
        if self.demo:
            t = now-self.start
            thigh = 15*math.sin(t)
            shank = thigh+35+25*math.sin(t*1.5)
            stamp = int(now*1e9)
            for name, angle in (('thigh', thigh), ('shank', shank)):
                self.state.segment(name, stamp, quaternion_from_euler_degrees(0, angle, 0), now)
                self.state.rpy[name] = ((0, angle, 0), now)
            self.state.update_joint(stamp, shank-thigh, 37.5*math.cos(t*1.5), now)
            self.state.diagnostic = (0, 'demo', now)
        self.state.record_history(now)
        self.draw(now)
        self.root.after(33, self.tick)

    def draw(self, now):
        c = self.canvas
        c.delete('all')
        w, h = max(900, c.winfo_width()), max(650, c.winfo_height())
        ready, reason = self.state.ready(now)
        self.text(28, 20, '双 IMU  /  桌面姿态' if self.state.raw else
                  '双 IMU  /  实时姿态', size=23)
        self.text(w-28, 25, '合成演示 · 非真实设备' if self.demo else 'ROS 2 · 只读监视',
                  ORANGE if self.demo else CYAN, anchor='ne')
        self.text(28, 61, LABELS.get(reason, reason), CYAN if ready else ORANGE)
        self.text(w-28, 62, '传感器姿态 · 未标定膝角' if self.state.raw else
                  '站立零位参考 · 二维投影 · 非解剖学测量', MUTED, 10, 'ne')
        directions = self.state.directions() if self.state.segments.keys() >= {'thigh', 'shank'} else ((0, 0, -1),)*2
        length = min(105, h*0.14)
        for col, axis, title in ((0, 0, '侧向投影  X–Z'), (1, 1, '正向投影  Y–Z')):
            cx = w*(0.20+col*0.34)
            self.text(cx, 112, title, MUTED, 12, 'n')
            hip = (cx, 165)
            points = [hip]
            for v in directions:
                x, y = points[-1]
                points.append((x+v[axis]*length, y-v[2]*length))
            if self.state.raw:
                origins = [(cx-45, 260), (cx+45, 260)]
                lines = [(origin, (origin[0]+v[axis]*length,
                                   origin[1]-v[2]*length))
                         for origin, v in zip(origins, directions)]
                points = [point for line in lines for point in line]
                self.text(cx-45, 390, 'I1', CYAN, 12, 'n')
                self.text(cx+45, 390, 'I2', ORANGE, 12, 'n')
            else:
                lines = [points[i:i+2] for i in range(2)]
            for (a, b), color in zip(lines, (CYAN, ORANGE)):
                c.create_line(*a, *b, fill=color if ready else '#4e6076', width=16,
                              capstyle='round')
            for x, y in points:
                c.create_oval(x-7, y-7, x+7, y+7, fill=FG if ready else MUTED, outline=BG, width=2)
            if not ready:
                self.text(cx, 405, '数据失效 / 等待标定', ORANGE, 11, 'n')
        x = w*0.74
        self.text(x, 125, '膝关节屈伸', MUTED)
        angle = f'{self.state.joint[1]:.1f}°' if ready and not self.state.raw else '未标定' if self.state.raw else '—'
        rate = f'{self.state.joint[2]:.1f} °/s' if ready and not self.state.raw else '—'
        self.text(x, 155, angle, CYAN if ready else MUTED, 38)
        self.text(x, 220, '角速度  '+rate, FG)
        for i, name in enumerate(('thigh', 'shank')):
            rpy = self.state.rpy.get(name)
            label = '—'
            if ready and rpy and now-rpy[1] <= 0.3:
                label = ' / '.join(f'{v:.1f}' for v in rpy[0])
            self.text(x, 280+i*58, ('I1 R/P/Y °' if i == 0 else 'I2 R/P/Y °') if self.state.raw else
                      ('大腿 R/P/Y °' if i == 0 else '小腿 R/P/Y °'), MUTED, 11)
            self.text(x, 303+i*58, label, FG, 11)
        self.chart(now, 30, h-255, w-60, 170, ready)
        self.text(30, h-48, '青色：I1 Pitch °    橙色：I2 Pitch °    |    最近 20 秒' if self.state.raw else
                  '青色：膝角 °    橙色：角速度 °/s    |    最近 20 秒', MUTED, 11)
        self.text(w-28, h-25, '仅显示数据 · 不启动串口或电机 · 标定服务由解算节点提供', MUTED, 10, 'se')

    def chart(self, now, x, y, width, height, ready):
        c = self.canvas
        for i in range(5):
            yy = y+i*height/4
            c.create_line(x, yy, x+width, yy, fill='#253449')
            self.text(x, yy-15, f'{180-i*90:g}°' if self.state.raw else f'{150-i*50:g}°', MUTED, 9)
            self.text(x+width, yy-15, f'{180-i*90:g}°' if self.state.raw else f'{500-i*250:g}°/s', MUTED, 9, 'ne')
        ranges = ((1, -180, 180, CYAN), (2, -180, 180, ORANGE)) if self.state.raw else (
            (1, -50, 150, CYAN), (2, -500, 500, ORANGE))
        for index, low, high, color in ranges:
            points = []
            previous = None
            for row in self.state.history:
                if now-row[0] > 20:
                    continue
                if previous is not None and row[0]-previous > 0.3:
                    if len(points) >= 4:
                        c.create_line(*points, fill=color, width=2)
                    points = []
                previous = row[0]
                points.extend((x+width*(1-(now-row[0])/20),
                               y+height*(1-(max(low, min(high, row[index]))-low)/(high-low))))
            if len(points) >= 4:
                c.create_line(*points, fill=color if ready else MUTED, width=2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', action='store_true', help='independent sensor poses without knee calibration')
    parser.add_argument('--demo', action='store_true', help='synthetic animation, no ROS')
    parser.add_argument('--smoke-seconds', type=float, default=0, help='close after bounded UI test')
    args, ros_args = parser.parse_known_args()
    state = AttitudeViewState(raw=args.raw)
    node = None
    try:
        if not args.demo:
            import rclpy
            rclpy.init(args=ros_args)
            node = make_subscriber(state)
        root = tk.Tk()
        def pump():
            for _ in range(32):
                rclpy.spin_once(node, timeout_sec=0.0)
        AttitudeDashboard(root, state, pump if node else None, args.demo)
        if args.smoke_seconds > 0:
            root.after(int(args.smoke_seconds*1000), root.destroy)
        root.mainloop()
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
