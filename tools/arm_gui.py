#!/usr/bin/env python3
"""arm_gui.py — 机械臂图形化上位机（3D 预览 + 末端位姿控制 + 串口发送）

用法:
    python arm_gui.py -p COM5              # 连接机械臂 (UART1, 115200)
    python arm_gui.py --demo               # 无硬件演示模式（假遥测，动画扫动）
    python arm_gui.py --screenshot out.png # 离线渲染一帧并保存（自检用）

依赖: pip install matplotlib pyserial

功能:
  - 3D 抽象机械臂预览：连杆 + 关节轴线小段 + TCP；几何与固件同源
    （直接解析 RobotArm/Inc/arm_config.h 的 DH，FK 与 MCU 一致）
  - 绿色点 = 实际 TCP（遥测 FK），红色菱形 = 目标位置
  - 滑条设置末端 x/y/z(±0.6m 显示为 mm) 与 roll/pitch/yaw(deg)
  - 按钮: 使能/失能、夹爪开/合、单次发送、"连续发送"开关(5Hz 跟随滑条)、
          J0~J5 点动小按钮（±0.1rad，用于方向验证）
  - 状态栏: 控制器状态/故障/跟踪误差/轨迹进度/串口统计
  - 自动心跳；关闭窗口自动失能

协议与 tools/uart_console.py 完全一致（复用其帧构建/解析代码）。
"""

import argparse
import math
import queue
import struct
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "test"))

import matplotlib

if "--screenshot" in sys.argv:
    matplotlib.use("Agg")

# Windows 中文字体（否则中文渲染为方框）
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "sans-serif"]
matplotlib.rcParams["axes.unicode_minus"] = False

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import gridspec
from matplotlib.widgets import Button, CheckButtons, Slider

import uart_console as UC
from verify_kinematics import CONFIG_H, fk_chain, parse_config

# ---------------------------------------------------------------- 几何(与固件同源)

CFG = parse_config(CONFIG_H.read_text(encoding="utf-8"))
DH = CFG["dh"]
LINK_LEN = 0.06   # 关节轴线小段长度(m)
REACH = max(sum(r[1] for r in DH) + sum(abs(r[0]) for r in DH), 0.45) + 0.12


def fk_points(q):
    """返回 (7 个关节/TCP 原点, 6 个关节轴方向) —— 基座系。"""
    T = fk_chain(CFG, q)
    origins = [np.array([t[0][3], t[1][3], t[2][3]]) for t in T]
    zaxes = [np.array([t[0][2], t[1][2], t[2][2]]) for t in T[:6]]
    return origins, zaxes


# ---------------------------------------------------------------- 串口/演示源

class SerialLink:
    """真实串口链路：发帧 + 收帧入队 + 心跳。"""

    def __init__(self, port):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.rx_q = queue.Queue()
        self.tx_count = 0
        self.rx_count = 0
        self.rx_bytes = 0   # 原始字节计数（含噪声），用于排查物理链路
        self._stop = threading.Event()
        self.feed = UC.make_parser(lambda t, p: (self.rx_q.put((t, p)),
                                                 setattr(self, "rx_count", self.rx_count + 1)))
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._heartbeat, daemon=True).start()

    def _reader(self):
        while not self._stop.is_set():
            try:
                data = self.ser.read(256)
                if data:
                    self.rx_bytes += len(data)
                    self.feed(data)
            except Exception:
                self._stop.set()

    def _heartbeat(self):
        while not self._stop.is_set():
            try:
                self.ser.write(UC.build_frame(UC.T_HEARTBEAT))
            except Exception:
                pass
            self._stop.wait(0.3)

    def send(self, ftype, payload=b""):
        if self._stop.is_set():
            return
        try:
            self.ser.write(UC.build_frame(ftype, payload))
            self.tx_count += 1
        except Exception:
            pass

    def close(self):
        self._stop.set()
        try:
            self.ser.write(UC.build_frame(UC.T_SET_ENABLE, bytes([0])))  # 退出失能
            time.sleep(0.15)
            self.ser.close()
        except Exception:
            pass


class DemoLink:
    """演示链路：假遥测（关节正弦摆动），不发串口。"""

    def __init__(self):
        self.rx_q = queue.Queue()
        self.tx_count = 0
        self.rx_count = 0
        self.t0 = time.time()
        self._stop = threading.Event()
        threading.Thread(target=self._gen, daemon=True).start()

    def _gen(self):
        while not self._stop.is_set():
            t = time.time() - self.t0
            q = [0.15 * math.sin(t * 0.7 + i) for i in range(6)]
            self.rx_q.put((UC.T_JOINTS, struct.pack("<6f", *q)))
            T = fk_chain(CFG, q)
            pose = [T[6][0][3], T[6][1][3], T[6][2][3], 0, 0, 0]
            self.rx_q.put((UC.T_POSE, struct.pack("<6f", *pose)))
            self.rx_q.put((UC.T_STATUS, struct.pack("<BBHff", 2, 0, 0, 0.01, 0.4)))
            self.rx_count += 3
            self._stop.wait(0.15)

    def send(self, ftype, payload=b""):
        self.tx_count += 1
        print(f"[demo] 发送 0x{ftype:02X} {payload.hex()}")

    def close(self):
        self._stop.set()


# ---------------------------------------------------------------- GUI

class ArmGui:
    def __init__(self, link):
        self.link = link
        self.q = [0.0] * 6
        self.pose = None
        self.status = {"state": 0, "req": 0, "fault": 0, "track": 0.0, "prog": 0.0}
        self.slider_locked = True          # 收到首帧遥测前不回填滑条
        self.continuous = False
        self.last_send_t = 0.0
        self.pending_target = None

        self.fig = plt.figure("机械臂上位机", figsize=(13, 7.5))
        gs = gridspec.GridSpec(6, 2, width_ratios=[2.1, 1.0])
        # 上部 ~70% 给 3D 视图和滑条，底部 30% 留给控制按钮；
        # 右侧留 10% 边距给滑条数值文字，避免被画布裁剪
        self.fig.subplots_adjust(left=0.02, right=0.90, top=0.97, bottom=0.30, wspace=0.04,
                                 hspace=0.35)
        self.ax = self.fig.add_subplot(gs[:, 0], projection="3d")
        self.ax.set_box_aspect((1, 1, 1))
        self._setup_3d()

        # 右侧：6 个滑条
        self.sliders = {}
        slider_spec = [
            ("x", -REACH * 1000, REACH * 1000, 0.0, "末端 X (mm)"),
            ("y", -REACH * 1000, REACH * 1000, 0.0, "末端 Y (mm)"),
            ("z", -REACH * 1000, REACH * 1000, 0.0, "末端 Z (mm)"),
            ("roll", -180, 180, 0.0, "Roll (deg)"),
            ("pitch", -180, 180, 0.0, "Pitch (deg)"),
            ("yaw", -180, 180, 0.0, "Yaw (deg)"),
        ]
        for i, (key, lo, hi, val, label) in enumerate(slider_spec):
            sax = self.fig.add_subplot(gs[i, 1])
            sax.set_facecolor("#f4f4f4")
            sl = Slider(sax, label, lo, hi, valinit=val, valstep=1.0)
            sl.on_changed(lambda _v: self._on_slider())
            self.sliders[key] = sl

        # 底部控制区（绝对坐标，位于 y<0.28 区域）
        self.btn_enable = Button(plt.axes([0.56, 0.09, 0.09, 0.05]), "使能")
        self.btn_disable = Button(plt.axes([0.67, 0.09, 0.09, 0.05]), "失能")
        self.btn_send = Button(plt.axes([0.78, 0.09, 0.09, 0.05]), "发送目标")
        self.btn_grip = Button(plt.axes([0.89, 0.09, 0.09, 0.05]), "夹爪开/合")
        self.chk_cont = CheckButtons(plt.axes([0.56, 0.015, 0.05, 0.055]), ["连续发送"], [False])
        self.btn_home = Button(plt.axes([0.80, 0.015, 0.18, 0.045]), "回零")
        self.btn_home.on_clicked(lambda _e: self.link.send(UC.T_GO_HOME))
        self.btn_enable.on_clicked(lambda _e: self.link.send(UC.T_SET_ENABLE, bytes([1])))
        self.btn_disable.on_clicked(lambda _e: self.link.send(UC.T_SET_ENABLE, bytes([0])))
        self.btn_send.on_clicked(lambda _e: self._send_target())
        self.btn_grip.on_clicked(lambda _e: self._toggle_grip())
        self.chk_cont.on_clicked(lambda _l: self._toggle_continuous())
        self.grip_state = False

        self.status_text = self.fig.text(0.02, 0.005, "", fontsize=9)

        # 点动小按钮（J0~J5 两行：上行 +0.1rad，下行 -0.1rad）
        self.fig.text(0.56, 0.265, "J0      J1      J2      J3      J4      J5   (点动±0.1rad)",
                      fontsize=8)
        self.jog_btns = []
        for j in range(6):
            for k, (sign, txt) in enumerate(((+0.1, "+"), (-0.1, "-"))):
                b = Button(plt.axes([0.56 + j * 0.072, 0.150 + (1 - k) * 0.052, 0.06, 0.040]),
                           txt)
                b.on_clicked(lambda _e, jj=j, s=sign: self.link.send(
                    UC.T_JOG, bytes([jj]) + struct.pack("<f", s)))
                self.jog_btns.append(b)

        self.timer = self.fig.canvas.new_timer(interval=100)
        self.timer.add_callback(self._on_timer)
        self.timer.start()

    # ---------------- 3D 绘制 ----------------

    def _setup_3d(self):
        self.ax.set_xlim(-REACH, REACH)
        self.ax.set_ylim(-REACH, REACH)
        self.ax.set_zlim(-REACH, REACH)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Z (m)")
        self.ax.set_title("机械臂 3D 预览（拖动旋转视角）")

        origins, zaxes = fk_points(self.q)
        pts = np.array(origins)
        self.line_arm, = self.ax.plot(pts[:, 0], pts[:, 1], pts[:, 2],
                                      "-o", lw=6, color="#2196f3", markersize=8,
                                      markerfacecolor="#1565c0")
        self.axis_lines = []
        for i in range(6):
            ln, = self.ax.plot([], [], [], color="#ff9800", lw=1.5, alpha=0.8)
            self.axis_lines.append(ln)
        # 基座：XY 平面小方框（基座系 Z = J0 轴）
        g = np.array([[0.08, 0.08, 0], [-0.08, 0.08, 0], [-0.08, -0.08, 0],
                      [0.08, -0.08, 0], [0.08, 0.08, 0]])
        self.ax.plot(g[:, 0], g[:, 1], g[:, 2], color="#616161", lw=2)
        # 目标点（红菱形）
        self.target_pt, = self.ax.plot([0], [0], [0], "D", color="#e53935", markersize=10)
        # 实际 TCP（绿点）
        self.tcp_pt, = self.ax.plot([0], [0], [0], "o", color="#43a047", markersize=9)

    def _redraw_arm(self):
        origins, zaxes = fk_points(self.q)
        pts = np.array(origins)
        self.line_arm.set_data(pts[:, 0], pts[:, 1])
        self.line_arm.set_3d_properties(pts[:, 2])
        for i, ln in enumerate(self.axis_lines):
            o, z = origins[i], zaxes[i]
            a, b = o - z * LINK_LEN, o + z * LINK_LEN
            ln.set_data([a[0], b[0]], [a[1], b[1]])
            ln.set_3d_properties([a[2], b[2]])
        self.tcp_pt.set_data([pts[-1][0]], [pts[-1][1]])
        self.tcp_pt.set_3d_properties([pts[-1][2]])

        # 视野自适应臂的包围盒（等比三轴），避免机械臂只占画面一小块
        lo = pts.min(axis=0) - 0.05
        hi = pts.max(axis=0) + 0.05
        c = (lo + hi) / 2
        r = float(np.max(hi - lo)) / 2 * 1.05
        self.ax.set_xlim(c[0] - r, c[0] + r)
        self.ax.set_ylim(c[1] - r, c[1] + r)
        self.ax.set_zlim(c[2] - r, c[2] + r)

    # ---------------- 交互 ----------------

    def _slider_pose(self):
        s = self.sliders
        return (s["x"].val / 1000.0, s["y"].val / 1000.0, s["z"].val / 1000.0,
                math.radians(s["roll"].val), math.radians(s["pitch"].val),
                math.radians(s["yaw"].val))

    def _on_slider(self):
        x, y, z = self._slider_pose()[:3]
        self.target_pt.set_data([x], [y])
        self.target_pt.set_3d_properties([z])
        self.fig.canvas.draw_idle()
        if self.continuous:
            self.pending_target = self._slider_pose()

    def _send_target(self):
        pose = self._slider_pose()
        self.link.send(UC.T_MOVE_CART, struct.pack("<6f", *pose))
        self.pending_target = None

    def _toggle_continuous(self):
        self.continuous = not self.continuous
        print("连续发送:", "开(拖滑条即下发, 限5Hz)" if self.continuous else "关")

    def _toggle_grip(self):
        self.grip_state = not self.grip_state
        self.link.send(UC.T_GRIPPER, bytes([1 if self.grip_state else 0]))

    # ---------------- 定时刷新 ----------------

    def _on_timer(self):
        got_joints = False
        try:
            while True:
                ftype, payload = self.link.rx_q.get_nowait()
                if ftype == UC.T_JOINTS and len(payload) >= 24:
                    self.q = list(struct.unpack_from("<6f", payload))
                    got_joints = True
                elif ftype == UC.T_POSE and len(payload) >= 24:
                    self.pose = struct.unpack_from("<6f", payload)
                elif ftype == UC.T_STATUS and len(payload) >= 12:
                    self.status["state"], self.status["req"] = payload[0], payload[1]
                    self.status["fault"] = struct.unpack_from("<H", payload, 2)[0]
                    self.status["track"], self.status["prog"] = struct.unpack_from("<2f", payload, 4)
                    if len(payload) >= 14:
                        merr = payload[12]
                        miss = payload[13]
                        self.status["motor_err"] = "".join(
                            f"J{i} " for i in range(6) if merr & (1 << i)).strip()
                        self.status["missing"] = "".join(
                            f"J{i} " for i in range(6) if miss & (1 << i)).strip()
                elif ftype == UC.T_ACK:
                    print("[ACK]", UC.REQUEST_NAMES.get(payload[1], payload[1]),
                          f"cmd=0x{payload[0]:02X}")
        except queue.Empty:
            pass

        if got_joints:
            self._redraw_arm()
            if self.slider_locked and self.pose is None:
                T = fk_chain(CFG, self.q)
                p = [T[6][0][3], T[6][1][3], T[6][2][3], 0, 0, 0]
                self._fill_sliders(p)
            elif self.slider_locked and self.pose is not None:
                self._fill_sliders(self.pose)

        # 连续发送限 5Hz
        if self.continuous and self.pending_target is not None \
                and time.time() - self.last_send_t > 0.2:
            self.link.send(UC.T_MOVE_CART, struct.pack("<6f", *self.pending_target))
            self.pending_target = None
            self.last_send_t = time.time()

        st = self.status
        extra = ""
        if st.get("motor_err"):
            extra += f" 电机报码:{st['motor_err']}"
        if st.get("missing"):
            extra += f" 反馈缺失:{st['missing']}"
        self.status_text.set_text(
            f"状态:{UC.STATE_NAMES.get(st['state'], st['state'])}  "
            f"故障:{UC.FAULT_NAMES.get(st['fault'], st['fault'])}  "
            f"跟踪误差:{st['track']:.3f}rad  轨迹:{st['prog']*100:.0f}%"
            f"{extra}"
            f" | 串口 原始字节:{getattr(self.link, 'rx_bytes', 0)} "
            f"有效帧:{self.link.rx_count} 发:{self.link.tx_count}"
            + ("  [DEMO]" if isinstance(self.link, DemoLink) else ""))
        self.fig.canvas.draw_idle()   # 定时器更新后主动触发重绘（否则只有交互时才刷新）

    def _fill_sliders(self, pose):
        """首帧遥测后把滑条对齐当前位姿。"""
        x, y, z, r, p, yw = pose
        vals = {"x": x * 1000, "y": y * 1000, "z": z * 1000,
                "roll": math.degrees(r), "pitch": math.degrees(p), "yaw": math.degrees(yw)}
        for k, sl in self.sliders.items():
            sl.set_val(min(max(vals[k], sl.valmin), sl.valmax))
        self.slider_locked = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port")
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--screenshot")
    args = ap.parse_args()

    if args.screenshot:
        link = DemoLink()
        gui = ArmGui(link)
        time.sleep(0.5)          # 演示源产生几帧遥测
        gui._on_timer()
        gui.fig.savefig(args.screenshot, dpi=110)
        print(f"已保存 {args.screenshot}")
        link.close()
        return

    if args.demo:
        link = DemoLink()
    else:
        if not args.port:
            UC.main()  # 打印串口列表/用法
            return
        link = SerialLink(args.port)

    gui = ArmGui(link)

    try:
        plt.show()
    finally:
        link.close()
        print("已断开（退出前已发失能命令）")


if __name__ == "__main__":
    main()
