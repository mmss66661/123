#!/usr/bin/env python3
"""uart_console.py — 机械臂 UART1 命令通道上位机控制台

用法:
    python uart_console.py -p COM5              # 连接串口（115200）
    python uart_console.py --list               # 列出可用串口
    python uart_console.py --selftest           # 离线自测帧编解码（无需硬件）

依赖: pip install pyserial

进入交互后:
    en                使能（当前位置保持）
    dis               失能
    j  f1 f2 f3 f4 f5 f6          MOVEJ  六关节角(rad，运动学坐标)
    jq f1 f2 f3 f4 f5 f6          MOVEJ 排队模式
    c  x y z roll pitch yaw       末端位姿(基座系, m/rad)
    jog 关节号 增量                单关节点动
    grip 0|1                       夹爪 打开|闭合
    q                              立即查询一轮状态
    hb on|off                      心跳开关（默认开，300ms 一次，防止链路超时失能）
    遥测自动解码显示：状态/关节角/末端位姿/命令回执
"""

import argparse
import struct
import sys
import threading
import time

HEAD1, HEAD2 = 0xFE, 0xEF
T_SET_ENABLE, T_MOVEJ, T_MOVEJ_QUEUE, T_MOVE_CART = 0x01, 0x02, 0x03, 0x04
T_JOG, T_GRIPPER, T_HEARTBEAT, T_QUERY = 0x05, 0x06, 0x07, 0x08
T_ACK, T_STATUS, T_JOINTS, T_POSE = 0x80, 0x81, 0x82, 0x83

STATE_NAMES = {0: "Disabled", 1: "Ready", 2: "Moving", 3: "Fault"}
REQUEST_NAMES = {0: "Ok", 1: "NotEnabled", 2: "NotCalibrated", 3: "IkFailed",
                 4: "InvalidTarget", 5: "QueueFull", 6: "EnableFailed"}
FAULT_NAMES = {0: "None", 1: "FeedbackTimeout", 2: "MotorError", 3: "CanTx",
               4: "TrackingTimeout", 5: "LimitViolation"}


def build_frame(ftype: int, payload: bytes = b"") -> bytes:
    assert len(payload) <= 27
    checksum = (ftype + sum(payload)) & 0xFF
    return bytes([HEAD1, HEAD2, ftype, len(payload)]) + payload + bytes([checksum])


def make_parser(on_frame):
    """返回 feed(data) 函数——与固件 UartProtocol::input 相同的状态机。"""
    state = {"s": 0, "t": 0, "buf": bytearray(), "need": 0}

    def feed(data: bytes):
        for b in data:
            s = state["s"]
            if s == 0:
                if b == HEAD1:
                    state["s"] = 1
            elif s == 1:
                state["s"] = 2 if b == HEAD2 else 0
            elif s == 2:
                state["t"] = b
                state["s"] = 3
            elif s == 3:
                if b > 27:
                    state["s"] = 0
                    continue
                state["need"] = b
                state["buf"] = bytearray()
                state["s"] = 5 if b == 0 else 4
            elif s == 4:
                state["buf"].append(b)
                if len(state["buf"]) >= state["need"]:
                    state["s"] = 5
            elif s == 5:
                expect = (state["t"] + sum(state["buf"])) & 0xFF
                if b == expect:
                    on_frame(state["t"], bytes(state["buf"]))
                state["s"] = 0

    return feed


def selftest():
    got = []
    feed = make_parser(lambda t, p: got.append((t, p)))
    cases = [
        (T_HEARTBEAT, b""),
        (T_MOVEJ, struct.pack("<6f", 0.1, -0.2, 0.3, 0.4, -0.5, 0.6)),
        (T_MOVE_CART, struct.pack("<6f", 0.278, -0.334, 0.391, 0, 0, 0)),
        (T_ACK, bytes([T_MOVEJ, 3])),
    ]
    for ftype, payload in cases:
        frame = build_frame(ftype, payload)
        # 混入噪声字节 + 拆两半喂入，验证状态机鲁棒性
        feed(bytes([0x00, HEAD1, 0x55]) + frame[:3] + frame[3:])
    assert got == cases, got
    print("[SELFTEST PASS] 帧构建/解析与固件一致（含噪声与拆包）")
    for ftype, payload in got:
        print(f"  type=0x{ftype:02X} len={len(payload)}")


def telemetry_line(ftype: int, payload: bytes) -> str:
    if ftype == T_ACK:
        cmd, status = payload[0], payload[1]
        return f"[ACK ] cmd=0x{cmd:02X} {REQUEST_NAMES.get(status, status)}"
    if ftype == T_STATUS and len(payload) >= 12:
        state, req, fault = payload[0], payload[1], struct.unpack_from("<H", payload, 2)[0]
        track, prog = struct.unpack_from("<2f", payload, 4)
        extra = ""
        if len(payload) >= 14:
            merr, miss = payload[12], payload[13]
            if merr:
                extra += " 电机报码:" + ",".join(f"J{i}" for i in range(6) if merr & (1 << i))
            if miss:
                extra += " 反馈缺失:" + ",".join(f"J{i}" for i in range(6) if miss & (1 << i))
        return (f"[STAT] {STATE_NAMES.get(state, state)} req={REQUEST_NAMES.get(req, req)} "
                f"fault={FAULT_NAMES.get(fault, fault)} track={track:.3f} prog={prog*100:.0f}%{extra}")
    if ftype == T_JOINTS and len(payload) >= 24:
        q = struct.unpack_from("<6f", payload)
        return "[JNT ] " + " ".join(f"{v:+.3f}" for v in q)
    if ftype == T_POSE and len(payload) >= 24:
        p = struct.unpack_from("<6f", payload)
        return (f"[POSE] xyz=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})m "
                f"rpy=({p[3]:+.2f},{p[4]:+.2f},{p[5]:+.2f})rad")
    return f"[?0x{ftype:02X}] {payload.hex()}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return

    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        sys.exit("缺少 pyserial：先运行  pip install pyserial")

    if args.list or not args.port:
        ports = list(list_ports.comports())
        for p in ports:
            print(f"{p.device}  {p.description}")
        if not args.port:
            sys.exit(0 if ports else "未发现串口")

    ser = serial.Serial(args.port, 115200, timeout=0.05)
    print(f"已连接 {args.port} @115200。命令: en/dis/j/jq/c/jog/grip/q/hb。Ctrl+C 退出。")

    stop = threading.Event()
    hb_on = [True]

    def heartbeat():
        while not stop.is_set():
            if hb_on[0]:
                try:
                    ser.write(build_frame(T_HEARTBEAT))
                except Exception:
                    pass
            stop.wait(0.3)

    def reader():
        feed = make_parser(lambda t, p: print(telemetry_line(t, p)))
        while not stop.is_set():
            try:
                data = ser.read(256)
                if data:
                    feed(data)
            except Exception:
                stop.set()

    threading.Thread(target=heartbeat, daemon=True).start()
    threading.Thread(target=reader, daemon=True).start()

    def send(ftype, payload=b""):
        ser.write(build_frame(ftype, payload))

    try:
        while not stop.is_set():
            line = input().strip()
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()
            try:
                if cmd == "en":
                    send(T_SET_ENABLE, bytes([1]))
                elif cmd == "dis":
                    send(T_SET_ENABLE, bytes([0]))
                elif cmd in ("j", "jq") and len(parts) == 7:
                    f = [float(x) for x in parts[1:]]
                    send(T_MOVEJ if cmd == "j" else T_MOVEJ_QUEUE, struct.pack("<6f", *f))
                elif cmd == "c" and len(parts) == 7:
                    f = [float(x) for x in parts[1:]]
                    send(T_MOVE_CART, struct.pack("<6f", *f))
                elif cmd == "jog" and len(parts) == 3:
                    send(T_JOG, bytes([int(parts[1]) & 0xFF]) + struct.pack("<f", float(parts[2])))
                elif cmd == "grip" and len(parts) == 2:
                    send(T_GRIPPER, bytes([1 if parts[1] in ("1", "on") else 0]))
                elif cmd == "q":
                    send(T_QUERY)
                elif cmd == "hb" and len(parts) == 2:
                    hb_on[0] = parts[1] in ("1", "on")
                    print(f"心跳 {'开' if hb_on[0] else '关'}")
                else:
                    print("命令格式: en | dis | j f×6 | jq f×6 | c f×6 | jog i d | grip 0/1 | q | hb on/off")
            except ValueError:
                print("数值格式错误")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        stop.set()
        try:
            send(T_SET_ENABLE, bytes([0]))  # 退出前失能，保安全
            time.sleep(0.1)
        except Exception:
            pass
        ser.close()
        print("\n已断开（退出前已发失能命令）")


if __name__ == "__main__":
    main()
