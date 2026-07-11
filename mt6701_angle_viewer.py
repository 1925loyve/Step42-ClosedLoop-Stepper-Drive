# -*- coding: utf-8 -*-
import queue
import struct
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import ttk, messagebox

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


BAUD_DEFAULT = 115200
RAW_MAX = 16384.0
RPM_WINDOW_SECONDS = 0.65
RPM_DISPLAY_INTERVAL = 0.35
WAVEFORM_HISTORY_SECONDS = 180.0
WAVEFORM_PIXELS_PER_SECOND = 18

REQ_SOF = b"\xA5\x5A"
RESP_SOF = b"\x5A\xA5"
MAX_PAYLOAD = 64

CMD_PING = 0x01
CMD_ENABLE = 0x02
CMD_ANGLE = 0x03
CMD_SPEED = 0x04
CMD_PID = 0x05
CMD_STREAM = 0x06
CMD_QUERY = 0x07
CMD_MODE = 0x08
CMD_STOP = 0x09
CMD_MOVE_STEPS = 0x0A
CMD_MICROSTEP = 0x0B
CMD_CLOSED_ACCEL = 0x0C
CMD_HOME_MODE = 0x0D
CMD_SET_ZERO = 0x0E
CMD_HOME = 0x0F
CMD_TRACK_CONFIG = 0x10
CMD_TRACK_PID = 0x11
CMD_TRACK_ADVANCED = 0x12
CMD_TRACK_INPUT = 0x13
CMD_TRACK_DEFAULT = 0x14
CMD_TRACK_QUERY = 0x15

MODE_OPEN_POSITION = 0
MODE_CLOSED_POSITION = 1
MODE_OPEN_SPEED = 2
MODE_TRACK_PID = 3

MODE_LABELS = {
    "速度模式": MODE_OPEN_SPEED,
    "开环位置": MODE_OPEN_POSITION,
    "闭环位置": MODE_CLOSED_POSITION,
    "内置误差PID": MODE_TRACK_PID,
}

MODE_NAMES = {value: key for key, value in MODE_LABELS.items()}

STATUS_NAMES = {
    0: "OK",
    1: "长度错误",
    2: "参数超范围",
    3: "未知命令",
    4: "当前模式不支持",
    5: "忙，请稍后再试",
    6: "当前接口无权限",
}

BOARD_MODE_NAMES = {
    0: "Factory Debug",
    1: "PWM",
    2: "CAN",
    3: "UART",
}

TRACK_MODE_LABELS = {"单环PID": 1, "串级双环PID": 2}
TRACK_MODE_NAMES = {value: key for key, value in TRACK_MODE_LABELS.items()}

HOME_MODE_LABELS = {
    "就近回零": 0,
    "正向无限位": 1,
    "反向无限位": 2,
}

HOME_MODE_NAMES = {value: key for key, value in HOME_MODE_LABELS.items()}

DEFAULT_MOTION_PARAMS = {
    1: {"kp": 5.63, "ki": 0.01, "kd": 0.0, "limit": 5000, "deadband": 2.56, "accel": 94},
    2: {"kp": 11.25, "ki": 0.01, "kd": 0.0, "limit": 5000, "deadband": 1.28, "accel": 188},
    4: {"kp": 22.50, "ki": 0.03, "kd": 0.0, "limit": 5000, "deadband": 0.64, "accel": 375},
    8: {"kp": 45.00, "ki": 0.05, "kd": 0.0, "limit": 5000, "deadband": 0.32, "accel": 750},
    16: {"kp": 90.00, "ki": 0.10, "kd": 0.0, "limit": 5000, "deadband": 0.16, "accel": 1500},
    32: {"kp": 180.00, "ki": 0.20, "kd": 0.0, "limit": 5000, "deadband": 0.08, "accel": 3000},
    64: {"kp": 360.00, "ki": 0.40, "kd": 0.0, "limit": 5000, "deadband": 0.04, "accel": 6000},
    128: {"kp": 720.00, "ki": 0.80, "kd": 0.0, "limit": 5000, "deadband": 0.02, "accel": 12000},
    256: {"kp": 1440.00, "ki": 1.60, "kd": 0.0, "limit": 5000, "deadband": 0.01, "accel": 24000},
}


def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_frame(cmd, seq, payload=b""):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes([len(payload), cmd & 0x7F, seq & 0xFF]) + payload
    return REQ_SOF + body + bytes([crc8(body)])


class ResponseParser:
    def __init__(self):
        self.reset()

    def reset(self):
        self.state = 0
        self.length = 0
        self.cmd = 0
        self.seq = 0
        self.payload = bytearray()

    def feed(self, value):
        if self.state == 0:
            if value == RESP_SOF[0]:
                self.state = 1
            return None
        if self.state == 1:
            if value == RESP_SOF[1]:
                self.state = 2
            else:
                self.reset()
            return None
        if self.state == 2:
            self.length = value
            self.payload.clear()
            if self.length > MAX_PAYLOAD:
                self.reset()
            else:
                self.state = 3
            return None
        if self.state == 3:
            self.cmd = value & 0x7F
            self.state = 4
            return None
        if self.state == 4:
            self.seq = value
            self.state = 6 if self.length == 0 else 5
            return None
        if self.state == 5:
            self.payload.append(value)
            if len(self.payload) >= self.length:
                self.state = 6
            return None
        if self.state == 6:
            body = bytes([self.length, self.cmd | 0x80, self.seq]) + bytes(self.payload)
            if crc8(body) == value:
                frame = (self.cmd, self.seq, bytes(self.payload))
                self.reset()
                return frame
            self.reset()
            return None
        self.reset()
        return None


class SerialReader:
    def __init__(self, on_frame, on_status):
        self.on_frame = on_frame
        self.on_status = on_status
        self.ser = None
        self.thread = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.parser = ResponseParser()

    def start(self, port, baud):
        self.stop()
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, args=(port, baud), daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        with self.lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def write_frame(self, frame):
        with self.lock:
            if self.ser is None or not self.ser.is_open:
                return False
            try:
                self.ser.write(frame)
                return True
            except Exception as exc:
                self.on_status(("error", f"串口发送失败：{exc}"))
                return False

    def _run(self, port, baud):
        try:
            ser = serial.Serial(port=port, baudrate=baud, bytesize=8, parity="N", stopbits=1, timeout=0.05)
            with self.lock:
                self.ser = ser
            self.on_status(("connected", f"已连接 {port} @ {baud}"))
        except Exception as exc:
            self.on_status(("error", f"打开串口失败：{exc}"))
            return

        while not self.stop_event.is_set():
            try:
                chunk = ser.read(128)
            except Exception as exc:
                self.on_status(("error", f"串口读取失败：{exc}"))
                break
            for value in chunk:
                frame = self.parser.feed(value)
                if frame is not None:
                    self.on_frame(frame)

        try:
            ser.close()
        finally:
            with self.lock:
                if self.ser is ser:
                    self.ser = None
            self.on_status(("closed", "串口已关闭"))


class AngleViewer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Step42 二进制协议调试")
        self.geometry("1380x900")
        self.minsize(1180, 760)

        self.frame_queue = queue.Queue()
        self.status_queue = queue.Queue()
        self.reader = SerialReader(self.frame_queue.put, self.status_queue.put)
        self.seq = 0
        self.connected_port = None
        self.last_raw = None
        self.unwrapped_raw = 0.0
        self.speed_window = deque()
        self.rpm_filtered = 0.0
        self.last_rpm_display_time = 0.0
        self.current_angle_deg = None
        self.target_angle_deg = None
        self.wave_history = deque()
        self.wave_pixels_per_second = WAVEFORM_PIXELS_PER_SECOND
        self.wave_paused_var = tk.BooleanVar(value=False)
        self.wave_follow_var = tk.BooleanVar(value=True)
        self.current_mode = MODE_CLOSED_POSITION

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(BAUD_DEFAULT))
        self.auto_var = tk.BooleanVar(value=True)
        self.status_var = tk.StringVar(value="等待串口")
        self.raw_var = tk.StringVar(value="--")
        self.abs_deg_var = tk.StringVar(value="--")
        self.deg_var = tk.StringVar(value="--")
        self.rpm_var = tk.StringVar(value="--")
        self.temp_var = tk.StringVar(value="--")
        self.board_mode_var = tk.StringVar(value="--")
        self.mode_var = tk.StringVar(value="闭环位置")
        self.mode_status_var = tk.StringVar(value="模式：未知")
        self.enable_status_var = tk.StringVar(value="电机：未知")
        self.stream_status_var = tk.StringVar(value="数据流：未知")
        self.microstep_var = tk.StringVar(value="32")
        self.microstep_status_var = tk.StringVar(value="细分：未知")
        self.pid_status_var = tk.StringVar(value="PID：未写入")
        self.kp_var = tk.StringVar(value="180.0")
        self.ki_var = tk.StringVar(value="0.2")
        self.kd_var = tk.StringVar(value="0.0")
        self.limit_var = tk.StringVar(value="5000")
        self.deadband_var = tk.StringVar(value="0.08")
        self.closed_accel_var = tk.StringVar(value="3000")
        self.closed_accel_status_var = tk.StringVar(value="闭环加速度：未知")
        self.home_mode_var = tk.StringVar(value="就近回零")
        self.home_status_var = tk.StringVar(value="零点：未知")
        self.target_angle_var = tk.StringVar(value="0.0")
        self.angle_status_var = tk.StringVar(value="角度：未发送")
        self.move_steps_var = tk.StringVar(value="3200")
        self.move_speed_var = tk.StringVar(value="2500")
        self.move_accel_var = tk.StringVar(value="5000")
        self.move_status_var = tk.StringVar(value="脉冲位置：未发送")
        self.speed_var = tk.StringVar(value="0")
        self.accel_var = tk.StringVar(value="3000")
        self.speed_status_var = tk.StringVar(value="速度：未发送")
        self.track_mode_var = tk.StringVar(value="单环PID")
        self.track_speed_pct_var = tk.StringVar(value="100")
        self.track_accel_pct_var = tk.StringVar(value="100")
        self.track_freq_var = tk.StringVar(value="1")
        self.track_single_kp_var = tk.StringVar(value="180.0")
        self.track_single_ki_var = tk.StringVar(value="0.2")
        self.track_single_kd_var = tk.StringVar(value="0.0")
        self.track_outer_kp_var = tk.StringVar(value="1.0")
        self.track_outer_ki_var = tk.StringVar(value="0.0")
        self.track_outer_kd_var = tk.StringVar(value="0.0")
        self.track_inner_kp_var = tk.StringVar(value="180.0")
        self.track_inner_ki_var = tk.StringVar(value="0.2")
        self.track_inner_kd_var = tk.StringVar(value="0.0")
        self.track_primary_var = tk.StringVar(value="0.0")
        self.track_secondary_var = tk.StringVar(value="0.0")
        self.track_status_var = tk.StringVar(value="误差PID：等待读取")
        self.track_filter_var = tk.StringVar(value="0.25")
        self.track_gain_min_var = tk.StringVar(value="1.0")
        self.track_gain_max_var = tk.StringVar(value="1.0")
        self.track_full_error_var = tk.StringVar(value="10.0")
        self.track_prediction_var = tk.StringVar(value="0")
        self.track_timeout_var = tk.StringVar(value="250")
        self.track_single_dead_var = tk.StringVar(value="0.08")
        self.track_outer_dead_var = tk.StringVar(value="0.08")
        self.track_inner_dead_var = tk.StringVar(value="0.08")

        self._build_ui()
        self.refresh_ports(auto_connect=True)
        self.after(50, self._poll_queues)
        self.after(1000, self._auto_refresh_ports)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        if serial is None:
            messagebox.showerror("缺少依赖", "没有安装 pyserial。\n\n请先运行：\npython -m pip install pyserial")

    def _next_seq(self):
        self.seq = (self.seq + 1) & 0xFF
        if self.seq == 0:
            self.seq = 1
        return self.seq

    def _send(self, cmd, payload=b""):
        frame = build_frame(cmd, self._next_seq(), payload)
        return self.reader.write_frame(frame)

    def _build_ui(self):
        root = ttk.Frame(self, padding=14)
        root.pack(fill=tk.BOTH, expand=True)

        top = ttk.Frame(root)
        top.pack(fill=tk.X)
        ttk.Label(top, text="串口").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=16, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(8, 14))
        ttk.Label(top, text="波特率").pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self.baud_var, width=10).pack(side=tk.LEFT, padx=(8, 14))
        ttk.Checkbutton(top, text="自动连接", variable=self.auto_var).pack(side=tk.LEFT)
        ttk.Button(top, text="刷新", command=self.refresh_ports).pack(side=tk.RIGHT)

        btns = ttk.Frame(root)
        btns.pack(fill=tk.X, pady=(12, 0))
        ttk.Button(btns, text="连接", command=self.connect_selected).pack(side=tk.LEFT)
        ttk.Button(btns, text="断开", command=self.disconnect).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(btns, text="查询状态", command=self.send_query).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(btns, text="停止", command=self.send_stop).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Label(root, textvariable=self.status_var).pack(anchor=tk.W, pady=(12, 0))

        data = ttk.Frame(root)
        data.pack(fill=tk.X, pady=(18, 0))
        for col, (title, var) in enumerate(
            [("Raw", self.raw_var), ("绝对角度", self.abs_deg_var), ("零点角度", self.deg_var),
             ("RPM", self.rpm_var), ("温度", self.temp_var), ("接口模式", self.board_mode_var)]
        ):
            ttk.Label(data, text=title, font=("Segoe UI", 10)).grid(row=0, column=col, sticky=tk.W, padx=(0 if col == 0 else 22, 0))
            ttk.Label(data, textvariable=var, font=("Consolas", 20, "bold")).grid(row=1, column=col, sticky=tk.W, padx=(0 if col == 0 else 22, 0))
            data.columnconfigure(col, weight=1)

        body = ttk.Frame(root)
        body.pack(fill=tk.BOTH, expand=True, pady=(12, 0))
        body.columnconfigure(0, weight=0, minsize=540)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        left = ttk.Frame(body)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        right = ttk.Frame(body)
        right.grid(row=0, column=1, sticky="nsew")
        right.rowconfigure(3, weight=1)
        right.columnconfigure(0, weight=1)

        mode = ttk.LabelFrame(left, text="工作模式 / 驱动参数")
        mode.pack(fill=tk.X)
        ttk.Combobox(mode, textvariable=self.mode_var, values=list(MODE_LABELS.keys()), width=12, state="readonly").pack(side=tk.LEFT, padx=(8, 8), pady=8)
        ttk.Button(mode, text="设置模式", command=self.send_mode).pack(side=tk.LEFT, padx=(0, 12), pady=8)
        ttk.Label(mode, text="细分").pack(side=tk.LEFT, padx=(0, 6), pady=8)
        self.microstep_combo = ttk.Combobox(mode, textvariable=self.microstep_var, values=["1", "2", "4", "8", "16", "32", "64", "128", "256"], width=6, state="readonly")
        self.microstep_combo.pack(side=tk.LEFT, padx=(0, 8), pady=8)
        self.microstep_combo.bind("<<ComboboxSelected>>", self._on_microstep_selected)
        ttk.Button(mode, text="设置细分", command=self.send_microstep).pack(side=tk.LEFT, padx=(0, 12), pady=8)

        drive = ttk.Frame(left)
        drive.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(drive, text="使能", command=lambda: self.send_enable(1)).pack(side=tk.LEFT)
        ttk.Button(drive, text="失能", command=lambda: self.send_enable(0)).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(drive, text="数据流开", command=lambda: self.send_stream(1)).pack(side=tk.LEFT, padx=(16, 0))
        ttk.Button(drive, text="数据流关", command=lambda: self.send_stream(0)).pack(side=tk.LEFT, padx=(8, 0))

        mode_status = ttk.Frame(left)
        mode_status.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(mode_status, textvariable=self.mode_status_var).pack(side=tk.LEFT, padx=(0, 12))
        ttk.Label(mode_status, textvariable=self.microstep_status_var).pack(side=tk.LEFT, padx=(0, 12))
        ttk.Label(mode_status, textvariable=self.enable_status_var).pack(side=tk.LEFT, padx=(0, 12))
        ttk.Label(mode_status, textvariable=self.stream_status_var).pack(side=tk.LEFT)

        speed = ttk.LabelFrame(left, text="速度模式")
        speed.pack(fill=tk.X, pady=(12, 0))
        ttk.Label(speed, text="Speed -5000~5000").pack(side=tk.LEFT, padx=(8, 6), pady=8)
        ttk.Entry(speed, textvariable=self.speed_var, width=10).pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Label(speed, text="Accel").pack(side=tk.LEFT, padx=(0, 6), pady=8)
        ttk.Entry(speed, textvariable=self.accel_var, width=10).pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Button(speed, text="运行速度", command=self.send_speed_value).pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Button(speed, text="停止", command=self.send_stop).pack(side=tk.LEFT, padx=(0, 12), pady=8)
        ttk.Label(speed, textvariable=self.speed_status_var).pack(side=tk.LEFT, padx=(0, 8), pady=8)

        pulse = ttk.LabelFrame(left, text="位置模式：脉冲数移动")
        pulse.pack(fill=tk.X, pady=(12, 0))
        ttk.Label(pulse, text="脉冲数").pack(side=tk.LEFT, padx=(8, 6), pady=8)
        ttk.Entry(pulse, textvariable=self.move_steps_var, width=12).pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Label(pulse, text="速度").pack(side=tk.LEFT, padx=(0, 6), pady=8)
        ttk.Entry(pulse, textvariable=self.move_speed_var, width=10).pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Label(pulse, text="加速度").pack(side=tk.LEFT, padx=(0, 6), pady=8)
        ttk.Entry(pulse, textvariable=self.move_accel_var, width=10).pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Button(pulse, text="发送脉冲", command=self.send_move_steps).pack(side=tk.LEFT, padx=(0, 12), pady=8)
        ttk.Label(pulse, textvariable=self.move_status_var).pack(side=tk.LEFT, padx=(0, 8), pady=8)

        angle = ttk.LabelFrame(left, text="位置模式：角度闭环")
        angle.pack(fill=tk.X, pady=(12, 0))
        ttk.Label(angle, text="目标 0~360°").pack(side=tk.LEFT, padx=(8, 6), pady=8)
        ttk.Entry(angle, textvariable=self.target_angle_var, width=12).pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Button(angle, text="发送角度", command=self.send_angle_value).pack(side=tk.LEFT, padx=(0, 12), pady=8)
        ttk.Label(angle, textvariable=self.angle_status_var).pack(side=tk.LEFT, padx=(0, 8), pady=8)

        zero = ttk.LabelFrame(right, text="零点 / 回零")
        zero.grid(row=0, column=0, sticky="ew")
        ttk.Combobox(zero, textvariable=self.home_mode_var, values=list(HOME_MODE_LABELS.keys()), width=12, state="readonly").pack(side=tk.LEFT, padx=(8, 8), pady=8)
        ttk.Button(zero, text="设置回零模式", command=self.send_home_mode).pack(side=tk.LEFT, padx=(0, 12), pady=8)
        ttk.Button(zero, text="当前位置设为零点", command=self.send_set_zero).pack(side=tk.LEFT, padx=(0, 12), pady=8)
        ttk.Button(zero, text="回零", command=self.send_home).pack(side=tk.LEFT, padx=(0, 12), pady=8)
        ttk.Label(zero, textvariable=self.home_status_var).pack(side=tk.LEFT, padx=(0, 8), pady=8)

        pid = ttk.LabelFrame(right, text="闭环位置参数")
        pid.grid(row=1, column=0, sticky="ew", pady=(12, 0))
        labels = [("Kp", self.kp_var), ("Ki", self.ki_var), ("Kd", self.kd_var), ("Limit", self.limit_var), ("Deadband", self.deadband_var), ("Accel", self.closed_accel_var)]
        for i, (label, var) in enumerate(labels):
            row = i // 3
            col = (i % 3) * 2
            ttk.Label(pid, text=label).grid(row=row, column=col, padx=(8, 4), pady=8, sticky=tk.W)
            ttk.Entry(pid, textvariable=var, width=10).grid(row=row, column=col + 1, padx=(0, 10), pady=8, sticky=tk.W)
        ttk.Button(pid, text="写入PID", command=self.send_pid_values).grid(row=0, column=6, padx=(0, 8), pady=8, sticky=tk.W)
        ttk.Button(pid, text="写入加速度", command=self.send_closed_accel).grid(row=1, column=6, padx=(0, 8), pady=8, sticky=tk.W)
        ttk.Button(pid, text="填入默认参数", command=self.fill_default_motion_params).grid(row=2, column=6, padx=(0, 8), pady=(0, 8), sticky=tk.W)
        ttk.Label(pid, textvariable=self.pid_status_var).grid(row=2, column=0, columnspan=6, padx=8, pady=(0, 8), sticky=tk.W)
        ttk.Label(pid, textvariable=self.closed_accel_status_var).grid(row=3, column=0, columnspan=6, padx=8, pady=(0, 8), sticky=tk.W)

        tracking = ttk.LabelFrame(right, text="内置误差PID（追踪模式固定32细分+插值）")
        tracking.grid(row=2, column=0, sticky="ew", pady=(12, 0))
        ttk.Combobox(tracking, textvariable=self.track_mode_var,
                     values=list(TRACK_MODE_LABELS.keys()), width=12,
                     state="readonly").grid(row=0, column=0, padx=8, pady=6)
        for col, (label, var) in enumerate((("速度%", self.track_speed_pct_var),
                                            ("加速度%", self.track_accel_pct_var),
                                            ("频率kHz", self.track_freq_var))):
            ttk.Label(tracking, text=label).grid(row=0, column=1 + col * 2, padx=(4, 2), pady=6)
            ttk.Entry(tracking, textvariable=var, width=6).grid(row=0, column=2 + col * 2, padx=(0, 6), pady=6)
        ttk.Button(tracking, text="写入基础", command=self.send_track_basic).grid(row=0, column=7, padx=4, pady=6)
        ttk.Button(tracking, text="恢复默认", command=self.send_track_defaults).grid(row=0, column=8, padx=4, pady=6)
        ttk.Button(tracking, text="高级参数", command=self.open_track_advanced).grid(row=0, column=9, padx=(4, 8), pady=6)

        pid_rows = (("单环", self.track_single_kp_var, self.track_single_ki_var, self.track_single_kd_var, 0),
                    ("外环", self.track_outer_kp_var, self.track_outer_ki_var, self.track_outer_kd_var, 1),
                    ("内环", self.track_inner_kp_var, self.track_inner_ki_var, self.track_inner_kd_var, 2))
        for row, (name, kp_var, ki_var, kd_var, loop_id) in enumerate(pid_rows, start=1):
            ttk.Label(tracking, text=name).grid(row=row, column=0, padx=8, pady=3, sticky=tk.W)
            for col, (label, var) in enumerate((("Kp", kp_var), ("Ki", ki_var), ("Kd", kd_var))):
                ttk.Label(tracking, text=label).grid(row=row, column=1 + col * 2, padx=(4, 2), pady=3)
                ttk.Entry(tracking, textvariable=var, width=8).grid(row=row, column=2 + col * 2, padx=(0, 6), pady=3)
            ttk.Button(tracking, text="写入", command=lambda loop=loop_id: self.send_track_pid(loop)).grid(row=row, column=7, padx=4, pady=3)

        ttk.Label(tracking, text="主误差").grid(row=4, column=0, padx=8, pady=6, sticky=tk.W)
        ttk.Entry(tracking, textvariable=self.track_primary_var, width=8).grid(row=4, column=1, columnspan=2, padx=4, pady=6)
        ttk.Label(tracking, text="内环反馈").grid(row=4, column=3, padx=4, pady=6)
        ttk.Entry(tracking, textvariable=self.track_secondary_var, width=8).grid(row=4, column=4, columnspan=2, padx=4, pady=6)
        ttk.Button(tracking, text="发送测试误差", command=self.send_track_input).grid(row=4, column=7, padx=4, pady=6)
        ttk.Button(tracking, text="读取追踪参数", command=self.send_track_query).grid(row=4, column=8, padx=4, pady=6)
        ttk.Label(tracking, textvariable=self.track_status_var).grid(row=5, column=0, columnspan=10, padx=8, pady=(2, 6), sticky=tk.W)

        wave = ttk.LabelFrame(right, text="目标/当前角度波形")
        wave.grid(row=3, column=0, sticky="nsew", pady=(12, 0))
        wave_toolbar = ttk.Frame(wave)
        wave_toolbar.pack(fill=tk.X, padx=8, pady=(8, 4))
        ttk.Checkbutton(wave_toolbar, text="暂停波形", variable=self.wave_paused_var).pack(side=tk.LEFT)
        ttk.Checkbutton(wave_toolbar, text="跟随最新", variable=self.wave_follow_var).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Button(wave_toolbar, text="放大", command=lambda: self.zoom_waveform(1.35)).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Button(wave_toolbar, text="缩小", command=lambda: self.zoom_waveform(1.0 / 1.35)).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(wave_toolbar, text="清空波形", command=self.clear_waveform).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Label(wave_toolbar, text="蓝色=current，橙色=target，拖动滚动条可回看").pack(side=tk.LEFT, padx=(16, 0))

        canvas_wrap = ttk.Frame(wave)
        canvas_wrap.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        self.wave_canvas = tk.Canvas(canvas_wrap, height=180, bg="#101418", highlightthickness=1, highlightbackground="#34404a")
        self.wave_canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.wave_scroll = ttk.Scrollbar(canvas_wrap, orient=tk.HORIZONTAL, command=self._wave_xview)
        self.wave_scroll.pack(side=tk.BOTTOM, fill=tk.X)
        self.wave_canvas.configure(xscrollcommand=self.wave_scroll.set)
        self.wave_canvas.bind("<Configure>", lambda _event: self._redraw_waveform())

    def refresh_ports(self, auto_connect=False):
        if list_ports is None:
            return
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        if not ports:
            self.status_var.set("没有发现串口设备")
            self.port_var.set("")
            return
        if auto_connect and self.auto_var.get() and self.connected_port is None:
            self.connect_selected()

    def connect_selected(self):
        if serial is None:
            self.status_var.set("缺少 pyserial，无法连接")
            return
        port = self.port_var.get().strip()
        if not port:
            self.status_var.set("没有选择串口")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            self.status_var.set("波特率不是有效数字")
            return
        self.connected_port = port
        self._reset_motion_state()
        self.status_var.set(f"正在连接 {port} ...")
        self.reader.start(port, baud)

    def disconnect(self):
        self.connected_port = None
        self._reset_motion_state()
        self.reader.stop()
        self.status_var.set("已请求断开")

    def send_mode(self):
        mode = MODE_LABELS.get(self.mode_var.get(), MODE_CLOSED_POSITION)
        if self._send(CMD_MODE, struct.pack("<B", mode)):
            self.mode_status_var.set(f"模式：已发送 {self.mode_var.get()}")

    def send_microstep(self):
        try:
            microsteps = int(self.microstep_var.get())
        except ValueError:
            self.microstep_status_var.set("细分：请输入有效数字")
            return
        if microsteps not in (1, 2, 4, 8, 16, 32, 64, 128, 256):
            self.microstep_status_var.set("细分：只支持 1/2/4/8/16/32/64/128/256")
            return
        if self._send(CMD_MICROSTEP, struct.pack("<H", microsteps)):
            self.fill_default_motion_params(show_status=False)
            self.microstep_status_var.set(f"细分：已发送 {microsteps}，已填入默认参数")

    def _on_microstep_selected(self, _event=None):
        self.fill_default_motion_params()

    def fill_default_motion_params(self, show_status=True):
        try:
            microsteps = int(self.microstep_var.get())
        except ValueError:
            if show_status:
                self.pid_status_var.set("PID：细分不是有效数字")
            return
        params = DEFAULT_MOTION_PARAMS.get(microsteps)
        if params is None:
            if show_status:
                self.pid_status_var.set("PID：当前细分没有默认参数")
            return

        self.kp_var.set(f"{params['kp']:.2f}")
        self.ki_var.set(f"{params['ki']:.2f}")
        self.kd_var.set(f"{params['kd']:.2f}")
        self.limit_var.set(str(params["limit"]))
        self.deadband_var.set(f"{params['deadband']:.2f}")
        self.closed_accel_var.set(str(params["accel"]))
        if show_status:
            self.pid_status_var.set(
                f"PID：已填入 {microsteps} 细分默认 Kp={params['kp']:.2f}, Ki={params['ki']:.2f}, "
                f"Kd={params['kd']:.2f}, Deadband={params['deadband']:.2f}"
            )
            self.closed_accel_status_var.set(f"闭环加速度：已填入默认 {params['accel']}")

    def send_closed_accel(self):
        try:
            accel = int(float(self.closed_accel_var.get()))
        except ValueError:
            self.closed_accel_status_var.set("闭环加速度：请输入有效数字")
            return
        if accel < 1 or accel > 65535:
            self.closed_accel_status_var.set("闭环加速度：范围 1~65535")
            return
        if self._send(CMD_CLOSED_ACCEL, struct.pack("<H", accel)):
            self.closed_accel_status_var.set(f"闭环加速度：已发送 {accel}")

    def send_home_mode(self):
        mode = HOME_MODE_LABELS.get(self.home_mode_var.get(), 0)
        if self._send(CMD_HOME_MODE, struct.pack("<B", mode)):
            self.home_status_var.set(f"回零模式：已发送 {self.home_mode_var.get()}")

    def send_set_zero(self):
        if self._send(CMD_SET_ZERO):
            self.home_status_var.set("零点：已发送保存请求")

    def send_home(self):
        if self.current_mode != MODE_CLOSED_POSITION:
            self.home_status_var.set("回零：当前不是闭环位置模式")
            return
        mode = HOME_MODE_LABELS.get(self.home_mode_var.get(), 0)
        if self._send(CMD_HOME, struct.pack("<B", mode)):
            self.target_angle_deg = 0.0
            self._append_wave_sample(time.monotonic())
            self.home_status_var.set(f"回零：已发送 {self.home_mode_var.get()}")

    def send_enable(self, enable):
        if self._send(CMD_ENABLE, struct.pack("<B", 1 if enable else 0)):
            self.enable_status_var.set("电机：已发送使能" if enable else "电机：已发送失能")

    def send_stream(self, enable):
        if self._send(CMD_STREAM, struct.pack("<B", 1 if enable else 0)):
            self.stream_status_var.set("数据流：已发送开启" if enable else "数据流：已发送关闭")

    def send_query(self):
        self._send(CMD_QUERY)

    def send_stop(self):
        if self._send(CMD_STOP):
            self.speed_status_var.set("速度：已发送停止")
            self.angle_status_var.set("角度：已发送停止")

    def send_pid_values(self):
        try:
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            kd = float(self.kd_var.get())
            limit = int(float(self.limit_var.get()))
            deadband = float(self.deadband_var.get())
        except ValueError:
            self.pid_status_var.set("PID：参数不是有效数字")
            return
        kp_i = int(round(kp * 100))
        ki_i = int(round(ki * 100))
        kd_i = int(round(kd * 100))
        dead_i = int(round(deadband * 1000))
        if not (-2147483648 <= kp_i <= 2147483647 and -2147483648 <= ki_i <= 2147483647 and -2147483648 <= kd_i <= 2147483647):
            self.pid_status_var.set("PID：Kp/Ki/Kd 超范围")
            return
        if not (1 <= limit <= 65535 and 0 <= dead_i <= 10000):
            self.pid_status_var.set("PID：Limit 或 Deadband 超范围")
            return
        payload = struct.pack(
            "<iiiHH",
            kp_i,
            ki_i,
            kd_i,
            limit,
            dead_i,
        )
        if self._send(CMD_PID, payload):
            self.pid_status_var.set("PID：已发送写入")

    def send_speed_value(self):
        if self.current_mode != MODE_OPEN_SPEED:
            self.speed_status_var.set("速度：当前不是速度模式，先设置为速度模式")
            return
        try:
            speed = int(float(self.speed_var.get()))
            accel = int(float(self.accel_var.get()))
        except ValueError:
            self.speed_status_var.set("速度：请输入有效数字")
            return
        if speed < -5000 or speed > 5000 or accel <= 0:
            self.speed_status_var.set("速度：参数超范围")
            return
        payload = struct.pack("<hH", speed, accel)
        if self._send(CMD_SPEED, payload):
            self.speed_status_var.set(f"速度：已发送 speed={speed}, accel={accel}")

    def send_move_steps(self):
        if self.current_mode not in (MODE_OPEN_POSITION, MODE_CLOSED_POSITION):
            self.move_status_var.set("脉冲位置：当前不是位置模式")
            return
        try:
            steps = int(float(self.move_steps_var.get()))
            speed = int(float(self.move_speed_var.get()))
            accel = int(float(self.move_accel_var.get()))
        except ValueError:
            self.move_status_var.set("脉冲位置：请输入有效数字")
            return
        if steps == 0 or speed <= 0 or speed > 5000 or accel <= 0 or accel > 65535:
            self.move_status_var.set("脉冲位置：参数超范围")
            return
        if steps < -2147483648 or steps > 2147483647:
            self.move_status_var.set("脉冲位置：脉冲数超范围")
            return
        payload = struct.pack("<iHH", steps, speed, accel)
        if self._send(CMD_MOVE_STEPS, payload):
            self.move_status_var.set(f"脉冲位置：已发送 steps={steps}, speed={speed}, accel={accel}")

    def send_angle_value(self):
        if self.current_mode != MODE_CLOSED_POSITION:
            self.angle_status_var.set("角度：当前不是闭环位置模式")
            return
        try:
            angle = float(self.target_angle_var.get())
        except ValueError:
            self.angle_status_var.set("角度：请输入有效数字")
            return
        if angle < 0.0 or angle > 360.0:
            self.angle_status_var.set("角度：范围必须是 0~360")
            return
        payload = struct.pack("<H", int(round(angle * 100)))
        if self._send(CMD_ANGLE, payload):
            self.target_angle_deg = angle
            self._append_wave_sample(time.monotonic())
            self.angle_status_var.set(f"角度：已发送 {angle:.3f}°")

    def send_track_basic(self):
        try:
            mode = TRACK_MODE_LABELS[self.track_mode_var.get()]
            speed_pct = int(self.track_speed_pct_var.get())
            accel_pct = int(self.track_accel_pct_var.get())
            frequency = int(self.track_freq_var.get())
        except (ValueError, KeyError):
            self.track_status_var.set("误差PID：基础参数格式错误")
            return
        if not (1 <= speed_pct <= 100 and 1 <= accel_pct <= 100 and 1 <= frequency <= 10):
            self.track_status_var.set("误差PID：速度/加速度为1~100%，频率为1~10kHz")
            return
        if self._send(CMD_TRACK_CONFIG, struct.pack("<BBBB", mode, speed_pct, accel_pct, frequency)):
            self.track_status_var.set(
                f"误差PID：已发送 {self.track_mode_var.get()}，速度{speed_pct}%={speed_pct * 50}，"
                f"加速度{accel_pct}%={accel_pct * 100}，{frequency}kHz"
            )

    def send_track_pid(self, loop):
        variables = (
            (self.track_single_kp_var, self.track_single_ki_var, self.track_single_kd_var),
            (self.track_outer_kp_var, self.track_outer_ki_var, self.track_outer_kd_var),
            (self.track_inner_kp_var, self.track_inner_ki_var, self.track_inner_kd_var),
        )
        try:
            kp, ki, kd = (float(var.get()) for var in variables[loop])
        except (ValueError, IndexError):
            self.track_status_var.set("误差PID：PID参数格式错误")
            return
        if min(kp, ki, kd) < 0 or max(kp, ki, kd) > 5000:
            self.track_status_var.set("误差PID：Kp/Ki/Kd范围为0~5000")
            return
        payload = struct.pack("<Biii", loop, int(round(kp * 100)), int(round(ki * 100)), int(round(kd * 100)))
        if self._send(CMD_TRACK_PID, payload):
            self.track_status_var.set(f"误差PID：已发送第{loop + 1}组PID")

    def send_track_input(self):
        if self.current_mode != MODE_TRACK_PID:
            self.track_status_var.set("误差PID：请先把工作模式设置为内置误差PID")
            return
        try:
            primary = float(self.track_primary_var.get())
            secondary = float(self.track_secondary_var.get())
        except ValueError:
            self.track_status_var.set("误差PID：误差输入格式错误")
            return
        if not (-327.68 <= primary <= 327.67 and -327.68 <= secondary <= 327.67):
            self.track_status_var.set("误差PID：输入范围为-327.68~327.67")
            return
        payload = struct.pack("<hh", int(round(primary * 100)), int(round(secondary * 100)))
        if self._send(CMD_TRACK_INPUT, payload):
            self.track_status_var.set(f"误差PID：已发送主误差{primary:.2f}，内环反馈{secondary:.2f}")

    def send_track_defaults(self):
        if self._send(CMD_TRACK_DEFAULT):
            self.track_status_var.set("误差PID：已发送恢复默认请求")
            self.after(120, self.send_track_query)

    def send_track_query(self):
        self._send(CMD_TRACK_QUERY)

    def send_track_advanced(self):
        try:
            values = [
                int(round(float(self.track_filter_var.get()) * 1000)),
                int(round(float(self.track_gain_min_var.get()) * 1000)),
                int(round(float(self.track_gain_max_var.get()) * 1000)),
                int(round(float(self.track_full_error_var.get()) * 100)),
                int(round(float(self.track_prediction_var.get()))),
                int(round(float(self.track_timeout_var.get()))),
                int(round(float(self.track_single_dead_var.get()) * 1000)),
                int(round(float(self.track_outer_dead_var.get()) * 1000)),
                int(round(float(self.track_inner_dead_var.get()) * 1000)),
            ]
        except ValueError:
            self.track_status_var.set("误差PID：高级参数格式错误")
            return
        if any(value < 0 or value > 65535 for value in values):
            self.track_status_var.set("误差PID：高级参数超出协议范围")
            return
        if self._send(CMD_TRACK_ADVANCED, struct.pack("<9H", *values)):
            self.track_status_var.set("误差PID：已发送高级参数")

    def open_track_advanced(self):
        window = tk.Toplevel(self)
        window.title("误差PID高级参数")
        window.resizable(False, False)
        fields = [
            ("误差滤波系数 0~1", self.track_filter_var),
            ("动态增益最小值", self.track_gain_min_var),
            ("动态增益最大值", self.track_gain_max_var),
            ("达到最大增益的误差", self.track_full_error_var),
            ("预测时间 ms", self.track_prediction_var),
            ("输入超时 ms", self.track_timeout_var),
            ("单环死区", self.track_single_dead_var),
            ("外环死区", self.track_outer_dead_var),
            ("内环死区", self.track_inner_dead_var),
        ]
        for row, (label, var) in enumerate(fields):
            ttk.Label(window, text=label).grid(row=row, column=0, padx=12, pady=6, sticky=tk.W)
            ttk.Entry(window, textvariable=var, width=14).grid(row=row, column=1, padx=12, pady=6)
        ttk.Button(window, text="写入高级参数", command=self.send_track_advanced).grid(
            row=len(fields), column=0, columnspan=2, padx=12, pady=12
        )

    def _auto_refresh_ports(self):
        self.refresh_ports(auto_connect=True)
        self.after(1000, self._auto_refresh_ports)

    def _poll_queues(self):
        while True:
            try:
                cmd, seq, payload = self.frame_queue.get_nowait()
            except queue.Empty:
                break
            self._process_frame(cmd, seq, payload)

        while True:
            try:
                kind, message = self.status_queue.get_nowait()
            except queue.Empty:
                break
            self.status_var.set(message)
            if kind == "connected":
                self.after(100, self.send_query)
                self.after(160, self.send_track_query)
            if kind in ("closed", "error"):
                self.connected_port = None

        self.after(30, self._poll_queues)

    def _process_frame(self, cmd, seq, payload):
        if cmd == CMD_QUERY and len(payload) >= 10:
            status, enabled, stream, mode, raw, angle_cdeg, pid_output = struct.unpack("<BBBBHHh", payload[:10])
            pid_params = None
            if len(payload) >= 32:
                kp_i, ki_i, kd_i, limit, dead_i = struct.unpack("<iiiHH", payload[10:26])
                pid_params = (kp_i / 100.0, ki_i / 100.0, kd_i / 100.0, limit, dead_i / 1000.0)
            elif len(payload) >= 20:
                kp_i, ki_i, kd_i, limit, dead_i = struct.unpack("<hhhHH", payload[10:20])
                pid_params = (kp_i / 100.0, ki_i / 100.0, kd_i / 100.0, limit, dead_i / 1000.0)
            microsteps = None
            closed_accel = None
            zero_valid = None
            home_mode = None
            if len(payload) >= 32:
                microsteps = struct.unpack("<H", payload[26:28])[0]
                closed_accel = struct.unpack("<H", payload[28:30])[0]
                zero_valid = payload[30]
                home_mode = payload[31]
            elif len(payload) >= 22:
                microsteps = struct.unpack("<H", payload[20:22])[0]
            extended = None
            if len(payload) >= 48:
                board_mode = payload[32]
                sensor_flags = payload[33]
                temp_cdeg, rpm_x10 = struct.unpack("<hh", payload[34:38])
                track_mode, track_freq, track_speed, track_accel = struct.unpack("<BBBB", payload[38:42])
                primary_x100, secondary_x100, track_output = struct.unpack("<hhh", payload[42:48])
                extended = (board_mode, sensor_flags, temp_cdeg / 100.0, rpm_x10 / 10.0,
                            track_mode, track_freq, track_speed, track_accel,
                            primary_x100 / 100.0, secondary_x100 / 100.0, track_output)
            self._display_status(status, enabled, stream, mode, raw, angle_cdeg / 100.0,
                                 pid_output, pid_params, microsteps, closed_accel,
                                 zero_valid, home_mode, extended)
            return
        if cmd == CMD_TRACK_QUERY and len(payload) >= 59:
            if payload[0] != 0:
                self.track_status_var.set(f"误差PID：{STATUS_NAMES.get(payload[0], payload[0])}")
                return
            mode, speed_pct, accel_pct, frequency = struct.unpack("<BBBB", payload[1:5])
            pid_values = struct.unpack("<9i", payload[5:41])
            advanced = struct.unpack("<9H", payload[41:59])
            self.track_mode_var.set(TRACK_MODE_NAMES.get(mode, "单环PID"))
            self.track_speed_pct_var.set(str(speed_pct))
            self.track_accel_pct_var.set(str(accel_pct))
            self.track_freq_var.set(str(frequency))
            variables = (
                self.track_single_kp_var, self.track_single_ki_var, self.track_single_kd_var,
                self.track_outer_kp_var, self.track_outer_ki_var, self.track_outer_kd_var,
                self.track_inner_kp_var, self.track_inner_ki_var, self.track_inner_kd_var,
            )
            for var, value in zip(variables, pid_values):
                var.set(f"{value / 100.0:.2f}")
            advanced_vars = (
                self.track_filter_var, self.track_gain_min_var, self.track_gain_max_var,
                self.track_full_error_var, self.track_prediction_var, self.track_timeout_var,
                self.track_single_dead_var, self.track_outer_dead_var, self.track_inner_dead_var,
            )
            scales = (1000.0, 1000.0, 1000.0, 100.0, 1.0, 1.0, 1000.0, 1000.0, 1000.0)
            for var, value, scale in zip(advanced_vars, advanced, scales):
                var.set(f"{value / scale:g}")
            self.track_status_var.set(
                f"误差PID：{TRACK_MODE_NAMES.get(mode, '未知')}，速度{speed_pct}%={speed_pct * 50}，"
                f"加速度{accel_pct}%={accel_pct * 100}，{frequency}kHz"
            )
            return
        if payload:
            status = payload[0]
            text = STATUS_NAMES.get(status, f"ERR {status}")
            if cmd == CMD_ENABLE:
                self.enable_status_var.set(f"电机：{text}")
            elif cmd == CMD_MODE:
                self.mode_status_var.set(f"模式：{text}")
            elif cmd == CMD_STREAM:
                self.stream_status_var.set(f"数据流：{text}")
            elif cmd == CMD_PID:
                self.pid_status_var.set(f"PID：{text}")
            elif cmd == CMD_SPEED:
                self.speed_status_var.set(f"速度：{text}")
            elif cmd == CMD_ANGLE:
                self.angle_status_var.set(f"角度：{text}")
            elif cmd == CMD_STOP:
                self.speed_status_var.set(f"停止：{text}")
            elif cmd == CMD_MOVE_STEPS:
                self.move_status_var.set(f"脉冲位置：{text}")
            elif cmd == CMD_MICROSTEP:
                self.microstep_status_var.set(f"细分：{text}")
            elif cmd == CMD_CLOSED_ACCEL:
                self.closed_accel_status_var.set(f"闭环加速度：{text}")
            elif cmd == CMD_HOME_MODE:
                self.home_status_var.set(f"回零模式：{text}")
            elif cmd == CMD_SET_ZERO:
                self.home_status_var.set(f"设置零点：{text}")
            elif cmd == CMD_HOME:
                self.home_status_var.set(f"回零：{text}")
            elif cmd in (CMD_TRACK_CONFIG, CMD_TRACK_PID, CMD_TRACK_ADVANCED,
                          CMD_TRACK_INPUT, CMD_TRACK_DEFAULT, CMD_TRACK_QUERY):
                self.track_status_var.set(f"误差PID：{text}")
        self._send(CMD_QUERY)

    def _display_status(self, status, enabled, stream, mode, raw, deg, pid_output,
                        pid_params=None, microsteps=None, closed_accel=None,
                        zero_valid=None, home_mode=None, extended=None):
        now = time.monotonic()
        rpm = None
        if extended is None:
            rpm = self._calc_rpm(raw, now)
        else:
            rpm = extended[3]
        abs_deg = (float(raw & 0x3FFF) * 360.0) / RAW_MAX
        self.current_angle_deg = deg
        self.raw_var.set(f"{raw:5d}")
        self.abs_deg_var.set(f"{abs_deg:7.3f}°")
        self.deg_var.set(f"{deg:7.3f}°")
        if (now - self.last_rpm_display_time) >= RPM_DISPLAY_INTERVAL:
            self.rpm_var.set(f"{rpm:7.2f}")
            self.last_rpm_display_time = now
        self._append_wave_sample(now)
        self.enable_status_var.set("电机：已使能" if enabled else "电机：已失能")
        self.stream_status_var.set("数据流：开启" if stream else "数据流：关闭")
        self.current_mode = mode
        self.mode_status_var.set(f"模式：{MODE_NAMES.get(mode, '未知')}")
        if microsteps is not None:
            self.microstep_status_var.set(f"细分：当前 {microsteps}")
        if closed_accel is not None:
            self.closed_accel_status_var.set(f"闭环加速度：当前 {closed_accel}")
        if zero_valid is not None:
            home_text = HOME_MODE_NAMES.get(home_mode, "未知")
            self.home_status_var.set(f"零点：{'已设置' if zero_valid else '未设置'}，回零模式：{home_text}")
        if pid_params is not None:
            kp, ki, kd, limit, deadband = pid_params
            self.pid_status_var.set(
                f"PID：当前 Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}, Limit={limit}, Deadband={deadband:.3f}"
            )
        if extended is not None:
            board_mode, sensor_flags, temperature, _rpm, track_mode, track_freq, track_speed, track_accel, primary, secondary, track_output = extended
            self.board_mode_var.set(BOARD_MODE_NAMES.get(board_mode, f"未知{board_mode}"))
            if sensor_flags & 0x01:
                suffix = " 关断" if sensor_flags & 0x04 else (" 警告" if sensor_flags & 0x02 else "")
                self.temp_var.set(f"{temperature:.1f}°C{suffix}")
            else:
                self.temp_var.set("传感器异常")
            self.track_status_var.set(
                f"误差PID：{TRACK_MODE_NAMES.get(track_mode, '未知')}，{track_freq}kHz，"
                f"速度{track_speed}% / 加速度{track_accel}%，误差{primary:.2f}，"
                f"内环反馈{secondary:.2f}，输出{track_output}"
            )
        if status != 0:
            self.status_var.set(f"设备状态错误：{status}")

    def _calc_rpm(self, raw, now):
        if self.last_raw is None:
            self.last_raw = raw
            self.unwrapped_raw = float(raw)
            self.speed_window.clear()
            self.speed_window.append((now, self.unwrapped_raw))
            self.rpm_filtered = 0.0
            return 0.0
        delta = raw - self.last_raw
        half = int(RAW_MAX / 2)
        if delta > half:
            delta -= int(RAW_MAX)
        elif delta < -half:
            delta += int(RAW_MAX)
        self.unwrapped_raw += delta
        self.last_raw = raw
        self.speed_window.append((now, self.unwrapped_raw))
        while self.speed_window and (now - self.speed_window[0][0]) > RPM_WINDOW_SECONDS:
            self.speed_window.popleft()
        if len(self.speed_window) < 2:
            return self.rpm_filtered
        first_time, first_raw = self.speed_window[0]
        last_time, last_raw = self.speed_window[-1]
        dt = last_time - first_time
        if dt <= 0.0:
            return self.rpm_filtered
        rpm = ((last_raw - first_raw) / RAW_MAX) / dt * 60.0
        self.rpm_filtered = self.rpm_filtered * 0.85 + rpm * 0.15
        return self.rpm_filtered

    def _reset_motion_state(self):
        self.last_raw = None
        self.unwrapped_raw = 0.0
        self.speed_window.clear()
        self.rpm_filtered = 0.0
        self.last_rpm_display_time = 0.0
        self.rpm_var.set("--")
        self.temp_var.set("--")
        self.board_mode_var.set("--")
        self.abs_deg_var.set("--")
        self.deg_var.set("--")
        self.current_angle_deg = None
        self.target_angle_deg = None
        self.wave_history.clear()
        if hasattr(self, "wave_canvas"):
            self._redraw_waveform()

    def clear_waveform(self):
        self.wave_history.clear()
        self._redraw_waveform()

    def zoom_waveform(self, scale):
        self.wave_pixels_per_second = max(5.0, min(180.0, self.wave_pixels_per_second * scale))
        self._redraw_waveform()

    def _wave_xview(self, *args):
        self.wave_follow_var.set(False)
        self.wave_canvas.xview(*args)

    def _append_wave_sample(self, now):
        if self.current_angle_deg is None and self.target_angle_deg is None:
            return
        if self.wave_paused_var.get():
            return

        self.wave_history.append((now, self.current_angle_deg, self.target_angle_deg))
        while self.wave_history and (now - self.wave_history[0][0]) > WAVEFORM_HISTORY_SECONDS:
            self.wave_history.popleft()
        self._redraw_waveform()

    def _angle_to_y(self, angle, top, height):
        angle = max(0.0, min(360.0, float(angle)))
        return top + (360.0 - angle) * height / 360.0

    def _redraw_waveform(self):
        if not hasattr(self, "wave_canvas"):
            return

        canvas = self.wave_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 600)
        height = max(canvas.winfo_height(), 160)
        left = 48
        right_pad = 16
        top = 14
        graph_h = height - 42

        history = list(self.wave_history)
        if len(history) == 0:
            canvas.configure(scrollregion=(0, 0, width, height))
            canvas.create_text(width / 2, height / 2, text="等待角度数据", fill="#b7c0c7", font=("Segoe UI", 12))
            return

        start_t = history[0][0]
        end_t = max(history[-1][0], start_t + 1.0)
        graph_w = max(width - left - right_pad, int((end_t - start_t) * self.wave_pixels_per_second) + 80)
        total_w = left + graph_w + right_pad
        canvas.configure(scrollregion=(0, 0, total_w, height))

        for angle in (0, 90, 180, 270, 360):
            y = self._angle_to_y(angle, top, graph_h)
            canvas.create_line(left, y, total_w - right_pad, y, fill="#26323a")
            canvas.create_text(8, y, anchor=tk.W, text=str(angle), fill="#8997a1", font=("Consolas", 9))

        canvas.create_line(left, top, left, top + graph_h, fill="#5b6872")
        canvas.create_line(left, top + graph_h, total_w - right_pad, top + graph_h, fill="#5b6872")

        def make_points(index):
            points = []
            last_value = None
            for t, current, target in history:
                value = current if index == 1 else target
                if value is None:
                    value = last_value
                if value is None:
                    continue
                last_value = value
                x = left + (t - start_t) * self.wave_pixels_per_second
                y = self._angle_to_y(value, top, graph_h)
                points.extend((x, y))
            return points

        current_points = make_points(1)
        target_points = make_points(2)
        if len(target_points) >= 4:
            canvas.create_line(*target_points, fill="#f0a23a", width=2, smooth=False)
        if len(current_points) >= 4:
            canvas.create_line(*current_points, fill="#4aa3ff", width=2, smooth=False)

        if self.wave_follow_var.get():
            canvas.xview_moveto(1.0)

    def _on_close(self):
        self.reader.stop()
        self.destroy()


if __name__ == "__main__":
    app = AngleViewer()
    app.mainloop()
