# -*- coding: utf-8 -*-
import json
import math
import queue
import struct
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import filedialog, ttk, messagebox

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
MAX_PAYLOAD = 72

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
CMD_CHOPPER_MODE = 0x16
CMD_RELATIVE_ANGLE = 0x17
CMD_PROGRAM_MODE = 0x18
CMD_PROGRAM_STEP = 0x19
CMD_PROGRAM_SAVE = 0x1A
CMD_PROGRAM_RUN = 0x1B
CMD_PROGRAM_QUERY = 0x1C
CMD_PROGRAM_STAGE = 0x1D
CMD_MOTION_PROFILE = 0x1E
CMD_TRACK_DIRECTION = 0x1F
CMD_SAVE_RUNTIME_CONFIG = 0x20

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
CHOPPER_MODE_LABELS = {"静音 StealthChop": 0, "力矩 SpreadCycle": 1}
CHOPPER_MODE_NAMES = {value: key for key, value in CHOPPER_MODE_LABELS.items()}
MOTION_PROFILE_LABELS = {"梯形加减速": 0, "S 曲线加减速": 1}
MOTION_PROFILE_NAMES = {value: key for key, value in MOTION_PROFILE_LABELS.items()}

HOME_MODE_LABELS = {
    "就近回零": 0,
    "正向单圈回零": 1,
    "反向单圈回零": 2,
    "多圈绝对回零": 3,
}

HOME_MODE_NAMES = {value: key for key, value in HOME_MODE_LABELS.items()}
HOME_MODE_ALIASES = {"正向无限位": 1, "反向无限位": 2, "负向无限位": 2}


def home_mode_value(name):
    return HOME_MODE_LABELS.get(name, HOME_MODE_ALIASES.get(name, 0))

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
        self.title("Step42 闭环步进驱动控制中心")
        # This is deliberately a fixed desktop-tool layout so packaged EXEs
        # keep the same usable proportions on every production computer.
        self.geometry("1600x980")
        self.resizable(False, False)

        self.frame_queue = queue.Queue()
        self.status_queue = queue.Queue()
        self.tx_queue = deque()
        self.pending_tx = None
        self.reader = SerialReader(self.frame_queue.put, self.status_queue.put)
        self.seq = 0
        self.connected_port = None
        self.last_raw = None
        self.unwrapped_raw = 0.0
        self.speed_window = deque()
        self.rpm_filtered = 0.0
        self.last_rpm_display_time = 0.0
        self.current_angle_deg = None
        self.current_multi_turn_deg = None
        self.target_angle_deg = None
        self.position_error_deg = 0.0
        self.track_primary_error = 0.0
        self.track_secondary_feedback = 0.0
        self.track_output = 0
        self.wave_history = deque()
        self.wave_pixels_per_second = WAVEFORM_PIXELS_PER_SECOND
        self.wave_range_limit = 1000.0
        self.wave_paused_var = tk.BooleanVar(value=False)
        self.wave_follow_var = tk.BooleanVar(value=True)
        self.current_mode = MODE_CLOSED_POSITION
        self.current_microsteps = 32

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(BAUD_DEFAULT))
        self.interface_role_var = tk.StringVar(value="Error")
        self.auto_var = tk.BooleanVar(value=True)
        self.status_var = tk.StringVar(value="等待串口")
        self.operation_var = tk.StringVar(value="等待操作。")
        self.raw_var = tk.StringVar(value="--")
        self.abs_deg_var = tk.StringVar(value="--")
        self.deg_var = tk.StringVar(value="--")
        self.rpm_var = tk.StringVar(value="--")
        self.temp_var = tk.StringVar(value="--")
        self.board_mode_var = tk.StringVar(value="--")
        self.mode_var = tk.StringVar(value="闭环位置")
        self.mode_status_var = tk.StringVar(value="模式：未知")
        self.driver_status_var = tk.StringVar(value="实际驱动状态：等待读取")
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
        self.position_target_mode_var = tk.StringVar(value="绝对目标")
        self.target_angle_var = tk.StringVar(value="0.0")
        self.angle_status_var = tk.StringVar(value="角度：未发送")
        self.move_steps_var = tk.StringVar(value="3200")
        self.move_speed_var = tk.StringVar(value="2500")
        self.move_accel_var = tk.StringVar(value="5000")
        self.move_status_var = tk.StringVar(value="脉冲位置：未发送")
        self.speed_var = tk.StringVar(value="0")
        self.speed_unit_var = tk.StringVar(value="RPM")
        self.accel_var = tk.StringVar(value="3000")
        self.speed_status_var = tk.StringVar(value="速度：未发送")
        self.chopper_mode_var = tk.StringVar(value="静音 StealthChop")
        self.chopper_status_var = tk.StringVar(value="斩波模式：未知")
        self.motion_profile_var = tk.StringVar(value="梯形加减速")
        self.motion_profile_status_var = tk.StringVar(value="曲线：未知")
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
        self.track_reverse_var = tk.BooleanVar(value=False)
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
        self.dynamic_gain_enabled_var = tk.BooleanVar(value=False)
        self.prediction_enabled_var = tk.BooleanVar(value=False)
        self.sequence_type_var = tk.StringVar(value="位置移动")
        self.sequence_param_vars = {}
        self.sequence_param_frame = None
        self.sequence_status_var = tk.StringVar(value="指令列表仅保存在电脑，尚未写入驱动板。")
        self.sequence_steps = []
        self.sequence_ram_count = None
        self.sequence_tree = None

        self._build_ui()
        self._update_advanced_control_states()
        self.interface_role_var.trace_add("write", self._on_interface_role_changed)
        self._on_interface_role_changed()
        self._sequence_command_changed()
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

    def _send(self, cmd, payload=b"", context=None):
        seq = self._next_seq()
        frame = build_frame(cmd, seq, payload)
        item = {"cmd": cmd, "seq": seq, "frame": frame,
                "context": context, "sent_at": None}
        if self.pending_tx is None:
            if not self.reader.write_frame(frame):
                return False
            item["sent_at"] = time.monotonic()
            self.pending_tx = item
        else:
            self.tx_queue.append(item)
        return True

    def _send_next_queued(self):
        if self.pending_tx is not None or not self.tx_queue:
            return
        item = self.tx_queue.popleft()
        if self.reader.write_frame(item["frame"]):
            item["sent_at"] = time.monotonic()
            self.pending_tx = item

    def _complete_pending(self, cmd, seq, payload):
        item = self.pending_tx
        if item is None or item["cmd"] != cmd or item["seq"] != seq:
            return False
        self.pending_tx = None
        status = payload[0] if payload else None
        context = item.get("context") or {}
        if cmd in (CMD_ANGLE, CMD_RELATIVE_ANGLE):
            if status == 0 and "target_angle" in context:
                self.target_angle_deg = context["target_angle"]
                self._append_wave_sample(time.monotonic())
                self.angle_status_var.set(f"角度：设备已确认 {self.target_angle_deg:.3f}°")
            elif status == 0 and "relative_angle" in context:
                self.angle_status_var.set(f"相对移动：设备已确认 {context['relative_angle']:+.3f}°，正在刷新目标")
                self.after(80, self.send_query)
            elif status is not None:
                self.angle_status_var.set(f"角度：设备拒绝，{STATUS_NAMES.get(status, status)}")
        self.after_idle(self._send_next_queued)
        return True

    def _build_ui_legacy(self):
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
        ttk.Combobox(drive, textvariable=self.chopper_mode_var,
                     values=list(CHOPPER_MODE_LABELS.keys()), width=18,
                     state="readonly").pack(side=tk.LEFT, padx=(16, 6))
        ttk.Button(drive, text="设置斩波", command=self.send_chopper_mode).pack(side=tk.LEFT)

        mode_status = ttk.Frame(left)
        mode_status.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(mode_status, textvariable=self.mode_status_var).pack(side=tk.LEFT, padx=(0, 12))
        ttk.Label(mode_status, textvariable=self.microstep_status_var).pack(side=tk.LEFT, padx=(0, 12))
        ttk.Label(mode_status, textvariable=self.enable_status_var).pack(side=tk.LEFT, padx=(0, 12))
        ttk.Label(mode_status, textvariable=self.stream_status_var).pack(side=tk.LEFT)
        ttk.Label(left, textvariable=self.driver_status_var).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(left, textvariable=self.chopper_status_var).pack(anchor=tk.W, pady=(2, 0))

        speed = ttk.LabelFrame(left, text="速度模式")
        speed.pack(fill=tk.X, pady=(12, 0))
        ttk.Label(speed, text="目标速度").pack(side=tk.LEFT, padx=(8, 6), pady=8)
        ttk.Entry(speed, textvariable=self.speed_var, width=10).pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Combobox(speed, textvariable=self.speed_unit_var,
                     values=["RPM", "脉冲/s"], width=8,
                     state="readonly").pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Label(speed, text="加速度 脉冲/s²").pack(side=tk.LEFT, padx=(0, 6), pady=8)
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

    def _build_ui(self):
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)
        root.columnconfigure(0, weight=0, minsize=190)
        root.columnconfigure(1, weight=1)
        root.columnconfigure(2, weight=0)
        root.columnconfigure(3, weight=0)
        # Keep the control area at its required height; the waveform receives
        # only the remaining vertical space instead of being squeezed under it.
        root.rowconfigure(1, weight=0)
        root.rowconfigure(2, weight=1)

        top = ttk.Frame(root)
        top.grid(row=0, column=0, columnspan=4, sticky="ew")
        top.columnconfigure(1, weight=1)
        connection = ttk.LabelFrame(top, text="连接与通信", padding=8)
        connection.grid(row=0, column=0, sticky="w", padx=(0, 10))
        ttk.Label(connection, text="串口").grid(row=0, column=0, sticky=tk.W)
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var,
                                       width=12, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=(4, 8))
        ttk.Label(connection, text="波特率").grid(row=0, column=2, sticky=tk.W)
        ttk.Combobox(connection, textvariable=self.baud_var, values=("115200",),
                     state="readonly", width=9).grid(row=0, column=3, padx=(4, 8))
        ttk.Label(connection, text="接口").grid(row=0, column=4, sticky=tk.W)
        ttk.Label(connection, textvariable=self.interface_role_var,
                  width=7).grid(row=0, column=5, padx=(4, 6), sticky=tk.W)
        ttk.Checkbutton(connection, text="自动连接", variable=self.auto_var).grid(row=0, column=6, sticky=tk.W)
        ttk.Button(connection, text="刷新", command=self.refresh_ports).grid(row=1, column=0, pady=(7, 0), sticky="ew")
        ttk.Button(connection, text="连接", command=self.connect_selected).grid(row=1, column=1, pady=(7, 0), sticky="ew")
        ttk.Button(connection, text="断开", command=self.disconnect).grid(row=1, column=2, pady=(7, 0), sticky="ew")
        ttk.Button(connection, text="查询状态", command=self.send_query).grid(row=1, column=3, columnspan=2, padx=(4, 0), pady=(7, 0), sticky="ew")

        telemetry = ttk.Frame(top)
        telemetry.grid(row=0, column=1, sticky="ew")
        metrics = (("绝对角度", self.abs_deg_var), ("零点角度", self.deg_var),
                   ("当前转速 RPM", self.rpm_var), ("温度", self.temp_var),
                   ("拨码通信模式", self.board_mode_var), ("连接接口", self.interface_role_var))
        for column, (label, variable) in enumerate(metrics):
            telemetry.columnconfigure(column, weight=1)
            ttk.Label(telemetry, text=label).grid(row=0, column=column, sticky=tk.W, padx=(0 if column == 0 else 16, 0))
            ttk.Label(telemetry, textvariable=variable, font=("Consolas", 21, "bold")).grid(
                row=1, column=column, sticky=tk.W, padx=(0 if column == 0 else 16, 0))

        left = ttk.Frame(root)
        left.grid(row=1, column=0, sticky="nsew", padx=(0, 10), pady=(12, 0))
        center = ttk.Frame(root)
        center.grid(row=1, column=1, sticky="nsew", padx=(0, 10), pady=(12, 0))
        tracking = ttk.LabelFrame(root, text="内置误差 PID", padding=8)
        tracking.grid(row=1, column=2, sticky="nsew", padx=(0, 10), pady=(12, 0))
        sequence = ttk.LabelFrame(root, text="指令添加", padding=8)
        # Keep the sequence editor in its own vertical lane.  It spans the
        # waveform row so a long command form cannot push the graph downward.
        sequence.grid(row=1, column=3, rowspan=2, sticky="ns", pady=(12, 0))

        mode = ttk.LabelFrame(left, text="模式状态", padding=8)
        mode.pack(fill=tk.X)
        mode.columnconfigure(1, weight=1)
        rows = (("模式", self.mode_var, list(MODE_LABELS.keys()), self.send_mode),
                ("细分", self.microstep_var, ["1", "2", "4", "8", "16", "32", "64", "128", "256"], self.send_microstep),
                ("斩波", self.chopper_mode_var, list(CHOPPER_MODE_LABELS.keys()), self.send_chopper_mode),
                ("曲线", self.motion_profile_var, list(MOTION_PROFILE_LABELS.keys()), self.send_motion_profile))
        for row, (label, variable, values, command) in enumerate(rows):
            ttk.Label(mode, text=label).grid(row=row, column=0, sticky=tk.W, pady=3)
            combo = ttk.Combobox(mode, textvariable=variable, values=values, state="readonly", width=6)
            combo.grid(row=row, column=1, sticky="ew", padx=3, pady=3)
            if variable is self.microstep_var:
                self.microstep_combo = combo
                combo.bind("<<ComboboxSelected>>", self._on_microstep_selected)
            ttk.Button(mode, text="设置", command=command, width=5).grid(row=row, column=2, pady=3)
        action_row = ttk.Frame(mode)
        action_row.grid(row=4, column=0, columnspan=3, sticky="ew", pady=(7, 4))
        ttk.Button(action_row, text="使能", command=lambda: self.send_enable(1)).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(action_row, text="失能", command=lambda: self.send_enable(0)).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        ttk.Button(action_row, text="停止", command=self.send_stop).pack(side=tk.LEFT, fill=tk.X, expand=True)
        stream_row = ttk.Frame(mode)
        stream_row.grid(row=5, column=0, columnspan=3, sticky="ew", pady=(0, 5))
        ttk.Button(stream_row, text="数据流开启", command=lambda: self.send_stream(1)).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(stream_row, text="数据流关闭", command=lambda: self.send_stream(0)).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(3, 0))
        for row, variable in enumerate((self.mode_status_var, self.microstep_status_var,
                                        self.chopper_status_var, self.motion_profile_status_var,
                                        self.enable_status_var, self.stream_status_var), start=6):
            ttk.Label(mode, textvariable=variable).grid(row=row, column=0, columnspan=3, sticky=tk.W, pady=1)

        position_pid = ttk.LabelFrame(left, text="闭环位置参数", padding=7)
        position_pid.pack(fill=tk.X, pady=(8, 0))
        position_fields = (("Kp", self.kp_var), ("Ki", self.ki_var),
                           ("Kd", self.kd_var), ("限幅", self.limit_var),
                           ("加速", self.closed_accel_var), ("死区", self.deadband_var))
        for index, (label, variable) in enumerate(position_fields):
            row = index // 2
            column = (index % 2) * 2
            ttk.Label(position_pid, text=label).grid(
                row=row, column=column, sticky=tk.W, padx=(0 if column == 0 else 7, 2), pady=2)
            ttk.Entry(position_pid, textvariable=variable, width=5).grid(
                row=row, column=column + 1, sticky=tk.W, pady=2)
        pid_actions = ttk.Frame(position_pid)
        pid_actions.grid(row=3, column=0, columnspan=4, sticky="ew", pady=(5, 0))
        ttk.Button(pid_actions, text="应用运行参数", command=self.apply_position_params).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(pid_actions, text="填入默认参数", command=self.fill_default_motion_params).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(3, 0))
        ttk.Label(position_pid, textvariable=self.pid_status_var, wraplength=185).grid(
            row=4, column=0, columnspan=4, sticky=tk.W, pady=(5, 0))

        zero = ttk.LabelFrame(center, text="零点设置", padding=8)
        zero.pack(fill=tk.X)
        ttk.Combobox(zero, textvariable=self.home_mode_var, values=list(HOME_MODE_LABELS.keys()),
                     width=10, state="readonly").grid(row=0, column=0, sticky=tk.W)
        ttk.Button(zero, text="设置回零模式", command=self.send_home_mode).grid(row=0, column=1, padx=5)
        ttk.Button(zero, text="当前位置设为零点", command=self.send_set_zero).grid(row=0, column=2, padx=5)
        ttk.Button(zero, text="执行回零", command=self.send_home).grid(row=0, column=3, padx=(5, 0))
        ttk.Label(zero, textvariable=self.home_status_var).grid(row=1, column=0, columnspan=4, sticky=tk.W, pady=(5, 0))

        test_motor = ttk.LabelFrame(center, text="测试电机", padding=8)
        test_motor.pack(fill=tk.X, pady=(10, 0))

        speed = ttk.LabelFrame(test_motor, text="开环速度", padding=7)
        speed.pack(fill=tk.X)
        ttk.Label(speed, text="目标速度").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(speed, textvariable=self.speed_var, width=8).grid(row=0, column=1, padx=4)
        ttk.Combobox(speed, textvariable=self.speed_unit_var, values=["RPM", "脉冲/s"],
                     state="readonly", width=7).grid(row=0, column=2, padx=(0, 8))
        ttk.Label(speed, text="加速度").grid(row=0, column=3, sticky=tk.W)
        ttk.Entry(speed, textvariable=self.accel_var, width=8).grid(row=0, column=4, padx=4)
        ttk.Button(speed, text="运行速度", command=self.send_speed_value).grid(row=1, column=0, columnspan=3, sticky="ew", pady=(5, 0))
        ttk.Button(speed, text="停止电机", command=self.send_stop).grid(row=1, column=3, columnspan=2, sticky="ew", padx=(5, 0), pady=(5, 0))
        ttk.Label(speed, textvariable=self.speed_status_var).grid(row=2, column=0, columnspan=5, sticky=tk.W, pady=(5, 0))

        pulse = ttk.LabelFrame(test_motor, text="脉冲位置移动", padding=7)
        pulse.pack(fill=tk.X, pady=(8, 0))
        for index, (label, variable, width) in enumerate(
                (("脉冲数", self.move_steps_var, 9), ("速度", self.move_speed_var, 7),
                 ("加速度", self.move_accel_var, 7))):
            column = index * 2
            ttk.Label(pulse, text=label).grid(row=0, column=column, sticky=tk.W, padx=(0 if index == 0 else 8, 3))
            ttk.Entry(pulse, textvariable=variable, width=width).grid(row=0, column=column + 1, padx=(0, 2))
        ttk.Button(pulse, text="发送脉冲位置指令", command=self.send_move_steps).grid(
            row=1, column=0, columnspan=6, sticky="ew", pady=(5, 0))
        ttk.Label(pulse, textvariable=self.move_status_var).grid(row=2, column=0, columnspan=6, sticky=tk.W, pady=(5, 0))

        absolute = ttk.LabelFrame(test_motor, text="闭环多圈位置", padding=7)
        absolute.pack(fill=tk.X, pady=(8, 0))
        ttk.Combobox(absolute, textvariable=self.position_target_mode_var,
                     values=("绝对目标", "相对移动"), state="readonly", width=9).grid(row=0, column=0, sticky=tk.W)
        ttk.Label(absolute, text="角度 -7200° ~ 7200°").grid(row=0, column=1, sticky=tk.W, padx=(7, 0))
        ttk.Entry(absolute, textvariable=self.target_angle_var, width=11).grid(row=0, column=2, padx=5)
        ttk.Button(absolute, text="发送闭环角度指令", command=self.send_angle_value).grid(
            row=1, column=0, columnspan=3, sticky="ew", pady=(5, 0))
        ttk.Label(absolute, textvariable=self.angle_status_var).grid(row=2, column=0, columnspan=3, sticky=tk.W, pady=(5, 0))

        operation = ttk.LabelFrame(test_motor, text="操作结果", padding=6)
        operation.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(operation, textvariable=self.operation_var, wraplength=360,
                  justify=tk.LEFT).pack(anchor=tk.W, fill=tk.X)

        tracking_base = ttk.LabelFrame(tracking, text="基础配置", padding=6)
        tracking_base.pack(fill=tk.X)
        ttk.Label(tracking_base, text="控制结构").grid(row=0, column=0, sticky=tk.W)
        ttk.Combobox(tracking_base, textvariable=self.track_mode_var,
                     values=list(TRACK_MODE_LABELS.keys()), state="readonly", width=11).grid(
                         row=0, column=1, sticky="ew", padx=(4, 6))
        ttk.Label(tracking_base, text="频率 kHz").grid(row=0, column=2, sticky=tk.W)
        ttk.Entry(tracking_base, textvariable=self.track_freq_var, width=5).grid(
            row=0, column=3, sticky=tk.W, padx=(4, 0))
        ttk.Label(tracking_base, text="速度 %").grid(row=1, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(tracking_base, textvariable=self.track_speed_pct_var, width=7).grid(
            row=1, column=1, sticky=tk.W, padx=(4, 6), pady=(5, 0))
        ttk.Label(tracking_base, text="100%=5000 脉冲/s").grid(row=1, column=2, columnspan=2, sticky=tk.W, pady=(5, 0))
        ttk.Label(tracking_base, text="加速度 %").grid(row=2, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Entry(tracking_base, textvariable=self.track_accel_pct_var, width=5).grid(
            row=2, column=1, sticky=tk.W, padx=(4, 6), pady=(5, 0))
        ttk.Label(tracking_base, text="100%=10000 脉冲/s²").grid(row=2, column=2, columnspan=2, sticky=tk.W, pady=(5, 0))
        ttk.Button(tracking_base, text="写入", command=self.send_track_basic).grid(
            row=3, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        ttk.Button(tracking_base, text="读取", command=self.send_track_query).grid(
            row=3, column=2, sticky="ew", padx=(5, 2), pady=(6, 0))
        ttk.Button(tracking_base, text="PID存Flash", command=self.send_save_runtime_config).grid(
            row=3, column=3, sticky="ew", padx=(2, 0), pady=(6, 0))

        tracking_loops = ttk.LabelFrame(tracking, text="PID 环参数", padding=6)
        tracking_loops.pack(fill=tk.X, pady=(8, 0))
        for column, text in enumerate(("环", "Kp", "Ki", "Kd", "")):
            ttk.Label(tracking_loops, text=text).grid(row=0, column=column, sticky=tk.W, pady=(0, 3))
        loop_rows = (("单环", self.track_single_kp_var, self.track_single_ki_var, self.track_single_kd_var, 0),
                     ("外环", self.track_outer_kp_var, self.track_outer_ki_var, self.track_outer_kd_var, 1),
                     ("内环", self.track_inner_kp_var, self.track_inner_ki_var, self.track_inner_kd_var, 2))
        for row, (name, kp, ki, kd, loop) in enumerate(loop_rows, start=1):
            ttk.Label(tracking_loops, text=name).grid(row=row, column=0, sticky=tk.W, pady=2)
            for column, variable in enumerate((kp, ki, kd), start=1):
                ttk.Entry(tracking_loops, textvariable=variable, width=7).grid(
                    row=row, column=column, padx=(0, 3), pady=2)
            ttk.Button(tracking_loops, text="写入", command=lambda value=loop: self.send_track_pid(value)).grid(
                row=row, column=4, padx=(3, 0), pady=2)

        tracking_input = ttk.LabelFrame(tracking, text="视觉误差测试", padding=6)
        tracking_input.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(tracking_input, text="主误差").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(tracking_input, textvariable=self.track_primary_var, width=8).grid(row=0, column=1, padx=4)
        ttk.Label(tracking_input, text="内环反馈").grid(row=0, column=2, sticky=tk.W)
        ttk.Entry(tracking_input, textvariable=self.track_secondary_var, width=8).grid(row=0, column=3, padx=4)
        ttk.Button(tracking_input, text="发送", command=self.send_track_input).grid(row=0, column=4)
        ttk.Checkbutton(tracking_input, text="误差反向", variable=self.track_reverse_var,
                        command=self.send_track_direction).grid(row=0, column=5, padx=(8, 0))

        self.tracking_advanced = ttk.LabelFrame(tracking, text="高级参数", padding=6)
        advanced = self.tracking_advanced
        ttk.Label(advanced, text="滤波 α (0.01~1)").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(advanced, textvariable=self.track_filter_var, width=7).grid(row=0, column=1, padx=(3, 8))
        ttk.Label(advanced, text="超时 ms (20~5000)").grid(row=0, column=2, sticky=tk.W)
        ttk.Entry(advanced, textvariable=self.track_timeout_var, width=7).grid(row=0, column=3, padx=(3, 0))

        ttk.Checkbutton(advanced, text="启用目标预测", variable=self.prediction_enabled_var,
                        command=self._update_advanced_control_states).grid(row=1, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Label(advanced, text="预测 ms (0~200)").grid(row=1, column=1, sticky=tk.W, pady=(5, 0))
        self.prediction_entry = ttk.Entry(advanced, textvariable=self.track_prediction_var, width=7)
        self.prediction_entry.grid(row=1, column=2, sticky=tk.W, padx=(3, 8), pady=(5, 0))

        ttk.Checkbutton(advanced, text="启用动态增益", variable=self.dynamic_gain_enabled_var,
                        command=self._update_advanced_control_states).grid(row=2, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Label(advanced, text="最小/最大 (0.1~10)").grid(row=2, column=1, sticky=tk.W, pady=(5, 0))
        self.dynamic_gain_min_entry = ttk.Entry(advanced, textvariable=self.track_gain_min_var, width=6)
        self.dynamic_gain_min_entry.grid(row=2, column=2, sticky=tk.W, padx=(3, 2), pady=(5, 0))
        self.dynamic_gain_max_entry = ttk.Entry(advanced, textvariable=self.track_gain_max_var, width=6)
        self.dynamic_gain_max_entry.grid(row=2, column=3, sticky=tk.W, padx=(0, 8), pady=(5, 0))
        ttk.Label(advanced, text="满增益误差 (0.01~655.35)").grid(row=3, column=0, columnspan=2, sticky=tk.W, pady=(5, 0))
        self.dynamic_full_error_entry = ttk.Entry(advanced, textvariable=self.track_full_error_var, width=8)
        self.dynamic_full_error_entry.grid(row=3, column=2, sticky=tk.W, padx=(3, 8), pady=(5, 0))

        for column, text in enumerate(("单环死区", "外环死区", "内环死区")):
            ttk.Label(advanced, text=f"{text} (0~65.535)").grid(row=4, column=column, sticky=tk.W, pady=(5, 0))
        ttk.Entry(advanced, textvariable=self.track_single_dead_var, width=7).grid(row=5, column=0, sticky=tk.W, pady=(2, 0))
        ttk.Entry(advanced, textvariable=self.track_outer_dead_var, width=7).grid(row=5, column=1, sticky=tk.W, pady=(2, 0))
        ttk.Entry(advanced, textvariable=self.track_inner_dead_var, width=7).grid(row=5, column=2, sticky=tk.W, pady=(2, 0))
        ttk.Button(advanced, text="写入高级参数", command=self.send_track_advanced).grid(
            row=5, column=3, sticky="ew", pady=(2, 0))
        self.tracking_advanced.pack(fill=tk.X, pady=(8, 0))
        self.track_status_label = ttk.Label(tracking, textvariable=self.track_status_var, wraplength=300)
        self.track_status_label.pack(anchor=tk.W, pady=(7, 0))

        sequence.columnconfigure(0, weight=1)
        command_values = ("位置移动", "速度运行", "延时", "回零", "设置细分", "设置斩波", "运行内置PID", "停止电机")
        command_box = ttk.Combobox(sequence, textvariable=self.sequence_type_var,
                                   values=command_values, state="readonly", width=20)
        command_box.grid(row=0, column=0, sticky="ew")
        command_box.bind("<<ComboboxSelected>>", self._sequence_command_changed)
        self.sequence_param_frame = ttk.Frame(sequence)
        self.sequence_param_frame.grid(row=1, column=0, sticky="ew", pady=(7, 0))
        ttk.Button(sequence, text="添加到流程", command=self.add_sequence_step).grid(
            row=2, column=0, sticky="ew", pady=(7, 5))
        self.sequence_tree = ttk.Treeview(sequence, columns=("index", "description"),
                                          show="headings", height=6, selectmode="browse")
        self.sequence_tree.heading("index", text="#")
        self.sequence_tree.heading("description", text="动作")
        self.sequence_tree.column("index", width=28, anchor=tk.CENTER, stretch=False)
        self.sequence_tree.column("description", width=255, anchor=tk.W)
        self.sequence_tree.grid(row=3, column=0, sticky="nsew")
        sequence.rowconfigure(3, weight=1)
        sequence_tools = ttk.Frame(sequence)
        sequence_tools.grid(row=4, column=0, sticky="ew", pady=(6, 0))
        for text, command in (("上移", lambda: self.move_sequence_step(-1)),
                              ("下移", lambda: self.move_sequence_step(1)),
                              ("删除", self.remove_sequence_step),
                              ("清空", self.clear_sequence_steps)):
            ttk.Button(sequence_tools, text=text, command=command).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
        program_tools = ttk.Frame(sequence)
        program_tools.grid(row=5, column=0, sticky="ew", pady=(6, 0))
        ttk.Button(program_tools, text="写入 RAM", command=self.write_program_to_ram).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2))
        ttk.Button(program_tools, text="烧写 Flash", command=self.write_program_to_flash).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(2, 0))
        ttk.Button(sequence, text="运行指令", command=self.run_program).grid(
            row=6, column=0, sticky="ew", pady=(4, 0))
        program_mode_tools = ttk.Frame(sequence)
        program_mode_tools.grid(row=7, column=0, sticky="ew", pady=(3, 0))
        ttk.Button(program_mode_tools, text="进入指令模式", command=lambda: self.set_program_mode(True)).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2))
        ttk.Button(program_mode_tools, text="退出指令模式", command=lambda: self.set_program_mode(False)).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(2, 0))
        ttk.Button(sequence, text="保存 JSON", command=self.save_sequence).grid(row=8, column=0, sticky="ew", pady=(6, 0))
        ttk.Button(sequence, text="载入 JSON", command=self.load_sequence).grid(row=9, column=0, sticky="ew", pady=(3, 0))
        ttk.Label(sequence, textvariable=self.sequence_status_var, wraplength=280).grid(
            row=10, column=0, sticky=tk.W, pady=(7, 0))

        self.wave_frame = ttk.LabelFrame(root, text="多圈位置波形：目标 / 当前位置", padding=8)
        wave = self.wave_frame
        wave.grid(row=2, column=0, columnspan=3, sticky="nsew", padx=(0, 10), pady=(12, 0))
        wave_toolbar = ttk.Frame(wave)
        wave_toolbar.pack(fill=tk.X, pady=(0, 5))
        ttk.Checkbutton(wave_toolbar, text="暂停波形", variable=self.wave_paused_var).pack(side=tk.LEFT)
        ttk.Checkbutton(wave_toolbar, text="跟随最新", variable=self.wave_follow_var).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Button(wave_toolbar, text="横向 +", command=lambda: self.zoom_waveform(1.35)).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Button(wave_toolbar, text="横向 -", command=lambda: self.zoom_waveform(1.0 / 1.35)).pack(side=tk.LEFT, padx=5)
        ttk.Button(wave_toolbar, text="纵向 +", command=lambda: self.zoom_waveform_vertical(1.0 / 1.5)).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(wave_toolbar, text="纵向 -", command=lambda: self.zoom_waveform_vertical(1.5)).pack(side=tk.LEFT, padx=5)
        ttk.Button(wave_toolbar, text="清空波形", command=self.clear_waveform).pack(side=tk.LEFT, padx=(7, 0))
        ttk.Label(wave_toolbar, textvariable=self.status_var).pack(side=tk.RIGHT)
        canvas_wrap = ttk.Frame(wave)
        canvas_wrap.pack(fill=tk.BOTH, expand=True)
        self.wave_axis_canvas = tk.Canvas(canvas_wrap, width=66, height=240, bg="#f8fafc", highlightthickness=1,
                                          highlightbackground="#cbd5e1")
        self.wave_axis_canvas.grid(row=0, column=0, sticky="ns")
        graph_wrap = ttk.Frame(canvas_wrap)
        graph_wrap.grid(row=0, column=1, sticky="nsew")
        canvas_wrap.columnconfigure(1, weight=1)
        canvas_wrap.rowconfigure(0, weight=1)
        self.wave_canvas = tk.Canvas(graph_wrap, height=240, bg="#f8fafc", highlightthickness=1, highlightbackground="#cbd5e1")
        self.wave_canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.wave_scroll = ttk.Scrollbar(graph_wrap, orient=tk.HORIZONTAL, command=self._wave_xview)
        self.wave_scroll.pack(side=tk.BOTTOM, fill=tk.X)
        self.wave_canvas.configure(xscrollcommand=self.wave_scroll.set)
        self.wave_canvas.bind("<Configure>", lambda _event: self._redraw_waveform())

    def _sequence_command_changed(self, _event=None):
        if self.sequence_param_frame is None:
            return
        for child in self.sequence_param_frame.winfo_children():
            child.destroy()

        specs = self._sequence_parameter_specs(self.sequence_type_var.get())
        self.sequence_param_vars = {}
        for row, (key, label, default, values) in enumerate(specs):
            variable = tk.StringVar(value=default)
            self.sequence_param_vars[key] = variable
            ttk.Label(self.sequence_param_frame, text=label).grid(row=row, column=0, sticky=tk.W, pady=2)
            if values:
                ttk.Combobox(self.sequence_param_frame, textvariable=variable, values=values,
                             state="readonly", width=17).grid(row=row, column=1, sticky="ew", padx=(5, 0), pady=2)
            else:
                ttk.Entry(self.sequence_param_frame, textvariable=variable, width=20).grid(
                    row=row, column=1, sticky="ew", padx=(5, 0), pady=2)
        self.sequence_param_frame.columnconfigure(1, weight=1)

    @staticmethod
    def _sequence_parameter_specs(command_type):
        specs = {
            "位置移动": (
                ("reference", "基准", "当前位置", ("当前位置", "绝对零点")),
                ("angle", "移动角度 °", "300", None),
                ("speed", "速度 脉冲/s", "2000", None),
                ("accel", "加速度 脉冲/s²", "100", None),
                ("control", "控制方式", "闭环位置", ("闭环位置", "开环脉冲")),
            ),
            "速度运行": (
                ("speed", "目标速度", "4", None),
                ("unit", "速度单位", "RPM", ("RPM", "脉冲/s")),
                ("accel", "加速度 脉冲/s²", "3000", None),
                ("duration", "持续时间 ms", "无限", None),
            ),
            "延时": (("duration", "延时 ms", "200", None),),
            "回零": (("mode", "回零方式", "就近回零", tuple(HOME_MODE_LABELS.keys())),),
            "设置细分": (("microsteps", "细分", "32", ("1", "2", "4", "8", "16", "32", "64", "128", "256")),),
            "设置斩波": (("chopper", "斩波模式", "静音 StealthChop", ("静音 StealthChop", "力矩 SpreadCycle")),),
            "运行内置PID": (
                ("pid_mode", "PID结构", "双环PID", ("单环PID", "双环PID")),
                ("error", "启动测试误差(0=等视觉)", "0", None),
                ("duration", "持续时间 ms", "无限", None),
            ),
            "停止电机": (),
        }
        return specs.get(command_type, ())

    @staticmethod
    def _describe_sequence_step(step):
        params = step.get("params", {})
        command_type = step.get("type", "未知指令")
        if command_type == "位置移动":
            return (f"{params.get('reference', '当前位置')}移动 {params.get('angle', '?')}°，"
                    f"速度 {params.get('speed', '?')}，加速度 {params.get('accel', '?')}，"
                    f"{params.get('control', '闭环位置')}")
        if command_type == "速度运行":
            return (f"速度 {params.get('speed', '?')} {params.get('unit', '脉冲/s')}，加速度 {params.get('accel', '?')}，"
                    f"持续 {params.get('duration', '?')} ms")
        if command_type == "延时":
            return f"延时 {params.get('duration', '?')} ms"
        if command_type == "回零":
            return params.get("mode", "就近回零")
        if command_type == "设置细分":
            return f"设置细分 {params.get('microsteps', '?')}"
        if command_type == "设置斩波":
            return f"设置斩波 {params.get('chopper', '?')}"
        if command_type == "运行内置PID":
            return (f"运行 {params.get('pid_mode', '?')}，启动误差 {params.get('error', '0')}，"
                    f"持续 {params.get('duration', '无限')} ms")
        return "停止电机"

    def _refresh_sequence_tree(self, select_index=None):
        for node in self.sequence_tree.get_children():
            self.sequence_tree.delete(node)
        for index, step in enumerate(self.sequence_steps):
            node = self.sequence_tree.insert("", tk.END, iid=str(index), values=(index + 1, self._describe_sequence_step(step)))
            if select_index == index:
                self.sequence_tree.selection_set(node)

    def add_sequence_step(self):
        command_type = self.sequence_type_var.get()
        params = {key: variable.get().strip() for key, variable in self.sequence_param_vars.items()}
        required = {
            "位置移动": ("angle", "speed", "accel"),
            "速度运行": ("speed", "accel"),
            "延时": ("duration",),
        }
        if any(not params.get(key) for key in required.get(command_type, ())):
            self.sequence_status_var.set("请先填写该指令的必填参数。")
            return
        step = {"type": command_type, "params": params}
        self.sequence_steps.append(step)
        self.sequence_ram_count = None
        self._refresh_sequence_tree(len(self.sequence_steps) - 1)
        self.sequence_status_var.set(f"已添加第 {len(self.sequence_steps)} 条，仅保存在上位机。")

    def _selected_sequence_index(self):
        selected = self.sequence_tree.selection() if self.sequence_tree else ()
        return int(selected[0]) if selected else None

    def remove_sequence_step(self):
        index = self._selected_sequence_index()
        if index is None:
            return
        self.sequence_steps.pop(index)
        self.sequence_ram_count = None
        self._refresh_sequence_tree(min(index, len(self.sequence_steps) - 1))

    def move_sequence_step(self, delta):
        index = self._selected_sequence_index()
        if index is None:
            return
        destination = index + delta
        if destination < 0 or destination >= len(self.sequence_steps):
            return
        self.sequence_steps[index], self.sequence_steps[destination] = self.sequence_steps[destination], self.sequence_steps[index]
        self.sequence_ram_count = None
        self._refresh_sequence_tree(destination)

    def clear_sequence_steps(self):
        self.sequence_steps.clear()
        self.sequence_ram_count = None
        self._refresh_sequence_tree()
        self.sequence_status_var.set("已清空本机指令列表。")

    @staticmethod
    def _program_duration(value):
        if str(value).strip() == "无限":
            return 0
        duration = int(float(value))
        if not 0 <= duration <= 65535:
            raise ValueError("持续时间必须为 0~65535 ms 或“无限”")
        return duration

    def _encode_program_step(self, index, step, active_microsteps=32):
        params = step.get("params", {})
        command_type = step.get("type", "")
        op = flags = speed = accel = duration = value = 0

        if command_type == "位置移动":
            angle = float(params["angle"])
            speed = int(float(params["speed"]))
            accel = int(float(params["accel"]))
            if not (1 <= speed <= 65535 and 1 <= accel <= 65535):
                raise ValueError("位置速度和加速度必须为 1~65535")
            if params.get("control") == "开环脉冲":
                microsteps = int(self.microstep_var.get())
                value = int(round(angle * 200.0 * microsteps / 360.0))
                if value == 0:
                    raise ValueError("开环脉冲移动不能为 0")
                op = 3
            else:
                value = int(round(angle * 100.0))
                if not -720000 <= value <= 720000:
                    raise ValueError("闭环位置范围为 -7200~7200 度")
                op = 2 if params.get("reference") == "当前位置" else 1
        elif command_type == "速度运行":
            requested_speed = float(params["speed"])
            value = int(round(requested_speed))
            accel = int(float(params["accel"]))
            duration = self._program_duration(params.get("duration", "无限"))
            if not (-5000 <= value <= 5000 and 0 < accel <= 65535):
                raise ValueError("速度范围为 -5000~5000，加速度为 1~65535")
            if params.get("unit") == "RPM":
                converted_speed = value * 200.0 * active_microsteps / 60.0
                if abs(converted_speed) > 5000.0:
                    max_rpm = 5000.0 * 60.0 / (200.0 * active_microsteps)
                    raise ValueError(
                        f"第 {index + 1} 条在 {active_microsteps} 细分下最大约 {max_rpm:.1f} RPM"
                    )
                flags = 0x01
            op = 4
        elif command_type == "延时":
            duration = self._program_duration(params["duration"])
            op = 5
        elif command_type == "回零":
            flags = home_mode_value(params.get("mode"))
            op = 6
        elif command_type == "设置细分":
            value = int(params["microsteps"])
            if value not in (1, 2, 4, 8, 16, 32, 64, 128, 256):
                raise ValueError("细分仅支持 1~256 的 2 次幂")
            op = 9
        elif command_type == "设置斩波":
            flags = 1 if params.get("chopper") == "力矩 SpreadCycle" else 0
            op = 10
        elif command_type == "运行内置PID":
            flags = 1 if params.get("pid_mode") == "双环PID" else 0
            value = int(round(float(params.get("error", "0")) * 100.0))
            duration = self._program_duration(params.get("duration", "无限"))
            if not -100000 <= value <= 100000:
                raise ValueError("内部测试误差范围为 -1000~1000")
            op = 7
        elif command_type == "停止电机":
            op = 8
        else:
            raise ValueError("未知指令类型")

        return struct.pack("<BBBHHHi", index, op, flags, speed, accel, duration, value)

    def _encode_sequence_payloads(self):
        if not self.sequence_steps:
            self.sequence_status_var.set("请先添加至少一条指令。")
            return None
        if len(self.sequence_steps) > 8:
            self.sequence_status_var.set("驱动 Flash 最多保存 8 条指令。")
            return None
        try:
            payloads = []
            active_microsteps = self.current_microsteps if self.current_microsteps else 32
            for index, step in enumerate(self.sequence_steps):
                payloads.append(self._encode_program_step(index, step, active_microsteps))
                if step.get("type") == "设置细分":
                    active_microsteps = int(step.get("params", {}).get("microsteps", active_microsteps))
            return payloads
        except (KeyError, ValueError) as error:
            self.sequence_status_var.set(f"指令格式错误：{error}")
            return None

    def write_program_to_ram(self):
        payloads = self._encode_sequence_payloads()
        if payloads is None:
            return

        for payload in payloads:
            self._send(CMD_PROGRAM_STEP, payload)
        if self._send(CMD_PROGRAM_STAGE, struct.pack("<B", len(payloads))):
            self.sequence_ram_count = len(payloads)
            self.sequence_status_var.set(f"已写入 {len(payloads)} 条指令到 RAM，可反复试跑；确认后再烧写 Flash。")

    def write_program_to_flash(self):
        if self.sequence_ram_count != len(self.sequence_steps):
            self.sequence_status_var.set("请先点击“写入 RAM”，试跑确认后才能烧写 Flash。")
            return
        if self._send(CMD_PROGRAM_SAVE, struct.pack("<B", self.sequence_ram_count)):
            self.sequence_status_var.set("正在烧写指令到 Flash；不要重复点击。")

    def set_program_mode(self, enabled):
        if self._send(CMD_PROGRAM_MODE, struct.pack("<B", 1 if enabled else 0)):
            self.sequence_status_var.set("正在进入指令模式。" if enabled else "正在退出指令模式。")

    def run_program(self):
        if self._send(CMD_PROGRAM_RUN):
            self.sequence_status_var.set("已发送运行内部指令请求。")

    def send_program_query(self):
        self._send(CMD_PROGRAM_QUERY)

    def save_sequence(self):
        path = filedialog.asksaveasfilename(title="保存指令列表", defaultextension=".json",
                                            initialfile="step42_command_sequence.json",
                                            filetypes=(("JSON 指令", "*.json"), ("所有文件", "*.*")))
        if not path:
            return
        with open(path, "w", encoding="utf-8") as file:
            json.dump({"format": "step42-command-sequence-v2", "steps": self.sequence_steps}, file,
                      ensure_ascii=False, indent=2)
        self.sequence_status_var.set("指令列表已保存到电脑，尚未写入驱动板。")

    def load_sequence(self):
        path = filedialog.askopenfilename(title="载入指令列表", filetypes=(("JSON 指令", "*.json"), ("所有文件", "*.*")))
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as file:
                data = json.load(file)
            if data.get("format") not in ("step42-command-sequence-v1", "step42-command-sequence-v2"):
                raise ValueError("不是 Step42 指令列表文件")
            self.sequence_steps = []
            for step in data.get("steps", []):
                if "params" in step:
                    self.sequence_steps.append(step)
                else:
                    # Preserve old two-parameter drafts as a readable position command.
                    self.sequence_steps.append({
                        "type": step.get("type", "位置移动"),
                        "params": {"angle": step.get("primary", ""), "speed": step.get("secondary", "")},
                    })
            self.sequence_ram_count = None
            self._refresh_sequence_tree()
            self.sequence_status_var.set(f"已载入 {len(self.sequence_steps)} 条本机指令。")
        except (OSError, ValueError, json.JSONDecodeError) as error:
            self.sequence_status_var.set(f"指令列表载入失败：{error}")

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
        self.tx_queue.clear()
        self.pending_tx = None
        self._reset_motion_state()
        self.status_var.set(f"正在连接 {port} ...")
        self.reader.start(port, baud)

    def disconnect(self):
        self.connected_port = None
        self.tx_queue.clear()
        self.pending_tx = None
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
                f"Kd={params['kd']:.2f}, Limit={params['limit']}, "
                f"Deadband={params['deadband']:.2f}, Accel={params['accel']}"
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
        mode = home_mode_value(self.home_mode_var.get())
        if self._send(CMD_HOME_MODE, struct.pack("<B", mode)):
            self.home_status_var.set(f"回零模式：已发送 {self.home_mode_var.get()}")

    def send_set_zero(self):
        if self._send(CMD_SET_ZERO):
            self.home_status_var.set("零点：已发送保存请求")

    def send_home(self):
        if self.current_mode != MODE_CLOSED_POSITION:
            self.home_status_var.set("回零：当前不是闭环位置模式")
            return
        mode = home_mode_value(self.home_mode_var.get())
        if self._send(CMD_HOME, struct.pack("<B", mode)):
            self.home_status_var.set(f"回零：已发送 {self.home_mode_var.get()}")

    def send_enable(self, enable):
        if self._send(CMD_ENABLE, struct.pack("<B", 1 if enable else 0)):
            self.enable_status_var.set("电机：已发送使能" if enable else "电机：已发送失能")

    def send_stream(self, enable):
        if self._send(CMD_STREAM, struct.pack("<B", 1 if enable else 0)):
            self.stream_status_var.set("数据流：已发送开启" if enable else "数据流：已发送关闭")

    def send_chopper_mode(self):
        mode = CHOPPER_MODE_LABELS.get(self.chopper_mode_var.get())
        if mode is None:
            self.chopper_status_var.set("斩波模式：选择无效")
            return
        if self._send(CMD_CHOPPER_MODE, struct.pack("<B", mode)):
            self.chopper_status_var.set(f"斩波模式：已发送 {self.chopper_mode_var.get()}")

    def send_motion_profile(self):
        profile = MOTION_PROFILE_LABELS.get(self.motion_profile_var.get())
        if profile is None:
            self.motion_profile_status_var.set("曲线：选择无效")
            return
        if self._send(CMD_MOTION_PROFILE, struct.pack("<B", profile)):
            self.motion_profile_status_var.set(f"曲线：已发送 {self.motion_profile_var.get()}")

    def send_query(self):
        self._send(CMD_QUERY)

    def send_stop(self):
        if self._send(CMD_STOP):
            self.speed_status_var.set("速度：已发送停止")
            self.angle_status_var.set("角度：已发送停止")

    def send_pid_values(self):
        payload = self._build_position_pid_payload()
        if payload is not None and self._send(CMD_PID, payload):
            self.pid_status_var.set(
                f"PID：已发送 Kp={self.kp_var.get()}, Ki={self.ki_var.get()}, "
                f"Kd={self.kd_var.get()}, Limit={self.limit_var.get()}, "
                f"Deadband={self.deadband_var.get()}, Accel={self.closed_accel_var.get()}"
            )

    def _build_position_pid_payload(self):
        try:
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            kd = float(self.kd_var.get())
            limit = int(float(self.limit_var.get()))
            deadband = float(self.deadband_var.get())
        except ValueError:
            self.pid_status_var.set("PID：参数不是有效数字")
            return None
        kp_i = int(round(kp * 100))
        ki_i = int(round(ki * 100))
        kd_i = int(round(kd * 100))
        dead_i = int(round(deadband * 1000))
        if not (-2147483648 <= kp_i <= 2147483647 and -2147483648 <= ki_i <= 2147483647 and -2147483648 <= kd_i <= 2147483647):
            self.pid_status_var.set("PID：Kp/Ki/Kd 超范围")
            return None
        if not (1 <= limit <= 65535 and 0 <= dead_i <= 10000):
            self.pid_status_var.set("PID：Limit 或 Deadband 超范围")
            return None
        return struct.pack(
            "<iiiHH",
            kp_i,
            ki_i,
            kd_i,
            limit,
            dead_i,
        )

    def apply_position_params(self):
        payload = self._build_position_pid_payload()
        try:
            accel = int(float(self.closed_accel_var.get()))
        except ValueError:
            self.pid_status_var.set("运行参数：闭环加速度不是有效整数")
            return
        if payload is None:
            return
        if not (1 <= accel <= 65535):
            self.pid_status_var.set("运行参数：闭环加速度范围为1~65535")
            return
        if self._send(CMD_PID, payload) and self._send(CMD_CLOSED_ACCEL, struct.pack("<H", accel)):
            self.pid_status_var.set(
                f"运行参数：Kp={self.kp_var.get()}, Ki={self.ki_var.get()}, Kd={self.kd_var.get()}, "
                f"Limit={self.limit_var.get()}, Deadband={self.deadband_var.get()}, Accel={accel} 已应用"
            )

    def send_save_runtime_config(self):
        if self._send(CMD_SAVE_RUNTIME_CONFIG):
            self.operation_var.set("正在保存当前闭环位置与内置误差 PID 参数到 Flash，请勿断电。")

    def send_speed_value(self):
        if self.current_mode != MODE_OPEN_SPEED:
            self.speed_status_var.set("速度：当前不是速度模式，先设置为速度模式")
            return
        try:
            requested_speed = float(self.speed_var.get())
            accel = int(float(self.accel_var.get()))
        except ValueError:
            self.speed_status_var.set("速度：请输入有效数字")
            return
        if self.speed_unit_var.get() == "RPM":
            speed = int(round(requested_speed * 200.0 * self.current_microsteps / 60.0))
        else:
            speed = int(round(requested_speed))
        if speed < -5000 or speed > 5000 or accel <= 0:
            max_rpm = 5000.0 * 60.0 / (200.0 * self.current_microsteps)
            self.speed_status_var.set(
                f"速度：换算后{speed}脉冲/s超范围；当前{self.current_microsteps}细分最大约{max_rpm:.1f}RPM"
            )
            return
        payload = struct.pack("<hH", speed, accel)
        if self._send(CMD_SPEED, payload):
            rpm = speed * 60.0 / (200.0 * self.current_microsteps)
            self.speed_status_var.set(
                f"速度：已发送{speed}脉冲/s，理论{rpm:.1f}RPM，accel={accel}"
            )

    def send_move_steps(self):
        if self.current_mode != MODE_OPEN_POSITION:
            self.move_status_var.set("脉冲位置：仅开环位置可用；闭环请使用相对/绝对角度")
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
        if angle < -7200.0 or angle > 7200.0:
            self.angle_status_var.set("角度：范围必须是 -7200~7200")
            return
        payload = struct.pack("<i", int(round(angle * 100)))
        if self.position_target_mode_var.get() == "相对移动":
            if self._send(CMD_RELATIVE_ANGLE, payload, {"relative_angle": angle}):
                self.angle_status_var.set(f"相对移动：等待设备确认 {angle:+.3f}°")
        elif self._send(CMD_ANGLE, payload, {"target_angle": angle}):
            self.angle_status_var.set(f"绝对目标：等待设备确认 {angle:.3f}°")

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
        if not (-1000.0 <= primary <= 1000.0 and -1000.0 <= secondary <= 1000.0):
            self.track_status_var.set("误差PID：输入范围为-1000~1000")
            return
        payload = struct.pack("<ii", int(round(primary * 100)), int(round(secondary * 100)))
        if self._send(CMD_TRACK_INPUT, payload):
            self.track_status_var.set(f"误差PID：已发送主误差{primary:.2f}，内环反馈{secondary:.2f}")

    def send_track_direction(self):
        reverse = 1 if self.track_reverse_var.get() else 0
        if self._send(CMD_TRACK_DIRECTION, struct.pack("<B", reverse)):
            self.track_status_var.set(f"误差PID：误差方向已设为{'反向' if reverse else '正向'}")

    def send_track_defaults(self):
        if self._send(CMD_TRACK_DEFAULT):
            self.track_status_var.set("误差PID：已发送恢复默认请求")
            self.after(120, self.send_track_query)

    def send_track_query(self):
        self._send(CMD_TRACK_QUERY)

    def send_track_advanced(self):
        try:
            filter_alpha = float(self.track_filter_var.get())
            gain_min = float(self.track_gain_min_var.get())
            gain_max = float(self.track_gain_max_var.get())
            full_error = float(self.track_full_error_var.get())
            prediction_ms = float(self.track_prediction_var.get())
            timeout_ms = int(round(float(self.track_timeout_var.get())))
            deadbands = [float(self.track_single_dead_var.get()),
                         float(self.track_outer_dead_var.get()),
                         float(self.track_inner_dead_var.get())]
        except ValueError:
            self.track_status_var.set("误差PID：高级参数格式错误")
            return
        if not (0.01 <= filter_alpha <= 1.0 and 20 <= timeout_ms <= 5000 and
                all(0.0 <= value <= 65.535 for value in deadbands)):
            self.track_status_var.set("误差PID：滤波0.01~1，超时20~5000，死区0~65.535")
            return
        if self.dynamic_gain_enabled_var.get():
            if not (0.1 <= gain_min <= gain_max <= 10.0 and 0.01 <= full_error <= 655.35):
                self.track_status_var.set("误差PID：动态增益最小/最大为0.1~10，满增益误差为0.01~655.35")
                return
        else:
            gain_min = gain_max = 1.0
        if self.prediction_enabled_var.get():
            if not (0.0 <= prediction_ms <= 200.0):
                self.track_status_var.set("误差PID：预测时间范围为0~200ms")
                return
        else:
            prediction_ms = 0.0
        values = [int(round(filter_alpha * 1000)),
                  int(round(gain_min * 1000)),
                  int(round(gain_max * 1000)),
                  int(round(full_error * 100)),
                  int(round(prediction_ms)), timeout_ms,
                  *(int(round(value * 1000)) for value in deadbands)]
        if self._send(CMD_TRACK_ADVANCED, struct.pack("<9H", *values)):
            self.track_status_var.set("误差PID：高级参数已写入运行时配置")

    def _update_advanced_control_states(self):
        gain_state = "normal" if self.dynamic_gain_enabled_var.get() else "disabled"
        prediction_state = "normal" if self.prediction_enabled_var.get() else "disabled"
        for entry in getattr(self, "dynamic_gain_min_entry", ()), getattr(self, "dynamic_gain_max_entry", ()), getattr(self, "dynamic_full_error_entry", ()):
            entry.configure(state=gain_state)
        if hasattr(self, "prediction_entry"):
            self.prediction_entry.configure(state=prediction_state)

    def _auto_refresh_ports(self):
        self.refresh_ports(auto_connect=True)
        self.after(1000, self._auto_refresh_ports)

    def _report_operation(self, message):
        """Keep explicit command results visible independently of telemetry."""
        self.operation_var.set(message)

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
                self.after(220, self.send_program_query)
            if kind in ("closed", "error"):
                self.connected_port = None
                self.tx_queue.clear()
                self.pending_tx = None

        if self.pending_tx is not None:
            sent_at = self.pending_tx.get("sent_at")
            if sent_at is not None and (time.monotonic() - sent_at) > 1.0:
                timed_out = self.pending_tx
                self.pending_tx = None
                self.status_var.set(
                    f"命令0x{timed_out['cmd']:02X}等待响应超时，后续命令已暂停"
                )
                self._report_operation(
                    f"命令 0x{timed_out['cmd']:02X} 未收到响应：请检查拨码模式、接口和接线。"
                )
                self.tx_queue.clear()

        self.after(30, self._poll_queues)

    def _process_frame(self, cmd, seq, payload):
        completed_pending = self._complete_pending(cmd, seq, payload)
        if cmd == CMD_STREAM and seq == 0 and len(payload) >= 16:
            status = payload[0]
            current_cdeg = struct.unpack("<H", payload[1:3])[0]
            target_cdeg, error_cdeg = struct.unpack("<ii", payload[3:11])
            pid_active = payload[11]
            rpm_x10 = struct.unpack("<h", payload[12:14])[0]
            now = time.monotonic()
            self.current_angle_deg = current_cdeg / 100.0
            self.target_angle_deg = target_cdeg / 100.0
            self.position_error_deg = error_cdeg / 100.0
            if len(payload) >= 31:
                self.current_multi_turn_deg = struct.unpack("<i", payload[27:31])[0] / 100.0
            else:
                self.current_multi_turn_deg = self.target_angle_deg - self.position_error_deg
            if len(payload) >= 27:
                self.track_primary_error, self.track_secondary_feedback = (
                    value / 100.0 for value in struct.unpack("<ii", payload[16:24])
                )
                self.track_output = struct.unpack("<h", payload[24:26])[0]
                track_mode = payload[26]
                self.track_status_var.set(
                    f"误差PID：{TRACK_MODE_NAMES.get(track_mode, '未知')}，"
                    f"主误差 {self.track_primary_error:+.2f}，输出 {self.track_output}"
                )
            self.deg_var.set(f"{self.current_angle_deg:7.3f}°")
            raw = struct.unpack("<H", payload[14:16])[0]
            absolute_deg = (float(raw & 0x3FFF) * 360.0) / RAW_MAX
            self.raw_var.set(f"{raw:5d}")
            self.abs_deg_var.set(f"{absolute_deg:7.3f}°")
            if (now - self.last_rpm_display_time) >= RPM_DISPLAY_INTERVAL:
                self.rpm_var.set(f"{rpm_x10 / 10.0:7.2f}")
                self.last_rpm_display_time = now
            self.stream_status_var.set("数据流：开启")
            self.angle_status_var.set(
                f"角度：驱动目标 {self.target_angle_deg:.2f}°，"
                f"当前 {self.current_angle_deg:.2f}°，误差 {self.position_error_deg:+.2f}°，"
                f"PID {'运行' if pid_active else '停止'}"
            )
            self._append_wave_sample(now)
            if status != 0:
                self.status_var.set(f"数据流状态错误：{status}")
            return
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
                chopper_mode = payload[48] if len(payload) >= 49 else None
                position_state = None
                if len(payload) >= 58:
                    target_cdeg = struct.unpack("<i", payload[49:53])[0]
                    error_cdeg = struct.unpack("<i", payload[53:57])[0]
                    position_state = (target_cdeg / 100.0, error_cdeg / 100.0,
                                      payload[57])
                multi_turn_angle = None
                if len(payload) >= 66:
                    multi_turn_angle = struct.unpack("<i", payload[62:66])[0] / 100.0
                motion_profile = payload[66] if len(payload) >= 67 else None
                tracking_reverse = payload[67] if len(payload) >= 68 else None
                extended = (board_mode, sensor_flags, temp_cdeg / 100.0, rpm_x10 / 10.0,
                            track_mode, track_freq, track_speed, track_accel,
                            primary_x100 / 100.0, secondary_x100 / 100.0,
                            track_output, chopper_mode, position_state, multi_turn_angle,
                            motion_profile, tracking_reverse)
            if len(payload) >= 59 and payload[58] in (0, 1):
                detected_role = "Debug" if payload[58] else "Error"
                if self.interface_role_var.get() != detected_role:
                    self.interface_role_var.set(detected_role)
                self.status_var.set(f"已自动识别接口：{detected_role}")
            if len(payload) >= 62:
                self.sequence_status_var.set(
                    f"指令模式：{'开启' if payload[59] else '关闭'}，"
                    f"{'运行中' if payload[60] else '未运行'}，"
                    f"Flash 指令 {payload[61]} 条"
                )
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
            self.dynamic_gain_enabled_var.set(advanced[1] != advanced[2])
            self.prediction_enabled_var.set(advanced[4] != 0)
            self._update_advanced_control_states()
            if len(payload) >= 60:
                self.track_reverse_var.set(payload[59] != 0)
            self.track_status_var.set(
                f"误差PID：{TRACK_MODE_NAMES.get(mode, '未知')}，速度{speed_pct}%={speed_pct * 50}，"
                f"加速度{accel_pct}%={accel_pct * 100}，{frequency}kHz，"
                f"误差{'反向' if self.track_reverse_var.get() else '正向'}"
            )
            return
        if payload:
            status = payload[0]
            text = STATUS_NAMES.get(status, f"ERR {status}")
            command_names = {
                CMD_ENABLE: "电机使能", CMD_MODE: "工作模式", CMD_STREAM: "数据流",
                CMD_PID: "闭环位置 PID", CMD_SPEED: "速度指令", CMD_ANGLE: "绝对角度",
                CMD_RELATIVE_ANGLE: "相对角度", CMD_STOP: "停止电机", CMD_MOVE_STEPS: "脉冲位置",
                CMD_MICROSTEP: "细分", CMD_CLOSED_ACCEL: "闭环加速度", CMD_HOME_MODE: "回零模式",
                CMD_SET_ZERO: "设置零点", CMD_HOME: "执行回零", CMD_TRACK_CONFIG: "误差PID基础配置",
                CMD_TRACK_PID: "误差PID环参数", CMD_TRACK_ADVANCED: "误差PID高级参数",
                CMD_TRACK_INPUT: "测试误差", CMD_TRACK_DEFAULT: "恢复误差PID默认", CMD_TRACK_DIRECTION: "误差方向",
                CMD_CHOPPER_MODE: "斩波模式", CMD_MOTION_PROFILE: "加减速曲线",
                CMD_PROGRAM_MODE: "指令模式", CMD_PROGRAM_STEP: "内部指令", CMD_PROGRAM_STAGE: "写入RAM",
                CMD_PROGRAM_SAVE: "保存内部指令", CMD_PROGRAM_RUN: "运行内部指令",
                CMD_SAVE_RUNTIME_CONFIG: "保存 PID 到 Flash",
            }
            self._report_operation(f"{command_names.get(cmd, f'命令0x{cmd:02X}')}：{text}")
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
            elif cmd in (CMD_ANGLE, CMD_RELATIVE_ANGLE) and not completed_pending:
                self.angle_status_var.set(f"角度：{text}")
            elif cmd == CMD_STOP:
                self.speed_status_var.set(f"停止：{text}")
            elif cmd == CMD_MOVE_STEPS:
                self.move_status_var.set(f"脉冲位置：{text}")
            elif cmd == CMD_MICROSTEP:
                self.microstep_status_var.set(f"细分：{text}")
            elif cmd == CMD_CLOSED_ACCEL:
                self.closed_accel_status_var.set(f"闭环加速度：{text}")
                self.pid_status_var.set(f"闭环加速度：{self.closed_accel_var.get()}，{text}")
            elif cmd == CMD_HOME_MODE:
                self.home_status_var.set(f"回零模式：{text}")
            elif cmd == CMD_SET_ZERO:
                self.home_status_var.set(f"设置零点：{text}")
            elif cmd == CMD_HOME:
                self.home_status_var.set(f"回零：{text}")
            elif cmd in (CMD_TRACK_CONFIG, CMD_TRACK_PID, CMD_TRACK_ADVANCED,
                          CMD_TRACK_INPUT, CMD_TRACK_DEFAULT, CMD_TRACK_QUERY):
                self.track_status_var.set(f"误差PID：{text}")
            elif cmd == CMD_TRACK_DIRECTION:
                self.track_status_var.set(f"误差PID：误差方向 {'反向' if self.track_reverse_var.get() else '正向'}，{text}")
            elif cmd == CMD_CHOPPER_MODE:
                self.chopper_status_var.set(f"斩波模式：{text}")
            elif cmd == CMD_MOTION_PROFILE:
                self.motion_profile_status_var.set(f"曲线：{text}")
            elif cmd == CMD_SAVE_RUNTIME_CONFIG:
                self.pid_status_var.set("PID：已保存到 Flash" if status == 0 else f"PID：保存 Flash {text}")
            elif cmd in (CMD_PROGRAM_MODE, CMD_PROGRAM_STEP, CMD_PROGRAM_STAGE,
                         CMD_PROGRAM_SAVE, CMD_PROGRAM_RUN):
                self.sequence_status_var.set(f"内部指令：{text}")
            elif cmd == CMD_PROGRAM_QUERY and len(payload) >= 4:
                self.sequence_status_var.set(
                    f"指令模式：{'开启' if payload[1] else '关闭'}，"
                    f"{'运行中' if payload[2] else '未运行'}，当前指令 {payload[3]} 条"
                )
        if self.pending_tx is None and not self.tx_queue:
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
        self.enable_status_var.set("电机：已使能" if enabled else "电机：已失能")
        self.stream_status_var.set("数据流：开启" if stream else "数据流：关闭")
        self.current_mode = mode
        self.mode_status_var.set(f"模式：{MODE_NAMES.get(mode, '未知')}")
        self._sync_waveform_presentation()
        if microsteps is not None:
            self.current_microsteps = microsteps
            self.microstep_status_var.set(f"细分：当前 {microsteps}")
        if closed_accel is not None:
            self.closed_accel_status_var.set(f"闭环加速度：当前 {closed_accel}")
        if zero_valid is not None:
            home_text = HOME_MODE_NAMES.get(home_mode, "未知")
            self.home_status_var.set(f"零点：{'已设置' if zero_valid else '未设置'}，回零模式：{home_text}")
        if pid_params is not None:
            kp, ki, kd, limit, deadband = pid_params
            accel_text = str(closed_accel) if closed_accel is not None else "--"
            self.pid_status_var.set(
                f"PID：当前 Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}, Limit={limit}, "
                f"Deadband={deadband:.3f}, Accel={accel_text}"
            )
        if extended is not None:
            board_mode, sensor_flags, temperature, _rpm, track_mode, track_freq, track_speed, track_accel, primary, secondary, track_output, chopper_mode, position_state, multi_turn_angle, motion_profile, tracking_reverse = extended
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
            self.track_primary_error = primary
            self.track_secondary_feedback = secondary
            self.track_output = track_output
            if multi_turn_angle is not None:
                self.current_multi_turn_deg = multi_turn_angle
            if chopper_mode is not None:
                chopper_name = CHOPPER_MODE_NAMES.get(chopper_mode, f"未知{chopper_mode}")
                self.chopper_status_var.set(f"斩波模式：当前 {chopper_name}")
            else:
                chopper_name = "旧固件未知"
            if motion_profile is not None:
                motion_name = MOTION_PROFILE_NAMES.get(motion_profile, f"未知{motion_profile}")
                self.motion_profile_var.set(motion_name)
                self.motion_profile_status_var.set(f"曲线：当前 {motion_name}")
            if tracking_reverse is not None:
                self.track_reverse_var.set(tracking_reverse != 0)
            self.driver_status_var.set(
                f"实际驱动状态：{MODE_NAMES.get(mode, '未知')} | {self.current_microsteps}细分 | {chopper_name}"
            )
            if position_state is not None and mode == MODE_CLOSED_POSITION:
                actual_target, position_error, pid_active = position_state
                self.target_angle_deg = actual_target
                self.position_error_deg = position_error
                self.angle_status_var.set(
                    f"角度：驱动目标 {actual_target:.2f}°，误差 {position_error:+.2f}°，"
                    f"PID {'运行' if pid_active else '停止'}"
                )
        self._append_wave_sample(now)
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
        self.current_multi_turn_deg = None
        self.target_angle_deg = None
        self.position_error_deg = 0.0
        self.track_primary_error = 0.0
        self.track_secondary_feedback = 0.0
        self.track_output = 0
        self.wave_history.clear()
        if hasattr(self, "wave_canvas"):
            self._redraw_waveform()

    def clear_waveform(self):
        self.wave_history.clear()
        self._redraw_waveform()

    def _on_interface_role_changed(self, *_args):
        """Refresh waveform semantics when the connected interface changes."""
        self._sync_waveform_presentation(force=True)

    def _sync_waveform_presentation(self, force=False):
        """Tracking data always takes priority over the physical UART role."""
        tracking_active = self.current_mode == MODE_TRACK_PID
        error_view = tracking_active or self.interface_role_var.get() == "Debug"
        view_kind = "error" if error_view else "position"

        if not force and getattr(self, "wave_view_kind", None) == view_kind:
            return

        self.wave_view_kind = view_kind
        self.wave_history.clear()
        if view_kind == "error":
            self.wave_range_limit = 1000.0
            if hasattr(self, "wave_frame"):
                self.wave_frame.configure(text="视觉追踪波形：目标误差 0 / 当前误差")
        else:
            self.wave_range_limit = 7200.0
            if hasattr(self, "wave_frame"):
                self.wave_frame.configure(text="多圈位置波形：目标 / 当前位置")
        self._redraw_waveform()

    def zoom_waveform(self, scale):
        self.wave_pixels_per_second = max(5.0, min(180.0, self.wave_pixels_per_second * scale))
        self._redraw_waveform()

    def zoom_waveform_vertical(self, scale):
        error_view = getattr(self, "wave_view_kind", "position") == "error"
        minimum = 5.0 if error_view else 90.0
        maximum = 1000.0 if error_view else 7200.0
        self.wave_range_limit = max(minimum, min(maximum, self.wave_range_limit * scale))
        self._redraw_waveform()

    def _wave_xview(self, *args):
        self.wave_follow_var.set(False)
        self.wave_canvas.xview(*args)

    def _append_wave_sample(self, now):
        if self.wave_paused_var.get():
            return
        if self.current_mode == MODE_TRACK_PID or self.interface_role_var.get() == "Debug":
            current = self.track_primary_error
            target = 0.0
        else:
            current = self.current_multi_turn_deg
            if current is None and self.target_angle_deg is not None:
                current = self.target_angle_deg - self.position_error_deg
            if current is None:
                return
            if self.current_mode == MODE_CLOSED_POSITION and self.target_angle_deg is not None:
                target = self.target_angle_deg
            else:
                target = current

        self.wave_history.append((now, current, target))
        while self.wave_history and (now - self.wave_history[0][0]) > WAVEFORM_HISTORY_SECONDS:
            self.wave_history.popleft()
        self._redraw_waveform()

    def _angle_to_y(self, angle, low, high, top, height):
        if high <= low:
            return top + height / 2.0
        value = max(low, min(high, float(angle)))
        return top + (high - value) * height / (high - low)

    def _redraw_waveform(self):
        if not hasattr(self, "wave_canvas") or not hasattr(self, "wave_axis_canvas"):
            return

        canvas = self.wave_canvas
        axis = self.wave_axis_canvas
        canvas.delete("all")
        axis.delete("all")
        width = max(canvas.winfo_width(), 600)
        height = max(canvas.winfo_height(), 160)
        right_pad = 16
        top = 14
        graph_h = height - 42
        low = -self.wave_range_limit
        high = self.wave_range_limit

        axis.configure(scrollregion=(0, 0, 66, height))
        for index in range(5):
            value = high - (high - low) * index / 4.0
            y = self._angle_to_y(value, low, high, top, graph_h)
            axis.create_text(62, y, anchor=tk.E, text=f"{value:g}", fill="#475569", font=("Consolas", 9))
        axis.create_text(62, top + graph_h + 16, anchor=tk.E,
                         text="deg" if self.interface_role_var.get() == "Error" else "error",
                         fill="#475569", font=("Consolas", 8))

        history = list(self.wave_history)
        if len(history) == 0:
            canvas.configure(scrollregion=(0, 0, width, height))
            text = "等待多圈位置数据" if self.interface_role_var.get() == "Error" else "等待视觉误差数据"
            canvas.create_text(width / 2, height / 2, text=text, fill="#64748b", font=("Segoe UI", 12))
            return

        start_t = history[0][0]
        end_t = max(history[-1][0], start_t + 1.0)
        graph_w = max(width - right_pad, int((end_t - start_t) * self.wave_pixels_per_second) + 80)
        total_w = graph_w + right_pad
        canvas.configure(scrollregion=(0, 0, total_w, height))

        for index in range(5):
            value = high - (high - low) * index / 4.0
            y = self._angle_to_y(value, low, high, top, graph_h)
            canvas.create_line(0, y, total_w - right_pad, y, fill="#d8e0e8")

        zero_y = self._angle_to_y(0.0, low, high, top, graph_h)
        canvas.create_line(0, zero_y, total_w - right_pad, zero_y, fill="#94a3b8", dash=(4, 3))
        canvas.create_line(0, top + graph_h, total_w - right_pad, top + graph_h, fill="#94a3b8")

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
                x = (t - start_t) * self.wave_pixels_per_second
                y = self._angle_to_y(value, low, high, top, graph_h)
                points.extend((x, y))
            return points

        current_points = make_points(1)
        target_points = make_points(2)
        if len(target_points) >= 4:
            canvas.create_line(*target_points, fill="#d69e00", width=2, smooth=False)
        if len(current_points) >= 4:
            canvas.create_line(*current_points, fill="#111827", width=2, smooth=False)

        if self.wave_follow_var.get():
            canvas.xview_moveto(1.0)

    def _on_close(self):
        self.reader.stop()
        self.destroy()


if __name__ == "__main__":
    app = AngleViewer()
    app.mainloop()
