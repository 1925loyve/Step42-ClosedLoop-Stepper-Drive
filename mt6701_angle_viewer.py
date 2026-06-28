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
MAX_PAYLOAD = 24

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

MODE_OPEN_POSITION = 0
MODE_CLOSED_POSITION = 1
MODE_OPEN_SPEED = 2

MODE_LABELS = {
    "速度模式": MODE_OPEN_SPEED,
    "开环位置": MODE_OPEN_POSITION,
    "闭环位置": MODE_CLOSED_POSITION,
}

MODE_NAMES = {value: key for key, value in MODE_LABELS.items()}

STATUS_NAMES = {
    0: "OK",
    1: "长度错误",
    2: "参数超范围",
    3: "未知命令",
    4: "当前模式不支持",
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
        self.geometry("1060x720")
        self.minsize(960, 640)

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
        self.deg_var = tk.StringVar(value="--")
        self.rpm_var = tk.StringVar(value="--")
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
        self.target_angle_var = tk.StringVar(value="0.0")
        self.angle_status_var = tk.StringVar(value="角度：未发送")
        self.move_steps_var = tk.StringVar(value="3200")
        self.move_speed_var = tk.StringVar(value="2500")
        self.move_accel_var = tk.StringVar(value="5000")
        self.move_status_var = tk.StringVar(value="脉冲位置：未发送")
        self.speed_var = tk.StringVar(value="0")
        self.accel_var = tk.StringVar(value="3000")
        self.speed_status_var = tk.StringVar(value="速度：未发送")

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
            [("Raw", self.raw_var), ("Current", self.deg_var), ("RPM", self.rpm_var)]
        ):
            ttk.Label(data, text=title, font=("Segoe UI", 12)).grid(row=0, column=col, sticky=tk.W, padx=(0 if col == 0 else 36, 0))
            ttk.Label(data, textvariable=var, font=("Consolas", 28, "bold")).grid(row=1, column=col, sticky=tk.W, padx=(0 if col == 0 else 36, 0))
            data.columnconfigure(col, weight=1)

        mode = ttk.LabelFrame(root, text="工作模式 / 驱动参数")
        mode.pack(fill=tk.X, pady=(12, 0))
        ttk.Combobox(mode, textvariable=self.mode_var, values=list(MODE_LABELS.keys()), width=12, state="readonly").pack(side=tk.LEFT, padx=(8, 8), pady=8)
        ttk.Button(mode, text="设置模式", command=self.send_mode).pack(side=tk.LEFT, padx=(0, 12), pady=8)
        ttk.Label(mode, text="细分").pack(side=tk.LEFT, padx=(0, 6), pady=8)
        ttk.Combobox(mode, textvariable=self.microstep_var, values=["1", "2", "4", "8", "16", "32", "64", "128", "256"], width=6, state="readonly").pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Button(mode, text="设置细分", command=self.send_microstep).pack(side=tk.LEFT, padx=(0, 12), pady=8)
        ttk.Button(mode, text="使能", command=lambda: self.send_enable(1)).pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Button(mode, text="失能", command=lambda: self.send_enable(0)).pack(side=tk.LEFT, padx=(0, 12), pady=8)
        ttk.Button(mode, text="数据流开", command=lambda: self.send_stream(1)).pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Button(mode, text="数据流关", command=lambda: self.send_stream(0)).pack(side=tk.LEFT, padx=(0, 12), pady=8)
        ttk.Label(mode, textvariable=self.mode_status_var).pack(side=tk.LEFT, padx=(0, 12), pady=8)
        ttk.Label(mode, textvariable=self.microstep_status_var).pack(side=tk.LEFT, padx=(0, 12), pady=8)
        ttk.Label(mode, textvariable=self.enable_status_var).pack(side=tk.LEFT, padx=(0, 12), pady=8)
        ttk.Label(mode, textvariable=self.stream_status_var).pack(side=tk.LEFT, padx=(0, 8), pady=8)

        speed = ttk.LabelFrame(root, text="速度模式")
        speed.pack(fill=tk.X, pady=(12, 0))
        ttk.Label(speed, text="Speed -5000~5000").pack(side=tk.LEFT, padx=(8, 6), pady=8)
        ttk.Entry(speed, textvariable=self.speed_var, width=10).pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Label(speed, text="Accel").pack(side=tk.LEFT, padx=(0, 6), pady=8)
        ttk.Entry(speed, textvariable=self.accel_var, width=10).pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Button(speed, text="运行速度", command=self.send_speed_value).pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Button(speed, text="停止", command=self.send_stop).pack(side=tk.LEFT, padx=(0, 12), pady=8)
        ttk.Label(speed, textvariable=self.speed_status_var).pack(side=tk.LEFT, padx=(0, 8), pady=8)

        pulse = ttk.LabelFrame(root, text="位置模式：脉冲数移动")
        pulse.pack(fill=tk.X, pady=(12, 0))
        ttk.Label(pulse, text="脉冲数").pack(side=tk.LEFT, padx=(8, 6), pady=8)
        ttk.Entry(pulse, textvariable=self.move_steps_var, width=12).pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Label(pulse, text="速度").pack(side=tk.LEFT, padx=(0, 6), pady=8)
        ttk.Entry(pulse, textvariable=self.move_speed_var, width=10).pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Label(pulse, text="加速度").pack(side=tk.LEFT, padx=(0, 6), pady=8)
        ttk.Entry(pulse, textvariable=self.move_accel_var, width=10).pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Button(pulse, text="发送脉冲", command=self.send_move_steps).pack(side=tk.LEFT, padx=(0, 12), pady=8)
        ttk.Label(pulse, textvariable=self.move_status_var).pack(side=tk.LEFT, padx=(0, 8), pady=8)

        angle = ttk.LabelFrame(root, text="位置模式：角度闭环")
        angle.pack(fill=tk.X, pady=(12, 0))
        ttk.Label(angle, text="目标 0~360°").pack(side=tk.LEFT, padx=(8, 6), pady=8)
        ttk.Entry(angle, textvariable=self.target_angle_var, width=12).pack(side=tk.LEFT, padx=(0, 8), pady=8)
        ttk.Button(angle, text="发送角度", command=self.send_angle_value).pack(side=tk.LEFT, padx=(0, 12), pady=8)
        ttk.Label(angle, textvariable=self.angle_status_var).pack(side=tk.LEFT, padx=(0, 8), pady=8)

        pid = ttk.LabelFrame(root, text="PID 参数")
        pid.pack(fill=tk.X, pady=(12, 0))
        labels = [("Kp", self.kp_var), ("Ki", self.ki_var), ("Kd", self.kd_var), ("Limit", self.limit_var), ("Deadband", self.deadband_var)]
        for i, (label, var) in enumerate(labels):
            ttk.Label(pid, text=label).grid(row=0, column=i * 2, padx=(8, 4), pady=8, sticky=tk.W)
            ttk.Entry(pid, textvariable=var, width=10).grid(row=0, column=i * 2 + 1, padx=(0, 10), pady=8)
        ttk.Button(pid, text="写入PID", command=self.send_pid_values).grid(row=0, column=10, padx=(0, 8), pady=8)
        ttk.Label(pid, textvariable=self.pid_status_var).grid(row=1, column=0, columnspan=11, padx=8, pady=(0, 8), sticky=tk.W)

        wave = ttk.LabelFrame(root, text="目标/当前角度波形")
        wave.pack(fill=tk.BOTH, expand=True, pady=(12, 0))
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
            self.microstep_status_var.set(f"细分：已发送 {microsteps}")

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
        if not (-32768 <= kp_i <= 32767 and -32768 <= ki_i <= 32767 and -32768 <= kd_i <= 32767):
            self.pid_status_var.set("PID：Kp/Ki/Kd 超范围，缩放后必须在 int16 内")
            return
        if not (1 <= limit <= 65535 and 0 <= dead_i <= 10000):
            self.pid_status_var.set("PID：Limit 或 Deadband 超范围")
            return
        payload = struct.pack(
            "<hhhHH",
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
                self.after(100, lambda: self._send(CMD_STREAM, b"\x01"))
                self.after(160, self.send_query)
            if kind in ("closed", "error"):
                self.connected_port = None

        self.after(30, self._poll_queues)

    def _process_frame(self, cmd, seq, payload):
        if cmd == CMD_QUERY and len(payload) >= 10:
            status, enabled, stream, mode, raw, angle_cdeg, pid_output = struct.unpack("<BBBBHHh", payload[:10])
            pid_params = None
            if len(payload) >= 20:
                kp_i, ki_i, kd_i, limit, dead_i = struct.unpack("<hhhHH", payload[10:20])
                pid_params = (kp_i / 100.0, ki_i / 100.0, kd_i / 100.0, limit, dead_i / 1000.0)
            microsteps = None
            if len(payload) >= 22:
                microsteps = struct.unpack("<H", payload[20:22])[0]
            self._display_status(status, enabled, stream, mode, raw, angle_cdeg / 100.0, pid_output, pid_params, microsteps)
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
        self._send(CMD_QUERY)

    def _display_status(self, status, enabled, stream, mode, raw, deg, pid_output, pid_params=None, microsteps=None):
        now = time.monotonic()
        rpm = self._calc_rpm(raw, now)
        self.current_angle_deg = deg
        self.raw_var.set(f"{raw:5d}")
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
        if pid_params is not None:
            kp, ki, kd, limit, deadband = pid_params
            self.pid_status_var.set(
                f"PID：当前 Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}, Limit={limit}, Deadband={deadband:.3f}"
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
