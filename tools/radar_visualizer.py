#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Display obstacle snapshots reported by the STM32 over a USB UART."""

import math
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import serial
from serial.tools import list_ports

if __package__:
    from .obstacle_protocol import LineDecoder, SnapshotState, vehicle_to_field
else:
    from obstacle_protocol import LineDecoder, SnapshotState, vehicle_to_field


class RadarVisualizer:
    MAP_SIZE_M = 2.4
    VEHICLE_X_M = 0.168
    VEHICLE_Y_M = 0.168
    MAX_RANGE_M = 6.2
    C = {
        "window": "#e9eef2", "surface": "#ffffff", "surface_2": "#f5f7f9",
        "ink": "#17212b", "muted": "#687684", "grid": "#d7dddf",
        "grid_major": "#aeb8bd", "green": "#27865f", "amber": "#d68a17",
        "red": "#c94747", "map_bg": "#eef1f1", "zone": "#fff9c9",
        "zone_edge": "#a99e59", "start": "#175bd1",
    }

    def __init__(self, root):
        self.root = root
        root.title("STM32 障碍物监视器")
        width = min(1240, root.winfo_screenwidth() - 80)
        height = min(800, root.winfo_screenheight() - 80)
        root.geometry(f"{width}x{height}")
        root.minsize(960, 620)
        root.configure(bg=self.C["window"])
        self.serial_port = None
        self.reader_thread = None
        self.stop_reader = None
        self.running = False
        self.session = 0
        self.events = queue.Queue()
        self.data_lock = threading.Lock()
        self.snapshot = SnapshotState()
        self.received_reports = 0
        self.report_errors = 0
        self.bytes_since_rate = self.reports_since_rate = 0
        self.report_rate = self.throughput_kbps = 0.0
        self.rate_started_at = time.monotonic()
        self.port_var = tk.StringVar()
        self.view_var = tk.StringVar(value="field")
        self.heading_text_var = tk.StringVar(value="0.0")
        self.vehicle_heading_deg = 0.0
        self.map_zoom = 1.0
        self.map_center_x_m = self.map_center_y_m = self.MAP_SIZE_M / 2
        self.pan_start = None
        self.connection_var = tk.StringVar(value="未连接")
        self.values = {name: tk.StringVar(value=value) for name, value in {
            "obstacles": "0", "front": "--", "blocked": "--", "age": "--",
            "scan_seq": "--", "reports": "0", "rate": "0.0 次/秒",
            "throughput": "0.0 KB/s", "errors": "0", "rx_frames": "0",
            "bad_frames": "0", "rx_dropped": "0", "uart_errors": "0",
        }.items()}
        self._setup_styles()
        self._build_ui()
        self.refresh_ports()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(50, self.refresh_display)

    def _setup_styles(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("Root.TFrame", background=self.C["window"])
        style.configure("Surface.TFrame", background=self.C["surface"])
        style.configure("Toolbar.TFrame", background=self.C["surface_2"])
        style.configure("Label.TLabel", background=self.C["surface"], foreground=self.C["muted"], font=("Microsoft YaHei UI", 9))
        style.configure("Section.TLabel", background=self.C["surface"], foreground=self.C["ink"], font=("Microsoft YaHei UI", 10, "bold"))
        style.configure("Tool.TButton", padding=(10, 6), font=("Microsoft YaHei UI", 9))

    def _build_ui(self):
        outer = ttk.Frame(self.root, style="Root.TFrame", padding=16)
        outer.pack(fill=tk.BOTH, expand=True)
        header = ttk.Frame(outer, style="Surface.TFrame", padding=(16, 12))
        header.pack(fill=tk.X, pady=(0, 12))
        ttk.Label(header, text="STM32 障碍物监视器", style="Section.TLabel").pack(side=tk.LEFT)
        self.status_badge = tk.Label(header, textvariable=self.connection_var, bg="#f9e6e6", fg=self.C["red"], padx=10, pady=6)
        self.status_badge.pack(side=tk.RIGHT, padx=(10, 0))
        self.connect_button = ttk.Button(header, text="连接", command=self.toggle_connection, style="Tool.TButton")
        self.connect_button.pack(side=tk.RIGHT)
        ttk.Button(header, text="刷新", command=self.refresh_ports, style="Tool.TButton").pack(side=tk.RIGHT, padx=6)
        self.port_combo = ttk.Combobox(header, textvariable=self.port_var, state="readonly", width=12)
        self.port_combo.pack(side=tk.RIGHT)
        body = ttk.Frame(outer, style="Root.TFrame")
        body.pack(fill=tk.BOTH, expand=True)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)
        sidebar = ttk.Frame(body, style="Surface.TFrame", padding=16, width=245)
        sidebar.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        sidebar.grid_propagate(False)
        self._metrics(sidebar, "当前障碍物", (("数量", "obstacles"), ("前方距离", "front"), ("前方状态", "blocked"), ("扫描数据年龄", "age"), ("扫描序号", "scan_seq")))
        ttk.Separator(sidebar).pack(fill=tk.X, pady=8)
        self._metrics(sidebar, "串口链路", (("快照数", "reports"), ("更新速率", "rate"), ("串口流量", "throughput"), ("报告错误", "errors"), ("雷达有效帧", "rx_frames"), ("雷达坏帧", "bad_frames"), ("接收丢字节", "rx_dropped"), ("UART 错误", "uart_errors")))
        ttk.Separator(sidebar).pack(fill=tk.X, pady=8)
        ttk.Label(sidebar, text="显示基准", style="Section.TLabel").pack(anchor="w", pady=(0, 8))
        ttk.Label(sidebar, text="车辆 X 0.168 m / Y 0.168 m", style="Label.TLabel").pack(anchor="w")
        row = ttk.Frame(sidebar, style="Surface.TFrame")
        row.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(row, text="显示朝向", style="Label.TLabel").pack(side=tk.LEFT)
        entry = ttk.Spinbox(row, from_=0, to=359, increment=1, textvariable=self.heading_text_var, width=6, command=self._apply_heading)
        entry.pack(side=tk.RIGHT)
        entry.bind("<Return>", self._apply_heading)
        entry.bind("<FocusOut>", self._apply_heading)
        viewer = ttk.Frame(body, style="Surface.TFrame")
        viewer.grid(row=0, column=1, sticky="nsew")
        viewer.columnconfigure(0, weight=1)
        viewer.rowconfigure(1, weight=1)
        toolbar = ttk.Frame(viewer, style="Toolbar.TFrame", padding=(12, 8))
        toolbar.grid(row=0, column=0, sticky="ew")
        for text, mode in (("场地图", "field"), ("车体坐标", "vehicle")):
            ttk.Radiobutton(toolbar, text=text, value=mode, variable=self.view_var, command=self.draw_view).pack(side=tk.LEFT, padx=(0, 14))
        self.canvas = tk.Canvas(viewer, bg="#f7f9fa", highlightthickness=0)
        self.canvas.grid(row=1, column=0, sticky="nsew")
        self.canvas.bind("<Configure>", lambda _event: self.draw_view())
        for event in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            self.canvas.bind(event, self._zoom_map)
        self.canvas.bind("<ButtonPress-2>", self._start_map_pan)
        self.canvas.bind("<B2-Motion>", self._pan_map)
        self.canvas.bind("<ButtonRelease-2>", self._stop_map_pan)
        self.footer_var = tk.StringVar(value="等待 STM32 数据")
        tk.Label(viewer, textvariable=self.footer_var, bg=self.C["surface_2"], fg=self.C["muted"], anchor="w", padx=12, pady=8).grid(row=2, column=0, sticky="ew")

    def _metrics(self, parent, title, entries):
        ttk.Label(parent, text=title, style="Section.TLabel").pack(anchor="w", pady=(0, 8))
        for name, key in entries:
            row = ttk.Frame(parent, style="Surface.TFrame")
            row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=name, style="Label.TLabel").pack(side=tk.LEFT)
            ttk.Label(row, textvariable=self.values[key], style="Label.TLabel").pack(side=tk.RIGHT)

    def refresh_ports(self):
        devices = [port.device for port in sorted(list_ports.comports(), key=lambda port: port.device)]
        self.port_combo["values"] = devices
        if devices and self.port_var.get() not in devices:
            self.port_var.set(devices[0])
        elif not devices:
            self.port_var.set("")

    def toggle_connection(self):
        self.disconnect() if self.running else self.connect()

    def connect(self):
        port_name = self.port_var.get().strip()
        if not port_name:
            messagebox.showwarning("未选择串口", "请连接 USB 串口设备并点击刷新。")
            return
        try:
            port = serial.Serial(port_name, 230400, timeout=0.1)
        except (ValueError, serial.SerialException) as exc:
            messagebox.showerror("连接失败", str(exc))
            return
        stop = threading.Event()
        with self.data_lock:
            self.session += 1
            session = self.session
            self.snapshot.clear()
            self.received_reports = self.report_errors = 0
            self.bytes_since_rate = self.reports_since_rate = 0
            self.report_rate = self.throughput_kbps = 0.0
            self.rate_started_at = time.monotonic()
            self.running = True
        self.serial_port, self.stop_reader = port, stop
        self.reader_thread = threading.Thread(target=self._read_serial, args=(port, stop, session), daemon=True, name="stm32-obstacles")
        self.reader_thread.start()
        self.port_combo.configure(state="disabled")
        self.connection_var.set(f"已连接 {port_name}")
        self.status_badge.configure(bg="#e1f2e9", fg=self.C["green"])
        self.connect_button.configure(text="断开")
        self.draw_view()

    def disconnect(self):
        with self.data_lock:
            self.running = False
            self.session += 1
            self.snapshot.clear()
        if self.stop_reader:
            self.stop_reader.set()
        port, self.serial_port = self.serial_port, None
        if port:
            try:
                port.close()
            except serial.SerialException:
                pass
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=0.25)
        self.reader_thread = self.stop_reader = None
        self.port_combo.configure(state="readonly")
        self.connection_var.set("未连接")
        self.status_badge.configure(bg="#f9e6e6", fg=self.C["red"])
        self.connect_button.configure(text="连接")
        self.draw_view()

    def _read_serial(self, port, stop, session):
        decoder = LineDecoder()
        while not stop.is_set():
            try:
                chunk = port.read(1024)
                if not chunk:
                    continue
                reports = decoder.feed(chunk)
                received_at = time.monotonic()
                with self.data_lock:
                    if session != self.session or not self.running:
                        return
                    self.bytes_since_rate += len(chunk)
                    self.report_errors = decoder.errors
                    for report in reports:
                        self.snapshot.update(report, received_at)
                    self.received_reports += len(reports)
                    self.reports_since_rate += len(reports)
            except (serial.SerialException, OSError) as exc:
                if not stop.is_set():
                    self.events.put((session, str(exc)))
                return

    def refresh_display(self):
        while True:
            try:
                session, detail = self.events.get_nowait()
            except queue.Empty:
                break
            if session == self.session:
                self.disconnect()
                messagebox.showerror("串口已断开", detail)
        now = time.monotonic()
        with self.data_lock:
            interval = now - self.rate_started_at
            if interval >= 0.75:
                self.report_rate = self.reports_since_rate / interval
                self.throughput_kbps = self.bytes_since_rate / interval / 1024.0
                self.bytes_since_rate = self.reports_since_rate = 0
                self.rate_started_at = now
            report = self.snapshot.latest
            fresh = self.snapshot.is_fresh(now)
            age = self.snapshot.age_ms(now)
            count = len(self.snapshot.obstacles(now))
            counters = {"reports": str(self.received_reports), "errors": str(self.report_errors),
                        "rate": f"{self.report_rate:.1f} 次/秒", "throughput": f"{self.throughput_kbps:.1f} KB/s"}
        counters.update(obstacles=str(count), age=f"{int(age)} ms" if age is not None else "--", scan_seq=str(report.scan_seq) if report else "--")
        counters["front"] = f"{report.front_m:.3g} m" if fresh and report.front_m is not None else "--"
        counters["blocked"] = ("有障碍" if report.blocked else "畅通") if fresh and report.blocked is not None else "--"
        for name in ("rx_frames", "bad_frames", "rx_dropped", "uart_errors"):
            counters[name] = str(getattr(report, name)) if report else "0"
        for name, value in counters.items():
            self.values[name].set(value)
        self.footer_var.set("未连接" if not self.running else "等待 STM32 数据" if report is None else "雷达无有效扫描" if not report.valid else "扫描数据已过期" if not fresh else f"障碍物 {count} 个")
        self.draw_view()
        self.root.after(50, self.refresh_display)

    def _apply_heading(self, _event=None):
        try:
            heading = float(self.heading_text_var.get())
            if not math.isfinite(heading):
                raise ValueError("invalid heading")
            self.vehicle_heading_deg = heading % 360
        except ValueError:
            pass
        self.heading_text_var.set(f"{self.vehicle_heading_deg:.1f}")
        self.draw_view()

    def draw_view(self):
        if not hasattr(self, "canvas"):
            return
        with self.data_lock:
            obstacles = self.snapshot.obstacles(time.monotonic())
        self.canvas.delete("all")
        if self.view_var.get() == "vehicle":
            self._draw_vehicle_view(obstacles)
        else:
            self._draw_field(obstacles)

    def _field_geometry(self):
        width, height = max(self.canvas.winfo_width(), 320), max(self.canvas.winfo_height(), 300)
        size = max(180, min(width - 120, height - 140)) * self.map_zoom
        return (width / 2 - self.map_center_x_m / self.MAP_SIZE_M * size,
                height / 2 - (1 - self.map_center_y_m / self.MAP_SIZE_M) * size, size)

    def _field_xy(self, x_m, y_m):
        left, top, size = self._field_geometry()
        return left + x_m / self.MAP_SIZE_M * size, top + (1 - y_m / self.MAP_SIZE_M) * size

    def _clamp_map_center(self):
        half = self.MAP_SIZE_M / (2 * self.map_zoom)
        self.map_center_x_m = max(half, min(self.MAP_SIZE_M - half, self.map_center_x_m))
        self.map_center_y_m = max(half, min(self.MAP_SIZE_M - half, self.map_center_y_m))

    def _zoom_map(self, event):
        if self.view_var.get() != "field":
            return
        delta, button = getattr(event, "delta", 0), getattr(event, "num", 0)
        factor = 1.18 if delta > 0 or button == 4 else 1 / 1.18 if delta < 0 or button == 5 else 1
        self.map_zoom = max(1.0, min(5.0, self.map_zoom * factor))
        self._clamp_map_center()
        self.draw_view()
        return "break"

    def _start_map_pan(self, event):
        if self.view_var.get() == "field" and self.map_zoom > 1:
            self.pan_start = (event.x, event.y, self.map_center_x_m, self.map_center_y_m)
            self.canvas.configure(cursor="fleur")

    def _pan_map(self, event):
        if not self.pan_start:
            return
        start_x, start_y, center_x, center_y = self.pan_start
        _, _, size = self._field_geometry()
        self.map_center_x_m = center_x - (event.x - start_x) / size * self.MAP_SIZE_M
        self.map_center_y_m = center_y + (event.y - start_y) / size * self.MAP_SIZE_M
        self._clamp_map_center()
        self.draw_view()

    def _stop_map_pan(self, event):
        self._pan_map(event)
        self.pan_start = None
        self.canvas.configure(cursor="")

    def _vehicle_polygon(self):
        points = []
        for x, y in ((-0.132, -0.132), (0.132, -0.132), (0.132, 0.132), (-0.132, 0.132)):
            world = vehicle_to_field(x, y, self.VEHICLE_X_M, self.VEHICLE_Y_M, self.vehicle_heading_deg)
            points.extend(self._field_xy(*world))
        return points

    def _obstacle_icon(self, x, y, obstacle):
        color = self.C["red"] if math.hypot(obstacle.x_m, obstacle.y_m) < 0.5 else self.C["amber"]
        self.canvas.create_polygon(x, y - 7, x + 7, y, x, y + 7, x - 7, y, fill=color, outline="white", width=1, tags="obstacle")
        label = self.canvas.create_text(0, 0, text=f"({obstacle.x_m:.2f}, {obstacle.y_m:.2f}) m", anchor="w", fill=self.C["ink"], font=("Segoe UI", 8), tags="obstacle-label")
        occupied = [self.canvas.bbox(item) for item in self.canvas.find_all() if item != label and self.canvas.type(item) == "text"]
        bounds = self.canvas.bbox(label)
        label_width = bounds[2] - bounds[0]
        width, height = max(320, self.canvas.winfo_width()), max(300, self.canvas.winfo_height())
        placed = False
        for step in range(20):
            for sign in (-1, 1):
                label_y = min(max(y + sign * (10 + step * 18), 12), height - 12)
                for candidate_x in (x + 12, x - label_width - 12):
                    label_x = min(max(candidate_x, 8), max(8, width - label_width - 8))
                    self.canvas.coords(label, label_x, label_y)
                    box = self.canvas.bbox(label)
                    if all(box[2] + 3 < old[0] or box[0] - 3 > old[2] or box[3] + 3 < old[1] or box[1] - 3 > old[3] for old in occupied):
                        placed = True
                        break
                if placed:
                    break
            if placed:
                break
        box = self.canvas.bbox(label)
        background = self.canvas.create_rectangle(box[0] - 2, box[1] - 1, box[2] + 2, box[3] + 1, fill=self.canvas.cget("bg"), outline="", tags="obstacle-label-bg")
        self.canvas.tag_lower(background, label)
        if abs(label_y - y) > 20 or label_x < x:
            end_x = min(max(x, label_x), label_x + label_width)
            leader = self.canvas.create_line(x, y, end_x, label_y, fill=color, width=1)
            self.canvas.tag_lower(leader, background)

    def _draw_field(self, obstacles):
        c = self.C
        left, top, size = self._field_geometry()
        self.canvas.create_rectangle(left, top, left + size, top + size, fill=c["map_bg"], outline="#4f595e", width=2)
        for index in range(25):
            value = index / 10
            color = c["grid_major"] if index % 5 == 0 else c["grid"]
            self.canvas.create_line(*self._field_xy(value, 0), *self._field_xy(value, 2.4), fill=color)
            self.canvas.create_line(*self._field_xy(0, value), *self._field_xy(2.4, value), fill=color)
        for zone in ((0.55, 1.40, 1.00, 1.85), (1.40, 1.40, 1.85, 1.85), (0.55, 0.55, 1.00, 1.00), (1.40, 0.55, 1.85, 1.00)):
            self.canvas.create_rectangle(*self._field_xy(*zone[:2]), *self._field_xy(*zone[2:]), fill=c["zone"], outline=c["zone_edge"], width=2)
        self.canvas.create_line(*self._field_xy(1.2, 0), *self._field_xy(1.2, 2.4), fill="#657177", dash=(8, 6))
        self.canvas.create_line(*self._field_xy(0, 1.2), *self._field_xy(2.4, 1.2), fill="#657177", dash=(8, 6))
        for number, y in ((1, 2.10), (2, 0.0)):
            self.canvas.create_rectangle(*self._field_xy(2.10, y), *self._field_xy(2.40, y + 0.30), fill=c["start"], outline="#0d3f9f")
            self.canvas.create_text(*self._field_xy(2.25, y + 0.15), text=f"启停\n区{number}", fill="white", font=("Microsoft YaHei UI", 8))
        raw_x, raw_y = self._field_xy(1.20, 2.50)
        radius = 0.15 / self.MAP_SIZE_M * size
        self.canvas.create_oval(raw_x - radius, raw_y - radius, raw_x + radius, raw_y + radius, fill="#f4f5f5", outline="#4f565a")
        for degrees in (90, 210, 330):
            angle = math.radians(degrees)
            x, y = raw_x + radius * 0.58 * math.cos(angle), raw_y - radius * 0.58 * math.sin(angle)
            self.canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill="white", outline="#4f565a")
        self.canvas.create_text(raw_x, raw_y + radius + 12, text="原料区", fill=c["ink"], font=("Microsoft YaHei UI", 9))
        for zone in ((0.0, 0.91, 0.15, 1.49), (0.91, 0.0, 1.49, 0.15)):
            self.canvas.create_rectangle(*self._field_xy(*zone[:2]), *self._field_xy(*zone[2:]), fill="#f5f6f6", outline="#7d858b")
        ring_radius = 0.045 / self.MAP_SIZE_M * size
        for number, value in enumerate((1.39, 1.20, 1.01), 1):
            for position in ((0.075, value), (2.40 - value, 0.075)):
                x, y = self._field_xy(*position)
                self.canvas.create_oval(x - ring_radius, y - ring_radius, x + ring_radius, y + ring_radius, fill="white", outline="#4f565a", width=2)
                self.canvas.create_text(x, y, text=str(number), fill=c["ink"], font=("Segoe UI", 7, "bold"))
        for x, y, text in ((0.19, 1.20, "暂存区"), (1.20, 0.20, "粗加工区")):
            self.canvas.create_text(*self._field_xy(x, y), text=text, fill=c["ink"], font=("Microsoft YaHei UI", 9), anchor="w" if x < 0.2 else "center")
        self.canvas.create_line(*self._field_xy(2.40, 1.0), *self._field_xy(2.40, 1.4), fill="#24282b", width=5)
        self.canvas.create_text(*self._field_xy(2.35, 1.20), text="二维码板", angle=90, fill=c["ink"], font=("Microsoft YaHei UI", 9))
        self.canvas.create_polygon(*self._vehicle_polygon(), fill="#edf7f5", outline=c["green"], width=2)
        front = vehicle_to_field(0, 0.10, self.VEHICLE_X_M, self.VEHICLE_Y_M, self.vehicle_heading_deg)
        self.canvas.create_line(*self._field_xy(self.VEHICLE_X_M, self.VEHICLE_Y_M), *self._field_xy(*front), fill=c["green"], width=3, arrow=tk.LAST)
        for value in (0.0, 0.6, 1.2, 1.8, 2.4):
            x, _ = self._field_xy(value, 0)
            _, y = self._field_xy(0, value)
            self.canvas.create_text(x, top + size + 16, text=f"{value:.1f}", fill=c["ink"], font=("Segoe UI", 9))
            self.canvas.create_text(left - 25, y, text=f"{value:.1f}", fill=c["ink"], font=("Segoe UI", 9))
        for obstacle in obstacles:
            world = vehicle_to_field(obstacle.x_m, obstacle.y_m, self.VEHICLE_X_M, self.VEHICLE_Y_M, self.vehicle_heading_deg)
            if 0 <= world[0] <= self.MAP_SIZE_M and 0 <= world[1] <= self.MAP_SIZE_M:
                self._obstacle_icon(*self._field_xy(*world), obstacle)

    def _draw_vehicle_view(self, obstacles):
        width, height = max(self.canvas.winfo_width(), 320), max(self.canvas.winfo_height(), 300)
        cx, cy = width / 2, height / 2
        scale = min(width - 110, height - 100) / (2 * self.MAX_RANGE_M)
        for meter in range(-6, 7):
            x, y = cx + meter * scale, cy - meter * scale
            self.canvas.create_line(x, cy - 6 * scale, x, cy + 6 * scale, fill=self.C["grid"])
            self.canvas.create_line(cx - 6 * scale, y, cx + 6 * scale, y, fill=self.C["grid"])
            if meter and meter % 2 == 0:
                self.canvas.create_text(x, cy + 14, text=str(meter), fill=self.C["muted"], font=("Segoe UI", 8))
                self.canvas.create_text(cx - 14, y, text=str(meter), fill=self.C["muted"], font=("Segoe UI", 8))
        self.canvas.create_line(cx - 6 * scale, cy, cx + 6 * scale, cy, fill=self.C["green"], arrow=tk.LAST)
        self.canvas.create_line(cx, cy + 6 * scale, cx, cy - 6 * scale, fill=self.C["green"], arrow=tk.LAST)
        self.canvas.create_text(cx + 6 * scale, cy - 15, text="X / 右 (m)", anchor="e", fill=self.C["ink"])
        self.canvas.create_text(cx + 8, cy - 6 * scale - 12, text="Y / 前 (m)", anchor="w", fill=self.C["ink"])
        half = max(4, 0.132 * scale)
        self.canvas.create_rectangle(cx - half, cy - half, cx + half, cy + half, fill="#edf7f5", outline=self.C["green"], width=2)
        for obstacle in obstacles:
            if abs(obstacle.x_m) <= self.MAX_RANGE_M and abs(obstacle.y_m) <= self.MAX_RANGE_M:
                self._obstacle_icon(cx + obstacle.x_m * scale, cy - obstacle.y_m * scale, obstacle)

    def on_close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="Connect to this USB-UART port at startup")
    args = parser.parse_args()
    app_root = tk.Tk()
    app = RadarVisualizer(app_root)
    if args.port:
        app.port_var.set(args.port)
        app_root.after(100, app.connect)
    app_root.mainloop()
