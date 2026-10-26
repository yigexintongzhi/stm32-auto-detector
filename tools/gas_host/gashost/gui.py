"""Tkinter 图形上位机：串口采集、实时曲线、标定拟合与离线回放。"""

from __future__ import annotations

import csv
from datetime import datetime
import math
import os
import queue
import subprocess
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Dict, Iterable, List, Optional, Tuple

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib import rcParams

rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
rcParams["axes.unicode_minus"] = False

from .calib import FIRMWARE_CAL, demo_points, linear_fit, load_points
from .plotter import find_plateaus
from .protocol import Dataset, Event, FrameParser, Result, Sample, Text
from .replay import ReplayResult, replay_dir, replay_file
from .simulator import SimulatedSerial

# 与图表规范一致的浅色主题。两条量程曲线的配色已通过 CVD/对比度校验。
PAGE_BG = "#f9f9f7"
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
SECONDARY = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
BLUE = "#2a78d6"       # 量程 A
ORANGE = "#eb6834"     # 量程 B
GOOD = "#0ca30c"
WARNING = "#a66b00"
CRITICAL = "#d03b3b"

STAGE_STYLE = {
    "A": {"color": BLUE, "marker": "o", "label": "量程 A"},
    "B": {"color": ORANGE, "marker": "s", "label": "量程 B"},
}
FINAL_INDEX = {"A": 119, "B": 19}


class GasHostGUI:
    """上位机主窗口。串口读取在线程中执行，Tk 更新统一回到主线程。"""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("STM32 气体检测仪上位机")
        self.root.geometry("1320x840")
        self.root.minsize(1120, 720)
        self.root.configure(bg=PAGE_BG)

        self._configure_styles()

        self.event_queue: queue.Queue[Tuple[str, object]] = queue.Queue()
        self.stream = None
        self.reader_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.raw_log = None
        self.session_dir: Optional[str] = None
        self.data_dir: Optional[str] = None
        self.parser = FrameParser()
        self.real_serial = False
        self.stream_on = False

        self.samples: Dict[str, List[Tuple[int, int]]] = {"A": [], "B": []}
        self.datasets: Dict[str, List[int]] = {}
        self.current_stage = "—"
        self.last_value: Optional[int] = None
        self.last_result = "等待检测"
        self.final_markers = []
        self.plateau_markers = []
        self.endpoint_label = None
        self.hover_annotation = None

        self.connection_var = tk.StringVar(value="● 未连接")
        self.stage_var = tk.StringVar(value="—")
        self.sample_var = tk.StringVar(value="0")
        self.value_var = tk.StringVar(value="—")
        self.result_var = tk.StringVar(value="等待检测")
        self.status_var = tk.StringVar(value="就绪 · 可连接设备或运行模拟演示")
        self.fit_file_var = tk.StringVar(value="未选择标定 CSV")
        self.replay_summary_var = tk.StringVar(value="请选择数据集或目录")

        self._build_menu()
        self._build_header()
        self._build_tabs()
        self._refresh_ports()

        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(50, self._poll_events)

    # ---------- 界面骨架 ----------

    def _configure_styles(self) -> None:
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(".", font=("Segoe UI", 10), background=PAGE_BG,
                        foreground=TEXT)
        style.configure("TFrame", background=PAGE_BG)
        style.configure("Surface.TFrame", background=SURFACE)
        style.configure("TLabel", background=PAGE_BG, foreground=TEXT)
        style.configure("Muted.TLabel", background=PAGE_BG, foreground=MUTED)
        style.configure("Surface.TLabel", background=SURFACE, foreground=TEXT)
        style.configure("CardTitle.TLabel", background=SURFACE, foreground=SECONDARY,
                        font=("Segoe UI", 9))
        style.configure("CardValue.TLabel", background=SURFACE, foreground=TEXT,
                        font=("Segoe UI Semibold", 18))
        style.configure("ResultValue.TLabel", background=SURFACE, foreground=TEXT,
                        font=("Segoe UI Semibold", 11))
        style.configure("Header.TLabel", background=PAGE_BG, foreground=TEXT,
                        font=("Segoe UI Semibold", 20))
        style.configure("Subheader.TLabel", background=PAGE_BG, foreground=SECONDARY,
                        font=("Segoe UI", 10))
        style.configure("Accent.TButton", background=BLUE, foreground="white",
                        bordercolor=BLUE, focusthickness=0, padding=(12, 7))
        style.map("Accent.TButton", background=[("active", "#1c5cab"),
                                                ("disabled", "#9ec5f4")])
        style.configure("Secondary.TButton", background=SURFACE, foreground=TEXT,
                        bordercolor=AXIS, padding=(10, 7))
        style.map("Secondary.TButton", background=[("active", "#f0efec")])
        style.configure("Danger.TButton", background=SURFACE, foreground=CRITICAL,
                        bordercolor="#e8b1b1", padding=(10, 7))
        style.configure("TNotebook", background=PAGE_BG, borderwidth=0)
        style.configure("TNotebook.Tab", background="#efefec", foreground=SECONDARY,
                        padding=(18, 9), borderwidth=0)
        style.map("TNotebook.Tab", background=[("selected", SURFACE)],
                  foreground=[("selected", TEXT)])
        style.configure("Treeview", background=SURFACE, fieldbackground=SURFACE,
                        foreground=TEXT, rowheight=25, bordercolor=GRID)
        style.configure("Treeview.Heading", background="#efefec", foreground=TEXT,
                        font=("Segoe UI Semibold", 9), relief="flat")
        style.map("Treeview", background=[("selected", "#dceafb")],
                  foreground=[("selected", TEXT)])

    def _build_menu(self) -> None:
        menu = tk.Menu(self.root)
        file_menu = tk.Menu(menu, tearoff=False)
        file_menu.add_command(label="打开本次数据目录", command=self._open_session_dir)
        file_menu.add_command(label="保存当前曲线图", command=self._save_chart)
        file_menu.add_separator()
        file_menu.add_command(label="退出", command=self.close)
        menu.add_cascade(label="文件", menu=file_menu)

        help_menu = tk.Menu(menu, tearoff=False)
        help_menu.add_command(label="使用说明", command=self._show_help)
        help_menu.add_command(label="关于", command=self._show_about)
        menu.add_cascade(label="帮助", menu=help_menu)
        self.root.config(menu=menu)

    def _build_header(self) -> None:
        header = ttk.Frame(self.root, padding=(22, 16, 22, 8))
        header.pack(fill="x")
        left = ttk.Frame(header)
        left.pack(side="left", fill="x", expand=True)
        ttk.Label(left, text="STM32 气体检测仪上位机",
                  style="Header.TLabel").pack(anchor="w")
        ttk.Label(left,
                  text="串口采集 · 实时曲线 · 标定回归 · 固件阈值离线复现",
                  style="Subheader.TLabel").pack(anchor="w", pady=(3, 0))
        self.header_state = ttk.Label(header, textvariable=self.connection_var,
                                      style="Subheader.TLabel")
        self.header_state.pack(side="right", anchor="e")

    def _build_tabs(self) -> None:
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=20, pady=(4, 12))

        self.monitor_tab = ttk.Frame(self.notebook, style="Surface.TFrame")
        self.calib_tab = ttk.Frame(self.notebook, style="Surface.TFrame")
        self.replay_tab = ttk.Frame(self.notebook, style="Surface.TFrame")
        self.notebook.add(self.monitor_tab, text="实时监测")
        self.notebook.add(self.calib_tab, text="标定分析")
        self.notebook.add(self.replay_tab, text="离线回放")

        self._build_monitor_tab()
        self._build_calibration_tab()
        self._build_replay_tab()

        status = ttk.Frame(self.root, padding=(22, 0, 22, 10))
        status.pack(fill="x")
        ttk.Label(status, textvariable=self.status_var,
                  style="Muted.TLabel").pack(side="left")

    # ---------- 实时监测页 ----------

    def _build_monitor_tab(self) -> None:
        controls = ttk.Frame(self.monitor_tab, style="Surface.TFrame",
                             padding=(14, 12))
        controls.pack(fill="x")

        ttk.Label(controls, text="串口", style="Surface.TLabel").grid(row=0, column=0,
                                                                       padx=(0, 5))
        self.port_combo = ttk.Combobox(controls, width=12, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=(0, 5))
        ttk.Button(controls, text="刷新", style="Secondary.TButton",
                   command=self._refresh_ports).grid(row=0, column=2, padx=(0, 10))

        ttk.Label(controls, text="波特率", style="Surface.TLabel").grid(row=0, column=3,
                                                                         padx=(0, 5))
        self.baud_combo = ttk.Combobox(controls, width=9, state="readonly",
                                       values=(9600, 57600, 115200, 230400))
        self.baud_combo.set("115200")
        self.baud_combo.grid(row=0, column=4, padx=(0, 10))

        self.connect_button = ttk.Button(controls, text="连接", style="Accent.TButton",
                                         command=self.connect_serial)
        self.connect_button.grid(row=0, column=5, padx=(0, 5))
        self.disconnect_button = ttk.Button(controls, text="断开",
                                            style="Secondary.TButton",
                                            command=self.disconnect)
        self.disconnect_button.grid(row=0, column=6, padx=(0, 14))

        ttk.Separator(controls, orient="vertical").grid(row=0, column=7, sticky="ns",
                                                         padx=(0, 14))
        self.start_button = ttk.Button(controls, text="开启实时上传 LON",
                                       style="Accent.TButton", command=self.start_stream)
        self.start_button.grid(row=0, column=8, padx=(0, 5))
        self.stop_button = ttk.Button(controls, text="关闭实时上传 LOF",
                                      style="Secondary.TButton", command=self.stop_stream)
        self.stop_button.grid(row=0, column=9, padx=(0, 14))

        ttk.Label(controls, text="无设备演示", style="Surface.TLabel").grid(
            row=0, column=10, padx=(0, 5))
        self.scenario_combo = ttk.Combobox(
            controls, width=13, state="readonly",
            values=("正常测量", "超量程切换", "通信失败"))
        self.scenario_combo.current(0)
        self.scenario_combo.grid(row=0, column=11, padx=(0, 5))
        ttk.Button(controls, text="运行演示", style="Secondary.TButton",
                   command=self.start_demo).grid(row=0, column=12, padx=(0, 5))
        ttk.Button(controls, text="清空", style="Secondary.TButton",
                   command=self.clear_monitor).grid(row=0, column=13)

        # 设备控制：串口命令封装为按钮，仅连接真实设备时可用。
        device = ttk.Frame(self.monitor_tab, style="Surface.TFrame", padding=(14, 0, 14, 8))
        device.pack(fill="x")
        ttk.Label(device, text="设备控制", style="CardTitle.TLabel").pack(side="left",
                                                                          padx=(0, 10))
        self.device_buttons: List[ttk.Button] = []
        device_commands = (
            ("启动检测", "RUN", "Accent"),
            ("水泵开", "PON", None),
            ("水泵关", "POF", None),
            ("电机1 正转", "M1F", None),
            ("电机1 反转", "M1R", None),
            ("电机1 停转", "M1S", None),
            ("电机2 正转", "M2F", None),
            ("电机2 反转", "M2R", None),
            ("电机2 停转", "M2S", None),
            ("电机1 转①圈", "M1F551", None),
            ("电机2 转①圈", "M2F551", None),
        )
        for label, command, style in device_commands:
            button = ttk.Button(
                device, text=label,
                style=f"{style}.TButton" if style else "Secondary.TButton",
                command=lambda cmd=command, text=label: self._send_device_command(cmd, text))
            button.pack(side="left", padx=(0, 5), pady=(0, 4))
            self.device_buttons.append(button)
        self._update_device_buttons()

        cards = ttk.Frame(self.monitor_tab, style="Surface.TFrame", padding=(14, 2, 14, 10))
        cards.pack(fill="x")
        cards.columnconfigure(tuple(range(5)), weight=1, uniform="cards")
        self._card(cards, 0, "连接状态", self.connection_var)
        self._card(cards, 1, "当前量程", self.stage_var)
        self._card(cards, 2, "采样点", self.sample_var)
        self._card(cards, 3, "当前原始值 y", self.value_var)
        self._card(cards, 4, "判定结果", self.result_var)

        pane = ttk.Panedwindow(self.monitor_tab, orient="horizontal")
        pane.pack(fill="both", expand=True, padx=14, pady=(0, 12))

        chart_frame = tk.Frame(pane, bg=SURFACE, highlightbackground=GRID,
                               highlightthickness=1)
        side_frame = tk.Frame(pane, bg=SURFACE, width=350,
                              highlightbackground=GRID, highlightthickness=1)
        pane.add(chart_frame, weight=4)
        pane.add(side_frame, weight=2)

        self.figure = Figure(figsize=(8.5, 5.2), dpi=100, facecolor=SURFACE)
        self.ax = self.figure.add_subplot(111)
        self._style_axis(self.ax, "实时采样曲线", "采样序号（1 秒/点）", "传感器原始值 y")
        self.lines = {
            stage: self.ax.plot([], [], color=cfg["color"], marker=cfg["marker"],
                                markersize=6, linewidth=2, label=cfg["label"])[0]
            for stage, cfg in STAGE_STYLE.items()
        }
        self.lines["B"].set_visible(False)
        self.canvas = FigureCanvasTkAgg(self.figure, master=chart_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=(8, 0))
        toolbar = NavigationToolbar2Tk(self.canvas, chart_frame, pack_toolbar=False)
        toolbar.update()
        toolbar.pack(fill="x", padx=8, pady=(0, 6))
        self.canvas.mpl_connect("motion_notify_event", self._on_chart_hover)

        side_notebook = ttk.Notebook(side_frame)
        side_notebook.pack(fill="both", expand=True, padx=6, pady=6)
        log_tab = ttk.Frame(side_notebook, style="Surface.TFrame")
        table_tab = ttk.Frame(side_notebook, style="Surface.TFrame")
        side_notebook.add(log_tab, text="事件日志")
        side_notebook.add(table_tab, text="采样数据")

        self.log_text = tk.Text(log_tab, wrap="word", relief="flat", bg=SURFACE,
                                fg=TEXT, insertbackground=TEXT, padx=10, pady=10,
                                font=("Consolas", 9), state="disabled")
        log_scroll = ttk.Scrollbar(log_tab, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_scroll.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        log_scroll.pack(side="right", fill="y")
        self.log_text.tag_configure("time", foreground=MUTED)
        self.log_text.tag_configure("ok", foreground="#006300")
        self.log_text.tag_configure("warn", foreground=WARNING)
        self.log_text.tag_configure("error", foreground=CRITICAL)

        self.sample_tree = ttk.Treeview(table_tab, columns=("stage", "index", "value"),
                                        show="headings", selectmode="browse")
        for col, label, width in (("stage", "量程", 70),
                                  ("index", "序号", 80),
                                  ("value", "原始值", 100)):
            self.sample_tree.heading(col, text=label)
            self.sample_tree.column(col, width=width, anchor="center")
        tree_scroll = ttk.Scrollbar(table_tab, orient="vertical",
                                    command=self.sample_tree.yview)
        self.sample_tree.configure(yscrollcommand=tree_scroll.set)
        self.sample_tree.pack(side="left", fill="both", expand=True)
        tree_scroll.pack(side="right", fill="y")

    def _card(self, parent, column: int, title: str, variable: tk.StringVar) -> None:
        card = tk.Frame(parent, bg=SURFACE, highlightbackground=GRID,
                        highlightthickness=1, padx=13, pady=9)
        card.grid(row=0, column=column, sticky="nsew",
                  padx=(0 if column == 0 else 4, 0 if column == 4 else 4))
        ttk.Label(card, text=title, style="CardTitle.TLabel").pack(anchor="w")
        value_style = "ResultValue.TLabel" if title == "判定结果" else "CardValue.TLabel"
        value_label = ttk.Label(card, textvariable=variable, style=value_style)
        if title == "判定结果":
            value_label.configure(wraplength=240)
        value_label.pack(anchor="w", pady=(3, 0))

    # ---------- 标定分析页 ----------

    def _build_calibration_tab(self) -> None:
        controls = ttk.Frame(self.calib_tab, style="Surface.TFrame", padding=14)
        controls.pack(fill="x")
        ttk.Button(controls, text="选择标定 CSV", style="Accent.TButton",
                   command=self._choose_calibration_file).pack(side="left")
        ttk.Button(controls, text="运行拟合", style="Secondary.TButton",
                   command=self._run_calibration_file).pack(side="left", padx=6)
        ttk.Button(controls, text="加载演示数据", style="Secondary.TButton",
                   command=self._run_calibration_demo).pack(side="left")
        ttk.Label(controls, textvariable=self.fit_file_var,
                  style="Muted.TLabel").pack(side="left", padx=14)

        body = ttk.Panedwindow(self.calib_tab, orient="horizontal")
        body.pack(fill="both", expand=True, padx=14, pady=(0, 14))
        chart_frame = tk.Frame(body, bg=SURFACE, highlightbackground=GRID,
                               highlightthickness=1)
        result_frame = tk.Frame(body, bg=SURFACE, highlightbackground=GRID,
                                highlightthickness=1, width=410)
        body.add(chart_frame, weight=3)
        body.add(result_frame, weight=2)

        self.calib_figure = Figure(figsize=(7, 5), dpi=100, facecolor=SURFACE)
        self.calib_ax = self.calib_figure.add_subplot(111)
        self._style_axis(self.calib_ax, "分段线性标定", "浓度 x（ppm）", "传感器原始值 y")
        self.calib_canvas = FigureCanvasTkAgg(self.calib_figure, master=chart_frame)
        self.calib_canvas.draw()
        self.calib_canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=8)

        ttk.Label(result_frame, text="拟合结果", style="Surface.TLabel",
                  font=("Segoe UI Semibold", 12)).pack(anchor="w", padx=12, pady=(12, 8))
        self.fit_tree = ttk.Treeview(result_frame,
                                     columns=("stage", "k", "b", "r2", "n", "dk", "db"),
                                     show="headings", height=5)
        fit_cols = (("stage", "量程", 55), ("k", "K", 72), ("b", "B", 80),
                    ("r2", "R²", 65), ("n", "点数", 55),
                    ("dk", "ΔK", 70), ("db", "ΔB", 70))
        for col, label, width in fit_cols:
            self.fit_tree.heading(col, text=label)
            self.fit_tree.column(col, width=width, anchor="center")
        self.fit_tree.pack(fill="x", padx=10)

        note = ("拟合模型：y = K·x + B\n\n"
                "ΔK / ΔB 表示拟合结果与固件 app_config.h 当前标定系数的差值。\n\n"
                "CSV 格式：x_ppm,y_adc")
        ttk.Label(result_frame, text=note, style="Surface.TLabel",
                  justify="left", wraplength=370).pack(anchor="w", padx=14, pady=18)

    # ---------- 离线回放页 ----------

    def _build_replay_tab(self) -> None:
        controls = ttk.Frame(self.replay_tab, style="Surface.TFrame", padding=14)
        controls.pack(fill="x")
        ttk.Button(controls, text="选择数据集 CSV", style="Accent.TButton",
                   command=self._replay_choose_file).pack(side="left")
        ttk.Button(controls, text="批量选择目录", style="Secondary.TButton",
                   command=self._replay_choose_dir).pack(side="left", padx=6)
        ttk.Label(controls, textvariable=self.replay_summary_var,
                  style="Muted.TLabel").pack(side="left", padx=14)

        frame = tk.Frame(self.replay_tab, bg=SURFACE, highlightbackground=GRID,
                         highlightthickness=1)
        frame.pack(fill="both", expand=True, padx=14, pady=(0, 14))
        self.replay_tree = ttk.Treeview(
            frame, columns=("file", "stage", "category", "x", "y", "expect", "match"),
            show="headings")
        replay_cols = (("file", "文件", 310), ("stage", "量程", 70),
                       ("category", "判定", 100), ("x", "浓度 ppm", 110),
                       ("y", "原始值 y", 95), ("expect", "预期", 90),
                       ("match", "一致性", 100))
        for col, label, width in replay_cols:
            self.replay_tree.heading(col, text=label)
            self.replay_tree.column(col, width=width,
                                    anchor="w" if col == "file" else "center")
        scroll = ttk.Scrollbar(frame, orient="vertical", command=self.replay_tree.yview)
        self.replay_tree.configure(yscrollcommand=scroll.set)
        self.replay_tree.pack(side="left", fill="both", expand=True, padx=(8, 0), pady=8)
        scroll.pack(side="right", fill="y", padx=(0, 8), pady=8)
        self.replay_tree.tag_configure("match", foreground="#006300")
        self.replay_tree.tag_configure("mismatch", foreground=CRITICAL)

    # ---------- 串口 / 模拟数据 ----------

    def _refresh_ports(self) -> None:
        try:
            from serial.tools import list_ports
            ports = [p.device for p in list_ports.comports()]
        except Exception:
            ports = []
        self.port_combo["values"] = ports
        if ports:
            self.port_combo.current(0)
        elif not self.port_combo.get():
            self.port_combo.set("无可用串口")

    def connect_serial(self) -> None:
        if self.stream is not None:
            messagebox.showinfo("提示", "请先断开当前连接。")
            return
        port = self.port_combo.get()
        if not port or port == "无可用串口":
            messagebox.showwarning("串口", "未检测到可用串口。可先运行无设备演示。")
            return
        try:
            import serial
            baud = int(self.baud_combo.get())
            stream = serial.Serial(port, baud, timeout=0.2)
        except Exception as exc:
            messagebox.showerror("连接失败", str(exc))
            return

        self.real_serial = True
        self._start_reader(stream, f"{port} @ {self.baud_combo.get()}")
        self._update_device_buttons()
        self._append_log(f"已连接 {port}，发送 LON 后开始流式采集", "ok")

    def disconnect(self) -> None:
        if self.stream is None:
            return
        if self.real_serial:
            self._write_command("LOF")
        self._stop_reader()
        self._update_device_buttons()
        self.connection_var.set("● 未连接")
        self.status_var.set("连接已断开")
        self._append_log("连接已断开", "warn")

    def start_stream(self) -> None:
        if self.stream is None or not self.real_serial:
            messagebox.showwarning("开始采集", "请先连接真实串口，或使用无设备演示。")
            return
        self._write_command("LON")
        self.stream_on = True
        self.status_var.set("已发送 LON，等待倒计时 STREAM 数据")
        self._append_log("→ LON（开启每秒流式采样）", "ok")

    def stop_stream(self) -> None:
        if self.stream is None or not self.real_serial:
            return
        self._write_command("LOF")
        self.stream_on = False
        self.status_var.set("已发送 LOF，连接保持")
        self._append_log("→ LOF（停止流式采样）", "warn")

    def start_demo(self, scenario: Optional[str] = None) -> None:
        if self.stream is not None:
            self.disconnect()
        self.clear_monitor()
        mapping = {"正常测量": "normal", "超量程切换": "over", "通信失败": "comm_fail"}
        scenario = scenario or mapping[self.scenario_combo.get()]
        labels = {"normal": "正常测量", "over": "超量程切换", "comm_fail": "通信失败"}
        self.real_serial = False
        self._start_reader(SimulatedSerial(scenario=scenario, fast=True),
                           f"模拟 · {labels[scenario]}")
        self._update_device_buttons()
        self.status_var.set(f"正在运行{labels[scenario]}演示")
        self._append_log(f"无设备演示启动：{labels[scenario]}", "ok")

    def _start_reader(self, stream, label: str) -> None:
        self._open_session()
        self.stream = stream
        self.parser = FrameParser()
        self.stop_event.clear()
        self.connection_var.set(f"● {label}")
        self.status_var.set(f"已连接：{label}")
        self.reader_thread = threading.Thread(target=self._reader_loop,
                                              name="uart-reader", daemon=True)
        self.reader_thread.start()

    def _reader_loop(self) -> None:
        try:
            while not self.stop_event.is_set():
                chunk = self.stream.read(256)
                if chunk:
                    if self.raw_log is not None:
                        self.raw_log.write(chunk)
                        self.raw_log.flush()
                    for event in self.parser.feed(chunk):
                        self.event_queue.put(("event", event))
                elif getattr(self.stream, "eof", False):
                    self.event_queue.put(("done", "模拟演示完成"))
                    break
        except Exception as exc:
            if not self.stop_event.is_set():
                self.event_queue.put(("error", str(exc)))
        finally:
            self.event_queue.put(("reader_stopped", None))

    def _stop_reader(self) -> None:
        self.stop_event.set()
        stream = self.stream
        self.stream = None
        if stream is not None:
            try:
                stream.close()
            except Exception:
                pass
        self._close_raw_log()
        self.stream_on = False

    def _write_command(self, command: str) -> None:
        if self.stream is None:
            return
        try:
            self.stream.write((command + "\r\n").encode("ascii"))
        except Exception as exc:
            self._append_log(f"发送失败：{exc}", "error")

    def _send_device_command(self, command: str, label: str) -> None:
        """设备控制按钮统一入口：发送串口命令，设备返回文本显示在事件日志。"""
        if self.stream is None or not self.real_serial:
            messagebox.showwarning("设备控制", "请先连接真实设备（演示模式不支持控制）。")
            return
        self._append_log(f"→ {command}（{label}）", "ok")
        self._write_command(command)

    def _update_device_buttons(self) -> None:
        """设备控制按钮仅在已连接真实设备时可用。"""
        state = "normal" if (self.stream is not None and self.real_serial) else "disabled"
        for button in getattr(self, "device_buttons", []):
            button.configure(state=state)

    def _open_session(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.join(os.path.dirname(os.path.dirname(__file__)), "runs",
                            f"gui_{stamp}")
        self.data_dir = os.path.join(base, "data")
        log_dir = os.path.join(base, "logs")
        os.makedirs(self.data_dir, exist_ok=True)
        os.makedirs(log_dir, exist_ok=True)
        self.session_dir = base
        self.raw_log = open(os.path.join(log_dir, "uart_raw.log"), "wb")

    def _close_raw_log(self) -> None:
        if self.raw_log is not None:
            try:
                self.raw_log.close()
            except Exception:
                pass
            self.raw_log = None

    # ---------- 事件处理与实时图表 ----------

    def _poll_events(self) -> None:
        try:
            while True:
                kind, payload = self.event_queue.get_nowait()
                if kind == "event":
                    self._handle_event(payload)
                elif kind == "done":
                    self.status_var.set(str(payload))
                    self.connection_var.set("● 演示完成")
                    self._append_log(str(payload), "ok")
                elif kind == "error":
                    self.status_var.set("读取错误")
                    self._append_log(f"读取错误：{payload}", "error")
                    messagebox.showerror("串口读取错误", str(payload))
                elif kind == "reader_stopped":
                    if not self.real_serial:
                        self._close_raw_log()
        except queue.Empty:
            pass
        self.root.after(50, self._poll_events)

    def _handle_event(self, event: Event) -> None:
        if isinstance(event, Sample):
            stage = "A" if event.stage == 1 else "B"
            self.samples[stage].append((event.index, event.value))
            self.current_stage = stage
            self.last_value = event.value
            self.stage_var.set(f"量程 {stage}")
            self.sample_var.set(f"{event.index + 1} / {120 if stage == 'A' else 20}")
            self.value_var.set(str(event.value))
            self._update_line(stage, self.samples[stage])
            self._insert_sample(stage, event.index, event.value)
        elif isinstance(event, Dataset):
            stage = event.stage
            self.datasets[stage] = event.values
            if not self.samples[stage]:
                self.samples[stage] = list(enumerate(event.values))
                for index, value in self.samples[stage]:
                    self._insert_sample(stage, index, value)
            self._update_line(stage, list(enumerate(event.values)))
            self._mark_dataset(stage, event.values)
            path = self._save_dataset(stage, event.values)
            self._append_log(f"完整数据集：量程 {stage}，{len(event.values)} 点", "ok")
            self.status_var.set(f"数据已保存：{os.path.basename(path)}")
        elif isinstance(event, Result):
            self.last_result = f"{event.stage}: {event.text}"
            self.result_var.set(self.last_result)
            self._append_log(f"Result {event.stage}: {event.text}", "ok")
            self.status_var.set("检测结果已接收")
            self.ax.set_title(f"实时采样曲线 · Result {event.stage}: {event.text}",
                              loc="left", fontsize=12, color=TEXT, pad=12)
            self.canvas.draw_idle()
        elif isinstance(event, Text):
            tag = "error" if event.line.startswith("ERR") else (
                "warn" if event.line.startswith("Busy") else "ok")
            self._append_log(event.line, tag)

    def _update_line(self, stage: str, points: Iterable[Tuple[int, int]]) -> None:
        points = list(points)
        if not points:
            return
        xs, ys = zip(*points)
        line = self.lines[stage]
        line.set_data(xs, ys)
        line.set_visible(True)
        self._update_endpoint_label(stage, xs[-1], ys[-1])
        self._rescale_monitor()
        visible = [line for line in self.lines.values() if line.get_visible() and len(line.get_xdata())]
        legend = self.ax.get_legend()
        if len(visible) >= 2:
            self.ax.legend(handles=visible, loc="upper left", frameon=False)
        elif legend is not None:
            legend.remove()
        self.canvas.draw_idle()

    def _update_endpoint_label(self, stage: str, x: int, y: int) -> None:
        if self.endpoint_label is not None:
            try:
                self.endpoint_label.remove()
            except Exception:
                pass
        self.endpoint_label = self.ax.annotate(
            f"{stage} · {y}", xy=(x, y), xytext=(8, 8), textcoords="offset points",
            fontsize=9, color=TEXT, bbox=dict(boxstyle="round,pad=0.25", fc=SURFACE,
                                               ec=GRID, lw=1))

    def _mark_dataset(self, stage: str, values: List[int]) -> None:
        for artist in self.final_markers + self.plateau_markers:
            try:
                artist.remove()
            except Exception:
                pass
        self.final_markers.clear()
        self.plateau_markers.clear()

        for known_stage, known_values in self.datasets.items():
            final = FINAL_INDEX[known_stage]
            if final < len(known_values):
                marker, = self.ax.plot([final], [known_values[final]], marker="v",
                                       color=CRITICAL, markersize=9, linestyle="none",
                                       label=f"量程 {known_stage} 判定点")
                self.final_markers.append(marker)
            for start, end in find_plateaus(known_values, min_len=5):
                xs = list(range(start, end))
                ys = [known_values[i] for i in xs]
                marker, = self.ax.plot(xs, ys, "o", mfc="none", mec=CRITICAL,
                                       mew=1.5, markersize=8, linestyle="none")
                self.plateau_markers.append(marker)
                self._append_log(
                    f"量程 {known_stage}：发现连续等值区间 {start}–{end - 1}", "warn")
        self.canvas.draw_idle()

    def _rescale_monitor(self) -> None:
        all_points = [p for stage in ("A", "B") for p in self.samples[stage]]
        for stage, values in self.datasets.items():
            all_points.extend(enumerate(values))
        if not all_points:
            return
        xs = [p[0] for p in all_points]
        ys = [p[1] for p in all_points]
        self.ax.set_xlim(-2, max(25, max(xs) + 8))
        lo, hi = min(ys), max(ys)
        margin = max(5, (hi - lo) * 0.15)
        self.ax.set_ylim(lo - margin, hi + margin)

    def _insert_sample(self, stage: str, index: int, value: int) -> None:
        self.sample_tree.insert("", "end", values=(stage, index, value))
        children = self.sample_tree.get_children()
        if children:
            self.sample_tree.see(children[-1])

    def _save_dataset(self, stage: str, values: List[int]) -> str:
        if self.data_dir is None:
            self._open_session()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.data_dir, f"dataset_{stage}_{stamp}.csv")
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            handle.write(f"# stage={stage} points={len(values)}\n")
            writer.writerow(("index", "value"))
            writer.writerows(enumerate(values))
        return path

    def _on_chart_hover(self, event) -> None:
        if event.inaxes != self.ax or event.x is None or event.y is None:
            self._hide_hover()
            return
        nearest = None
        for stage, line in self.lines.items():
            if not line.get_visible():
                continue
            for x, y in zip(line.get_xdata(), line.get_ydata()):
                px, py = self.ax.transData.transform((x, y))
                distance = math.hypot(px - event.x, py - event.y)
                if distance <= 12 and (nearest is None or distance < nearest[0]):
                    nearest = (distance, stage, x, y)
        if nearest is None:
            self._hide_hover()
            return
        _, stage, x, y = nearest
        if self.hover_annotation is None:
            self.hover_annotation = self.ax.annotate(
                "", xy=(x, y), xytext=(12, 12), textcoords="offset points",
                fontsize=9, color=TEXT,
                bbox=dict(boxstyle="round,pad=0.35", fc=SURFACE, ec=AXIS, lw=1))
        self.hover_annotation.xy = (x, y)
        self.hover_annotation.set_text(f"量程 {stage}\n序号 {int(x)}\ny = {int(y)}")
        self.hover_annotation.set_visible(True)
        self.canvas.draw_idle()

    def _hide_hover(self) -> None:
        if self.hover_annotation is not None and self.hover_annotation.get_visible():
            self.hover_annotation.set_visible(False)
            self.canvas.draw_idle()

    def clear_monitor(self) -> None:
        self.samples = {"A": [], "B": []}
        self.datasets = {}
        self.current_stage = "—"
        self.last_value = None
        self.last_result = "等待检测"
        self.stage_var.set("—")
        self.sample_var.set("0")
        self.value_var.set("—")
        self.result_var.set("等待检测")
        for line in self.lines.values():
            line.set_data([], [])
            line.set_visible(line is self.lines["A"])
        for artist in self.final_markers + self.plateau_markers:
            try:
                artist.remove()
            except Exception:
                pass
        self.final_markers.clear()
        self.plateau_markers.clear()
        if self.endpoint_label is not None:
            try:
                self.endpoint_label.remove()
            except Exception:
                pass
            self.endpoint_label = None
        self.ax.set_xlim(0, 125)
        self.ax.set_ylim(0, 100)
        self.ax.set_title("实时采样曲线", loc="left", fontsize=12, color=TEXT, pad=12)
        for item in self.sample_tree.get_children():
            self.sample_tree.delete(item)
        self.canvas.draw_idle()
        self._append_log("曲线与采样表已清空", "warn")

    # ---------- 标定 ----------

    def _choose_calibration_file(self) -> None:
        path = filedialog.askopenfilename(title="选择标定数据",
                                          filetypes=(("CSV", "*.csv"), ("所有文件", "*.*")))
        if path:
            self.calibration_path = path
            self.fit_file_var.set(path)

    def _run_calibration_file(self) -> None:
        path = getattr(self, "calibration_path", None)
        if not path:
            self._choose_calibration_file()
            path = getattr(self, "calibration_path", None)
        if not path:
            return
        try:
            points = load_points(path)
            self._show_calibration(points, os.path.basename(path))
        except Exception as exc:
            messagebox.showerror("标定失败", str(exc))

    def _run_calibration_demo(self) -> None:
        self.fit_file_var.set("演示数据 · 根据固件系数生成并加入随机噪声")
        self._show_calibration(demo_points(), "演示数据")

    def _show_calibration(self, points: List[Tuple[float, float]], title: str) -> None:
        self.calib_ax.clear()
        self._style_axis(self.calib_ax, f"分段线性标定 · {title}",
                         "浓度 x（ppm）", "传感器原始值 y")
        for item in self.fit_tree.get_children():
            self.fit_tree.delete(item)

        for stage, cfg in STAGE_STYLE.items():
            fit = linear_fit(stage, points)
            if fit is None:
                continue
            fw = FIRMWARE_CAL[stage]
            selected = [(x, y) for x, y in points if fw["min"] <= x <= fw["max"]]
            xs = [p[0] for p in selected]
            ys = [p[1] for p in selected]
            self.calib_ax.scatter(xs, ys, s=44, color=cfg["color"], marker=cfg["marker"],
                                  label=f"量程 {stage} 标定点")
            x_line = [min(xs), max(xs)]
            y_line = [fit.K * x + fit.B for x in x_line]
            self.calib_ax.plot(x_line, y_line, color=cfg["color"], linewidth=2,
                               label=f"{stage}: y={fit.K:.3f}x{fit.B:+.3f}")
            d_k, d_b = fit.firmware_delta()
            self.fit_tree.insert("", "end", values=(
                stage, f"{fit.K:.4f}", f"{fit.B:.4f}", f"{fit.r_squared:.4f}",
                fit.points_used, f"{d_k:+.4f}", f"{d_b:+.4f}"))
        self.calib_ax.legend(frameon=False, loc="upper left")
        self.calib_canvas.draw_idle()

    # ---------- 离线回放 ----------

    def _replay_choose_file(self) -> None:
        path = filedialog.askopenfilename(title="选择采样数据集",
                                          filetypes=(("CSV", "*.csv"), ("所有文件", "*.*")))
        if path:
            try:
                self._show_replay_results([replay_file(path)])
            except Exception as exc:
                messagebox.showerror("回放失败", str(exc))

    def _replay_choose_dir(self) -> None:
        path = filedialog.askdirectory(title="选择数据集目录")
        if path:
            try:
                self._show_replay_results(replay_dir(path))
            except Exception as exc:
                messagebox.showerror("回放失败", str(exc))

    def _show_replay_results(self, results: List[ReplayResult]) -> None:
        for item in self.replay_tree.get_children():
            self.replay_tree.delete(item)
        checked = 0
        matched = 0
        for result in results:
            if result.expect is None:
                expectation, match_text, tag = "—", "未设置", ""
            else:
                checked += 1
                expectation = result.expect
                if result.matched:
                    matched += 1
                    match_text, tag = "✓ 一致", "match"
                else:
                    match_text, tag = "✕ 不一致", "mismatch"
            self.replay_tree.insert("", "end", values=(
                result.path, result.stage, result.category,
                f"{result.x:.2f}", result.y, expectation, match_text), tags=(tag,))
        summary = f"已回放 {len(results)} 个数据集"
        if checked:
            summary += f" · 预期一致 {matched}/{checked}"
        self.replay_summary_var.set(summary)

    # ---------- 公共辅助 ----------

    def _style_axis(self, axis, title: str, xlabel: str, ylabel: str) -> None:
        axis.set_facecolor(SURFACE)
        axis.set_title(title, loc="left", fontsize=12, color=TEXT, pad=12)
        axis.set_xlabel(xlabel, color=SECONDARY)
        axis.set_ylabel(ylabel, color=SECONDARY)
        axis.grid(True, color=GRID, linewidth=0.8, linestyle="-")
        axis.tick_params(colors=MUTED, labelsize=9)
        for spine in axis.spines.values():
            spine.set_color(AXIS)
            spine.set_linewidth(0.8)
        axis.set_axisbelow(True)

    def _append_log(self, text: str, tag: str = "") -> None:
        now = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{now}] ", "time")
        self.log_text.insert("end", text + "\n", tag)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _save_chart(self) -> None:
        path = filedialog.asksaveasfilename(defaultextension=".png",
                                            filetypes=(("PNG", "*.png"),))
        if path:
            self.figure.savefig(path, dpi=150, bbox_inches="tight")
            self.status_var.set(f"曲线图已保存：{path}")

    def _open_session_dir(self) -> None:
        path = self.session_dir or os.path.join(os.path.dirname(os.path.dirname(__file__)),
                                                "runs")
        os.makedirs(path, exist_ok=True)
        try:
            os.startfile(path)
        except AttributeError:
            subprocess.Popen(["explorer", path])

    def _show_help(self) -> None:
        messagebox.showinfo(
            "使用说明",
            "真实设备：选择串口 → 连接 → 开启实时上传 LON。\n"
            "LON 只控制每秒 STREAM 数据上传，不会启动检测流程。\n"
            "检测流程可用“设备控制 → 启动检测”按钮启动（串口 RUN 命令，等效按设备 K2）。\n"
            "LOF 只关闭实时上传，不会停止电机、水泵或状态机。\n\n"
            "无设备：选择场景并点击“运行演示”。\n\n"
            "采集数据自动保存到 tools/gas_host/runs/。")

    def _show_about(self) -> None:
        messagebox.showinfo(
            "关于",
            "STM32 气体检测仪上位机 1.0\n\n"
            "Python · Tkinter · pyserial · NumPy · Matplotlib\n"
            "与 STM32F103 固件的 UART 文本协议配套。")

    def close(self) -> None:
        self._stop_reader()
        self.root.destroy()


def run_gui(demo: Optional[str] = None,
            screenshot: Optional[str] = None,
            screenshot_delay_ms: int = 2800) -> None:
    """启动 GUI；demo/screenshot 参数用于自动化展示和布局检查。"""
    root = tk.Tk()
    app = GasHostGUI(root)
    if demo:
        root.after(250, lambda: app.start_demo(demo))
    if screenshot:
        def capture_window() -> None:
            try:
                from PIL import ImageGrab
                root.update_idletasks()
                x, y = root.winfo_rootx(), root.winfo_rooty()
                w, h = root.winfo_width(), root.winfo_height()
                os.makedirs(os.path.dirname(os.path.abspath(screenshot)), exist_ok=True)
                ImageGrab.grab(bbox=(x, y, x + w, y + h)).save(screenshot)
            finally:
                app.close()
        root.after(screenshot_delay_ms, capture_window)
    root.mainloop()
