"""BIMG录制、核心0日志配准和异常图像回放界面。"""

from __future__ import annotations

import math
import queue
import threading
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from PIL import Image, ImageDraw, ImageFont, ImageTk

from .alignment import (
    AlignmentResult,
    CoreLogRow,
    align_bimg_to_core_log,
    camera_beacons,
    camera_lamp,
    decode_cross_check_status,
    decode_lamp_shape,
    decode_track_geometry,
    export_alignment_csv,
    read_core_log,
)
from .bimg import BimgIndexEntry, read_bimg_frame, scan_bimg_file
from .projection import from_center
from .recorder import BimgRecorder


IMAGE_SCALE = 4
EDGE_GATE_PX = 5.0
LAMP_SHORT_LENGTH_PX = 12.0
LAMP_BEACON_OVERLAP_PX = 8.0
TRACK_STATE_NAMES = ("SEARCH", "ACQUIRE", "TRACKED", "COAST", "LOST")
CAMERA_NAMES_CN = ("前摄", "下摄", "后摄")
FILTERS = (
    "全部帧",
    "冲突帧",
    "灯与信标重合",
    "短车灯候选",
    "图像边缘车灯",
    "轨迹非TRACKED",
    "未匹配或CRC错误",
)


class ImageLogReviewApp:
    """提供BIMG录制和CSV同步回放的桌面上位机。"""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("三摄图像与核心0日志回放")
        self.root.geometry("1240x820")
        self.root.minsize(1040, 700)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._event_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self._recorder: BimgRecorder | None = None
        self._recorder_thread: threading.Thread | None = None
        self._bimg_path: Path | None = None
        self._csv_path: Path | None = None
        self._entries: list[BimgIndexEntry] = []
        self._rows: list[CoreLogRow] = []
        self._rows_by_index: dict[int, CoreLogRow] = {}
        self._alignment: AlignmentResult | None = None
        self._matches_by_frame = {}
        self._visible_frames: list[int] = []
        self._current_visible_index = 0
        self._photo: ImageTk.PhotoImage | None = None
        self._current_render: Image.Image | None = None
        self._playing = False

        self._configure_style()
        self._build_ui()
        self.root.after(100, self._poll_recorder_events)

    def _configure_style(self) -> None:
        """配置紧凑、适合调试工具的系统风格。"""

        style = ttk.Style(self.root)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        style.configure("TButton", padding=(9, 5))
        style.configure("TNotebook.Tab", padding=(14, 7))
        style.configure("Status.TLabel", foreground="#315a42")
        style.configure("Warn.TLabel", foreground="#9a4b1c")

    def _build_ui(self) -> None:
        """创建录制和回放两个主工作区。"""

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        capture_tab = ttk.Frame(notebook, padding=12)
        review_tab = ttk.Frame(notebook, padding=8)
        notebook.add(capture_tab, text="BIMG录制")
        notebook.add(review_tab, text="图像与日志回放")
        self._build_capture_tab(capture_tab)
        self._build_review_tab(review_tab)

    def _build_capture_tab(self, parent: ttk.Frame) -> None:
        """创建前摄TCP监听与原始流保存控件。"""

        parent.columnconfigure(1, weight=1)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.capture_host_var = tk.StringVar(value="0.0.0.0")
        self.capture_port_var = tk.StringVar(value="8086")
        self.capture_path_var = tk.StringVar(
            value=str(Path.cwd() / f"front_{timestamp}.bimg")
        )
        self.capture_state_var = tk.StringVar(value="未启动")
        self.capture_stats_var = tk.StringVar(value="帧 0 | CRC错误 0 | 0.00 FPS")

        ttk.Label(parent, text="监听地址").grid(row=0, column=0, sticky="w", pady=6)
        ttk.Entry(parent, textvariable=self.capture_host_var, width=18).grid(
            row=0, column=1, sticky="w", pady=6
        )
        ttk.Label(parent, text="端口").grid(row=1, column=0, sticky="w", pady=6)
        ttk.Entry(parent, textvariable=self.capture_port_var, width=10).grid(
            row=1, column=1, sticky="w", pady=6
        )
        ttk.Label(parent, text="原始BIMG文件").grid(row=2, column=0, sticky="w", pady=6)
        path_row = ttk.Frame(parent)
        path_row.grid(row=2, column=1, sticky="ew", pady=6)
        path_row.columnconfigure(0, weight=1)
        ttk.Entry(path_row, textvariable=self.capture_path_var).grid(
            row=0, column=0, sticky="ew"
        )
        ttk.Button(path_row, text="选择...", command=self._choose_capture_path).grid(
            row=0, column=1, padx=(8, 0)
        )

        command_row = ttk.Frame(parent)
        command_row.grid(row=3, column=1, sticky="w", pady=(12, 8))
        self.capture_start_button = ttk.Button(
            command_row, text="开始录制", command=self._start_capture
        )
        self.capture_start_button.pack(side=tk.LEFT)
        self.capture_stop_button = ttk.Button(
            command_row, text="停止", command=self._stop_capture, state=tk.DISABLED
        )
        self.capture_stop_button.pack(side=tk.LEFT, padx=(8, 0))

        ttk.Separator(parent).grid(row=4, column=0, columnspan=2, sticky="ew", pady=12)
        ttk.Label(parent, textvariable=self.capture_state_var, style="Status.TLabel").grid(
            row=5, column=0, columnspan=2, sticky="w", pady=4
        )
        ttk.Label(parent, textvariable=self.capture_stats_var).grid(
            row=6, column=0, columnspan=2, sticky="w", pady=4
        )
        notes = (
            "前摄物理板连接端口8086，后摄物理板连接端口8087。\n"
            "电脑网卡需配置为192.168.110.30，图传模式使用RAW(0)。\n"
            "工具原样保存TCP字节为.bimg；不要用MP4替代原始文件。"
        )
        ttk.Label(parent, text=notes, justify=tk.LEFT).grid(
            row=7, column=0, columnspan=2, sticky="w", pady=(18, 0)
        )

    def _build_review_tab(self, parent: ttk.Frame) -> None:
        """创建文件加载、配准、筛选和逐帧回放控件。"""

        parent.rowconfigure(1, weight=1)
        parent.columnconfigure(0, weight=1)
        toolbar = ttk.Frame(parent)
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 8))

        ttk.Button(toolbar, text="打开BIMG", command=self._open_bimg).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="打开核心0 CSV", command=self._open_csv).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        ttk.Label(toolbar, text="周期偏移").pack(side=tk.LEFT, padx=(18, 4))
        self.manual_offset_var = tk.StringVar()
        ttk.Entry(toolbar, textvariable=self.manual_offset_var, width=10).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="自动配准", command=self._align).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        ttk.Button(toolbar, text="导出配准表", command=self._export_alignment).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        ttk.Button(toolbar, text="导出当前图", command=self._export_current_image).pack(
            side=tk.LEFT, padx=(6, 0)
        )

        paned = ttk.Panedwindow(parent, orient=tk.HORIZONTAL)
        paned.grid(row=1, column=0, sticky="nsew")
        image_frame = ttk.Frame(paned)
        detail_frame = ttk.Frame(paned)
        paned.add(image_frame, weight=3)
        paned.add(detail_frame, weight=2)
        image_frame.rowconfigure(0, weight=1)
        image_frame.columnconfigure(0, weight=1)
        detail_frame.rowconfigure(1, weight=1)
        detail_frame.columnconfigure(0, weight=1)

        self.image_canvas = tk.Canvas(
            image_frame,
            width=188 * IMAGE_SCALE,
            height=120 * IMAGE_SCALE,
            background="#161a1d",
            highlightthickness=1,
            highlightbackground="#8c949a",
        )
        self.image_canvas.grid(row=0, column=0, sticky="nsew")
        self.image_canvas.create_text(
            376, 240, text="打开BIMG和核心0 CSV后开始配准", fill="#d5dadd"
        )

        self.alignment_summary_var = tk.StringVar(value="尚未加载数据")
        ttk.Label(
            detail_frame,
            textvariable=self.alignment_summary_var,
            justify=tk.LEFT,
            style="Status.TLabel",
        ).grid(row=0, column=0, sticky="ew", pady=(0, 6))
        self.detail_text = tk.Text(
            detail_frame,
            width=45,
            wrap=tk.WORD,
            font=("Consolas", 10),
            state=tk.DISABLED,
            background="#f6f7f8",
            relief=tk.SOLID,
            borderwidth=1,
        )
        self.detail_text.grid(row=1, column=0, sticky="nsew")

        controls = ttk.Frame(parent)
        controls.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        controls.columnconfigure(4, weight=1)
        ttk.Button(controls, text="上一帧", command=lambda: self._step_frame(-1)).grid(
            row=0, column=0
        )
        self.play_button = ttk.Button(controls, text="播放", command=self._toggle_play)
        self.play_button.grid(row=0, column=1, padx=6)
        ttk.Button(controls, text="下一帧", command=lambda: self._step_frame(1)).grid(
            row=0, column=2
        )
        ttk.Label(controls, text="筛选").grid(row=0, column=3, padx=(14, 4))
        self.filter_var = tk.StringVar(value=FILTERS[0])
        filter_box = ttk.Combobox(
            controls,
            textvariable=self.filter_var,
            values=FILTERS,
            width=18,
            state="readonly",
        )
        filter_box.grid(row=0, column=4, sticky="w")
        filter_box.bind("<<ComboboxSelected>>", lambda _event: self._apply_filter())
        self.position_var = tk.StringVar(value="0 / 0")
        ttk.Label(controls, textvariable=self.position_var).grid(
            row=0, column=5, padx=(12, 0)
        )

        self.frame_scale = ttk.Scale(
            parent, from_=0, to=0, orient=tk.HORIZONTAL, command=self._seek_frame
        )
        self.frame_scale.grid(row=3, column=0, sticky="ew", pady=(6, 0))

    def _choose_capture_path(self) -> None:
        """选择不会被自动覆盖的BIMG输出路径。"""

        path = filedialog.asksaveasfilename(
            title="保存原始BIMG",
            defaultextension=".bimg",
            filetypes=(("BIMG原始流", "*.bimg"), ("所有文件", "*.*")),
        )
        if path:
            self.capture_path_var.set(path)

    def _start_capture(self) -> None:
        """在后台线程启动TCP监听和原始字节录制。"""

        if self._recorder_thread is not None and self._recorder_thread.is_alive():
            return
        try:
            port = int(self.capture_port_var.get())
        except ValueError:
            messagebox.showerror("端口错误", "端口必须是1至65535之间的整数。")
            return
        output_path = Path(self.capture_path_var.get()).expanduser()
        if output_path.exists() or output_path.with_suffix(output_path.suffix + ".jsonl").exists():
            messagebox.showerror("文件已存在", "为保护已有数据，录制器不会覆盖同名文件。")
            return

        self._recorder = BimgRecorder(
            output_path=output_path,
            host=self.capture_host_var.get().strip() or "0.0.0.0",
            port=port,
            event_callback=lambda event, payload: self._event_queue.put((event, payload)),
        )

        def worker() -> None:
            try:
                assert self._recorder is not None
                self._recorder.run()
            except Exception as error:  # Tk线程通过消息队列统一报告错误。
                self._event_queue.put(("error", str(error)))

        self._recorder_thread = threading.Thread(target=worker, daemon=True)
        self._recorder_thread.start()
        self.capture_start_button.configure(state=tk.DISABLED)
        self.capture_stop_button.configure(state=tk.NORMAL)
        self.capture_state_var.set("正在启动监听...")

    def _stop_capture(self) -> None:
        """停止当前BIMG录制并保留已写入的原始文件。"""

        if self._recorder is not None:
            self._recorder.stop()
        self.capture_stop_button.configure(state=tk.DISABLED)

    def _poll_recorder_events(self) -> None:
        """在Tk主线程消费后台录制状态。"""

        try:
            while True:
                event, payload = self._event_queue.get_nowait()
                if event == "listening":
                    self.capture_state_var.set(
                        f"正在监听 {payload['host']}:{payload['port']}，等待图像板连接"
                    )
                elif event == "connected":
                    self.capture_state_var.set(f"图像板已连接：{payload['address']}")
                elif event == "frame":
                    assert self._recorder is not None
                    stats = self._recorder.stats
                    self.capture_stats_var.set(
                        f"帧 {stats.decoded_frames} | CRC错误 {stats.crc_error_count} | "
                        f"{stats.frame_rate_hz:.2f} FPS | 来源帧 {stats.last_frame_sequence}"
                    )
                elif event == "disconnected":
                    self.capture_state_var.set("连接已断开，继续等待重连")
                elif event == "stopped":
                    self.capture_state_var.set("录制已停止，原始BIMG和JSONL索引已保存")
                    self.capture_start_button.configure(state=tk.NORMAL)
                    self.capture_stop_button.configure(state=tk.DISABLED)
                elif event == "error":
                    self.capture_state_var.set("录制失败")
                    self.capture_start_button.configure(state=tk.NORMAL)
                    self.capture_stop_button.configure(state=tk.DISABLED)
                    messagebox.showerror("录制失败", str(payload))
        except queue.Empty:
            pass
        self.root.after(100, self._poll_recorder_events)

    def _open_bimg(self) -> None:
        """选择并扫描原始BIMG文件。"""

        path = filedialog.askopenfilename(
            title="打开原始BIMG",
            filetypes=(("BIMG原始流", "*.bimg"), ("所有文件", "*.*")),
        )
        if not path:
            return
        try:
            entries = scan_bimg_file(path)
        except Exception as error:
            messagebox.showerror("BIMG解析失败", str(error))
            return
        if not entries:
            messagebox.showerror("BIMG为空", "文件中没有找到结构完整的BIMG v3帧。")
            return
        non_raw_count = sum(entry.stream_mode != 0 for entry in entries)
        if non_raw_count:
            messagebox.showwarning(
                "包含非RAW图像",
                f"文件中有{non_raw_count}帧不是RAW(0)模式。非RAW像素不会参与自动配准或算法分析。",
            )
        self._bimg_path = Path(path)
        self._entries = entries
        self._clear_alignment_view()
        self.alignment_summary_var.set(
            f"BIMG：{self._bimg_path.name} | {len(entries)}帧 | 等待核心0 CSV"
        )
        self._try_auto_align()

    def _open_csv(self) -> None:
        """选择并读取核心0 I0至I35日志。"""

        path = filedialog.askopenfilename(
            title="打开核心0 CSV",
            filetypes=(("CSV日志", "*.csv"), ("所有文件", "*.*")),
        )
        if not path:
            return
        try:
            rows = read_core_log(path)
        except Exception as error:
            messagebox.showerror("CSV解析失败", str(error))
            return
        self._csv_path = Path(path)
        self._rows = rows
        self._rows_by_index = {row.row_index: row for row in rows}
        self._clear_alignment_view()
        self.alignment_summary_var.set(
            f"CSV：{self._csv_path.name} | {len(rows)}行 | 等待BIMG"
        )
        self._try_auto_align()

    def _clear_alignment_view(self) -> None:
        """清除上一组文件的匹配、筛选和图像，避免配准失败后显示旧叠加。"""

        self._alignment = None
        self._matches_by_frame = {}
        self._visible_frames = []
        self._current_visible_index = 0
        self._current_render = None
        self._photo = None
        self.position_var.set("0 / 0")
        self.frame_scale.configure(to=0)
        self.frame_scale.set(0)
        self.image_canvas.delete("all")
        self.image_canvas.create_text(
            max(1, self.image_canvas.winfo_width()) // 2,
            max(1, self.image_canvas.winfo_height()) // 2,
            text="等待BIMG与核心0 CSV配准",
            fill="#d5dadd",
        )

    def _try_auto_align(self) -> None:
        """在两类文件均已加载时自动执行一次配准。"""

        if self._entries and self._rows:
            self._align()

    def _align(self) -> None:
        """执行自动或指定周期偏移的逐帧配准。"""

        if not self._entries or not self._rows:
            messagebox.showinfo("缺少文件", "请先打开BIMG和核心0 CSV。")
            return
        source_cameras = [
            entry.source_camera
            for entry in self._entries
            if (
                entry.frame_valid
                and entry.crc_ok
                and entry.stream_mode == 0
                and entry.source_camera in range(3)
            )
        ]
        if not source_cameras:
            messagebox.showerror("帧元数据无效", "BIMG中没有有效的摄像头来源编号。")
            return
        camera_index = max(set(source_cameras), key=source_cameras.count)
        manual_text = self.manual_offset_var.get().strip()
        try:
            manual_offset = int(manual_text) if manual_text else None
            alignment = align_bimg_to_core_log(
                self._entries,
                self._rows,
                camera_index=camera_index,
                manual_sequence_offset=manual_offset,
            )
        except Exception as error:
            messagebox.showerror("配准失败", str(error))
            return

        best = next(
            candidate
            for candidate in alignment.candidates
            if candidate.sequence_offset == alignment.sequence_offset
        )
        self.alignment_summary_var.set(
            f"{CAMERA_NAMES_CN[camera_index]} | 配准{best.matched_frames}/{len(self._entries)}帧 | "
            f"偏移{alignment.sequence_offset:+d} | 置信度{alignment.confidence.upper()} | "
            f"姿态误差{best.pose_error:.2f}"
        )
        if alignment.confidence == "low" and manual_offset is None:
            self._alignment = None
            self._matches_by_frame = {}
            self._visible_frames = [entry.frame_index for entry in self._entries]
            self._current_visible_index = 0
            self.frame_scale.configure(to=max(0, len(self._visible_frames) - 1))
            self.frame_scale.set(0)
            self._show_current_frame()
            candidates = "\n".join(
                f"偏移{candidate.sequence_offset:+d}：匹配{candidate.matched_frames}帧，姿态误差{candidate.pose_error:.2f}"
                for candidate in alignment.candidates[:3]
            )
            messagebox.showwarning(
                "配准置信度低",
                "自动配准存在多个近似周期，本次没有应用日志叠加。\n\n"
                f"{candidates}\n\n"
                "请在“周期偏移”中填写128的整数倍后重新配准。",
            )
            return
        self._alignment = alignment
        self._matches_by_frame = {
            match.bimg_frame_index: match for match in alignment.matches
        }
        self._apply_filter()

    def _apply_filter(self) -> None:
        """按当前异常类型建立回放帧列表。"""

        selected = self.filter_var.get()
        self._visible_frames = [
            entry.frame_index
            for entry in self._entries
            if self._frame_matches_filter(entry, selected)
        ]
        self._current_visible_index = 0
        maximum = max(0, len(self._visible_frames) - 1)
        self.frame_scale.configure(to=maximum)
        self.frame_scale.set(0)
        self._show_current_frame()

    def _frame_matches_filter(self, entry: BimgIndexEntry, selected: str) -> bool:
        """判断一帧是否满足当前回放筛选条件。"""

        if selected == "全部帧":
            return True
        row = self._matched_row(entry.frame_index)
        if selected == "未匹配或CRC错误":
            return row is None or not entry.crc_ok
        if row is None or self._alignment is None:
            return False
        camera_index = self._alignment.camera_index
        status = decode_cross_check_status(row.value("I32"))
        lamp = camera_lamp(row, camera_index)
        shape = decode_lamp_shape(row.value(f"I{10 + camera_index}"))

        if selected == "冲突帧":
            return bool(status.conflict_mask & (1 << camera_index))
        if selected == "短车灯候选":
            return lamp is not None and shape.valid and shape.length_px < LAMP_SHORT_LENGTH_PX
        if selected == "图像边缘车灯":
            return lamp is not None and min(94.0 - abs(lamp[0]), 60.0 - abs(lamp[1])) <= EDGE_GATE_PX
        if selected == "轨迹非TRACKED":
            return status.state != 2
        if selected == "灯与信标重合" and lamp is not None:
            return any(
                math.hypot(lamp[0] - beacon_x, lamp[1] - beacon_y)
                <= LAMP_BEACON_OVERLAP_PX
                for _slot, beacon_x, beacon_y, _area in camera_beacons(row, camera_index)
            )
        return False

    def _matched_row(self, frame_index: int) -> CoreLogRow | None:
        """读取指定BIMG帧对应的核心0日志行。"""

        match = self._matches_by_frame.get(frame_index)
        if match is None or match.csv_row_index is None:
            return None
        return self._rows_by_index.get(match.csv_row_index)

    def _seek_frame(self, value: str) -> None:
        """响应滑条位置变化并显示对应帧。"""

        if not self._visible_frames:
            return
        index = max(0, min(int(float(value)), len(self._visible_frames) - 1))
        if index != self._current_visible_index:
            self._current_visible_index = index
            self._show_current_frame()

    def _step_frame(self, delta: int) -> None:
        """前进或后退一帧。"""

        if not self._visible_frames:
            return
        self._current_visible_index = max(
            0, min(self._current_visible_index + delta, len(self._visible_frames) - 1)
        )
        self.frame_scale.set(self._current_visible_index)
        self._show_current_frame()

    def _toggle_play(self) -> None:
        """切换10 FPS诊断回放。"""

        self._playing = not self._playing
        self.play_button.configure(text="暂停" if self._playing else "播放")
        if self._playing:
            self._play_next()

    def _play_next(self) -> None:
        """在播放状态下循环推进可见帧。"""

        if not self._playing or not self._visible_frames:
            return
        if self._current_visible_index >= len(self._visible_frames) - 1:
            self._playing = False
            self.play_button.configure(text="播放")
            return
        self._step_frame(1)
        self.root.after(100, self._play_next)

    def _show_current_frame(self) -> None:
        """读取、绘制并显示当前BIMG帧及对应日志诊断。"""

        if not self._visible_frames or self._bimg_path is None:
            self.position_var.set("0 / 0")
            return
        frame_index = self._visible_frames[self._current_visible_index]
        entry = self._entries[frame_index]
        try:
            frame = read_bimg_frame(self._bimg_path, entry)
        except Exception as error:
            self._set_detail(f"读取BIMG帧失败：{error}")
            return
        row = self._matched_row(frame_index)
        camera_index = (
            self._alignment.camera_index
            if self._alignment is not None
            else frame.header.source_camera
        )
        rendered = self._render_frame(frame.image, frame.header.width, frame.header.height, row, camera_index)
        self._current_render = rendered
        self._photo = ImageTk.PhotoImage(rendered)
        self.image_canvas.delete("all")
        canvas_width = max(self.image_canvas.winfo_width(), rendered.width)
        canvas_height = max(self.image_canvas.winfo_height(), rendered.height)
        self.image_canvas.create_image(
            canvas_width // 2, canvas_height // 2, image=self._photo, anchor=tk.CENTER
        )
        self.position_var.set(
            f"{self._current_visible_index + 1} / {len(self._visible_frames)}"
        )
        self._set_detail(self._build_detail(entry, row, camera_index))

    def _render_frame(
        self,
        image_bytes: bytes,
        width: int,
        height: int,
        row: CoreLogRow | None,
        camera_index: int,
    ) -> Image.Image:
        """将原始灰度图放大并绘制日志侧诊断叠加。"""

        image = Image.frombytes("L", (width, height), image_bytes).convert("RGB")
        image = image.resize(
            (width * IMAGE_SCALE, height * IMAGE_SCALE), Image.Resampling.NEAREST
        )
        draw = ImageDraw.Draw(image)
        font = ImageFont.load_default(size=13)
        scale = IMAGE_SCALE

        # 橙色框表示边缘截断候选的5像素风险区域。
        draw.rectangle(
            (
                EDGE_GATE_PX * scale,
                EDGE_GATE_PX * scale,
                (width - 1 - EDGE_GATE_PX) * scale,
                (height - 1 - EDGE_GATE_PX) * scale,
            ),
            outline="#ef8a3a",
            width=1,
        )
        if row is None:
            draw.text((8, 8), "NO CSV MATCH", fill="#ff5a5f", font=font)
            return image

        geometry = decode_track_geometry(row.value("I31"))
        status = decode_cross_check_status(row.value("I32"))
        if (
            geometry.valid
            and status.projection_enabled
            and (status.roi_valid_mask & (1 << camera_index))
        ):
            expected = from_center(camera_index, geometry.center_x, geometry.center_y)
            if expected is not None:
                px = (expected[0] + width * 0.5) * scale
                py = (expected[1] + height * 0.5) * scale
                gate = (12.0 if camera_index == 2 else 8.0) * scale
                draw.ellipse((px - gate, py - gate, px + gate, py + gate), outline="#ffd43b", width=2)
                draw.line((px - 6, py, px + 6, py), fill="#ffd43b", width=2)
                draw.line((px, py - 6, px, py + 6), fill="#ffd43b", width=2)

        lamp = camera_lamp(row, camera_index)
        shape = decode_lamp_shape(row.value(f"I{10 + camera_index}"))
        if lamp is not None:
            px = (lamp[0] + width * 0.5) * scale
            py = (lamp[1] + height * 0.5) * scale
            half_width = max(1.0, shape.width_px * 0.5) * scale
            half_length = max(1.0, shape.length_px * 0.5) * scale
            color = "#ff4d4f" if shape.length_px < LAMP_SHORT_LENGTH_PX else "#37d67a"
            angle = math.radians(shape.angle_deg)
            length_x = math.cos(angle) * half_length
            length_y = math.sin(angle) * half_length
            width_x = -math.sin(angle) * half_width
            width_y = math.cos(angle) * half_width
            corners = (
                (px + length_x + width_x, py + length_y + width_y),
                (px + length_x - width_x, py + length_y - width_y),
                (px - length_x - width_x, py - length_y - width_y),
                (px - length_x + width_x, py - length_y + width_y),
            )
            draw.polygon(corners, outline=color, width=2)
            draw.line(
                (px - length_x, py - length_y, px + length_x, py + length_y),
                fill=color,
                width=2,
            )
            draw.line((px - 7, py, px + 7, py), fill=color, width=2)
            draw.line((px, py - 7, px, py + 7), fill=color, width=2)

        beacon_colors = ("#21c7d9", "#c77dff")
        for slot, beacon_x, beacon_y, area in camera_beacons(row, camera_index):
            px = (beacon_x + width * 0.5) * scale
            py = (beacon_y + height * 0.5) * scale
            radius = max(2.0, math.sqrt(area / math.pi)) * scale
            color = beacon_colors[slot % len(beacon_colors)]
            draw.ellipse((px - radius, py - radius, px + radius, py + radius), outline=color, width=2)
            draw.text((px + radius + 2, py - 7), f"B{slot}", fill=color, font=font)

        if status.conflict_mask & (1 << camera_index):
            draw.text((8, 8), "CONFLICT", fill="#ff4d4f", font=font)
        return image

    def _build_detail(
        self, entry: BimgIndexEntry, row: CoreLogRow | None, camera_index: int
    ) -> str:
        """生成当前帧右侧的可复制诊断文本。"""

        lines = [
            f"BIMG frame       {entry.frame_index}",
            f"stream sequence  {entry.stream_sequence}",
            f"source sequence  {entry.frame_sequence}",
            f"capture time      {entry.capture_time_ms} ms",
            f"CRC               {'OK' if entry.crc_ok else 'ERROR'}",
            f"camera            {CAMERA_NAMES_CN[camera_index]}",
            f"stream mode       {entry.stream_mode}",
        ]
        if row is None:
            lines.extend(("", "该图像帧没有匹配到核心0 CSV。"))
            return "\n".join(lines)

        status = decode_cross_check_status(row.value("I32"))
        geometry = decode_track_geometry(row.value("I31"))
        lamp = camera_lamp(row, camera_index)
        shape = decode_lamp_shape(row.value(f"I{10 + camera_index}"))
        beacons = camera_beacons(row, camera_index)
        state_name = (
            TRACK_STATE_NAMES[status.state]
            if status.state < len(TRACK_STATE_NAMES)
            else f"UNKNOWN({status.state})"
        )
        lines.extend(
            (
                "",
                f"CSV row / I0      {row.row_index} / {row.value('I0'):.0f} ms",
                f"roll / pitch      {row.value('I7'):.2f} / {row.value('I8'):.2f} deg",
                f"height / rel yaw  {row.value('I9'):.1f} mm / {row.value('I34'):.2f} deg",
                f"three-camera skew {row.value('I35'):.0f} ms",
                "",
                f"track state       {state_name}",
                f"support mask      0x{status.support_mask:X}",
                f"ROI valid / hit   0x{status.roi_valid_mask:X} / 0x{status.roi_hit_mask:X}",
                f"conflict / meas   0x{status.conflict_mask:X} / 0x{status.measured_mask:X}",
                f"actual ROI mode   {int(status.actual_roi_mode)}",
                f"projection        {int(status.projection_enabled)}",
                f"log-time prediction ({geometry.center_x:.1f}, {geometry.center_y:.1f})"
                if geometry.valid
                else "public center     invalid",
            )
        )
        if lamp is None:
            lines.extend(("", "car lamp           invalid"))
        else:
            lines.extend(
                (
                    "",
                    f"car lamp center   ({lamp[0]:.2f}, {lamp[1]:.2f})",
                    f"lamp width/length {shape.width_px:.2f} / {shape.length_px:.2f} px",
                    f"lamp angle        {shape.angle_deg:.1f} deg",
                    "nearest beacon    unavailable"
                    if shape.nearest_beacon_distance_px is None
                    else f"nearest beacon    {shape.nearest_beacon_distance_px:.1f} px",
                )
            )
        lines.append("")
        if not beacons:
            lines.append("beacons            none")
        for slot, beacon_x, beacon_y, area in beacons:
            lines.append(
                f"beacon {slot}           ({beacon_x:.2f}, {beacon_y:.2f}), area {area:.1f}"
            )
        return "\n".join(lines)

    def _set_detail(self, value: str) -> None:
        """更新只读诊断文本框。"""

        self.detail_text.configure(state=tk.NORMAL)
        self.detail_text.delete("1.0", tk.END)
        self.detail_text.insert("1.0", value)
        self.detail_text.configure(state=tk.DISABLED)

    def _export_alignment(self) -> None:
        """导出逐BIMG帧对应的完整核心0通道表。"""

        if self._alignment is None:
            messagebox.showinfo("尚未配准", "请先加载BIMG和CSV并完成配准。")
            return
        path = filedialog.asksaveasfilename(
            title="导出配准CSV",
            defaultextension=".csv",
            filetypes=(("CSV", "*.csv"),),
        )
        if not path:
            return
        try:
            export_alignment_csv(path, self._entries, self._rows, self._alignment)
        except Exception as error:
            messagebox.showerror("导出失败", str(error))
            return
        messagebox.showinfo("导出完成", f"已写入：\n{path}")

    def _export_current_image(self) -> None:
        """保存当前带诊断叠加的无损PNG。"""

        if self._current_render is None:
            messagebox.showinfo("没有图像", "请先显示一帧BIMG图像。")
            return
        path = filedialog.asksaveasfilename(
            title="导出当前诊断图",
            defaultextension=".png",
            filetypes=(("PNG", "*.png"),),
        )
        if path:
            self._current_render.save(path)

    def _on_close(self) -> None:
        """关闭窗口前停止后台TCP监听。"""

        if self._recorder is not None:
            self._recorder.stop()
        if self._recorder_thread is not None and self._recorder_thread.is_alive():
            self._recorder_thread.join(timeout=2.0)
        self.root.destroy()


def launch_gui() -> None:
    """启动三摄图像日志回放桌面界面。"""

    root = tk.Tk()
    ImageLogReviewApp(root)
    root.mainloop()
