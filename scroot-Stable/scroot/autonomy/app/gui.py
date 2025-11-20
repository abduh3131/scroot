"""Tkinter-based GUI for configuring and launching the scooter autonomy stack."""

from __future__ import annotations

import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, scrolledtext, ttk
from typing import Callable, Optional, Tuple

import cv2
import numpy as np
from PIL import Image, ImageTk

from autonomy.app.environment import (
    install_dependencies,
    load_environment_status,
    resolve_environment_plan,
)
from autonomy.app.hardware import detect_hardware
from autonomy.app.profiles import (
    DEPENDENCY_PROFILES,
    MODEL_PROFILES,
    recommend_profiles,
)
from autonomy.app.state import AppState, AppStateManager
from autonomy.control.config import NavigationIntentConfig, SafetyMindsetConfig
from autonomy.pilot import AutonomyPilot, PilotConfig
from autonomy.utils.data_structures import LidarSnapshot, PilotTickData


def summarize_lidar_ranges(snapshot: Optional[LidarSnapshot]) -> tuple[str, Optional[float], Optional[float]]:
    """Return LiDAR summary text, closest hit, and ultrasonic reading."""

    if snapshot is None:
        return "n/a", None, None
    ranges = snapshot.ranges
    try:
        values = np.asarray(ranges, dtype=np.float32).reshape(-1)
    except Exception:
        return "n/a", None, snapshot.ultrasonic_distance
    if values.size == 0:
        return "n/a", None, snapshot.ultrasonic_distance
    mask = np.isfinite(values) & (values > 0.0)
    if not np.any(mask):
        return "n/a", None, snapshot.ultrasonic_distance
    filtered = values[mask]
    closest = float(np.min(filtered))
    median = float(np.median(filtered))
    summary = f"{closest:.2f} m min | {median:.2f} m med"
    if snapshot.ultrasonic_distance is not None and snapshot.ultrasonic_distance > 0.0:
        summary += f" | US {snapshot.ultrasonic_distance:.2f} m"
    return summary, closest, snapshot.ultrasonic_distance


ACCELERATION_CHOICES = ["auto", "cuda", "cpu"]

ACCELERATION_NOTES = {
    "cpu": "CPU-only mode enabled; YOLO will stay off CUDA for this run.",
    "cuda": "Explicit CUDA acceleration requested; ensure NVIDIA drivers are available.",
}


class PilotRunner(threading.Thread):
    """Background thread that runs the autonomy pilot."""

    def __init__(
        self,
        config: PilotConfig,
        log_callback: Callable[[str], None],
        on_stop: Callable[[], None],
        tick_callback: Optional[Callable[[PilotTickData], None]] = None,
    ) -> None:
        super().__init__(daemon=True)
        self.config = config
        self.log_callback = log_callback
        self.on_stop = on_stop
        self._stop_event = threading.Event()
        self._pilot: Optional[AutonomyPilot] = None
        self._tick_callback = tick_callback

    def stop(self) -> None:
        self._stop_event.set()
        if self._pilot:
            self._pilot.stop()

    def run(self) -> None:  # pragma: no cover - interacts with hardware
        try:
            self._pilot = AutonomyPilot(self.config, tick_callback=self._handle_tick)
            start = time.time()
            for command in self._pilot.run():
                if self._stop_event.is_set():
                    break
                elapsed = time.time() - start
                goal = ""
                review = None
                companion = ""
                decision = self._pilot.latest_decision
                if decision:
                    goal = decision.goal_context or ""
                if hasattr(self._pilot, "latest_review"):
                    review = self._pilot.latest_review
                if hasattr(self._pilot, "latest_companion") and self._pilot.latest_companion:
                    companion = self._pilot.latest_companion or ""
                line = (
                    f"time={elapsed:5.2f}s steer={command.steer:+.3f} "
                    f"throttle={command.throttle:.3f} brake={command.brake:.3f}"
                )
                if goal:
                    line += f" goal={goal}"
                lidar_summary, _, ultrasonic = summarize_lidar_ranges(
                    self._pilot.latest_lidar if self._pilot else None
                )
                if lidar_summary != "n/a":
                    line += f" lidar={lidar_summary}"
                if ultrasonic and ultrasonic > 0.0:
                    line += f" us={ultrasonic:.2f}m"
                if review:
                    line += f" arbiter={review.verdict.value}"
                    if review.reason_tags:
                        joined = ",".join(review.reason_tags)
                        line += f" reasons={joined}"
                if companion:
                    line += f" companion=\"{companion}\""
                self.log_callback(line)
                if self._stop_event.is_set():
                    break
        except Exception as exc:  # pragma: no cover - hardware specific failures
            self.log_callback(f"[error] Pilot crashed: {exc}")
        finally:
            if self._pilot:
                self._pilot.stop()
            self.on_stop()

    def _handle_tick(self, payload: PilotTickData) -> None:
        if self._tick_callback:
            try:
                self._tick_callback(payload)
            except Exception as exc:  # pragma: no cover - GUI safety
                self.log_callback(f"[warn] Failed to render tick data: {exc}")

    def send_command(self, text: str) -> None:
        if self._pilot:
            try:
                self._pilot.submit_command(text)
            except Exception as exc:  # pragma: no cover - defensive
                self.log_callback(f"[warn] Unable to submit command: {exc}")


class MediaFrameSource:
    """Lightweight frame iterator that replays an image or video file."""

    IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}

    def __init__(self, path: Path, fallback_fps: int = 24) -> None:
        self.path = path
        self.fallback_fps = max(1, fallback_fps or 1)
        self._capture: Optional[cv2.VideoCapture] = None
        self._image_frame: Optional[np.ndarray] = None
        self._repeat_frames = 0
        self.effective_fps: int = self.fallback_fps
        self.kind = "video"

    def prepare(self) -> None:
        suffix = self.path.suffix.lower()
        if suffix in self.IMAGE_EXTENSIONS:
            frame = cv2.imread(str(self.path))
            if frame is None:
                raise RuntimeError(f"Unable to load image {self.path}")
            self._image_frame = frame
            self.kind = "image"
            self.effective_fps = self.fallback_fps
            self._repeat_frames = max(1, self.fallback_fps)
            return

        capture = cv2.VideoCapture(str(self.path))
        if not capture.isOpened():
            raise RuntimeError(f"Unable to open video {self.path}")
        fps = capture.get(cv2.CAP_PROP_FPS)
        if fps and fps > 0:
            self.effective_fps = int(max(1, round(fps)))
        else:
            self.effective_fps = self.fallback_fps
        self._capture = capture
        self.kind = "video"

    def frames(self) -> "Iterator[tuple[bool, Optional[np.ndarray]]]":
        if self.kind == "image" and self._image_frame is not None:
            for _ in range(self._repeat_frames):
                yield True, self._image_frame.copy()
            return

        if self._capture is None:
            raise RuntimeError("MediaFrameSource not prepared")

        while True:
            success, frame = self._capture.read()
            if not success:
                break
            yield True, frame

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None


class MediaTestWorker(threading.Thread):
    """Applies the pilot to offline media and records the overlay video."""

    def __init__(
        self,
        config: PilotConfig,
        media_path: Path,
        output_path: Path,
        log_callback: Callable[[str], None],
        frame_callback: Optional[Callable[[np.ndarray, PilotTickData], None]] = None,
        on_complete: Optional[Callable[[Optional[Path], Optional[str]], None]] = None,
    ) -> None:
        super().__init__(daemon=True)
        self.config = config
        self.media_path = media_path
        self.output_path = output_path
        self.log_callback = log_callback
        self.frame_callback = frame_callback
        self.on_complete = on_complete
        self._writer: Optional[cv2.VideoWriter] = None
        self._frame_count = 0
        self._error: Optional[str] = None
        self._stop_event = threading.Event()
        self._pilot: Optional[AutonomyPilot] = None
        self._cancelled = False
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

    def cancel(self) -> None:
        """Signal the worker to stop after the current tick."""

        self._cancelled = True
        self._stop_event.set()
        if self._pilot:
            self._pilot.stop()

    def run(self) -> None:  # pragma: no cover - integrates heavy models
        try:
            source = MediaFrameSource(self.media_path, fallback_fps=self.config.camera_fps)
            self.log_callback(f"Loading {self.media_path.name}...")
            source.prepare()
            config = self.config
            config.visualize = False  # avoid GUI popups during offline playback
            config.camera_fps = source.effective_fps

            pilot = AutonomyPilot(config, tick_callback=self._handle_tick)
            self._pilot = pilot
            pilot._camera = source
            self.log_callback("Applying pilot to offline media...")
            for _ in pilot.run():
                if self._stop_event.is_set():
                    self.log_callback("Stop requested, waiting for pilot to shut down...")
                    break
            if self._stop_event.is_set():
                self.log_callback("Media test cancelled; cleaning up partial results...")
            else:
                self.log_callback("Pilot finished, finalizing video...")
        except Exception as exc:
            self._error = str(exc)
            self.log_callback(f"[error] Media test failed: {exc}")
        finally:
            if self._pilot:
                self._pilot.stop()
                self._pilot = None
            if self._writer is not None:
                self._writer.release()
                self._writer = None
            if self._cancelled and self.output_path.exists():
                try:
                    self.output_path.unlink()
                    self.log_callback("Partial overlay removed after cancellation.")
                except Exception as exc:  # pragma: no cover - filesystem specific
                    self.log_callback(f"[warn] Unable to remove partial overlay: {exc}")
            if self.on_complete:
                if self._cancelled:
                    output: Optional[Path] = None
                else:
                    output = self.output_path if self._frame_count > 0 else None
                error = self._error
                if self._cancelled and not error:
                    error = "Cancelled by user"
                self.on_complete(output, error)

    def _handle_tick(self, payload: PilotTickData) -> None:
        overlay = payload.overlay
        if overlay is None:
            return
        if self._writer is None:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            height, width = overlay.shape[:2]
            fps = max(1, getattr(payload, "fps", self.config.camera_fps))
            self._writer = cv2.VideoWriter(str(self.output_path), fourcc, fps, (width, height))
            if not self._writer.isOpened():
                raise RuntimeError(f"Unable to create video writer for {self.output_path}")
            self.log_callback(f"Recording overlay to {self.output_path}")
        self._writer.write(overlay)
        self._frame_count += 1
        if self.frame_callback:
            self.frame_callback(overlay, payload)


class ScooterApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Scooter Autonomy Suite")
        self.geometry("1024x720")
        self.minsize(900, 600)

        self.state_manager = AppStateManager()
        self.hardware_profile = self.state_manager.load_hardware() or detect_hardware()
        self.state_manager.save_hardware(self.hardware_profile)

        self._is_jetson_orin = (
            self.hardware_profile.environment == "jetson"
            and "orin" in (self.hardware_profile.gpu_name or "").lower()
        )

        recommended_model, recommended_dep = recommend_profiles(self.hardware_profile)
        stored_state = self.state_manager.load_state()
        if stored_state is None:
            self.app_state = AppState.default(recommended_model, recommended_dep)
            self._apply_hardware_tuning(initial=True)
            self.state_manager.save_state(self.app_state)
        else:
            self.app_state = stored_state
            self._apply_hardware_tuning(initial=False)

        self.pilot_thread: Optional[PilotRunner] = None
        self.media_test_thread: Optional[MediaTestWorker] = None
        self._video_photo: Optional[ImageTk.PhotoImage] = None
        self._media_test_photo: Optional[ImageTk.PhotoImage] = None
        self._last_tick_time: float = 0.0
        self._last_arbiter_verdict: str = ""
        self._last_companion: str = ""
        self._last_lidar_summary: str = ""
        self._auto_connect_enabled: bool = True
        self._last_auto_log: float = 0.0

        self._build_ui()
        self._render_hardware_summary()
        self._update_model_description()
        self._update_dependency_description()
        self.after(1000, self._maybe_autostart)

    # ------------------------------------------------------------------
    # UI construction
    def _apply_hardware_tuning(self, *, initial: bool) -> None:
        """Tune defaults for Jetson Orin platforms to prioritise smooth operation."""

        if not self._is_jetson_orin:
            return

        tuned = False

        if initial or self.app_state.model_profile not in MODEL_PROFILES:
            self.app_state.model_profile = "jetson_orin"
            tuned = True
        elif not initial and self.app_state.model_profile == "standard":
            self.app_state.model_profile = "jetson_orin"
            tuned = True

        if initial or self.app_state.dependency_profile not in DEPENDENCY_PROFILES:
            self.app_state.dependency_profile = "jetson"
            tuned = True
        elif not initial and self.app_state.dependency_profile == "modern":
            self.app_state.dependency_profile = "jetson"
            tuned = True

        if initial:
            if self.app_state.resolution_width != 960:
                self.app_state.resolution_width = 960
                tuned = True
            if self.app_state.resolution_height != 544:
                self.app_state.resolution_height = 544
                tuned = True
            if self.app_state.fps != 24:
                self.app_state.fps = 24
                tuned = True
            if self.app_state.enable_visualization:
                self.app_state.enable_visualization = False
                tuned = True
            preferred_accel = "cuda" if self._is_jetson_orin else "auto"
            if self.app_state.acceleration_mode != preferred_accel:
                self.app_state.acceleration_mode = preferred_accel
                tuned = True

        if tuned and not initial:
            self.state_manager.save_state(self.app_state)

    def _create_scrolling_text(self, parent: tk.Widget, **kwargs) -> scrolledtext.ScrolledText:
        widget = scrolledtext.ScrolledText(parent, **kwargs)
        self._bind_mousewheel(widget)
        return widget

    def _bind_mousewheel(self, widget: tk.Widget, *, target: Optional[tk.Widget] = None) -> None:
        scroll_target = target or widget
        yview_scroll = getattr(scroll_target, "yview_scroll", None)
        if yview_scroll is None:
            return

        def _on_mousewheel(event: tk.Event) -> str:
            delta_units = 0
            if getattr(event, "delta", 0):
                delta_units = -1 if event.delta > 0 else 1
            elif getattr(event, "num", None) == 4:
                delta_units = -1
            elif getattr(event, "num", None) == 5:
                delta_units = 1
            if delta_units:
                yview_scroll(delta_units, "units")
            return "break"

        widget.bind("<MouseWheel>", _on_mousewheel, add="+")
        widget.bind("<Button-4>", _on_mousewheel, add="+")
        widget.bind("<Button-5>", _on_mousewheel, add="+")

    def _create_scrollable_tab(self, parent: ttk.Notebook, *, padding: int = 0) -> tuple[ttk.Frame, ttk.Frame]:
        container = ttk.Frame(parent)
        canvas = tk.Canvas(container, highlightthickness=0, borderwidth=0)
        vscroll = ttk.Scrollbar(container, orient=tk.VERTICAL, command=canvas.yview)
        canvas.configure(yscrollcommand=vscroll.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        vscroll.pack(side=tk.RIGHT, fill=tk.Y)

        frame = ttk.Frame(canvas, padding=padding)
        window_id = canvas.create_window((0, 0), window=frame, anchor="nw")

        def _update_scroll_region(_event: tk.Event) -> None:
            canvas.configure(scrollregion=canvas.bbox("all"))
            canvas.itemconfigure(window_id, width=canvas.winfo_width())

        frame.bind("<Configure>", _update_scroll_region)
        canvas.bind("<Configure>", _update_scroll_region)

        self._bind_mousewheel(canvas)
        self._bind_mousewheel(frame, target=canvas)

        return container, frame

    def _build_ui(self) -> None:
        style = ttk.Style(self)
        style.configure("Headline.TLabel", font=("Segoe UI", 16, "bold"))
        style.configure("Body.TLabel", font=("Segoe UI", 11))

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        self.setup_container, self.setup_frame = self._create_scrollable_tab(
            self.notebook, padding=20
        )
        self.run_container, self.run_frame = self._create_scrollable_tab(
            self.notebook, padding=20
        )
        self.test_container, self.test_frame = self._create_scrollable_tab(
            self.notebook, padding=20
        )
        self.notebook.add(self.setup_container, text="Setup")
        self.notebook.add(self.run_container, text="Launch")
        self.notebook.add(self.test_container, text="Media Test")

        # Setup Tab -----------------------------------------------------
        self.hardware_label = ttk.Label(self.setup_frame, style="Body.TLabel", justify=tk.LEFT)
        self.hardware_label.grid(row=0, column=0, columnspan=3, sticky="w")

        ttk.Button(self.setup_frame, text="Rescan Hardware", command=self._rescan_hardware).grid(
            row=0, column=3, padx=(16, 0), sticky="e"
        )

        ttk.Separator(self.setup_frame, orient=tk.HORIZONTAL).grid(
            row=1, column=0, columnspan=4, pady=12, sticky="ew"
        )

        self._build_vehicle_section()

        ttk.Label(self.setup_frame, text="Model Intensity", style="Headline.TLabel").grid(
            row=3, column=0, sticky="w"
        )

        self.model_var = tk.StringVar(value=self.app_state.model_profile)
        self.model_combo = ttk.Combobox(
            self.setup_frame,
            textvariable=self.model_var,
            values=list(MODEL_PROFILES.keys()),
            state="readonly",
        )
        self.model_combo.grid(row=4, column=0, sticky="w")
        self.model_combo.bind("<<ComboboxSelected>>", lambda _event: self._update_model_description())

        self.model_desc = ttk.Label(self.setup_frame, style="Body.TLabel", wraplength=500, justify=tk.LEFT)
        self.model_desc.grid(row=4, column=1, columnspan=3, sticky="w", padx=16)

        ttk.Label(self.setup_frame, text="Dependency Stack", style="Headline.TLabel").grid(
            row=5, column=0, sticky="w", pady=(20, 0)
        )

        self.dependency_var = tk.StringVar(value=self.app_state.dependency_profile)
        self.dependency_combo = ttk.Combobox(
            self.setup_frame,
            textvariable=self.dependency_var,
            values=list(DEPENDENCY_PROFILES.keys()),
            state="readonly",
        )
        self.dependency_combo.grid(row=6, column=0, sticky="w")
        self.dependency_combo.bind("<<ComboboxSelected>>", lambda _event: self._update_dependency_description())

        self.dependency_desc = ttk.Label(
            self.setup_frame, style="Body.TLabel", wraplength=500, justify=tk.LEFT
        )
        self.dependency_desc.grid(row=6, column=1, columnspan=3, sticky="w", padx=16)

        self.install_button = ttk.Button(
            self.setup_frame,
            text="Install Dependencies",
            command=self._install_dependencies,
        )
        self.install_button.grid(row=7, column=0, pady=(24, 0), sticky="w")

        self.save_setup_button = ttk.Button(
            self.setup_frame,
            text="Save Setup",
            command=self._save_setup,
        )
        self.save_setup_button.grid(row=7, column=1, pady=(24, 0), sticky="w")

        self.setup_log = self._create_scrolling_text(
            self.setup_frame, height=10, width=80, wrap=tk.WORD
        )
        self.setup_log.configure(state=tk.DISABLED)
        self.setup_log.grid(row=8, column=0, columnspan=4, pady=(24, 0), sticky="nsew")

        self.setup_frame.rowconfigure(8, weight=1)
        self.setup_frame.columnconfigure(2, weight=1)

        # Launch Tab ----------------------------------------------------
        ttk.Label(self.run_frame, text="Launch Autonomy", style="Headline.TLabel").grid(
            row=0, column=0, sticky="w"
        )

        ttk.Label(self.run_frame, text="Camera Source", style="Body.TLabel").grid(row=1, column=0, sticky="w")
        self.camera_var = tk.StringVar(value=self.app_state.camera_source)
        ttk.Entry(
            self.run_frame, textvariable=self.camera_var, width=28, state="readonly"
        ).grid(row=1, column=1, columnspan=2, sticky="w")

        ttk.Label(self.run_frame, text="Resolution", style="Body.TLabel").grid(row=2, column=0, sticky="w")
        self.width_var = tk.IntVar(value=self.app_state.resolution_width)
        self.height_var = tk.IntVar(value=self.app_state.resolution_height)
        ttk.Entry(self.run_frame, textvariable=self.width_var, width=8).grid(row=2, column=1, sticky="w")
        ttk.Entry(self.run_frame, textvariable=self.height_var, width=8).grid(row=2, column=2, sticky="w")

        ttk.Label(self.run_frame, text="FPS", style="Body.TLabel").grid(row=3, column=0, sticky="w")
        self.fps_var = tk.IntVar(value=self.app_state.fps)
        ttk.Entry(self.run_frame, textvariable=self.fps_var, width=8).grid(row=3, column=1, sticky="w")

        ttk.Label(self.run_frame, text="Acceleration Mode", style="Body.TLabel").grid(row=4, column=0, sticky="w")
        self.acceleration_var = tk.StringVar(value=self.app_state.acceleration_mode)
        self.acceleration_combo = ttk.Combobox(
            self.run_frame,
            textvariable=self.acceleration_var,
            values=ACCELERATION_CHOICES,
            state="readonly",
            width=8,
        )
        self.acceleration_combo.grid(row=4, column=1, sticky="w")
        ttk.Label(
            self.run_frame,
            text=("Auto picks CUDA on Jetson; use CPU only when debugging."),
            style="Body.TLabel",
            wraplength=360,
            justify=tk.LEFT,
        ).grid(row=4, column=2, columnspan=2, sticky="w", padx=12)

        self.visual_var = tk.BooleanVar(value=self.app_state.enable_visualization)
        ttk.Checkbutton(self.run_frame, text="Enable Visualization", variable=self.visual_var).grid(
            row=5, column=0, columnspan=2, sticky="w"
        )

        ttk.Label(self.run_frame, text="Safety Mindset", style="Body.TLabel").grid(row=6, column=0, sticky="w")
        self.mindset_var = tk.StringVar(value=self.app_state.safety_mindset)
        ttk.Combobox(
            self.run_frame,
            textvariable=self.mindset_var,
            values=["off", "on"],
            state="readonly",
        ).grid(row=6, column=1, sticky="w")

        ttk.Label(self.run_frame, text="Ambient Mode", style="Body.TLabel").grid(row=7, column=0, sticky="w")
        self.ambient_var = tk.StringVar(value=self.app_state.ambient_mode)
        ttk.Combobox(
            self.run_frame,
            textvariable=self.ambient_var,
            values=["on", "off"],
            state="readonly",
        ).grid(row=7, column=1, sticky="w")

        self.advisor_var = tk.BooleanVar(value=self.app_state.advisor_enabled)
        ttk.Checkbutton(
            self.run_frame,
            text="Enable Advisor Narration",
            variable=self.advisor_var,
            command=self._update_persona_state,
        ).grid(row=8, column=0, columnspan=2, sticky="w")

        ttk.Label(self.run_frame, text="Companion Persona", style="Body.TLabel").grid(row=9, column=0, sticky="w")
        self.persona_var = tk.StringVar(value=self.app_state.persona)
        self.persona_combo = ttk.Combobox(
            self.run_frame,
            textvariable=self.persona_var,
            values=["calm_safe", "smart_scout", "playful"],
            state="readonly",
        )
        self.persona_combo.grid(row=9, column=1, sticky="w")
        self._update_persona_state()

        self.start_button = ttk.Button(
            self.run_frame, text="Start Pilot", command=lambda: self._start_pilot()
        )
        self.start_button.grid(row=10, column=0, pady=(24, 0), sticky="w")

        self.stop_button = ttk.Button(
            self.run_frame, text="Stop Pilot", command=self._stop_pilot, state=tk.DISABLED
        )
        self.stop_button.grid(row=10, column=1, pady=(24, 0), sticky="w")

        self.video_frame = ttk.LabelFrame(self.run_frame, text="Live Camera & Lane Overlay")
        self.video_frame.grid(row=11, column=0, columnspan=4, pady=(16, 0), sticky="nsew")
        self.video_label = ttk.Label(self.video_frame, text="Video feed will appear here", anchor="center")
        self.video_label.pack(fill=tk.BOTH, expand=True)

        telemetry_frame = ttk.Frame(self.run_frame)
        telemetry_frame.grid(row=11, column=0, columnspan=4, sticky="ew", pady=(12, 0))
        telemetry_frame.columnconfigure(1, weight=1)

        ttk.Label(telemetry_frame, text="Throttle").grid(row=0, column=0, sticky="w")
        self.throttle_bar = ttk.Progressbar(
            telemetry_frame, orient=tk.HORIZONTAL, length=220, mode="determinate", maximum=100
        )
        self.throttle_bar.grid(row=0, column=1, sticky="ew", padx=(6, 12))

        ttk.Label(telemetry_frame, text="Brake").grid(row=1, column=0, sticky="w")
        self.brake_bar = ttk.Progressbar(
            telemetry_frame, orient=tk.HORIZONTAL, length=220, mode="determinate", maximum=100
        )
        self.brake_bar.grid(row=1, column=1, sticky="ew", padx=(6, 12))

        ttk.Label(telemetry_frame, text="Steer").grid(row=0, column=2, sticky="w")
        self.steer_value = tk.StringVar(value="0.00")
        ttk.Label(telemetry_frame, textvariable=self.steer_value, width=8).grid(row=0, column=3, sticky="w")

        ttk.Label(telemetry_frame, text="Safety Arbiter").grid(row=1, column=2, sticky="w")
        self.arbiter_status = tk.StringVar(value="Idle")
        ttk.Label(telemetry_frame, textvariable=self.arbiter_status, width=18).grid(row=1, column=3, sticky="w")

        ttk.Label(telemetry_frame, text="LiDAR (closest | median | ultrasonic)").grid(
            row=2, column=0, sticky="w"
        )
        self.lidar_status = tk.StringVar(value="n/a")
        ttk.Label(telemetry_frame, textvariable=self.lidar_status, width=32).grid(
            row=2, column=1, sticky="w"
        )

        self.message_frame = ttk.LabelFrame(self.run_frame, text="Command Console")
        self.message_frame.grid(row=14, column=0, columnspan=4, sticky="nsew", pady=(12, 0))
        self.message_frame.rowconfigure(0, weight=1)
        self.message_frame.columnconfigure(0, weight=1)
        self.message_text = self._create_scrolling_text(
            self.message_frame, height=6, wrap=tk.WORD
        )
        self.message_text.configure(state=tk.DISABLED)
        self.message_text.grid(row=0, column=0, columnspan=3, sticky="nsew")
        self.command_var = tk.StringVar()
        self.command_entry = ttk.Entry(self.message_frame, textvariable=self.command_var)
        self.command_entry.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        self.command_entry.bind("<Return>", lambda _event: self._send_command())
        self.send_button = ttk.Button(self.message_frame, text="Send", command=self._send_command)
        self.send_button.grid(row=1, column=2, sticky="e", padx=(6, 0), pady=(6, 0))

        self.launch_log = self._create_scrolling_text(
            self.run_frame, height=10, width=100, wrap=tk.WORD
        )
        self.launch_log.configure(state=tk.DISABLED)
        self.launch_log.grid(row=15, column=0, columnspan=4, pady=(16, 0), sticky="nsew")

        self.run_frame.rowconfigure(10, weight=3)
        self.run_frame.rowconfigure(12, weight=1)
        self.run_frame.rowconfigure(13, weight=1)
        self.run_frame.columnconfigure(3, weight=1)

        # Media Test Tab ------------------------------------------------
        self._build_media_test_tab()

    # ------------------------------------------------------------------
    def _build_vehicle_section(self) -> None:
        self.vehicle_description_var = tk.StringVar(value=self.app_state.vehicle_description)
        self.vehicle_width_var = tk.DoubleVar(value=self.app_state.vehicle_width_m)
        self.vehicle_length_var = tk.DoubleVar(value=self.app_state.vehicle_length_m)
        self.vehicle_height_var = tk.DoubleVar(value=self.app_state.vehicle_height_m)
        self.vehicle_margin_var = tk.DoubleVar(value=self.app_state.vehicle_clearance_margin_m)
        self.calibration_distance_var = tk.DoubleVar(value=self.app_state.calibration_reference_distance_m)
        self.calibration_pixels_var = tk.DoubleVar(value=self.app_state.calibration_reference_pixels)

        frame = ttk.LabelFrame(self.setup_frame, text="Vehicle Envelope", padding=(12, 8))
        frame.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(0, 16))
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(3, weight=1)

        ttk.Label(frame, text="Description", style="Body.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.vehicle_description_var, width=32).grid(
            row=0, column=1, columnspan=3, sticky="ew"
        )

        ttk.Label(frame, text="Width (m)").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=self.vehicle_width_var, width=10).grid(
            row=1, column=1, sticky="w", pady=(8, 0)
        )
        ttk.Label(frame, text="Length (m)").grid(row=1, column=2, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=self.vehicle_length_var, width=10).grid(
            row=1, column=3, sticky="w", pady=(8, 0)
        )

        ttk.Label(frame, text="Height (m)").grid(row=2, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.vehicle_height_var, width=10).grid(row=2, column=1, sticky="w")
        ttk.Label(frame, text="Side Margin (m)").grid(row=2, column=2, sticky="w")
        ttk.Entry(frame, textvariable=self.vehicle_margin_var, width=10).grid(
            row=2, column=3, sticky="w"
        )

        ttk.Label(frame, text="Calibration distance (m)").grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=self.calibration_distance_var, width=10).grid(
            row=3, column=1, sticky="w", pady=(8, 0)
        )
        ttk.Label(frame, text="Reference width (px)").grid(row=3, column=2, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=self.calibration_pixels_var, width=10).grid(
            row=3, column=3, sticky="w", pady=(8, 0)
        )

        hint = (
            "Describe your scooter/cart and provide physical dimensions so obstacle clearance"
            " stays realistic. Calibration pairs a typical camera distance with the apparent"
            " width of the vehicle in pixels to ground distance estimates."
        )
        ttk.Label(frame, text=hint, style="Body.TLabel", wraplength=720, justify=tk.LEFT).grid(
            row=4, column=0, columnspan=4, sticky="w", pady=(8, 0)
        )

    def _read_vehicle_settings(self) -> dict[str, float]:
        try:
            width = float(self.vehicle_width_var.get())
            length = float(self.vehicle_length_var.get())
            height = float(self.vehicle_height_var.get())
            margin = float(self.vehicle_margin_var.get())
            calibration_distance = float(self.calibration_distance_var.get())
            calibration_pixels = float(self.calibration_pixels_var.get())
        except (TypeError, ValueError) as exc:
            raise ValueError("Vehicle calibration values must be numeric.") from exc

        if width <= 0 or length <= 0 or height <= 0:
            raise ValueError("Vehicle dimensions must be greater than zero.")
        if margin < 0:
            raise ValueError("Side margin cannot be negative.")
        if calibration_distance <= 0 or calibration_pixels <= 0:
            raise ValueError("Calibration distance and pixels must be positive.")

        return {
            "width": width,
            "length": length,
            "height": height,
            "margin": margin,
            "calibration_distance": calibration_distance,
            "calibration_pixels": calibration_pixels,
        }

    # ------------------------------------------------------------------
    # Setup tab actions
    def _render_hardware_summary(self) -> None:
        summary = (
            f"Detected Hardware:\n"
            f"  OS: {self.hardware_profile.os_name} {self.hardware_profile.os_version}\n"
            f"  CPU: {self.hardware_profile.cpu_model} ({self.hardware_profile.cpu_count} cores)\n"
            f"  Memory: {self.hardware_profile.total_memory_gb:.2f} GB\n"
            f"  GPU: {self.hardware_profile.gpu_name or 'None'}\n"
            f"  Tier: {self.hardware_profile.compute_tier.title()}"
        )
        self.hardware_label.configure(text=summary)

    def _rescan_hardware(self) -> None:
        self._append_setup_log("Rescanning hardware...")

        def worker() -> None:
            profile = detect_hardware()
            self.state_manager.save_hardware(profile)
            self.hardware_profile = profile
            recommended_model, recommended_dep = recommend_profiles(profile)
            self.after(0, self._render_hardware_summary)
            self.after(0, lambda: self.model_var.set(recommended_model.key))
            self.after(0, lambda: self.dependency_var.set(recommended_dep.key))
            self.after(0, self._update_model_description)
            self.after(0, self._update_dependency_description)
            self._append_setup_log("Hardware scan complete.")

        threading.Thread(target=worker, daemon=True).start()

    def _build_media_test_tab(self) -> None:
        ttk.Label(
            self.test_frame,
            text=(
                "Upload a road clip or still photo to replay the pilot offline. "
                "The AI will annotate the footage with steering, throttle, and lane cues."
            ),
            style="Body.TLabel",
            wraplength=760,
            justify=tk.LEFT,
        ).grid(row=0, column=0, columnspan=3, sticky="w")

        self.test_media_path_var = tk.StringVar()
        ttk.Label(self.test_frame, text="Media file", style="Headline.TLabel").grid(
            row=1, column=0, sticky="w", pady=(18, 0)
        )
        path_entry = ttk.Entry(self.test_frame, textvariable=self.test_media_path_var, width=60)
        path_entry.grid(row=2, column=0, sticky="we", pady=(4, 0))
        ttk.Button(self.test_frame, text="Browse", command=self._browse_media_file).grid(
            row=2, column=1, padx=(8, 0), sticky="w", pady=(4, 0)
        )

        self.test_output_path_var = tk.StringVar()
        ttk.Label(self.test_frame, text="Overlay output", style="Headline.TLabel").grid(
            row=3, column=0, sticky="w", pady=(12, 0)
        )
        out_entry = ttk.Entry(self.test_frame, textvariable=self.test_output_path_var, width=60)
        out_entry.grid(row=4, column=0, sticky="we")
        ttk.Button(self.test_frame, text="Browse", command=self._browse_output_file).grid(
            row=4, column=1, padx=(8, 0), sticky="w"
        )
        ttk.Button(self.test_frame, text="Use default", command=lambda: self.test_output_path_var.set("")).grid(
            row=4, column=2, padx=(8, 0), sticky="w"
        )
        ttk.Label(
            self.test_frame,
            text="Leave blank to write into logs/media_tests/. Point to a folder or filename to override.",
            style="Body.TLabel",
            wraplength=760,
            justify=tk.LEFT,
        ).grid(row=5, column=0, columnspan=3, sticky="w", pady=(4, 0))

        self.test_status_var = tk.StringVar(value="Idle")
        ttk.Label(self.test_frame, textvariable=self.test_status_var, style="Body.TLabel").grid(
            row=6, column=0, sticky="w", pady=(12, 0)
        )

        self.test_output_var = tk.StringVar()
        ttk.Label(
            self.test_frame,
            textvariable=self.test_output_var,
            style="Body.TLabel",
            wraplength=760,
            justify=tk.LEFT,
        ).grid(row=7, column=0, columnspan=3, sticky="w")

        self.test_start_button = ttk.Button(
            self.test_frame,
            text="Generate Overlay",
            command=self._start_media_test,
        )
        self.test_start_button.grid(row=8, column=0, sticky="w", pady=(10, 0))
        self.test_stop_button = ttk.Button(
            self.test_frame,
            text="Stop",
            command=self._stop_media_test,
            state=tk.DISABLED,
        )
        self.test_stop_button.grid(row=8, column=1, sticky="w", pady=(10, 0))

        preview_frame = ttk.LabelFrame(self.test_frame, text="Preview")
        preview_frame.grid(row=9, column=0, columnspan=3, sticky="nsew", pady=(16, 0))
        self.test_preview_label = ttk.Label(
            preview_frame,
            text="Overlay preview will appear here.",
            anchor="center",
        )
        self.test_preview_label.pack(fill=tk.BOTH, expand=True)

        self.test_log = self._create_scrolling_text(
            self.test_frame,
            height=10,
            width=90,
            wrap=tk.WORD,
        )
        self.test_log.configure(state=tk.DISABLED)
        self.test_log.grid(row=10, column=0, columnspan=3, sticky="nsew", pady=(16, 0))

        self.test_frame.rowconfigure(9, weight=3)
        self.test_frame.rowconfigure(10, weight=1)

    def _update_persona_state(self) -> None:
        if not hasattr(self, "persona_combo"):
            return
        state = "readonly" if self.advisor_var.get() else "disabled"
        self.persona_combo.configure(state=state)
        self.test_frame.columnconfigure(0, weight=1)

    def _browse_media_file(self) -> None:
        path = filedialog.askopenfilename(
            title="Select media", filetypes=[("Media", "*.mp4 *.mov *.avi *.mkv *.png *.jpg *.jpeg *.bmp"), ("All", "*.*")]
        )
        if path:
            self.test_media_path_var.set(path)

    def _browse_output_file(self) -> None:
        current = self.test_output_path_var.get().strip()
        default_dir = Path("logs") / "media_tests"
        initial_dir = default_dir
        initial_file = "overlay.mp4"
        if current:
            candidate = Path(current).expanduser()
            if candidate.is_dir() or not candidate.suffix:
                initial_dir = candidate
            else:
                initial_dir = candidate.parent
                initial_file = candidate.name
        path = filedialog.asksaveasfilename(
            title="Save overlay as",
            defaultextension=".mp4",
            filetypes=[("MP4", "*.mp4"), ("All", "*.*")],
            initialdir=str(initial_dir.expanduser()),
            initialfile=initial_file,
        )
        if path:
            self.test_output_path_var.set(path)

    def _append_test_log(self, text: str) -> None:
        def update() -> None:
            self.test_log.configure(state=tk.NORMAL)
            self.test_log.insert(tk.END, text + "\n")
            self.test_log.configure(state=tk.DISABLED)
            self.test_log.see(tk.END)

        self.after(0, update)

    def _start_media_test(self) -> None:
        if self.media_test_thread and self.media_test_thread.is_alive():
            messagebox.showwarning("Media Test", "A media test is already running.")
            return

        media_path_str = self.test_media_path_var.get().strip()
        if not media_path_str:
            messagebox.showerror("Media Test", "Please select an image or video file first.")
            return
        media_path = Path(media_path_str).expanduser()
        if not media_path.exists():
            messagebox.showerror("Media Test", f"File not found: {media_path}")
            return

        config_tuple = self._build_pilot_config()
        if not config_tuple:
            return
        config, jetson_note = config_tuple

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_dir = Path("logs") / "media_tests"
        default_dir.mkdir(parents=True, exist_ok=True)
        custom_output = self.test_output_path_var.get().strip()
        if custom_output:
            output_path = Path(custom_output).expanduser()
            if output_path.is_dir() or not output_path.suffix:
                output_path = output_path / f"{media_path.stem}_{timestamp}.mp4"
            elif output_path.suffix.lower() != ".mp4":
                output_path = output_path.with_suffix(".mp4")
            output_path.parent.mkdir(parents=True, exist_ok=True)
        else:
            output_path = default_dir / f"{media_path.stem}_{timestamp}.mp4"

        self.test_status_var.set("Running media test...")
        self.test_output_var.set("")
        self._append_test_log(f"Starting offline pilot replay for {media_path}...")
        self._append_test_log(f"Overlay will be saved to {output_path}")
        if jetson_note:
            self._append_test_log("Jetson Orin optimisations enabled (CUDA acceleration).")
        self._log_acceleration_note(self._append_test_log, config.detector_device)

        self.media_test_thread = MediaTestWorker(
            config=config,
            media_path=media_path,
            output_path=output_path,
            log_callback=self._append_test_log,
            frame_callback=self._handle_media_frame,
            on_complete=self._media_test_finished,
        )
        self.test_start_button.configure(state=tk.DISABLED)
        self.test_stop_button.configure(state=tk.NORMAL)
        self.media_test_thread.start()

    def _stop_media_test(self) -> None:
        if not self.media_test_thread:
            return
        self.test_status_var.set("Stopping current media test...")
        self._append_test_log("Stop requested; waiting for exporter to finish cleanup...")
        self.media_test_thread.cancel()
        self.test_stop_button.configure(state=tk.DISABLED)

    def _handle_media_frame(self, frame: np.ndarray, payload: PilotTickData) -> None:
        def update() -> None:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(rgb)
            self._media_test_photo = ImageTk.PhotoImage(image=image)
            summary = (
                f"steer={payload.command.steer:+.2f} throttle={payload.command.throttle:.2f} "
                f"brake={payload.command.brake:.2f}"
            )
            lidar_text, _, _ = summarize_lidar_ranges(payload.lidar)
            if lidar_text != "n/a":
                summary += f"\nLiDAR {lidar_text}"
            self.test_preview_label.configure(image=self._media_test_photo, text=summary, compound=tk.TOP)

        self.after(0, update)

    def _media_test_finished(self, output_path: Optional[Path], error: Optional[str]) -> None:
        def update() -> None:
            self.media_test_thread = None
            if hasattr(self, "test_start_button"):
                self.test_start_button.configure(state=tk.NORMAL)
            if hasattr(self, "test_stop_button"):
                self.test_stop_button.configure(state=tk.DISABLED)
            if error == "Cancelled by user":
                self.test_status_var.set("Cancelled")
                self.test_output_var.set("Overlay export cancelled.")
                return
            if error:
                self.test_status_var.set(f"Failed: {error}")
            else:
                self.test_status_var.set("Complete")
            if output_path:
                self.test_output_var.set(f"Overlay saved to {output_path}")
            else:
                self.test_output_var.set("No video was generated.")

        self.after(0, update)

    def _append_setup_log(self, message: str) -> None:
        self.after(0, lambda: self._write_log(self.setup_log, message))

    def _append_launch_log(self, message: str) -> None:
        self.after(0, lambda: self._write_log(self.launch_log, message))

    def _append_message(self, message: str) -> None:
        if not hasattr(self, "message_text"):
            return

        def writer() -> None:
            self.message_text.configure(state=tk.NORMAL)
            self.message_text.insert(tk.END, message + "\n")
            self.message_text.see(tk.END)
            self.message_text.configure(state=tk.DISABLED)

        self.after(0, writer)

    def _log_acceleration_note(self, writer: Callable[[str], None], mode: str) -> None:
        note = ACCELERATION_NOTES.get(mode)
        if note:
            writer(note)

    def _write_log(self, widget: tk.Text, message: str) -> None:
        widget.configure(state=tk.NORMAL)
        widget.insert(tk.END, message + "\n")
        widget.see(tk.END)
        widget.configure(state=tk.DISABLED)

    def _update_model_description(self) -> None:
        profile = MODEL_PROFILES[self.model_var.get()]
        text = (
            f"{profile.label}: {profile.description}\n"
            f"YOLO checkpoint: {profile.yolo_model}"
        )
        self.model_desc.configure(text=text)

    def _update_dependency_description(self) -> None:
        profile = DEPENDENCY_PROFILES[self.dependency_var.get()]
        text = f"{profile.label}: {profile.description}\n" + "\n".join(profile.requirements)
        self.dependency_desc.configure(text=text)

    def _save_setup(self) -> None:
        self._append_setup_log("Saving setup preferences...")
        try:
            vehicle_settings = self._read_vehicle_settings()
        except ValueError as exc:
            messagebox.showerror("Invalid Input", str(exc))
            self._append_setup_log(f"Failed to save: {exc}")
            return
        self.app_state.model_profile = self.model_var.get()
        self.app_state.dependency_profile = self.dependency_var.get()
        self.app_state.vehicle_description = self.vehicle_description_var.get().strip() or "Scooter"
        self.app_state.vehicle_width_m = vehicle_settings["width"]
        self.app_state.vehicle_length_m = vehicle_settings["length"]
        self.app_state.vehicle_height_m = vehicle_settings["height"]
        self.app_state.vehicle_clearance_margin_m = vehicle_settings["margin"]
        self.app_state.calibration_reference_distance_m = vehicle_settings["calibration_distance"]
        self.app_state.calibration_reference_pixels = vehicle_settings["calibration_pixels"]
        self.state_manager.save_state(self.app_state)
        self._append_setup_log("Setup saved.")
        messagebox.showinfo("Setup", "Configuration saved successfully.")

    def _install_dependencies(self) -> None:
        dependency_profile = DEPENDENCY_PROFILES[self.dependency_var.get()]
        status = load_environment_status()
        plan = resolve_environment_plan(dependency_profile, status=status)

        self.install_button.configure(state=tk.DISABLED)
        self._append_setup_log("Starting dependency installation...")

        def worker() -> None:
            report = install_dependencies(plan, logger=self._append_setup_log)
            if report.error:
                self._append_setup_log(f"Installation failed, see log at {report.log_path}")
                self.after(0, lambda: messagebox.showerror("Install", report.error or "Installation failed"))
            else:
                self._append_setup_log("Installation succeeded.")
                self._append_setup_log(
                    f"Interpreter: {report.plan.python_executable}"
                    + (
                        f" (virtualenv {report.plan.venv_path})"
                        if report.plan.venv_path
                        else ""
                    )
                )
                self.after(0, lambda: messagebox.showinfo("Install", "Dependencies installed."))
            self.after(0, lambda: self.install_button.configure(state=tk.NORMAL))

        threading.Thread(target=worker, daemon=True).start()

    def _maybe_autostart(self) -> None:
        if not self._auto_connect_enabled:
            return
        running = self.pilot_thread and self.pilot_thread.is_alive()
        if running:
            self.after(1000, self._maybe_autostart)
            return
        camera_path = Path(self.camera_var.get()).expanduser()
        lidar_path = camera_path.parent / "lidar.npy"
        now = time.time()
        if camera_path.exists():
            if now - self._last_auto_log > 2.0:
                self._append_launch_log("Auto-connecting to SensorHub inputs...")
                self._last_auto_log = now
            started = self._start_pilot(auto_invoked=True)
            if started:
                self.after(1500, self._maybe_autostart)
                return
        if lidar_path.exists():
            # If only LiDAR has arrived, keep polling until camera arrives too.
            if now - self._last_auto_log > 2.0:
                self._append_launch_log("Waiting for camera.jpg from SensorHub...")
                self._last_auto_log = now
        self.after(1000, self._maybe_autostart)

    # ------------------------------------------------------------------
    # Launch tab actions
    def _start_pilot(self, *, auto_invoked: bool = False) -> bool:
        if self.pilot_thread and self.pilot_thread.is_alive():
            if not auto_invoked:
                messagebox.showwarning("Pilot", "Pilot is already running.")
            return False

        config_tuple = self._build_pilot_config(show_errors=not auto_invoked)
        if not config_tuple:
            if not auto_invoked:
                self._append_launch_log("Pilot launch aborted; fix the inputs above.")
            return False
        config, jetson_note = config_tuple

        if jetson_note:
            self._append_launch_log("Jetson Orin optimisations enabled (CUDA acceleration).")
        self._log_acceleration_note(self._append_launch_log, config.detector_device)

        self._append_launch_log("Launching autonomy pilot...")
        self.start_button.configure(state=tk.DISABLED)
        self.stop_button.configure(state=tk.NORMAL)

        def on_stop() -> None:
            self.after(0, lambda: self.start_button.configure(state=tk.NORMAL))
            self.after(0, lambda: self.stop_button.configure(state=tk.DISABLED))
            self._append_launch_log("Pilot stopped.")

        self.pilot_thread = PilotRunner(
            config,
            log_callback=self._append_launch_log,
            on_stop=on_stop,
            tick_callback=self._handle_pilot_tick,
        )
        self.pilot_thread.start()
        return True

    def _stop_pilot(self) -> None:
        if not self.pilot_thread:
            return
        self._append_launch_log("Stopping pilot...")
        self.pilot_thread.stop()
        self.pilot_thread = None

    def _build_pilot_config(
        self, *, show_errors: bool = True
    ) -> Optional[tuple[PilotConfig, bool]]:
        try:
            width = int(self.width_var.get())
            height = int(self.height_var.get())
            fps = int(self.fps_var.get())
        except (TypeError, ValueError):
            if show_errors:
                messagebox.showerror("Invalid Input", "Resolution and FPS must be integers.")
            return None

        try:
            vehicle_settings = self._read_vehicle_settings()
        except ValueError as exc:
            if show_errors:
                messagebox.showerror("Invalid Input", str(exc))
            return None

        self.app_state.camera_source = self.camera_var.get()
        self.app_state.resolution_width = width
        self.app_state.resolution_height = height
        self.app_state.fps = fps
        self.app_state.enable_visualization = bool(self.visual_var.get())
        self.app_state.safety_mindset = self.mindset_var.get()
        self.app_state.ambient_mode = self.ambient_var.get()
        self.app_state.persona = self.persona_var.get()
        self.app_state.acceleration_mode = self.acceleration_var.get()
        self.app_state.advisor_enabled = bool(self.advisor_var.get())
        self.app_state.vehicle_description = self.vehicle_description_var.get().strip() or "Scooter"
        self.app_state.vehicle_width_m = vehicle_settings["width"]
        self.app_state.vehicle_length_m = vehicle_settings["length"]
        self.app_state.vehicle_height_m = vehicle_settings["height"]
        self.app_state.vehicle_clearance_margin_m = vehicle_settings["margin"]
        self.app_state.calibration_reference_distance_m = vehicle_settings["calibration_distance"]
        self.app_state.calibration_reference_pixels = vehicle_settings["calibration_pixels"]
        self.state_manager.save_state(self.app_state)

        model_profile = MODEL_PROFILES[self.model_var.get()]

        navigation_intent = NavigationIntentConfig()
        navigation_intent.ambient_mode = self.ambient_var.get() == "on"

        safety_mindset = SafetyMindsetConfig()
        safety_mindset.enabled = self.mindset_var.get() == "on"

        config = PilotConfig(
            camera_source=self.camera_var.get(),
            camera_width=width,
            camera_height=height,
            camera_fps=fps,
            model_name=model_profile.yolo_model,
            visualize=self.visual_var.get(),
            navigation_intent=navigation_intent,
            safety_mindset=safety_mindset,
            safety_mindset_enabled=safety_mindset.enabled,
            companion_persona=self.persona_var.get(),
            companion_enabled=bool(self.advisor_var.get()),
            detector_device=self.acceleration_var.get(),
            vehicle_description=self.vehicle_description_var.get().strip() or "Scooter",
            vehicle_width_m=vehicle_settings["width"],
            vehicle_length_m=vehicle_settings["length"],
            vehicle_height_m=vehicle_settings["height"],
            vehicle_clearance_margin_m=vehicle_settings["margin"],
            calibration_reference_distance_m=vehicle_settings["calibration_distance"],
            calibration_reference_pixels=vehicle_settings["calibration_pixels"],
        )

        jetson_note = False
        if self._is_jetson_orin:
            if self.model_var.get() == "jetson_orin":
                config.model_name = MODEL_PROFILES["jetson_orin"].yolo_model
            config.confidence_threshold = max(config.confidence_threshold, 0.35)
            config.iou_threshold = max(config.iou_threshold, 0.45)
            jetson_note = True

        return config, jetson_note

    def _handle_pilot_tick(self, payload: PilotTickData) -> None:
        def update() -> None:
            self._last_tick_time = payload.timestamp

            if payload.overlay is not None:
                rgb = cv2.cvtColor(payload.overlay, cv2.COLOR_BGR2RGB)
                image = Image.fromarray(rgb)
                self._video_photo = ImageTk.PhotoImage(image=image)
                self.video_label.configure(image=self._video_photo, text="")

            steer = payload.command.steer
            throttle = payload.command.throttle
            brake = payload.command.brake
            self.throttle_bar["value"] = max(0, min(100, throttle * 100.0))
            self.brake_bar["value"] = max(0, min(100, brake * 100.0))
            self.steer_value.set(f"{steer:+.2f}")

            lidar_text, _, _ = summarize_lidar_ranges(payload.lidar)
            self.lidar_status.set(lidar_text)
            if lidar_text != "n/a":
                if lidar_text != self._last_lidar_summary:
                    self._append_message(f"LiDAR: {lidar_text}")
                    self._last_lidar_summary = lidar_text
            else:
                self._last_lidar_summary = ""

            if payload.review:
                reason = ", ".join(payload.review.reason_tags)
                status = f"{payload.review.verdict.value} ({reason})"
                verdict_signature = f"{payload.review.verdict.value}:{reason}"
                self.arbiter_status.set(status)
                if verdict_signature != self._last_arbiter_verdict:
                    self._append_message(f"Arbiter: {status}")
                    self._last_arbiter_verdict = verdict_signature
            else:
                self.arbiter_status.set("Arbiter idle")
                self._last_arbiter_verdict = ""

            if payload.companion:
                if payload.companion != self._last_companion:
                    self._append_message(f"Companion: {payload.companion}")
                    self._last_companion = payload.companion
            else:
                self._last_companion = ""

            actuator_summary = (
                f"steer={steer:+.2f} throttle={throttle:.2f} brake={brake:.2f}"
                f" | gate={'/'.join(payload.gate_tags) if payload.gate_tags else 'nominal'}"
            )
            self.video_label.configure(compound=tk.TOP)
            self.video_label.configure(text="" if self._video_photo else actuator_summary)

        self.after(0, update)

    def _send_command(self) -> None:
        text = self.command_var.get().strip()
        if not text:
            return
        self.command_var.set("")
        self._append_message(f"You: {text}")
        if self.pilot_thread and self.pilot_thread.is_alive():
            self.pilot_thread.send_command(text)
            self._append_launch_log(f"Command sent to pilot: {text}")
        else:
            self._append_launch_log("Command queued but pilot is not running.")


def launch_app() -> None:  # pragma: no cover - UI entry point
    app = ScooterApp()
    app.mainloop()
