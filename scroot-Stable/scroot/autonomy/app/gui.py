"""Tkinter-based GUI for configuring and launching the scooter autonomy stack."""

from __future__ import annotations

import csv
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Callable, Optional, TextIO

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
    ADVISOR_MODEL_PROFILES,
    DEPENDENCY_PROFILES,
    MODEL_PROFILES,
    recommend_profiles,
)
from autonomy.app.state import AppState, AppStateManager
from autonomy.control.config import AdvisorRuntimeConfig, NavigationIntentConfig, SafetyMindsetConfig
from autonomy.pilot import AutonomyPilot, PilotConfig
from autonomy.utils.data_structures import PilotTickData


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
                directive = ""
                goal = ""
                review = None
                companion = ""
                if self._pilot.latest_decision:
                    directive = self._pilot.latest_decision.directive or ""
                    goal = self._pilot.latest_decision.goal_context or ""
                if hasattr(self._pilot, "latest_review"):
                    review = self._pilot.latest_review
                if hasattr(self._pilot, "latest_companion") and self._pilot.latest_companion:
                    companion = self._pilot.latest_companion or ""
                line = (
                    f"time={elapsed:5.2f}s steer={command.steer:+.3f} "
                    f"throttle={command.throttle:.3f} brake={command.brake:.3f}"
                )
                if directive:
                    line += f" directive={directive}"
                if goal:
                    line += f" goal={goal}"
                if review:
                    line += f" advisor={review.verdict.value}"
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


class VideoBatchRunner(threading.Thread):
    """Background job that replays a recorded video through the autonomy pilot."""

    def __init__(
        self,
        *,
        config: PilotConfig,
        input_path: Path,
        output_video: Path,
        output_csv: Path,
        fps: float,
        expected_frames: int,
        log_callback: Callable[[str], None],
        on_complete: Callable[[bool, Optional[str]], None],
    ) -> None:
        super().__init__(daemon=True)
        self.config = config
        self.input_path = input_path
        self.output_video_path = output_video
        self.output_csv_path = output_csv
        self._fps = fps
        self._expected_frames = expected_frames
        self.log_callback = log_callback
        self._on_complete = on_complete
        self._stop_event = threading.Event()
        self._pilot: Optional[AutonomyPilot] = None
        self._video_writer: Optional[cv2.VideoWriter] = None
        self._csv_file: Optional[TextIO] = None
        self._csv_writer: Optional[csv.writer] = None
        self._frame_index = 0
        self._last_progress_log = time.time()

    def stop(self) -> None:
        self._stop_event.set()
        if self._pilot:
            self._pilot.stop()

    def run(self) -> None:  # pragma: no cover - interacts with filesystem and heavy models
        error: Optional[str] = None
        try:
            self.log_callback(
                f"Processing video {self.input_path.name} at {self._fps:.2f} FPS"
                + (
                    ""
                    if self._expected_frames <= 0
                    else f" (~{self._expected_frames} frames)"
                )
            )
            self._prepare_outputs()
            self._pilot = AutonomyPilot(self.config, tick_callback=self._handle_tick)
            for _command in self._pilot.run():
                if self._stop_event.is_set():
                    break
            if self._stop_event.is_set():
                self.log_callback("Video analysis cancelled by user.")
            elif self._frame_index == 0:
                self.log_callback("No frames were processed from the selected video.")
            else:
                self.log_callback(
                    f"Video analysis complete: {self._frame_index} frames exported."
                )
        except Exception as exc:  # pragma: no cover - defensive
            error = str(exc)
            self.log_callback(f"[error] Video processing failed: {exc}")
        finally:
            if self._pilot:
                self._pilot.stop()
            self._close_streams()
            if self._on_complete:
                try:
                    self._on_complete(error is None and not self._stop_event.is_set(), error)
                except Exception:  # pragma: no cover - safety
                    pass

    def _prepare_outputs(self) -> None:
        self.output_video_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = self.output_csv_path.open("w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(
            [
                "timestamp_s",
                "steer",
                "throttle",
                "brake",
                "desired_speed",
                "hazard_level",
                "advisor_verdict",
                "advisor_reasons",
            ]
        )
        self._csv_file.flush()
        self.log_callback(f"Writing actuator CSV to {self.output_csv_path}")

    def _ensure_video_writer(self, frame: np.ndarray) -> None:
        if self._video_writer is not None:
            return
        height, width = frame.shape[:2]
        if height <= 0 or width <= 0:
            raise RuntimeError("Unable to determine frame dimensions for video writer")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self._video_writer = cv2.VideoWriter(
            str(self.output_video_path),
            fourcc,
            self._fps if self._fps > 0 else 30.0,
            (width, height),
        )
        if not self._video_writer.isOpened():
            raise RuntimeError(f"Unable to open video writer at {self.output_video_path}")
        self.log_callback(f"Writing overlay video to {self.output_video_path}")

    def _handle_tick(self, payload: PilotTickData) -> None:
        if self._stop_event.is_set():
            return
        if self._csv_writer is None or self._csv_file is None:
            return

        if payload.overlay is not None:
            try:
                self._ensure_video_writer(payload.overlay)
                if self._video_writer is not None:
                    self._video_writer.write(payload.overlay)
            except Exception as exc:  # pragma: no cover - defensive logging
                self.log_callback(f"[warn] Failed to write overlay frame: {exc}")

        frame_timestamp = (
            (self._frame_index / self._fps) if self._fps > 0 else payload.timestamp
        )
        verdict = payload.review.verdict.value if payload.review else ""
        reasons = ",".join(payload.review.reason_tags) if payload.review else ""

        self._csv_writer.writerow(
            [
                round(frame_timestamp, 4),
                round(payload.command.steer, 6),
                round(payload.command.throttle, 6),
                round(payload.command.brake, 6),
                round(payload.decision.desired_speed, 6),
                round(payload.decision.hazard_level, 6),
                verdict,
                reasons,
            ]
        )
        self._csv_file.flush()

        self._frame_index += 1
        if self._should_log_progress():
            percent = 0.0
            if self._expected_frames > 0:
                percent = min(100.0, (self._frame_index / self._expected_frames) * 100.0)
            self.log_callback(
                f"Processed {self._frame_index} frames"
                + ("" if percent <= 0 else f" ({percent:.1f}% complete)")
            )
            self._last_progress_log = time.time()

    def _should_log_progress(self) -> bool:
        if self._frame_index == 0:
            return False
        elapsed = time.time() - self._last_progress_log
        if elapsed >= 5.0:
            return True
        if self._fps > 0:
            return self._frame_index % max(1, int(self._fps)) == 0
        return self._frame_index % 30 == 0

    def _close_streams(self) -> None:
        if self._video_writer is not None:
            self._video_writer.release()
            self._video_writer = None
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None

class ScooterApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Scooter Autonomy Suite")
        self.geometry("1024x720")
        self.minsize(900, 600)

        self.state_manager = AppStateManager()
        self.hardware_profile = self.state_manager.load_hardware() or detect_hardware()
        self.state_manager.save_hardware(self.hardware_profile)

        recommended_model, recommended_dep = recommend_profiles(self.hardware_profile)
        stored_state = self.state_manager.load_state()
        self.app_state = stored_state or AppState.default(recommended_model, recommended_dep)
        if not self.hardware_profile.has_cuda:
            self.app_state.force_cpu = True
        if stored_state is None:
            self.state_manager.save_state(self.app_state)

        self.pilot_thread: Optional[PilotRunner] = None
        self.video_job: Optional[VideoBatchRunner] = None
        self._latest_video_outputs: Optional[tuple[Path, Path]] = None
        self._video_photo: Optional[ImageTk.PhotoImage] = None
        self._last_tick_time: float = 0.0
        self._last_advisor_verdict: str = ""
        self._last_directive: str = ""
        self._last_companion: str = ""
        self._video_max_width = 480
        self._video_max_height = 320

        self._build_ui()
        self._render_hardware_summary()
        self._update_model_description()
        self._update_dependency_description()
        self._update_advisor_profile_description()

    # ------------------------------------------------------------------
    # UI construction
    def _build_ui(self) -> None:
        style = ttk.Style(self)
        style.configure("Headline.TLabel", font=("Segoe UI", 16, "bold"))
        style.configure("Body.TLabel", font=("Segoe UI", 11))

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        self.setup_frame = ttk.Frame(self.notebook, padding=20)
        self.run_frame = ttk.Frame(self.notebook, padding=20)
        self.video_batch_frame = ttk.Frame(self.notebook, padding=20)
        self.notebook.add(self.setup_frame, text="Setup")
        self.notebook.add(self.run_frame, text="Launch")
        self.notebook.add(self.video_batch_frame, text="Video Analysis")

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

        self.setup_log = tk.Text(self.setup_frame, height=10, width=80, state=tk.DISABLED)
        self.setup_log.grid(row=8, column=0, columnspan=4, pady=(24, 0), sticky="nsew")

        self.setup_frame.rowconfigure(8, weight=1)
        self.setup_frame.columnconfigure(2, weight=1)

        # Launch Tab ----------------------------------------------------
        ttk.Label(self.run_frame, text="Launch Autonomy", style="Headline.TLabel").grid(
            row=0, column=0, sticky="w"
        )

        ttk.Label(self.run_frame, text="Camera Source", style="Body.TLabel").grid(row=1, column=0, sticky="w")
        self.camera_var = tk.StringVar(value=self.app_state.camera_source)
        ttk.Entry(self.run_frame, textvariable=self.camera_var, width=10).grid(
            row=1, column=1, sticky="w"
        )

        ttk.Label(self.run_frame, text="Resolution", style="Body.TLabel").grid(row=2, column=0, sticky="w")
        self.width_var = tk.IntVar(value=self.app_state.resolution_width)
        self.height_var = tk.IntVar(value=self.app_state.resolution_height)
        ttk.Entry(self.run_frame, textvariable=self.width_var, width=8).grid(row=2, column=1, sticky="w")
        ttk.Entry(self.run_frame, textvariable=self.height_var, width=8).grid(row=2, column=2, sticky="w")

        ttk.Label(self.run_frame, text="FPS", style="Body.TLabel").grid(row=3, column=0, sticky="w")
        self.fps_var = tk.IntVar(value=self.app_state.fps)
        ttk.Entry(self.run_frame, textvariable=self.fps_var, width=8).grid(row=3, column=1, sticky="w")

        self.visual_var = tk.BooleanVar(value=self.app_state.enable_visualization)
        ttk.Checkbutton(self.run_frame, text="Enable Visualization", variable=self.visual_var).grid(
            row=4, column=0, columnspan=2, sticky="w"
        )

        self.advisor_var = tk.BooleanVar(value=self.app_state.enable_advisor)
        ttk.Checkbutton(self.run_frame, text="Enable Advisor", variable=self.advisor_var).grid(
            row=5, column=0, columnspan=2, sticky="w"
        )

        initial_cpu_only = self.app_state.force_cpu or (not self.hardware_profile.has_cuda)
        self.cpu_only_var = tk.BooleanVar(value=initial_cpu_only)
        ttk.Checkbutton(
            self.run_frame,
            text="Force CPU-only mode",
            variable=self.cpu_only_var,
            command=self._render_hardware_summary,
        ).grid(row=6, column=0, columnspan=2, sticky="w")

        ttk.Label(self.run_frame, text="Advisor Mode", style="Body.TLabel").grid(
            row=7, column=0, sticky="w", pady=(8, 0)
        )
        self.advisor_mode_var = tk.StringVar(value=self.app_state.advisor_mode)
        ttk.Combobox(
            self.run_frame,
            textvariable=self.advisor_mode_var,
            values=["normal", "strict"],
            state="readonly",
        ).grid(row=7, column=1, sticky="w")

        ttk.Label(self.run_frame, text="Advisor Model", style="Body.TLabel").grid(row=8, column=0, sticky="w")
        self.advisor_model_var = tk.StringVar(value=self.app_state.advisor_model_profile)
        self.advisor_model_combo = ttk.Combobox(
            self.run_frame,
            textvariable=self.advisor_model_var,
            values=list(ADVISOR_MODEL_PROFILES.keys()),
            state="readonly",
        )
        self.advisor_model_combo.grid(row=8, column=1, sticky="w")
        self.advisor_model_combo.bind(
            "<<ComboboxSelected>>", lambda _event: self._update_advisor_profile_description()
        )
        self.advisor_model_desc = ttk.Label(
            self.run_frame, style="Body.TLabel", wraplength=400, justify=tk.LEFT
        )
        self.advisor_model_desc.grid(row=8, column=2, columnspan=2, sticky="w", padx=12)

        ttk.Label(self.run_frame, text="Safety Mindset", style="Body.TLabel").grid(row=9, column=0, sticky="w")
        self.mindset_var = tk.StringVar(value=self.app_state.safety_mindset)
        ttk.Combobox(
            self.run_frame,
            textvariable=self.mindset_var,
            values=["off", "on"],
            state="readonly",
        ).grid(row=9, column=1, sticky="w")

        ttk.Label(self.run_frame, text="Ambient Mode", style="Body.TLabel").grid(row=10, column=0, sticky="w")
        self.ambient_var = tk.StringVar(value=self.app_state.ambient_mode)
        ttk.Combobox(
            self.run_frame,
            textvariable=self.ambient_var,
            values=["on", "off"],
            state="readonly",
        ).grid(row=10, column=1, sticky="w")

        ttk.Label(self.run_frame, text="Companion Persona", style="Body.TLabel").grid(row=11, column=0, sticky="w")
        self.persona_var = tk.StringVar(value=self.app_state.persona)
        ttk.Combobox(
            self.run_frame,
            textvariable=self.persona_var,
            values=["calm_safe", "smart_scout", "playful"],
            state="readonly",
        ).grid(row=11, column=1, sticky="w")

        self.start_button = ttk.Button(self.run_frame, text="Start Pilot", command=self._start_pilot)
        self.start_button.grid(row=12, column=0, pady=(24, 0), sticky="w")

        self.stop_button = ttk.Button(
            self.run_frame, text="Stop Pilot", command=self._stop_pilot, state=tk.DISABLED
        )
        self.stop_button.grid(row=12, column=1, pady=(24, 0), sticky="w")

        self.video_frame = ttk.LabelFrame(self.run_frame, text="Live Camera & Advisor Overlay")
        self.video_frame.grid(row=13, column=0, columnspan=2, pady=(16, 0), sticky="w")
        self.video_label = ttk.Label(
            self.video_frame,
            text="Video feed will appear here",
            anchor="center",
            width=60,
        )
        self.video_label.pack(padx=8, pady=8)

        telemetry_frame = ttk.Frame(self.run_frame)
        telemetry_frame.grid(row=14, column=0, columnspan=4, sticky="ew", pady=(12, 0))
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

        ttk.Label(telemetry_frame, text="Advisor Verdict").grid(row=1, column=2, sticky="w")
        self.advisor_status = tk.StringVar(value="Idle")
        ttk.Label(telemetry_frame, textvariable=self.advisor_status, width=18).grid(row=1, column=3, sticky="w")

        self.message_frame = ttk.LabelFrame(self.run_frame, text="Advisor Messaging")
        self.message_frame.grid(row=15, column=0, columnspan=4, sticky="nsew", pady=(12, 0))
        self.message_frame.rowconfigure(0, weight=1)
        self.message_frame.columnconfigure(0, weight=1)
        self.message_text = tk.Text(self.message_frame, height=6, state=tk.DISABLED, wrap=tk.WORD)
        self.message_text.grid(row=0, column=0, columnspan=3, sticky="nsew")
        self.command_var = tk.StringVar()
        self.command_entry = ttk.Entry(self.message_frame, textvariable=self.command_var)
        self.command_entry.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        self.command_entry.bind("<Return>", lambda _event: self._send_command())
        self.send_button = ttk.Button(self.message_frame, text="Send", command=self._send_command)
        self.send_button.grid(row=1, column=2, sticky="e", padx=(6, 0), pady=(6, 0))

        self.launch_log = tk.Text(self.run_frame, height=10, width=100, state=tk.DISABLED)
        self.launch_log.grid(row=16, column=0, columnspan=4, pady=(16, 0), sticky="nsew")

        self.run_frame.rowconfigure(13, weight=3)
        self.run_frame.rowconfigure(15, weight=1)
        self.run_frame.rowconfigure(16, weight=1)
        self.run_frame.columnconfigure(3, weight=1)

        # Video Analysis Tab -------------------------------------------
        self._build_video_tab()

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
            "Describe your scooter/cart and provide physical dimensions so the advisor can"
            " respect real clearance. Calibration pairs a typical camera distance with the"
            " apparent width of the vehicle in pixels to ground distance estimates."
        )
        ttk.Label(frame, text=hint, style="Body.TLabel", wraplength=720, justify=tk.LEFT).grid(
            row=4, column=0, columnspan=4, sticky="w", pady=(8, 0)
        )

    def _build_video_tab(self) -> None:
        ttk.Label(
            self.video_batch_frame,
            text="Offline Video Analysis",
            style="Headline.TLabel",
        ).grid(row=0, column=0, columnspan=3, sticky="w")

        ttk.Label(self.video_batch_frame, text="Input Video", style="Body.TLabel").grid(
            row=1, column=0, sticky="w", pady=(12, 0)
        )
        self.video_input_var = tk.StringVar()
        ttk.Entry(self.video_batch_frame, textvariable=self.video_input_var, width=50).grid(
            row=1, column=1, sticky="ew", pady=(12, 0)
        )
        ttk.Button(
            self.video_batch_frame,
            text="Browse...",
            command=self._browse_video_input,
        ).grid(row=1, column=2, padx=(8, 0), pady=(12, 0), sticky="w")

        ttk.Label(
            self.video_batch_frame, text="Output Directory", style="Body.TLabel"
        ).grid(row=2, column=0, sticky="w", pady=(8, 0))
        self.video_output_dir_var = tk.StringVar()
        ttk.Entry(
            self.video_batch_frame,
            textvariable=self.video_output_dir_var,
            width=50,
        ).grid(row=2, column=1, sticky="ew", pady=(8, 0))
        ttk.Button(
            self.video_batch_frame,
            text="Choose...",
            command=self._browse_video_output_dir,
        ).grid(row=2, column=2, padx=(8, 0), pady=(8, 0), sticky="w")

        ttk.Label(self.video_batch_frame, text="Output Prefix", style="Body.TLabel").grid(
            row=3, column=0, sticky="w", pady=(8, 0)
        )
        self.video_output_prefix_var = tk.StringVar(value="analysis")
        ttk.Entry(
            self.video_batch_frame,
            textvariable=self.video_output_prefix_var,
            width=32,
        ).grid(row=3, column=1, sticky="w", pady=(8, 0))

        initial_cpu_only = self.app_state.force_cpu or (not self.hardware_profile.has_cuda)
        self.video_cpu_var = tk.BooleanVar(value=initial_cpu_only)
        cpu_check = ttk.Checkbutton(
            self.video_batch_frame,
            text="Force CPU-only mode",
            variable=self.video_cpu_var,
        )
        cpu_check.grid(row=4, column=0, columnspan=2, sticky="w", pady=(8, 0))
        if not self.hardware_profile.has_cuda:
            cpu_check.state(["disabled"])

        ttk.Label(
            self.video_batch_frame,
            text="Run the autonomy stack over a recorded clip and export an annotated video plus actuator CSV.",
            style="Body.TLabel",
            wraplength=640,
            justify=tk.LEFT,
        ).grid(row=5, column=0, columnspan=3, sticky="w", pady=(8, 0))

        self.video_start_button = ttk.Button(
            self.video_batch_frame, text="Process Video", command=self._start_video_batch
        )
        self.video_start_button.grid(row=6, column=0, pady=(16, 0), sticky="w")
        self.video_stop_button = ttk.Button(
            self.video_batch_frame,
            text="Cancel",
            command=self._stop_video_batch,
            state=tk.DISABLED,
        )
        self.video_stop_button.grid(row=6, column=1, pady=(16, 0), sticky="w")

        self.video_log = tk.Text(
            self.video_batch_frame,
            height=12,
            state=tk.DISABLED,
            wrap=tk.WORD,
        )
        self.video_log.grid(row=7, column=0, columnspan=3, pady=(16, 0), sticky="nsew")

        self.video_batch_frame.columnconfigure(1, weight=1)
        self.video_batch_frame.rowconfigure(7, weight=1)

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
        runtime_mode = "GPU-enabled" if self.hardware_profile.has_cuda else "CPU-only (no CUDA detected)"
        if hasattr(self, "cpu_only_var"):
            if not self.hardware_profile.has_cuda:
                runtime_mode = "CPU-only (no CUDA detected)"
            elif self.cpu_only_var.get():
                runtime_mode = "CPU-only (forced)"
        summary = (
            f"Detected Hardware:\n"
            f"  OS: {self.hardware_profile.os_name} {self.hardware_profile.os_version}\n"
            f"  CPU: {self.hardware_profile.cpu_model} ({self.hardware_profile.cpu_count} cores)\n"
            f"  Memory: {self.hardware_profile.total_memory_gb:.2f} GB\n"
            f"  GPU: {self.hardware_profile.gpu_name or 'None'}\n"
            f"  Tier: {self.hardware_profile.compute_tier.title()}\n"
            f"  Runtime Mode: {runtime_mode}"
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

    def _append_setup_log(self, message: str) -> None:
        self.after(0, lambda: self._write_log(self.setup_log, message))

    def _append_launch_log(self, message: str) -> None:
        self.after(0, lambda: self._write_log(self.launch_log, message))

    def _append_video_log(self, message: str) -> None:
        if not hasattr(self, "video_log"):
            return
        self.after(0, lambda: self._write_log(self.video_log, message))

    def _append_message(self, message: str) -> None:
        if not hasattr(self, "message_text"):
            return

        def writer() -> None:
            self.message_text.configure(state=tk.NORMAL)
            self.message_text.insert(tk.END, message + "\n")
            self.message_text.see(tk.END)
            self.message_text.configure(state=tk.DISABLED)

        self.after(0, writer)

    def _write_log(self, widget: tk.Text, message: str) -> None:
        widget.configure(state=tk.NORMAL)
        widget.insert(tk.END, message + "\n")
        widget.see(tk.END)
        widget.configure(state=tk.DISABLED)

    def _update_model_description(self) -> None:
        profile = MODEL_PROFILES[self.model_var.get()]
        text = (
            f"{profile.label}: {profile.description}\n"
            f"YOLO checkpoint: {profile.yolo_model}\n"
            f"Advisor models: {profile.advisor_image_model}, {profile.advisor_language_model}"
        )
        self.model_desc.configure(text=text)

    def _update_dependency_description(self) -> None:
        profile = DEPENDENCY_PROFILES[self.dependency_var.get()]
        text = f"{profile.label}: {profile.description}\n" + "\n".join(profile.requirements)
        self.dependency_desc.configure(text=text)

    def _update_advisor_profile_description(self) -> None:
        profile = ADVISOR_MODEL_PROFILES[self.advisor_model_var.get()]
        text = (
            f"{profile.label}: {profile.description}\n"
            f"Vision model: {profile.advisor_image_model}\n"
            f"Language model: {profile.advisor_language_model}"
        )
        self.advisor_model_desc.configure(text=text)

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
        self.app_state.advisor_model_profile = self.advisor_model_var.get()
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

    # ------------------------------------------------------------------
    # Launch tab actions
    def _start_pilot(self) -> None:
        if self.pilot_thread and self.pilot_thread.is_alive():
            messagebox.showwarning("Pilot", "Pilot is already running.")
            return

        try:
            width = int(self.width_var.get())
            height = int(self.height_var.get())
            fps = int(self.fps_var.get())
        except (TypeError, ValueError):
            messagebox.showerror("Invalid Input", "Resolution and FPS must be integers.")
            return

        try:
            vehicle_settings = self._read_vehicle_settings()
        except ValueError as exc:
            messagebox.showerror("Invalid Input", str(exc))
            return

        self.app_state.camera_source = self.camera_var.get()
        self.app_state.resolution_width = width
        self.app_state.resolution_height = height
        self.app_state.fps = fps
        self.app_state.enable_visualization = bool(self.visual_var.get())
        self.app_state.enable_advisor = bool(self.advisor_var.get())
        self.app_state.advisor_mode = self.advisor_mode_var.get()
        self.app_state.advisor_model_profile = self.advisor_model_var.get()
        self.app_state.safety_mindset = self.mindset_var.get()
        self.app_state.ambient_mode = self.ambient_var.get()
        self.app_state.persona = self.persona_var.get()
        self.app_state.force_cpu = bool(self.cpu_only_var.get())
        self.app_state.vehicle_description = self.vehicle_description_var.get().strip() or "Scooter"
        self.app_state.vehicle_width_m = vehicle_settings["width"]
        self.app_state.vehicle_length_m = vehicle_settings["length"]
        self.app_state.vehicle_height_m = vehicle_settings["height"]
        self.app_state.vehicle_clearance_margin_m = vehicle_settings["margin"]
        self.app_state.calibration_reference_distance_m = vehicle_settings["calibration_distance"]
        self.app_state.calibration_reference_pixels = vehicle_settings["calibration_pixels"]
        self.state_manager.save_state(self.app_state)

        model_profile = MODEL_PROFILES[self.model_var.get()]
        advisor_profile = ADVISOR_MODEL_PROFILES[self.advisor_model_var.get()]

        advisor_settings = AdvisorRuntimeConfig()
        mode = self.advisor_mode_var.get()
        if mode == "strict":
            advisor_settings.mode = "strict"
            advisor_settings.min_conf_for_allow = 0.7
            advisor_settings.ttc_block_s = 1.5
            advisor_settings.block_debounce_ms = 1000
            advisor_settings.timeout_grace_ticks = 0
        else:
            advisor_settings.mode = "normal"

        navigation_intent = NavigationIntentConfig()
        navigation_intent.ambient_mode = self.ambient_var.get() == "on"

        safety_mindset = SafetyMindsetConfig()
        safety_mindset.enabled = self.mindset_var.get() == "on"

        cpu_only = self.app_state.force_cpu
        if cpu_only:
            self._append_launch_log("CPU-only mode enabled; using CPU for all models.")
        elif not self.hardware_profile.has_cuda:
            self._append_launch_log("No CUDA GPU detected; models will run on CPU.")

        config = PilotConfig(
            camera_source=self.camera_var.get(),
            camera_width=width,
            camera_height=height,
            camera_fps=fps,
            model_name=model_profile.yolo_model,
            visualize=self.visual_var.get(),
            advisor_enabled=self.advisor_var.get(),
            advisor_image_model=advisor_profile.advisor_image_model,
            advisor_language_model=advisor_profile.advisor_language_model,
            advisor_device="cpu" if cpu_only else None,
            advisor=advisor_settings,
            navigation_intent=navigation_intent,
            safety_mindset=safety_mindset,
            safety_mindset_enabled=safety_mindset.enabled,
            companion_persona=self.persona_var.get(),
            vehicle_description=self.vehicle_description_var.get().strip() or "Scooter",
            vehicle_width_m=vehicle_settings["width"],
            vehicle_length_m=vehicle_settings["length"],
            vehicle_height_m=vehicle_settings["height"],
            vehicle_clearance_margin_m=vehicle_settings["margin"],
            calibration_reference_distance_m=vehicle_settings["calibration_distance"],
            calibration_reference_pixels=vehicle_settings["calibration_pixels"],
            force_cpu=cpu_only,
        )

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

    def _stop_pilot(self) -> None:
        if not self.pilot_thread:
            return
        self._append_launch_log("Stopping pilot...")
        self.pilot_thread.stop()
        self.pilot_thread = None

    # ------------------------------------------------------------------
    # Video analysis tab actions
    def _browse_video_input(self) -> None:
        path = filedialog.askopenfilename(
            title="Select video file",
            filetypes=[
                ("Video Files", "*.mp4 *.mov *.avi *.mkv *.m4v"),
                ("All Files", "*.*"),
            ],
        )
        if not path:
            return
        resolved = Path(path).expanduser()
        self.video_input_var.set(str(resolved))
        if not self.video_output_dir_var.get():
            self.video_output_dir_var.set(str(resolved.parent))
        current_prefix = self.video_output_prefix_var.get().strip()
        suggested = f"{resolved.stem}_analysis"
        if not current_prefix or current_prefix == "analysis":
            self.video_output_prefix_var.set(suggested)

    def _browse_video_output_dir(self) -> None:
        directory = filedialog.askdirectory(title="Select output directory")
        if directory:
            self.video_output_dir_var.set(directory)

    def _start_video_batch(self) -> None:
        if self.video_job and self.video_job.is_alive():
            messagebox.showwarning(
                "Video Analysis", "An offline analysis job is already running."
            )
            return

        source = self.video_input_var.get().strip()
        if not source:
            messagebox.showerror("Video Analysis", "Please choose a video file to process.")
            return

        input_path = Path(source).expanduser()
        if not input_path.exists():
            messagebox.showerror(
                "Video Analysis", f"Input video was not found: {input_path}"
            )
            return

        output_dir_str = self.video_output_dir_var.get().strip()
        if not output_dir_str:
            output_dir = input_path.parent
            self.video_output_dir_var.set(str(output_dir))
        else:
            output_dir = Path(output_dir_str).expanduser()

        try:
            output_dir.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            messagebox.showerror(
                "Video Analysis", f"Unable to create output directory: {exc}"
            )
            return

        prefix = self.video_output_prefix_var.get().strip()
        if not prefix:
            prefix = f"{input_path.stem}_analysis"
            self.video_output_prefix_var.set(prefix)

        output_video = output_dir / f"{prefix}_overlay.mp4"
        output_csv = output_dir / f"{prefix}_actuators.csv"
        log_dir = output_dir / f"{prefix}_logs"

        capture = cv2.VideoCapture(str(input_path))
        if not capture.isOpened():
            messagebox.showerror(
                "Video Analysis", f"Unable to open video source: {input_path}"
            )
            return

        try:
            width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
            height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
            fps_value = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
            frame_count_raw = capture.get(cv2.CAP_PROP_FRAME_COUNT)
            frame_count = (
                int(frame_count_raw)
                if frame_count_raw and frame_count_raw > 0
                else 0
            )
        finally:
            capture.release()

        if width <= 0 or height <= 0:
            width = int(self.app_state.resolution_width)
            height = int(self.app_state.resolution_height)
            self._append_video_log(
                "Using configured resolution because video metadata was unavailable."
            )

        if fps_value <= 0:
            fps_value = float(self.app_state.fps or 30)
            if fps_value <= 0:
                fps_value = 30.0
            self._append_video_log(
                "Using configured FPS because video metadata was unavailable."
            )

        camera_fps = max(1, int(round(fps_value)))

        try:
            vehicle_settings = self._read_vehicle_settings()
        except ValueError as exc:
            messagebox.showerror("Video Analysis", str(exc))
            return

        model_profile = MODEL_PROFILES[self.model_var.get()]
        advisor_profile = ADVISOR_MODEL_PROFILES[self.advisor_model_var.get()]

        advisor_settings = AdvisorRuntimeConfig()
        if self.advisor_mode_var.get() == "strict":
            advisor_settings.mode = "strict"
            advisor_settings.min_conf_for_allow = 0.7
            advisor_settings.ttc_block_s = 1.5
            advisor_settings.block_debounce_ms = 1000
            advisor_settings.timeout_grace_ticks = 0
        else:
            advisor_settings.mode = "normal"

        navigation_intent = NavigationIntentConfig()
        navigation_intent.ambient_mode = self.ambient_var.get() == "on"

        safety_mindset = SafetyMindsetConfig()
        safety_mindset.enabled = self.mindset_var.get() == "on"

        advisor_enabled = bool(self.advisor_var.get())
        cpu_only = bool(self.video_cpu_var.get()) or (not self.hardware_profile.has_cuda)

        self._append_video_log(f"Input: {input_path}")
        meta_line = f"Resolution: {width}x{height} @ {fps_value:.2f} FPS"
        if frame_count > 0:
            meta_line += f" (~{frame_count} frames)"
        self._append_video_log(meta_line)
        self._append_video_log(
            f"Outputs → video: {output_video.name}, actuators: {output_csv.name}"
        )
        if cpu_only:
            self._append_video_log("CPU-only mode enabled for offline analysis.")

        config = PilotConfig(
            camera_source=str(input_path),
            camera_width=width,
            camera_height=height,
            camera_fps=camera_fps,
            camera_auto_reconnect=False,
            model_name=model_profile.yolo_model,
            visualize=False,
            log_dir=log_dir,
            advisor_enabled=advisor_enabled,
            advisor_image_model=advisor_profile.advisor_image_model,
            advisor_language_model=advisor_profile.advisor_language_model,
            advisor_device="cpu" if cpu_only else None,
            advisor_state_path=None,
            command_state_path=None,
            command_file=None,
            advisor=advisor_settings,
            navigation_intent=navigation_intent,
            safety_mindset=safety_mindset,
            safety_mindset_enabled=safety_mindset.enabled,
            companion_persona=self.persona_var.get(),
            vehicle_description=self.vehicle_description_var.get().strip() or "Scooter",
            vehicle_width_m=vehicle_settings["width"],
            vehicle_length_m=vehicle_settings["length"],
            vehicle_height_m=vehicle_settings["height"],
            vehicle_clearance_margin_m=vehicle_settings["margin"],
            calibration_reference_distance_m=vehicle_settings["calibration_distance"],
            calibration_reference_pixels=vehicle_settings["calibration_pixels"],
            force_cpu=cpu_only,
        )

        self._latest_video_outputs = (output_video, output_csv)

        def notify(success: bool, error: Optional[str]) -> None:
            self.after(0, lambda s=success, e=error: self._on_video_job_complete(s, e))

        try:
            self.video_job = VideoBatchRunner(
                config=config,
                input_path=input_path,
                output_video=output_video,
                output_csv=output_csv,
                fps=fps_value,
                expected_frames=frame_count,
                log_callback=self._append_video_log,
                on_complete=notify,
            )
        except Exception as exc:
            self.video_job = None
            messagebox.showerror("Video Analysis", f"Unable to start analysis: {exc}")
            return

        self.video_start_button.configure(state=tk.DISABLED)
        self.video_stop_button.configure(state=tk.NORMAL)
        self._append_video_log("Starting offline analysis...")
        self.video_job.start()

    def _stop_video_batch(self) -> None:
        if not self.video_job:
            return
        self._append_video_log("Cancelling video analysis...")
        self.video_job.stop()
        self.video_stop_button.configure(state=tk.DISABLED)

    def _on_video_job_complete(self, success: bool, error: Optional[str]) -> None:
        self.video_job = None
        self.video_start_button.configure(state=tk.NORMAL)
        self.video_stop_button.configure(state=tk.DISABLED)

        if success:
            if self._latest_video_outputs:
                video_path, csv_path = self._latest_video_outputs
                self._append_video_log(
                    f"Saved overlay video to {video_path} and actuator log to {csv_path}"
                )
                messagebox.showinfo(
                    "Video Analysis",
                    f"Offline analysis complete.\nVideo: {video_path}\nActuators: {csv_path}",
                )
        elif error:
            messagebox.showerror("Video Analysis", f"Video processing failed: {error}")
        else:
            self._append_video_log("Video analysis cancelled.")

    def _prepare_video_image(self, frame: np.ndarray) -> Image.Image:
        height, width = frame.shape[:2]
        if height <= 0 or width <= 0:
            blank = np.zeros((self._video_max_height, self._video_max_width, 3), dtype=np.uint8)
            return Image.fromarray(blank)

        scale_w = self._video_max_width / float(width)
        scale_h = self._video_max_height / float(height)
        scale = min(scale_w, scale_h, 1.0)
        new_width = max(1, int(width * scale))
        new_height = max(1, int(height * scale))

        if new_width != width or new_height != height:
            resized = cv2.resize(
                frame,
                (new_width, new_height),
                interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR,
            )
        else:
            resized = frame

        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        return Image.fromarray(rgb)

    def _handle_pilot_tick(self, payload: PilotTickData) -> None:
        def update() -> None:
            self._last_tick_time = payload.timestamp

            if payload.overlay is not None:
                image = self._prepare_video_image(payload.overlay)
                self._video_photo = ImageTk.PhotoImage(image=image)
                self.video_label.configure(image=self._video_photo, text="")

            steer = payload.command.steer
            throttle = payload.command.throttle
            brake = payload.command.brake
            self.throttle_bar["value"] = max(0, min(100, throttle * 100.0))
            self.brake_bar["value"] = max(0, min(100, brake * 100.0))
            self.steer_value.set(f"{steer:+.2f}")

            if payload.review:
                reason = ", ".join(payload.review.reason_tags)
                status = f"{payload.review.verdict.value} ({reason})"
                verdict_signature = f"{payload.review.verdict.value}:{reason}"
                self.advisor_status.set(status)
                if verdict_signature != self._last_advisor_verdict:
                    self._append_message(f"Advisor: {status}")
                    self._last_advisor_verdict = verdict_signature
            else:
                self.advisor_status.set("Advisor idle")
                self._last_advisor_verdict = ""

            directive = payload.decision.directive.strip()
            if directive and directive != self._last_directive:
                self._append_message(f"Directive → {directive}")
                self._last_directive = directive
            elif not directive:
                self._last_directive = ""

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
            self._append_launch_log(f"Command sent to advisor: {text}")
        else:
            self._append_launch_log("Command queued but pilot is not running.")


def launch_app() -> None:  # pragma: no cover - UI entry point
    app = ScooterApp()
    app.mainloop()
