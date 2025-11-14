"""Tkinter-based GUI for configuring and launching the scooter autonomy stack."""

from __future__ import annotations

import threading
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk
from typing import Callable, Optional

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
        self._video_photo: Optional[ImageTk.PhotoImage] = None
        self._last_tick_time: float = 0.0
        self._last_advisor_verdict: str = ""
        self._last_directive: str = ""
        self._last_companion: str = ""

        self._build_ui()
        self._render_hardware_summary()
        self._update_model_description()
        self._update_dependency_description()
        self._update_advisor_profile_description()

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
            if self.app_state.advisor_model_profile != "light":
                self.app_state.advisor_model_profile = "light"
                tuned = True
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
            if not self.app_state.enable_advisor:
                self.app_state.enable_advisor = True
                tuned = True

        if tuned and not initial:
            self.state_manager.save_state(self.app_state)

    def _create_scrolling_text(self, parent: tk.Widget, **kwargs) -> scrolledtext.ScrolledText:
        widget = scrolledtext.ScrolledText(parent, **kwargs)
        self._bind_mousewheel(widget)
        return widget

    def _bind_mousewheel(self, widget: tk.Text) -> None:
        def _on_mousewheel(event: tk.Event) -> str:
            delta_units = 0
            if getattr(event, "delta", 0):
                delta_units = -1 if event.delta > 0 else 1
            elif getattr(event, "num", None) == 4:
                delta_units = -1
            elif getattr(event, "num", None) == 5:
                delta_units = 1
            if delta_units:
                widget.yview_scroll(delta_units, "units")
            return "break"

        widget.bind("<MouseWheel>", _on_mousewheel, add="+")
        widget.bind("<Button-4>", _on_mousewheel, add="+")
        widget.bind("<Button-5>", _on_mousewheel, add="+")

    def _build_ui(self) -> None:
        style = ttk.Style(self)
        style.configure("Headline.TLabel", font=("Segoe UI", 16, "bold"))
        style.configure("Body.TLabel", font=("Segoe UI", 11))

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        self.setup_frame = ttk.Frame(self.notebook, padding=20)
        self.run_frame = ttk.Frame(self.notebook, padding=20)
        self.notebook.add(self.setup_frame, text="Setup")
        self.notebook.add(self.run_frame, text="Launch")

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

        ttk.Label(self.run_frame, text="Advisor Mode", style="Body.TLabel").grid(
            row=6, column=0, sticky="w", pady=(8, 0)
        )
        self.advisor_mode_var = tk.StringVar(value=self.app_state.advisor_mode)
        ttk.Combobox(
            self.run_frame,
            textvariable=self.advisor_mode_var,
            values=["normal", "strict"],
            state="readonly",
        ).grid(row=6, column=1, sticky="w")

        ttk.Label(self.run_frame, text="Advisor Model", style="Body.TLabel").grid(row=7, column=0, sticky="w")
        self.advisor_model_var = tk.StringVar(value=self.app_state.advisor_model_profile)
        self.advisor_model_combo = ttk.Combobox(
            self.run_frame,
            textvariable=self.advisor_model_var,
            values=list(ADVISOR_MODEL_PROFILES.keys()),
            state="readonly",
        )
        self.advisor_model_combo.grid(row=7, column=1, sticky="w")
        self.advisor_model_combo.bind(
            "<<ComboboxSelected>>", lambda _event: self._update_advisor_profile_description()
        )
        self.advisor_model_desc = ttk.Label(
            self.run_frame, style="Body.TLabel", wraplength=400, justify=tk.LEFT
        )
        self.advisor_model_desc.grid(row=7, column=2, columnspan=2, sticky="w", padx=12)

        ttk.Label(self.run_frame, text="Safety Mindset", style="Body.TLabel").grid(row=8, column=0, sticky="w")
        self.mindset_var = tk.StringVar(value=self.app_state.safety_mindset)
        ttk.Combobox(
            self.run_frame,
            textvariable=self.mindset_var,
            values=["off", "on"],
            state="readonly",
        ).grid(row=8, column=1, sticky="w")

        ttk.Label(self.run_frame, text="Ambient Mode", style="Body.TLabel").grid(row=9, column=0, sticky="w")
        self.ambient_var = tk.StringVar(value=self.app_state.ambient_mode)
        ttk.Combobox(
            self.run_frame,
            textvariable=self.ambient_var,
            values=["on", "off"],
            state="readonly",
        ).grid(row=9, column=1, sticky="w")

        ttk.Label(self.run_frame, text="Companion Persona", style="Body.TLabel").grid(row=10, column=0, sticky="w")
        self.persona_var = tk.StringVar(value=self.app_state.persona)
        ttk.Combobox(
            self.run_frame,
            textvariable=self.persona_var,
            values=["calm_safe", "smart_scout", "playful"],
            state="readonly",
        ).grid(row=10, column=1, sticky="w")

        self.start_button = ttk.Button(self.run_frame, text="Start Pilot", command=self._start_pilot)
        self.start_button.grid(row=11, column=0, pady=(24, 0), sticky="w")

        self.stop_button = ttk.Button(
            self.run_frame, text="Stop Pilot", command=self._stop_pilot, state=tk.DISABLED
        )
        self.stop_button.grid(row=11, column=1, pady=(24, 0), sticky="w")

        self.video_frame = ttk.LabelFrame(self.run_frame, text="Live Camera & Advisor Overlay")
        self.video_frame.grid(row=12, column=0, columnspan=4, pady=(16, 0), sticky="nsew")
        self.video_label = ttk.Label(self.video_frame, text="Video feed will appear here", anchor="center")
        self.video_label.pack(fill=tk.BOTH, expand=True)

        telemetry_frame = ttk.Frame(self.run_frame)
        telemetry_frame.grid(row=13, column=0, columnspan=4, sticky="ew", pady=(12, 0))
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

        self.run_frame.rowconfigure(12, weight=3)
        self.run_frame.rowconfigure(14, weight=1)
        self.run_frame.rowconfigure(15, weight=1)
        self.run_frame.columnconfigure(3, weight=1)

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
        )

        if self._is_jetson_orin:
            config.advisor_device = config.advisor_device or "cuda"
            if self.model_var.get() == "jetson_orin":
                config.model_name = MODEL_PROFILES["jetson_orin"].yolo_model
            config.confidence_threshold = max(config.confidence_threshold, 0.35)
            config.iou_threshold = max(config.iou_threshold, 0.45)
            self._append_launch_log("Jetson Orin optimisations enabled (CUDA acceleration).")

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
