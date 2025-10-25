"""Tkinter-based GUI for configuring and launching the scooter autonomy stack."""

from __future__ import annotations

import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Callable, Optional

from autonomy.app.environment import EnvironmentPlan, install_dependencies
from autonomy.app.hardware import detect_hardware
from autonomy.app.profiles import DEPENDENCY_PROFILES, MODEL_PROFILES, recommend_profiles
from autonomy.app.state import AppState, AppStateManager
from autonomy.control.config import AdvisorRuntimeConfig, NavigationIntentConfig, SafetyMindsetConfig
from autonomy.pilot import AutonomyPilot, PilotConfig


class PilotRunner(threading.Thread):
    """Background thread that runs the autonomy pilot."""

    def __init__(
        self,
        config: PilotConfig,
        log_callback: Callable[[str], None],
        on_stop: Callable[[], None],
    ) -> None:
        super().__init__(daemon=True)
        self.config = config
        self.log_callback = log_callback
        self.on_stop = on_stop
        self._stop_event = threading.Event()
        self._pilot: Optional[AutonomyPilot] = None

    def stop(self) -> None:
        self._stop_event.set()
        if self._pilot:
            self._pilot.stop()

    def run(self) -> None:  # pragma: no cover - interacts with hardware
        try:
            self._pilot = AutonomyPilot(self.config)
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
        if stored_state is None:
            self.state_manager.save_state(self.app_state)

        self.pilot_thread: Optional[PilotRunner] = None

        self._build_ui()
        self._render_hardware_summary()
        self._update_model_description()
        self._update_dependency_description()

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

        ttk.Label(self.setup_frame, text="Model Intensity", style="Headline.TLabel").grid(
            row=2, column=0, sticky="w"
        )

        self.model_var = tk.StringVar(value=self.app_state.model_profile)
        self.model_combo = ttk.Combobox(
            self.setup_frame,
            textvariable=self.model_var,
            values=list(MODEL_PROFILES.keys()),
            state="readonly",
        )
        self.model_combo.grid(row=3, column=0, sticky="w")
        self.model_combo.bind("<<ComboboxSelected>>", lambda _event: self._update_model_description())

        self.model_desc = ttk.Label(self.setup_frame, style="Body.TLabel", wraplength=500, justify=tk.LEFT)
        self.model_desc.grid(row=3, column=1, columnspan=3, sticky="w", padx=16)

        ttk.Label(self.setup_frame, text="Dependency Stack", style="Headline.TLabel").grid(
            row=4, column=0, sticky="w", pady=(20, 0)
        )

        self.dependency_var = tk.StringVar(value=self.app_state.dependency_profile)
        self.dependency_combo = ttk.Combobox(
            self.setup_frame,
            textvariable=self.dependency_var,
            values=list(DEPENDENCY_PROFILES.keys()),
            state="readonly",
        )
        self.dependency_combo.grid(row=5, column=0, sticky="w")
        self.dependency_combo.bind("<<ComboboxSelected>>", lambda _event: self._update_dependency_description())

        self.dependency_desc = ttk.Label(
            self.setup_frame, style="Body.TLabel", wraplength=500, justify=tk.LEFT
        )
        self.dependency_desc.grid(row=5, column=1, columnspan=3, sticky="w", padx=16)

        self.install_button = ttk.Button(
            self.setup_frame,
            text="Install Dependencies",
            command=self._install_dependencies,
        )
        self.install_button.grid(row=6, column=0, pady=(24, 0), sticky="w")

        self.save_setup_button = ttk.Button(
            self.setup_frame,
            text="Save Setup",
            command=self._save_setup,
        )
        self.save_setup_button.grid(row=6, column=1, pady=(24, 0), sticky="w")

        self.setup_log = tk.Text(self.setup_frame, height=10, width=80, state=tk.DISABLED)
        self.setup_log.grid(row=7, column=0, columnspan=4, pady=(24, 0), sticky="nsew")

        self.setup_frame.rowconfigure(7, weight=1)
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

        ttk.Label(self.run_frame, text="Safety Mindset", style="Body.TLabel").grid(row=7, column=0, sticky="w")
        self.mindset_var = tk.StringVar(value=self.app_state.safety_mindset)
        ttk.Combobox(
            self.run_frame,
            textvariable=self.mindset_var,
            values=["off", "on"],
            state="readonly",
        ).grid(row=7, column=1, sticky="w")

        ttk.Label(self.run_frame, text="Ambient Mode", style="Body.TLabel").grid(row=8, column=0, sticky="w")
        self.ambient_var = tk.StringVar(value=self.app_state.ambient_mode)
        ttk.Combobox(
            self.run_frame,
            textvariable=self.ambient_var,
            values=["on", "off"],
            state="readonly",
        ).grid(row=8, column=1, sticky="w")

        ttk.Label(self.run_frame, text="Companion Persona", style="Body.TLabel").grid(row=9, column=0, sticky="w")
        self.persona_var = tk.StringVar(value=self.app_state.persona)
        ttk.Combobox(
            self.run_frame,
            textvariable=self.persona_var,
            values=["calm_safe", "smart_scout", "playful"],
            state="readonly",
        ).grid(row=9, column=1, sticky="w")

        self.start_button = ttk.Button(self.run_frame, text="Start Pilot", command=self._start_pilot)
        self.start_button.grid(row=10, column=0, pady=(24, 0), sticky="w")

        self.stop_button = ttk.Button(
            self.run_frame, text="Stop Pilot", command=self._stop_pilot, state=tk.DISABLED
        )
        self.stop_button.grid(row=10, column=1, pady=(24, 0), sticky="w")

        self.launch_log = tk.Text(self.run_frame, height=16, width=100, state=tk.DISABLED)
        self.launch_log.grid(row=11, column=0, columnspan=4, pady=(24, 0), sticky="nsew")

        self.run_frame.rowconfigure(11, weight=1)
        self.run_frame.columnconfigure(3, weight=1)

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

    def _save_setup(self) -> None:
        self._append_setup_log("Saving setup preferences...")
        self.app_state.model_profile = self.model_var.get()
        self.app_state.dependency_profile = self.dependency_var.get()
        self.state_manager.save_state(self.app_state)
        self._append_setup_log("Setup saved.")
        messagebox.showinfo("Setup", "Configuration saved successfully.")

    def _install_dependencies(self) -> None:
        dependency_profile = DEPENDENCY_PROFILES[self.dependency_var.get()]
        plan = EnvironmentPlan(dependency_profile=dependency_profile)

        self.install_button.configure(state=tk.DISABLED)
        self._append_setup_log("Starting dependency installation...")

        def worker() -> None:
            report = install_dependencies(plan, logger=self._append_setup_log)
            if report.error:
                self._append_setup_log(f"Installation failed, see log at {report.log_path}")
                self.after(0, lambda: messagebox.showerror("Install", report.error or "Installation failed"))
            else:
                self._append_setup_log("Installation succeeded.")
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

        self.app_state.camera_source = self.camera_var.get()
        self.app_state.resolution_width = width
        self.app_state.resolution_height = height
        self.app_state.fps = fps
        self.app_state.enable_visualization = bool(self.visual_var.get())
        self.app_state.enable_advisor = bool(self.advisor_var.get())
        self.app_state.advisor_mode = self.advisor_mode_var.get()
        self.app_state.safety_mindset = self.mindset_var.get()
        self.app_state.ambient_mode = self.ambient_var.get()
        self.app_state.persona = self.persona_var.get()
        self.state_manager.save_state(self.app_state)

        model_profile = MODEL_PROFILES[self.model_var.get()]

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
            advisor_image_model=model_profile.advisor_image_model,
            advisor_language_model=model_profile.advisor_language_model,
            advisor=advisor_settings,
            navigation_intent=navigation_intent,
            safety_mindset=safety_mindset,
            safety_mindset_enabled=safety_mindset.enabled,
            companion_persona=self.persona_var.get(),
        )

        self._append_launch_log("Launching autonomy pilot...")
        self.start_button.configure(state=tk.DISABLED)
        self.stop_button.configure(state=tk.NORMAL)

        def on_stop() -> None:
            self.after(0, lambda: self.start_button.configure(state=tk.NORMAL))
            self.after(0, lambda: self.stop_button.configure(state=tk.DISABLED))
            self._append_launch_log("Pilot stopped.")

        self.pilot_thread = PilotRunner(config, log_callback=self._append_launch_log, on_stop=on_stop)
        self.pilot_thread.start()

    def _stop_pilot(self) -> None:
        if not self.pilot_thread:
            return
        self._append_launch_log("Stopping pilot...")
        self.pilot_thread.stop()
        self.pilot_thread = None


def launch_app() -> None:  # pragma: no cover - UI entry point
    app = ScooterApp()
    app.mainloop()
