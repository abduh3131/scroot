from __future__ import annotations

import glob
import json
import logging
import platform
import time
from pathlib import Path
from typing import Optional

import serial

from autonomy.utils.data_structures import ActuatorCommand


class ActuatorOutput:
    """Writes actuator commands to disk and streams them to a USB MCU.

    The class keeps a persistent serial handle so low-power MCUs on Linux or
    Windows receive a continuous stream of normalized actuator values while the
    same data is mirrored to a JSONL file for debugging.
    """

    def __init__(
        self,
        log_path: Path | str | None = None,
        serial_device: str | None = None,
        baud_rate: int = 115200,
        auto_discover_serial: bool = False,
    ) -> None:
        self.log_path = Path(log_path) if log_path else None
        self.serial_device = serial_device
        self.baud_rate = baud_rate
        self._serial: Optional[serial.Serial] = None
        self._auto_discover = auto_discover_serial

        if self.log_path:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)

        if self.serial_device is None and self._auto_discover:
            self.serial_device = self._detect_serial_device()

    @staticmethod
    def _detect_serial_device() -> Optional[str]:
        """Best-effort discovery of a Rosmaster/MCU serial device on Linux."""

        if platform.system().lower() == "windows":
            return None

        candidates: list[str] = []
        for pattern in ("/dev/ttyTHS*", "/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyAMA*"):
            candidates.extend(glob.glob(pattern))

        if not candidates:
            logging.warning("No serial devices detected; continuing without MCU output")
            return None

        chosen = sorted(candidates)[0]
        logging.info("Auto-selected serial device %s for actuator streaming", chosen)
        return chosen

    def _ensure_serial(self) -> None:
        if self.serial_device is None:
            return
        if self._serial is not None and self._serial.is_open:
            return
        try:
            self._serial = serial.Serial(self.serial_device, self.baud_rate, timeout=0.1)
        except serial.SerialException as exc:
            logging.error("Unable to open serial device %s: %s", self.serial_device, exc)
            self._serial = None

    def publish(self, command: ActuatorCommand) -> None:
        """Persist the latest actuator command to disk and USB."""

        timestamp = time.time()
        payload = {
            "timestamp": timestamp,
            "steer": float(command.steer),
            "throttle": float(command.throttle),
            "brake": float(command.brake),
        }

        if self.log_path:
            with self.log_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(payload) + "\n")

        if self.serial_device:
            self._ensure_serial()
            if self._serial and self._serial.is_open:
                line = f"{payload['steer']:.3f},{payload['throttle']:.3f},{payload['brake']:.3f}\n"
                try:
                    self._serial.write(line.encode("utf-8"))
                except serial.SerialException as exc:
                    logging.error("Failed to write to %s: %s", self.serial_device, exc)

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None
