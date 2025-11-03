"""Serial GPS reader that parses NMEA sentences from an external dongle."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional

from autonomy.utils.data_structures import GPSFix

try:  # pragma: no cover - optional dependency during setup
    import serial  # type: ignore
except Exception:  # pragma: no cover
    serial = None  # type: ignore


@dataclass(slots=True)
class GPSConfig:
    port: Optional[str] = None
    baudrate: int = 9600
    enabled: bool = False


class GPSModule:
    """Background reader that exposes the most recent GPS fix."""

    def __init__(self, config: GPSConfig | None = None) -> None:
        self.config = config or GPSConfig()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._fix: Optional[GPSFix] = None
        self._serial: Optional["serial.Serial"] = None

    def start(self) -> None:
        if not self.config.enabled:
            logging.debug("GPS module start requested while disabled; ignoring.")
            return
        if not self.config.port:
            logging.warning("GPS module enabled but no serial port configured; skipping start.")
            return
        if serial is None:
            raise RuntimeError(
                "pyserial is required for GPS integration. Install it with 'pip install pyserial'."
            )
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="gps-reader", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:  # pragma: no cover - serial close errors ignored
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def latest_fix(self) -> Optional[GPSFix]:
        with self._lock:
            return self._fix

    # ------------------------------------------------------------------
    # Internal helpers
    def _run(self) -> None:
        try:
            self._serial = serial.Serial(self.config.port, self.config.baudrate, timeout=1.0)
        except Exception as exc:  # pragma: no cover - hardware specific
            logging.error("Failed to open GPS serial port %s: %s", self.config.port, exc)
            self._stop.set()
            return

        while not self._stop.is_set():
            try:
                line = self._serial.readline().decode("ascii", errors="ignore").strip()
            except Exception:  # pragma: no cover - IO specific
                time.sleep(0.5)
                continue
            if not line or not line.startswith("$"):
                continue
            fix = self._parse_nmea(line)
            if fix:
                with self._lock:
                    self._fix = fix

    def _parse_nmea(self, sentence: str) -> Optional[GPSFix]:
        parts = sentence.split(",")
        if len(parts) < 6:
            return None
        tag = parts[0][3:]
        if tag == "RMC":
            return self._parse_rmc(parts)
        if tag == "GGA":
            fix = self._parse_gga(parts)
            if fix:
                return fix
        return None

    def _parse_rmc(self, parts) -> Optional[GPSFix]:
        try:
            status = parts[2]
            if status != "A":
                return None
            latitude = self._parse_coordinate(parts[3], parts[4])
            longitude = self._parse_coordinate(parts[5], parts[6])
            speed_knots = float(parts[7]) if parts[7] else 0.0
            course = float(parts[8]) if parts[8] else None
            timestamp = time.time()
        except (ValueError, IndexError):
            return None
        speed_mps = speed_knots * 0.514444
        return GPSFix(
            latitude=latitude,
            longitude=longitude,
            altitude_m=None,
            speed_mps=speed_mps,
            course_deg=course,
            timestamp=timestamp,
        )

    def _parse_gga(self, parts) -> Optional[GPSFix]:
        try:
            quality = int(parts[6])
            if quality == 0:
                return None
            latitude = self._parse_coordinate(parts[2], parts[3])
            longitude = self._parse_coordinate(parts[4], parts[5])
            altitude = float(parts[9]) if parts[9] else None
            timestamp = time.time()
        except (ValueError, IndexError):
            return None
        return GPSFix(
            latitude=latitude,
            longitude=longitude,
            altitude_m=altitude,
            speed_mps=None,
            course_deg=None,
            timestamp=timestamp,
        )

    def _parse_coordinate(self, value: str, direction: str) -> float:
        if not value:
            return 0.0
        degrees = float(value[:2]) if len(value) >= 4 else 0.0
        minutes = float(value[2:]) if len(value) > 2 else 0.0
        coordinate = degrees + minutes / 60.0
        if direction in {"S", "W"}:
            coordinate *= -1
        return coordinate
