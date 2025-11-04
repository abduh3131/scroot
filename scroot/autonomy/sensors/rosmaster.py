from __future__ import annotations

import json
import logging
import time
from typing import Dict, Iterator, Optional

from .base import SensorSample, StreamingSensor, global_sensor_registry

LOG = logging.getLogger(__name__)

try:  # pragma: no cover - optional dependency
    import serial  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    serial = None  # type: ignore


class RosmasterSensor(StreamingSensor):
    """Serial bridge that ingests sensor packets emitted by the Yahboom Rosmaster X3."""

    def __init__(
        self,
        port: str = "/dev/ttyTHS1",
        baudrate: int = 115200,
        timeout: float = 0.5,
        frame_rate: float = 30.0,
        auto_reconnect: bool = True,
        reconnect_delay: float = 0.5,
        name: Optional[str] = None,
    ) -> None:
        super().__init__(name or "rosmaster")
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.frame_interval = 1.0 / max(frame_rate, 1e-3)
        self.auto_reconnect = auto_reconnect
        self.reconnect_delay = max(reconnect_delay, 0.1)
        self._serial: Optional["serial.Serial"] = None  # type: ignore[name-defined]

    def start(self) -> None:
        if self._serial is not None:
            return
        if serial is None:  # pragma: no cover - optional dependency
            raise RuntimeError("pyserial is required for the RosmasterSensor (pip install pyserial)")
        self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)  # type: ignore[arg-type]
        LOG.info("Rosmaster serial link opened on %s @ %d baud", self.port, self.baudrate)
        self._serial.reset_input_buffer()

    def stop(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:  # pragma: no cover - best effort cleanup
                LOG.debug("Rosmaster serial close raised", exc_info=True)
            finally:
                self._serial = None
                LOG.info("Rosmaster serial link on %s closed", self.port)

    def read(self) -> SensorSample:
        if self._serial is None:
            self.start()
        assert self._serial is not None
        try:
            raw = self._serial.readline()
        except Exception as exc:  # pragma: no cover - serial transport errors
            LOG.error("Rosmaster read failed: %s", exc)
            self._handle_disconnect()
            return self._make_sample(data=None, ok=False, metadata=self._metadata_base("read_error"))

        if not raw:
            return self._make_sample(data=None, ok=False, metadata=self._metadata_base("timeout"))

        text = raw.decode("utf-8", errors="replace").strip()
        payload = self._parse_payload(text)
        metadata = self._metadata_base("ok" if payload is not None else "parse_error")
        metadata["raw"] = text
        return self._make_sample(data=payload, ok=payload is not None, metadata=metadata)

    def stream(self) -> Iterator[SensorSample]:
        while True:
            sample = self.read()
            yield sample
            time.sleep(self.frame_interval)

    def _handle_disconnect(self) -> None:
        if not self.auto_reconnect:
            return
        self.stop()
        time.sleep(self.reconnect_delay)
        try:
            self.start()
        except Exception as exc:  # pragma: no cover - reconnection failure
            LOG.error("Rosmaster reconnect failed: %s", exc)

    def _parse_payload(self, text: str) -> Optional[Dict[str, float]]:
        if not text:
            return None
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            parsed = self._parse_key_value_csv(text)
        if isinstance(parsed, dict):
            return {str(key): self._coerce_number(value) for key, value in parsed.items()}
        if isinstance(parsed, list):
            enumerated = {f"value_{idx}": self._coerce_number(value) for idx, value in enumerate(parsed)}
            return enumerated or None
        return None

    def _parse_key_value_csv(self, text: str) -> Dict[str, float]:
        result: Dict[str, float] = {}
        for token in text.split(","):
            if ":" not in token:
                continue
            key, raw_value = token.split(":", 1)
            key = key.strip()
            if not key:
                continue
            result[key] = self._coerce_number(raw_value.strip())
        if result:
            return result
        values = [value.strip() for value in text.split(",") if value.strip()]
        return {f"value_{idx}": self._coerce_number(value) for idx, value in enumerate(values)}

    def _coerce_number(self, value: object) -> float:
        if isinstance(value, (int, float)):
            return float(value)
        text = str(value).strip()
        for cast in (int, float):
            try:
                return float(cast(text))
            except (TypeError, ValueError):
                continue
        return float("nan")

    def _metadata_base(self, status: str) -> Dict[str, object]:
        return {
            "port": self.port,
            "baudrate": self.baudrate,
            "status": status,
        }


global_sensor_registry.register(
    "rosmaster",
    lambda **kwargs: RosmasterSensor(**kwargs),
    "Serial bridge for Yahboom Rosmaster X3 sensor packets",
)
