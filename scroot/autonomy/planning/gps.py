"""GPS-assisted navigation utilities."""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Optional, Tuple

from autonomy.utils.data_structures import GPSFix, RoutePlan

try:  # pragma: no cover - optional dependency
    from geopy.distance import geodesic  # type: ignore
    from geopy.geocoders import Nominatim  # type: ignore
except Exception:  # pragma: no cover
    geodesic = None  # type: ignore
    Nominatim = None  # type: ignore


@dataclass(slots=True)
class Destination:
    label: str
    latitude: float
    longitude: float


class GPSRoutePlanner:
    """Resolves addresses to coordinates and generates coarse route guidance."""

    def __init__(self) -> None:
        self._destination: Optional[Destination] = None
        self._geolocator = Nominatim(user_agent="scooter_pilot") if Nominatim else None
        if self._geolocator is None:
            logging.debug(
                "geopy not available; GPS planner will accept raw coordinates only."
            )

    def has_destination(self) -> bool:
        return self._destination is not None

    def set_destination(self, address: str) -> bool:
        if not address:
            self._destination = None
            return False
        if self._geolocator is not None:
            try:
                location = self._geolocator.geocode(address, timeout=10)
            except Exception:  # pragma: no cover - network error
                logging.warning("Failed to resolve address '%s' via geopy", address)
            else:
                if location is not None:
                    self._destination = Destination(
                        address, location.latitude, location.longitude
                    )
                    return True
                logging.warning("No results returned for address '%s'", address)

        coords = self._parse_coordinate_text(address)
        if coords:
            latitude, longitude = coords
            label = address if address else f"{latitude:.5f},{longitude:.5f}"
            self._destination = Destination(label, latitude, longitude)
            return True

        logging.warning(
            "GPS destination '%s' could not be resolved. Install geopy for geocoding or "
            "enter coordinates as 'lat,lon'.",
            address,
        )
        return False

    def clear_destination(self) -> None:
        self._destination = None

    def destination_label(self) -> Optional[str]:
        return self._destination.label if self._destination else None

    def plan(self, fix: GPSFix) -> Optional[RoutePlan]:
        if self._destination is None:
            return None
        start = (fix.latitude, fix.longitude)
        end = (self._destination.latitude, self._destination.longitude)
        if geodesic is None:
            distance_m = self._haversine_distance(start, end)
        else:
            try:
                distance_m = float(geodesic(start, end).meters)
            except Exception:  # pragma: no cover - geodesic failure
                logging.warning("Geodesic distance computation failed; falling back to haversine.")
                distance_m = self._haversine_distance(start, end)

        bearing = self._bearing(start, end)
        eta = None
        if fix.speed_mps and fix.speed_mps > 0.5:
            eta = distance_m / fix.speed_mps

        return RoutePlan(
            destination_label=self._destination.label,
            target_latitude=self._destination.latitude,
            target_longitude=self._destination.longitude,
            distance_m=distance_m,
            bearing_deg=bearing,
            eta_s=eta,
        )

    @staticmethod
    def _bearing(start: tuple[float, float], end: tuple[float, float]) -> float:
        lat1 = math.radians(start[0])
        lat2 = math.radians(end[0])
        diff_long = math.radians(end[1] - start[1])
        x = math.sin(diff_long) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(diff_long)
        initial_bearing = math.atan2(x, y)
        initial_bearing = math.degrees(initial_bearing)
        return (initial_bearing + 360) % 360

    @staticmethod
    def _parse_coordinate_text(address: str) -> Optional[Tuple[float, float]]:
        cleaned = address.strip()
        if not cleaned:
            return None
        compact = cleaned.replace(" ", "")
        if "," in compact:
            parts = [p for p in compact.split(",") if p]
            if len(parts) == 2:
                try:
                    return float(parts[0]), float(parts[1])
                except ValueError:
                    return None
        parts = cleaned.split()
        if len(parts) == 2:
            try:
                return float(parts[0]), float(parts[1])
            except ValueError:
                return None
        return None

    @staticmethod
    def _haversine_distance(start: tuple[float, float], end: tuple[float, float]) -> float:
        radius = 6371000.0  # meters
        lat1, lon1 = map(math.radians, start)
        lat2, lon2 = map(math.radians, end)
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return radius * c
