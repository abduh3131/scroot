from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, TYPE_CHECKING

from autonomy.utils.data_structures import NavigationDecision, VehicleMode

if TYPE_CHECKING:  # pragma: no cover
    from autonomy.safety.guardian import GuardianDecision


@dataclass
class ModeManagerConfig:
    """Configuration values controlling mode-specific speed limits."""

    cruise_speed_limit: float = 4.5  # m/s (~16 km/h)
    summon_speed_limit: float = 3.0  # m/s
    idle_speed_limit: float = 0.0


@dataclass
class ModeContext:
    """Result of applying guardian feedback and current mode to a navigation decision."""

    mode: VehicleMode
    desired_speed: float
    force_stop: bool
    alerts: List[str] = field(default_factory=list)


class ModeManager:
    """Coordinates high-level modes and enforces mode-specific constraints."""

    def __init__(
        self,
        config: ModeManagerConfig | None = None,
        initial_mode: VehicleMode = VehicleMode.CRUISE,
    ) -> None:
        self.config = config or ModeManagerConfig()
        self._mode = initial_mode
        self._requested_mode = initial_mode

    @property
    def current_mode(self) -> VehicleMode:
        return self._mode

    @property
    def requested_mode(self) -> VehicleMode:
        return self._requested_mode

    def request_mode(self, mode: VehicleMode) -> None:
        if mode == self._requested_mode:
            return
        self._requested_mode = mode

    def apply(
        self,
        decision: NavigationDecision,
        guardian_decision: Optional["GuardianDecision"] = None,
    ) -> ModeContext:
        alerts: List[str] = []
        force_stop = decision.enforced_stop
        desired_speed = decision.desired_speed
        mode = self._mode

        if guardian_decision:
            alerts.extend(guardian_decision.alerts)
            if guardian_decision.force_stop:
                mode = VehicleMode.EMERGENCY_STOP
                desired_speed = 0.0
                force_stop = True
            else:
                desired_speed *= guardian_decision.speed_scale

        if mode != VehicleMode.EMERGENCY_STOP or not force_stop:
            # Adopt requested mode when not in a forced stop.
            mode = self._requested_mode

        if mode == VehicleMode.IDLE:
            desired_speed = 0.0
        elif mode == VehicleMode.CRUISE:
            desired_speed = min(desired_speed, self.config.cruise_speed_limit)
        elif mode == VehicleMode.SUMMON:
            desired_speed = min(desired_speed, self.config.summon_speed_limit)
        elif mode == VehicleMode.EMERGENCY_STOP:
            desired_speed = 0.0
            force_stop = True

        self._mode = mode

        return ModeContext(
            mode=mode,
            desired_speed=max(0.0, desired_speed),
            force_stop=force_stop,
            alerts=alerts,
        )
