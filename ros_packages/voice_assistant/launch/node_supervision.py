"""Bounded respawn policy for supervised voice-assistant ROS nodes (PR-2070)."""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Optional

# One log line per respawn; cap attempts so a crash loop cannot spin forever.
DEFAULT_MAX_RESPAWN_ATTEMPTS = 5
_BACKOFF_SECONDS = (1.0, 2.0, 4.0, 8.0, 16.0)
# Requirement: schedule the next start within 5 s of the exit.
MAX_RESPAWN_DELAY_SECONDS = 5.0

HEALTH_OK = "ok"
HEALTH_RESPAWNING = "respawning"
HEALTH_GAVE_UP = "gave_up"

GAVE_UP_LOG_PREFIX = "voice launch supervision: gave up"


@dataclass
class RespawnDecision:
    """Whether to respawn, how long to wait, and the single supervision log line."""

    respawn: bool
    delay_seconds: float = 0.0
    log_line: str = ""
    health: str = HEALTH_OK


@dataclass
class RespawnState:
    attempts: int = 0
    health: str = HEALTH_OK
    last_exit_code: Optional[int] = None


class RespawnPolicy:
    """Tracks respawn attempts for one supervised node executable."""

    def __init__(
        self,
        node_name: str,
        *,
        max_attempts: int = DEFAULT_MAX_RESPAWN_ATTEMPTS,
    ):
        self.node_name = node_name
        self.max_attempts = max(1, int(max_attempts))
        self.state = RespawnState()

    def on_process_exit(self, returncode: int, is_shutdown: bool) -> RespawnDecision:
        """Decide whether to respawn after a child process exits."""
        if is_shutdown or returncode == 0:
            return RespawnDecision(respawn=False, health=HEALTH_OK)

        self.state.last_exit_code = returncode
        if self.state.attempts >= self.max_attempts:
            self.state.health = HEALTH_GAVE_UP
            line = (
                f"{GAVE_UP_LOG_PREFIX} {self.node_name} after "
                f"{self.state.attempts} respawns (exit {returncode})"
            )
            return RespawnDecision(
                respawn=False,
                log_line=line,
                health=HEALTH_GAVE_UP,
            )

        self.state.attempts += 1
        index = min(self.state.attempts - 1, len(_BACKOFF_SECONDS) - 1)
        delay = min(_BACKOFF_SECONDS[index], MAX_RESPAWN_DELAY_SECONDS)
        self.state.health = HEALTH_RESPAWNING
        line = (
            f"voice launch supervision: respawning {self.node_name} "
            f"attempt={self.state.attempts}/{self.max_attempts} "
            f"exit={returncode} delay_s={delay:.1f}"
        )
        return RespawnDecision(
            respawn=True,
            delay_seconds=delay,
            log_line=line,
            health=HEALTH_RESPAWNING,
        )


def max_respawn_attempts_from_env() -> int:
    raw = os.environ.get("PIB_VOICE_MAX_RESPAWNS", "").strip()
    if not raw:
        return DEFAULT_MAX_RESPAWN_ATTEMPTS
    try:
        return max(1, int(raw))
    except ValueError:
        return DEFAULT_MAX_RESPAWN_ATTEMPTS
