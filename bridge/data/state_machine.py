from __future__ import annotations
# state_machine.py — Bridge-Zustandsmaschine (unchanged)

import time
import enum
import logging
import threading
from typing import Callable, Optional

logger = logging.getLogger("egm.state")


class State(enum.Enum):
    INIT = "INIT"
    READY = "READY"
    RUN = "RUN"
    DEGRADED = "DEGRADED"
    STOP = "STOP"
    FAULT = "FAULT"


VALID_TRANSITIONS = {
    State.INIT: {State.READY, State.FAULT},
    State.READY: {State.RUN, State.STOP, State.FAULT},
    State.RUN: {State.DEGRADED, State.STOP, State.FAULT},
    State.DEGRADED: {State.RUN, State.STOP, State.FAULT},
    State.STOP: {State.READY, State.FAULT},
    State.FAULT: {State.INIT},
}


class StateChangeEvent:
    __slots__ = ("timestamp", "from_state", "to_state", "reason", "details")

    def __init__(self, from_state: State, to_state: State,
                 reason: str, details: Optional[dict] = None):
        self.timestamp = time.monotonic()
        self.from_state = from_state
        self.to_state = to_state
        self.reason = reason
        self.details = details or {}

    def __repr__(self):
        return (f"StateChange({self.from_state.value} → {self.to_state.value}"
                f" | {self.reason})")


class BridgeStateMachine:
    def __init__(self):
        self._state = State.INIT
        self._lock = threading.Lock()
        self._history: list[StateChangeEvent] = []
        self._listeners: list[Callable[[StateChangeEvent], None]] = []
        self._max_history = 500

    @property
    def state(self) -> State:
        return self._state

    @property
    def is_running(self) -> bool:
        return self._state in (State.RUN, State.DEGRADED)

    @property
    def is_operational(self) -> bool:
        return self._state in (State.READY, State.RUN, State.DEGRADED)

    @property
    def history(self) -> list[StateChangeEvent]:
        with self._lock:
            return list(self._history)

    def add_listener(self, callback: Callable[[StateChangeEvent], None]):
        self._listeners.append(callback)

    def remove_listener(self, callback: Callable[[StateChangeEvent], None]):
        self._listeners.remove(callback)

    def transition(self, target: State, reason: str,
                   details: Optional[dict] = None) -> bool:
        with self._lock:
            current = self._state
            if target == current:
                return True
            if target not in VALID_TRANSITIONS.get(current, set()):
                logger.error("STATE: Ungültiger Übergang %s → %s (%s)",
                             current.value, target.value, reason)
                return False
            event = StateChangeEvent(current, target, reason, details)
            self._state = target
            self._history.append(event)
            if len(self._history) > self._max_history:
                self._history = self._history[-self._max_history:]

        logger.info("STATE: %s → %s | %s", current.value, target.value,
                     reason)
        for listener in self._listeners:
            try:
                listener(event)
            except Exception as e:
                logger.error("STATE: Listener-Fehler: %s", e)
        return True

    def to_ready(self, reason: str = "Verbindungen hergestellt"):
        return self.transition(State.READY, reason)

    def to_run(self, reason: str = "Job gestartet"):
        return self.transition(State.RUN, reason)

    def to_degraded(self, reason: str = "Sync-Problem", details=None):
        return self.transition(State.DEGRADED, reason, details)

    def to_stop(self, reason: str = "Geordneter Halt"):
        return self.transition(State.STOP, reason)

    def to_fault(self, reason: str = "Schwerer Fehler", details=None):
        return self.transition(State.FAULT, reason, details)

    def reset(self, reason: str = "Manueller Reset"):
        return self.transition(State.INIT, reason)

    def require_state(self, *allowed: State):
        def decorator(func):
            def wrapper(*args, **kwargs):
                if self._state not in allowed:
                    raise RuntimeError(
                        f"{func.__name__} benötigt Zustand "
                        f"{[s.value for s in allowed]}, "
                        f"aktuell: {self._state.value}")
                return func(*args, **kwargs)
            return wrapper
        return decorator

    def snapshot(self) -> dict:
        with self._lock:
            last = self._history[-1] if self._history else None
        return {
            "state": self._state.value,
            "last_change": repr(last) if last else None,
            "transition_count": len(self._history),
        }
