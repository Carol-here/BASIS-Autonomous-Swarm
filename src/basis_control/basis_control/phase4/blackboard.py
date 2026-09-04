from dataclasses import dataclass, field
from threading import RLock
from typing import Any, Dict, List, Optional
import copy
import time


@dataclass
class Blackboard:
    """
    Shared world model for the BASIS swarm.

    The Blackboard stores knowledge about:
    - drones
    - tasks
    - detections
    - events
    - mission state

    It is intentionally independent of ROS 2 and PX4.
    """

    drones: Dict[str, Any] = field(default_factory=dict)
    tasks: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    detections: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    events: List[Dict[str, Any]] = field(default_factory=list)
    mission: Dict[str, Any] = field(
        default_factory=lambda: {
            "status": "IDLE",
            "active_task": None,
            "started_at": None,
        }
    )

    def __post_init__(self):
        self._lock = RLock()

    # ============================================================
    # DRONES
    # ============================================================

    def update_drone(self, drone_id: str, state: Any) -> None:
        """
        Add or replace the current state of a drone.
        """

        with self._lock:
            self.drones[drone_id] = copy.deepcopy(state)

    def get_drone(self, drone_id: str) -> Optional[Any]:
        """
        Return a copy of one drone's current state.
        """

        with self._lock:
            state = self.drones.get(drone_id)

            if state is None:
                return None

            return copy.deepcopy(state)

    def get_all_drones(self) -> Dict[str, Any]:
        """
        Return a consistent copy of all drone states.
        """

        with self._lock:
            return copy.deepcopy(self.drones)

    # ============================================================
    # TASKS
    # ============================================================

    def add_task(self, task_id: str, task: Dict[str, Any]) -> None:
        """
        Add a new task to the Blackboard.
        """

        with self._lock:
            self.tasks[task_id] = copy.deepcopy(task)

    def update_task(
        self,
        task_id: str,
        updates: Dict[str, Any],
    ) -> bool:
        """
        Update an existing task.

        Returns:
            True  -> task existed and was updated
            False -> task did not exist
        """

        with self._lock:
            if task_id not in self.tasks:
                return False

            self.tasks[task_id].update(copy.deepcopy(updates))
            return True

    def get_tasks(self) -> Dict[str, Dict[str, Any]]:
        """
        Return a consistent copy of all tasks.
        """

        with self._lock:
            return copy.deepcopy(self.tasks)

    # ============================================================
    # DETECTIONS
    # ============================================================

    def add_detection(
        self,
        detection_id: str,
        detection: Dict[str, Any],
    ) -> None:
        """
        Add a perception/detection result.
        """

        with self._lock:
            self.detections[detection_id] = copy.deepcopy(detection)

    def get_detections(self) -> Dict[str, Dict[str, Any]]:
        """
        Return all known detections.
        """

        with self._lock:
            return copy.deepcopy(self.detections)

    # ============================================================
    # EVENTS
    # ============================================================

    def add_event(self, event: Dict[str, Any]) -> None:
        """
        Add an event to the Blackboard.

        A timestamp is automatically added if one is not supplied.
        """

        with self._lock:
            event_copy = copy.deepcopy(event)

            if "timestamp" not in event_copy:
                event_copy["timestamp"] = time.time()

            self.events.append(event_copy)

    def get_events(self) -> List[Dict[str, Any]]:
        """
        Return a copy of all events.
        """

        with self._lock:
            return copy.deepcopy(self.events)

    # ============================================================
    # MISSION
    # ============================================================

    def update_mission(self, updates: Dict[str, Any]) -> None:
        """
        Update mission-level state.
        """

        with self._lock:
            self.mission.update(copy.deepcopy(updates))

    def get_mission(self) -> Dict[str, Any]:
        """
        Return the current mission state.
        """

        with self._lock:
            return copy.deepcopy(self.mission)

    # ============================================================
    # SNAPSHOT
    # ============================================================

    def get_snapshot(self) -> Dict[str, Any]:
        """
        Return a consistent snapshot of the entire world model.

        Agents will eventually use this method to reason about
        the current swarm state.
        """

        with self._lock:
            return {
                "timestamp": time.time(),
                "drones": copy.deepcopy(self.drones),
                "tasks": copy.deepcopy(self.tasks),
                "detections": copy.deepcopy(self.detections),
                "events": copy.deepcopy(self.events),
                "mission": copy.deepcopy(self.mission),
            }

    # ============================================================
    # HEALTH
    # ============================================================

    def get_health(self) -> Dict[str, int]:
        """
        Return basic Blackboard statistics.
        """

        with self._lock:
            return {
                "drones": len(self.drones),
                "tasks": len(self.tasks),
                "detections": len(self.detections),
                "events": len(self.events),
            }
