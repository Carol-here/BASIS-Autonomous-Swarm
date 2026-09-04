from dataclasses import dataclass, field
from typing import Dict, Optional
import math


@dataclass
class DroneState:
    """Normalized state representation for one PX4 drone."""

    drone_id: int

    connected: bool = False

    # Position in local NED frame
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    # Velocity in local NED frame
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    # PX4 state
    armed: bool = False
    nav_state: int = 0
    failsafe: bool = False

    # Timing
    state_age: float = float("inf")

    # Derived health information
    healthy: bool = False

    @property
    def altitude(self) -> float:
        """Altitude above local origin in meters."""
        return -self.z

    @property
    def speed(self) -> float:
        """3D ground/vertical speed magnitude."""
        return math.sqrt(
            self.vx ** 2 +
            self.vy ** 2 +
            self.vz ** 2
        )

    def update_health(self, stale_threshold: float = 2.0) -> None:
        """Update derived health status."""
        self.healthy = (
            self.connected
            and not self.failsafe
            and self.state_age <= stale_threshold
        )


@dataclass
class SwarmState:
    """Global normalized state representation for the entire swarm."""

    drones: Dict[int, DroneState] = field(default_factory=dict)

    # Timestamp of latest swarm update
    timestamp: float = 0.0

    # ------------------------------------------------------------------
    # Drone management
    # ------------------------------------------------------------------

    def add_drone(self, drone_id: int) -> DroneState:
        """Create and register a drone if it does not already exist."""

        if drone_id not in self.drones:
            self.drones[drone_id] = DroneState(
                drone_id=drone_id
            )

        return self.drones[drone_id]

    def get_drone(self, drone_id: int) -> Optional[DroneState]:
        """Return a drone state."""
        return self.drones.get(drone_id)

    # ------------------------------------------------------------------
    # Swarm statistics
    # ------------------------------------------------------------------

    @property
    def total_drones(self) -> int:
        return len(self.drones)

    @property
    def connected_drones(self) -> int:
        return sum(
            1 for drone in self.drones.values()
            if drone.connected
        )

    @property
    def armed_drones(self) -> int:
        return sum(
            1 for drone in self.drones.values()
            if drone.armed
        )

    @property
    def failsafe_drones(self) -> int:
        return sum(
            1 for drone in self.drones.values()
            if drone.failsafe
        )

    @property
    def healthy_drones(self) -> int:
        return sum(
            1 for drone in self.drones.values()
            if drone.healthy
        )

    @property
    def stale_drones(self) -> int:
        return sum(
            1 for drone in self.drones.values()
            if drone.state_age > 2.0
        )

    # ------------------------------------------------------------------
    # Swarm geometry
    # ------------------------------------------------------------------

    @property
    def centroid(self):
        """Return the 3D centroid of connected drones."""

        connected = [
            drone
            for drone in self.drones.values()
            if drone.connected
        ]

        if not connected:
            return (0.0, 0.0, 0.0)

        x = sum(drone.x for drone in connected) / len(connected)
        y = sum(drone.y for drone in connected) / len(connected)
        z = sum(drone.z for drone in connected) / len(connected)

        return (x, y, z)

    @property
    def minimum_separation(self) -> float:
        """
        Minimum 3D distance between any two connected drones.

        Returns infinity when fewer than two drones are connected.
        """

        connected = [
            drone
            for drone in self.drones.values()
            if drone.connected
        ]

        if len(connected) < 2:
            return float("inf")

        minimum = float("inf")

        for i in range(len(connected)):
            for j in range(i + 1, len(connected)):

                a = connected[i]
                b = connected[j]

                dx = a.x - b.x
                dy = a.y - b.y
                dz = a.z - b.z

                distance = math.sqrt(
                    dx ** 2 +
                    dy ** 2 +
                    dz ** 2
                )

                minimum = min(minimum, distance)

        return minimum

    # ------------------------------------------------------------------
    # Swarm health
    # ------------------------------------------------------------------

    @property
    def healthy(self) -> bool:
        """True when every registered drone is connected and healthy."""

        if not self.drones:
            return False

        return self.healthy_drones == self.total_drones

    def update_health(self) -> None:
        """Update derived health for every drone."""

        for drone in self.drones.values():
            drone.update_health()

    # ------------------------------------------------------------------
    # Serialization
    # ------------------------------------------------------------------

    def to_dict(self) -> dict:
        """Convert the swarm state into a Blackboard-friendly dictionary."""

        centroid = self.centroid

        return {
            "timestamp": self.timestamp,

            "summary": {
                "total_drones": self.total_drones,
                "connected": self.connected_drones,
                "armed": self.armed_drones,
                "failsafe": self.failsafe_drones,
                "healthy": self.healthy_drones,
                "stale": self.stale_drones,
            },

            "centroid": {
                "x": centroid[0],
                "y": centroid[1],
                "z": centroid[2],
            },

            "minimum_separation": self.minimum_separation,

            "swarm_healthy": self.healthy,

            "drones": {
                drone_id: {
                    "connected": drone.connected,
                    "position": {
                        "x": drone.x,
                        "y": drone.y,
                        "z": drone.z,
                    },
                    "velocity": {
                        "vx": drone.vx,
                        "vy": drone.vy,
                        "vz": drone.vz,
                    },
                    "altitude": drone.altitude,
                    "speed": drone.speed,
                    "armed": drone.armed,
                    "nav_state": drone.nav_state,
                    "failsafe": drone.failsafe,
                    "state_age": drone.state_age,
                    "healthy": drone.healthy,
                }
                for drone_id, drone in self.drones.items()
            },
        }

    # ------------------------------------------------------------------
    # Human-readable output
    # ------------------------------------------------------------------

    def print_summary(self) -> None:
        """Print a compact swarm status."""

        centroid = self.centroid
        separation = self.minimum_separation

        if math.isinf(separation):
            separation_text = "N/A"
        else:
            separation_text = f"{separation:.2f} m"

        print()
        print("=" * 70)
        print("SWARM STATE")
        print("=" * 70)

        print(
            f"Connected : "
            f"{self.connected_drones}/{self.total_drones}"
        )

        print(
            f"Armed     : "
            f"{self.armed_drones}/{self.total_drones}"
        )

        print(
            f"Failsafe  : "
            f"{self.failsafe_drones}"
        )

        print(
            f"Healthy   : "
            f"{self.healthy_drones}/{self.total_drones}"
        )

        print(
            f"Stale     : "
            f"{self.stale_drones}"
        )

        print(
            f"Centroid  : "
            f"({centroid[0]:.2f}, "
            f"{centroid[1]:.2f}, "
            f"{centroid[2]:.2f})"
        )

        print(
            f"Min separation : "
            f"{separation_text}"
        )

        print(
            f"Swarm health   : "
            f"{'HEALTHY' if self.healthy else 'DEGRADED'}"
        )

        print("-" * 70)

        for drone_id in sorted(self.drones):

            drone = self.drones[drone_id]

            print(
                f"PX4_{drone_id} | "
                f"connected={drone.connected} | "
                f"armed={drone.armed} | "
                f"failsafe={drone.failsafe} | "
                f"healthy={drone.healthy} | "
                f"pos=("
                f"{drone.x:.2f}, "
                f"{drone.y:.2f}, "
                f"{drone.z:.2f}) | "
                f"alt={drone.altitude:.2f}m | "
                f"age={drone.state_age:.3f}s"
            )

        print("=" * 70)
