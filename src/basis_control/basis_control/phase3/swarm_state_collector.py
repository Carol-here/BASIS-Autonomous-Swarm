#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry

from .swarm_state import SwarmState


class SwarmStateCollector(Node):

    def __init__(self):

        super().__init__("basis_swarm_state_collector")

        # ==============================================================
        # Configuration
        # ==============================================================

        self.drone_ids = [1, 2, 3, 4, 5]

        # State considered stale after this duration
        self.stale_threshold = 2.0

        # How frequently the swarm state is printed
        self.print_interval = 1.0

        self.last_print_time = time.monotonic()

        # ==============================================================
        # Swarm state model
        # ==============================================================

        self.swarm_state = SwarmState()

        for drone_id in self.drone_ids:
            self.swarm_state.add_drone(drone_id)

        # ==============================================================
        # PX4 QoS
        # ==============================================================

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ==============================================================
        # Subscription containers
        # ==============================================================

        self.status_subscribers = []
        self.odometry_subscribers = []

        # ==============================================================
        # Last message timestamps
        # ==============================================================

        self.last_status_time = {
            drone_id: 0.0
            for drone_id in self.drone_ids
        }

        self.last_odometry_time = {
            drone_id: 0.0
            for drone_id in self.drone_ids
        }

        # ==============================================================
        # Create subscriptions
        # ==============================================================

        for drone_id in self.drone_ids:

            namespace = f"/px4_{drone_id}"

            status_topic = (
                f"{namespace}/fmu/out/vehicle_status_v4"
            )

            odometry_topic = (
                f"{namespace}/fmu/out/vehicle_odometry"
            )

            # ----------------------------------------------------------
            # Vehicle status
            # ----------------------------------------------------------

            status_subscription = self.create_subscription(
                VehicleStatus,
                status_topic,
                lambda msg, drone_id=drone_id:
                    self.status_callback(msg, drone_id),
                px4_qos,
            )

            self.status_subscribers.append(
                status_subscription
            )

            # ----------------------------------------------------------
            # Vehicle odometry
            # ----------------------------------------------------------

            odometry_subscription = self.create_subscription(
                VehicleOdometry,
                odometry_topic,
                lambda msg, drone_id=drone_id:
                    self.odometry_callback(msg, drone_id),
                px4_qos,
            )

            self.odometry_subscribers.append(
                odometry_subscription
            )

        # ==============================================================
        # State monitoring timer
        # ==============================================================

        self.timer = self.create_timer(
            0.1,
            self.monitor_callback
        )

        self.get_logger().info(
            "BASIS Swarm State Collector started."
        )

        self.get_logger().info(
            "Monitoring PX4_1 → PX4_5."
        )

        self.get_logger().info(
            "State model: DroneState + SwarmState."
        )

        self.get_logger().info(
            "PX4 QoS: BEST_EFFORT."
        )

    # ==================================================================
    # Vehicle Status Callback
    # ==================================================================

    def status_callback(self, msg, drone_id):

        drone = self.swarm_state.get_drone(drone_id)

        if drone is None:
            return

        now = time.monotonic()

        # --------------------------------------------------------------
        # Connection
        # --------------------------------------------------------------

        drone.connected = True

        self.last_status_time[drone_id] = now

        # --------------------------------------------------------------
        # PX4 arming state
        # --------------------------------------------------------------

        drone.armed = (
            msg.arming_state ==
            VehicleStatus.ARMING_STATE_ARMED
        )

        # --------------------------------------------------------------
        # Navigation state
        # --------------------------------------------------------------

        drone.nav_state = msg.nav_state

        # --------------------------------------------------------------
        # Failsafe
        # --------------------------------------------------------------

        drone.failsafe = bool(
            msg.failsafe
        )

        # --------------------------------------------------------------
        # State freshness
        # --------------------------------------------------------------

        self.update_state_age(drone_id)

    # ==================================================================
    # Vehicle Odometry Callback
    # ==================================================================

    def odometry_callback(self, msg, drone_id):

        drone = self.swarm_state.get_drone(drone_id)

        if drone is None:
            return

        now = time.monotonic()

        # --------------------------------------------------------------
        # Position
        #
        # PX4 VehicleOdometry position:
        # [x, y, z]
        #
        # Local NED:
        # +X = North
        # +Y = East
        # +Z = Down
        # --------------------------------------------------------------

        drone.x = float(msg.position[0])
        drone.y = float(msg.position[1])
        drone.z = float(msg.position[2])

        # --------------------------------------------------------------
        # Velocity
        # --------------------------------------------------------------

        drone.vx = float(msg.velocity[0])
        drone.vy = float(msg.velocity[1])
        drone.vz = float(msg.velocity[2])

        # --------------------------------------------------------------
        # Connection
        # --------------------------------------------------------------

        drone.connected = True

        self.last_odometry_time[drone_id] = now

        # --------------------------------------------------------------
        # State freshness
        # --------------------------------------------------------------

        self.update_state_age(drone_id)

    # ==================================================================
    # State Age
    # ==================================================================

    def update_state_age(self, drone_id):

        drone = self.swarm_state.get_drone(drone_id)

        if drone is None:
            return

        status_time = self.last_status_time[drone_id]
        odometry_time = self.last_odometry_time[drone_id]

        latest_message = max(
            status_time,
            odometry_time
        )

        if latest_message == 0.0:
            drone.state_age = float("inf")
        else:
            drone.state_age = (
                time.monotonic() -
                latest_message
            )

    # ==================================================================
    # Monitor
    # ==================================================================

    def monitor_callback(self):

        now = time.monotonic()

        # --------------------------------------------------------------
        # Update age and health
        # --------------------------------------------------------------

        for drone_id in self.drone_ids:

            self.update_state_age(drone_id)

        self.swarm_state.timestamp = now

        self.swarm_state.update_health()

        # --------------------------------------------------------------
        # Print once per second
        # --------------------------------------------------------------

        if (
            now - self.last_print_time
            >= self.print_interval
        ):

            self.print_swarm_state()

            self.last_print_time = now

    # ==================================================================
    # Print
    # ==================================================================

    def print_swarm_state(self):

        self.get_logger().info(
            "======================================================================"
        )

        self.get_logger().info(
            "SWARM STATE"
        )

        self.get_logger().info(
            "======================================================================"
        )

        self.get_logger().info(
            f"Connected : "
            f"{self.swarm_state.connected_drones}/"
            f"{self.swarm_state.total_drones}"
        )

        self.get_logger().info(
            f"Armed     : "
            f"{self.swarm_state.armed_drones}/"
            f"{self.swarm_state.total_drones}"
        )

        self.get_logger().info(
            f"Failsafe  : "
            f"{self.swarm_state.failsafe_drones}"
        )

        self.get_logger().info(
            f"Healthy   : "
            f"{self.swarm_state.healthy_drones}/"
            f"{self.swarm_state.total_drones}"
        )

        self.get_logger().info(
            f"Stale     : "
            f"{self.swarm_state.stale_drones}"
        )

        # --------------------------------------------------------------
        # Centroid
        # --------------------------------------------------------------

        cx, cy, cz = self.swarm_state.centroid

        self.get_logger().info(
            f"Centroid  : "
            f"X={cx:.2f} "
            f"Y={cy:.2f} "
            f"Z={cz:.2f}"
        )

        # --------------------------------------------------------------
        # Minimum separation
        # --------------------------------------------------------------

        separation = (
            self.swarm_state.minimum_separation
        )

        if separation == float("inf"):
            separation_text = "N/A"
        else:
            separation_text = f"{separation:.2f}m"

        self.get_logger().info(
            f"Min separation : {separation_text}"
        )

        # --------------------------------------------------------------
        # Overall health
        # --------------------------------------------------------------

        health_text = (
            "HEALTHY"
            if self.swarm_state.healthy
            else "DEGRADED"
        )

        self.get_logger().info(
            f"SWARM HEALTH : {health_text}"
        )

        # --------------------------------------------------------------
        # Individual drones
        # --------------------------------------------------------------

        for drone_id in sorted(
            self.swarm_state.drones
        ):

            drone = self.swarm_state.drones[
                drone_id
            ]

            self.get_logger().info(
                f"PX4_{drone_id} | "
                f"connected={drone.connected} | "
                f"armed={drone.armed} | "
                f"nav={drone.nav_state} | "
                f"failsafe={drone.failsafe} | "
                f"healthy={drone.healthy} | "
                f"pos=("
                f"{drone.x:.2f}, "
                f"{drone.y:.2f}, "
                f"{drone.z:.2f}) | "
                f"alt={drone.altitude:.2f}m | "
                f"speed={drone.speed:.2f}m/s | "
                f"age={drone.state_age:.3f}s"
            )

        self.get_logger().info(
            "======================================================================"
        )

    # ==================================================================
    # Shutdown
    # ==================================================================

    def destroy_node(self):

        self.get_logger().info(
            "Stopping BASIS Swarm State Collector."
        )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = SwarmStateCollector()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
