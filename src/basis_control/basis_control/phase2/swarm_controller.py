#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleOdometry,
    VehicleStatus,
)


# ============================================================
# BASIS
# Blackboard-based Autonomous Swarm Intelligence System
#
# PHASE 2
# Five independent PX4 SITL vehicles controlled from ROS 2.
#
# Collision prevention:
# Each drone receives a unique altitude layer.
#
#   PX4_1 -> 5 m  -> Z = -5
#   PX4_2 -> 6 m  -> Z = -6
#   PX4_3 -> 7 m  -> Z = -7
#   PX4_4 -> 8 m  -> Z = -8
#   PX4_5 -> 9 m  -> Z = -9
#
# This guarantees vertical separation even when horizontal
# trajectories cross.
#
# PX4 uses NED:
#   X = North
#   Y = East
#   Z = Down
#
# Therefore:
#   Z = -5 means 5 meters above ground.
# ============================================================


# ============================================================
# DRONE
# ============================================================

class Drone:

    def __init__(
        self,
        node,
        drone_id,
        home_x,
        home_y,
        target_x,
        target_y,
        flight_altitude,
    ):

        self.node = node
        self.id = int(drone_id)

        # ----------------------------------------------------
        # ROS namespace
        # ----------------------------------------------------

        self.namespace = f"/px4_{self.id}"

        # ----------------------------------------------------
        # PX4 MAV system ID
        #
        # Instance 1 -> MAV_SYS_ID 2
        # Instance 2 -> MAV_SYS_ID 3
        # Instance 3 -> MAV_SYS_ID 4
        # Instance 4 -> MAV_SYS_ID 5
        # Instance 5 -> MAV_SYS_ID 6
        # ----------------------------------------------------

        self.target_system = self.id + 1

        # ----------------------------------------------------
        # Mission coordinates
        # ----------------------------------------------------

        self.home_x = float(home_x)
        self.home_y = float(home_y)

        self.target_x = float(target_x)
        self.target_y = float(target_y)

        # ----------------------------------------------------
        # Unique flight altitude
        #
        # NED convention:
        # negative Z = above ground
        # ----------------------------------------------------

        self.flight_altitude = float(flight_altitude)

        # ----------------------------------------------------
        # Current position
        # ----------------------------------------------------

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # ----------------------------------------------------
        # PX4 state
        # ----------------------------------------------------

        self.arming_state = 0
        self.nav_state = 0
        self.failsafe = False

        self.status_received = False
        self.position_received = False

        # ----------------------------------------------------
        # Commanded setpoint
        # ----------------------------------------------------

        self.setpoint_x = self.home_x
        self.setpoint_y = self.home_y
        self.setpoint_z = 0.0
        self.setpoint_yaw = 0.0

        # ----------------------------------------------------
        # ACK tracking
        # ----------------------------------------------------

        self.last_ack_command = None
        self.last_ack_result = None
        self.last_ack_time = 0.0

        # ----------------------------------------------------
        # Command retry timestamps
        # ----------------------------------------------------

        self.last_offboard_command_time = 0.0
        self.last_arm_command_time = 0.0
        self.last_land_command_time = 0.0

        # ----------------------------------------------------
        # PX4 QoS
        # ----------------------------------------------------

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ----------------------------------------------------
        # Publishers
        # ----------------------------------------------------

        self.offboard_pub = node.create_publisher(
            OffboardControlMode,
            f"{self.namespace}/fmu/in/offboard_control_mode",
            10,
        )

        self.trajectory_pub = node.create_publisher(
            TrajectorySetpoint,
            f"{self.namespace}/fmu/in/trajectory_setpoint",
            10,
        )

        self.command_pub = node.create_publisher(
            VehicleCommand,
            f"{self.namespace}/fmu/in/vehicle_command",
            10,
        )

        # ----------------------------------------------------
        # Subscribers
        # ----------------------------------------------------

        node.create_subscription(
            VehicleStatus,
            f"{self.namespace}/fmu/out/vehicle_status_v4",
            self.status_callback,
            qos,
        )

        node.create_subscription(
            VehicleOdometry,
            f"{self.namespace}/fmu/out/vehicle_odometry",
            self.odometry_callback,
            qos,
        )

        node.create_subscription(
            VehicleCommandAck,
            f"{self.namespace}/fmu/out/vehicle_command_ack_v1",
            self.ack_callback,
            qos,
        )

        self.node.get_logger().info(
            f"Drone {self.id}: namespace={self.namespace}"
        )

        self.node.get_logger().info(
            f"Drone {self.id}: target_system={self.target_system}"
        )

        self.node.get_logger().info(
            f"Drone {self.id}: flight altitude="
            f"{abs(self.flight_altitude):.1f} m"
        )

    # ========================================================
    # CALLBACKS
    # ========================================================

    def status_callback(self, msg):

        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state
        self.failsafe = msg.failsafe

        self.status_received = True

    def odometry_callback(self, msg):

        self.x = float(msg.position[0])
        self.y = float(msg.position[1])
        self.z = float(msg.position[2])

        self.position_received = True

    def ack_callback(self, msg):

        self.last_ack_command = msg.command
        self.last_ack_result = msg.result
        self.last_ack_time = time.monotonic()

    # ========================================================
    # TIMESTAMP
    # ========================================================

    @staticmethod
    def timestamp():

        return int(time.time_ns() // 1000)

    # ========================================================
    # OFFBOARD CONTROL MODE
    # ========================================================

    def publish_offboard_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.timestamp()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_pub.publish(msg)

    # ========================================================
    # TRAJECTORY SETPOINT
    # ========================================================

    def publish_setpoint(self):

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        msg.position = [
            float(self.setpoint_x),
            float(self.setpoint_y),
            float(self.setpoint_z),
        ]

        msg.velocity = [
            float("nan"),
            float("nan"),
            float("nan"),
        ]

        msg.acceleration = [
            float("nan"),
            float("nan"),
            float("nan"),
        ]

        msg.jerk = [
            float("nan"),
            float("nan"),
            float("nan"),
        ]

        msg.yaw = float(self.setpoint_yaw)
        msg.yawspeed = float("nan")

        self.trajectory_pub.publish(msg)

    # ========================================================
    # VEHICLE COMMAND
    # ========================================================

    def send_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0,
    ):

        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.command = command

        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)

        # Multi-vehicle addressing

        msg.target_system = self.target_system
        msg.target_component = 1

        msg.source_system = 255
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)

    # ========================================================
    # REQUEST OFFBOARD
    # ========================================================

    def request_offboard(self):

        now = time.monotonic()

        if now - self.last_offboard_command_time < 1.0:
            return

        self.last_offboard_command_time = now

        self.send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )

        self.node.get_logger().info(
            f"PX4_{self.id}: OFFBOARD command sent"
        )

    # ========================================================
    # ARM
    # ========================================================

    def request_arm(self):

        now = time.monotonic()

        if now - self.last_arm_command_time < 1.0:
            return

        self.last_arm_command_time = now

        self.send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )

        self.node.get_logger().info(
            f"PX4_{self.id}: ARM command sent"
        )

    # ========================================================
    # LAND
    # ========================================================

    def request_land(self):

        now = time.monotonic()

        if now - self.last_land_command_time < 2.0:
            return

        self.last_land_command_time = now

        self.send_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )

        self.node.get_logger().info(
            f"PX4_{self.id}: LAND command sent"
        )

    # ========================================================
    # DISTANCE
    # ========================================================

    def distance_to_setpoint(self):

        dx = self.x - self.setpoint_x
        dy = self.y - self.setpoint_y
        dz = self.z - self.setpoint_z

        return math.sqrt(
            dx * dx +
            dy * dy +
            dz * dz
        )

    def reached(self, tolerance=0.8):

        return (
            self.position_received
            and self.distance_to_setpoint() < tolerance
        )


# ============================================================
# SWARM CONTROLLER
# ============================================================

class SwarmController(Node):

    NUM_DRONES = 5

    # --------------------------------------------------------
    # Collision prevention
    # --------------------------------------------------------

    # Vertical separation between drones.
    ALTITUDE_SEPARATION = 1.0

    # Minimum horizontal separation expected between homes
    # and assigned targets.
    MIN_HORIZONTAL_SEPARATION = 2.0

    # --------------------------------------------------------
    # Base altitude
    # --------------------------------------------------------

    BASE_ALTITUDE = -5.0

    # --------------------------------------------------------
    # Mission
    # --------------------------------------------------------

    ROTATION_ANGLE = math.pi / 2.0

    HOLD_TIME = 5.0

    POSITION_TOLERANCE = 0.8

    # --------------------------------------------------------
    # PX4 states
    # --------------------------------------------------------

    NAV_STATE_OFFBOARD = 14

    ARMING_STATE_ARMED = 2

    # --------------------------------------------------------
    # Timeouts
    # --------------------------------------------------------

    OFFBOARD_TIMEOUT = 20.0
    ARM_TIMEOUT = 20.0
    TAKEOFF_TIMEOUT = 40.0
    WAYPOINT_TIMEOUT = 60.0
    RETURN_TIMEOUT = 60.0

    # ========================================================
    # INITIALIZATION
    # ========================================================

    def __init__(self):

        super().__init__(
            "basis_swarm_controller"
        )

        self.get_logger().info("")
        self.get_logger().info(
            "=" * 70
        )
        self.get_logger().info(
            "       BASIS — 5 DRONE COLLISION-SAFE SWARM"
        )
        self.get_logger().info(
            "=" * 70
        )

        # ----------------------------------------------------
        # Create five drones
        #
        # Every drone receives a different altitude.
        # ----------------------------------------------------

        self.drones = [

            # PX4_1
            Drone(
                self,
                1,
                0.0,
                0.0,
                0.0,
                8.0,
                -5.0,
            ),

            # PX4_2
            Drone(
                self,
                2,
                5.0,
                0.0,
                8.0,
                0.0,
                -6.0,
            ),

            # PX4_3
            Drone(
                self,
                3,
                -5.0,
                0.0,
                -8.0,
                0.0,
                -7.0,
            ),

            # PX4_4
            Drone(
                self,
                4,
                0.0,
                5.0,
                0.0,
                -8.0,
                -8.0,
            ),

            # PX4_5
            Drone(
                self,
                5,
                0.0,
                -5.0,
                6.0,
                -6.0,
                -9.0,
            ),
        ]

        # ----------------------------------------------------
        # State machine
        # ----------------------------------------------------

        self.state = "WAIT_FOR_PX4"

        self.state_start = time.monotonic()

        self.last_status_log = time.monotonic()

        # ----------------------------------------------------
        # 20 Hz control loop
        # ----------------------------------------------------

        self.control_timer = self.create_timer(
            0.05,
            self.control_loop,
        )

        # ----------------------------------------------------
        # 5 Hz state machine
        # ----------------------------------------------------

        self.state_timer = self.create_timer(
            0.20,
            self.state_machine,
        )

        # ----------------------------------------------------
        # Status output
        # ----------------------------------------------------

        self.status_timer = self.create_timer(
            2.0,
            self.print_status,
        )

        self.get_logger().info(
            "Controller initialized."
        )

        self.get_logger().info(
            "Collision protection: UNIQUE ALTITUDE LAYERS"
        )

    # ========================================================
    # STATE HELPERS
    # ========================================================

    def elapsed(self):

        return (
            time.monotonic()
            - self.state_start
        )

    def change_state(self, new_state):

        self.state = new_state

        self.state_start = time.monotonic()

        self.get_logger().info("")
        self.get_logger().info(
            "=" * 60
        )
        self.get_logger().info(
            f"STATE → {new_state}"
        )
        self.get_logger().info(
            "=" * 60
        )

    # ========================================================
    # CONNECTION CHECKS
    # ========================================================

    def all_status_received(self):

        return all(
            drone.status_received
            for drone in self.drones
        )

    def all_positions_received(self):

        return all(
            drone.position_received
            for drone in self.drones
        )

    # ========================================================
    # STATE CHECKS
    # ========================================================

    def all_offboard(self):

        return all(
            drone.nav_state ==
            self.NAV_STATE_OFFBOARD
            for drone in self.drones
        )

    def all_armed(self):

        return all(
            drone.arming_state ==
            self.ARMING_STATE_ARMED
            for drone in self.drones
        )

    def all_reached(self, tolerance=None):

        if tolerance is None:
            tolerance = self.POSITION_TOLERANCE

        return all(
            drone.reached(tolerance)
            for drone in self.drones
        )

    # ========================================================
    # COLLISION MONITOR
    # ========================================================

    def check_separation(self):

        danger = False

        for i in range(len(self.drones)):

            drone_a = self.drones[i]

            for j in range(i + 1, len(self.drones)):

                drone_b = self.drones[j]

                if not (
                    drone_a.position_received
                    and drone_b.position_received
                ):
                    continue

                dx = drone_a.x - drone_b.x
                dy = drone_a.y - drone_b.y
                dz = drone_a.z - drone_b.z

                distance_3d = math.sqrt(
                    dx * dx +
                    dy * dy +
                    dz * dz
                )

                horizontal_distance = math.sqrt(
                    dx * dx +
                    dy * dy
                )

                # ------------------------------------------------
                # Emergency warning.
                #
                # The altitude layers should normally keep this
                # from occurring.
                # ------------------------------------------------

                if distance_3d < 2.0:

                    danger = True

                    self.get_logger().warn(
                        f"COLLISION WARNING: "
                        f"PX4_{drone_a.id} ↔ PX4_{drone_b.id} "
                        f"| 3D={distance_3d:.2f}m "
                        f"| horizontal={horizontal_distance:.2f}m "
                        f"| dz={abs(dz):.2f}m"
                    )

        return not danger

    # ========================================================
    # STATUS
    # ========================================================

    def print_status(self):

        self.get_logger().info(
            f"[STATE={self.state}]"
        )

        for drone in self.drones:

            self.get_logger().info(
                f"PX4_{drone.id} "
                f"| nav={drone.nav_state} "
                f"| arm={drone.arming_state} "
                f"| failsafe={drone.failsafe} "
                f"| pos=("
                f"{drone.x:.2f}, "
                f"{drone.y:.2f}, "
                f"{drone.z:.2f}) "
                f"| target=("
                f"{drone.setpoint_x:.2f}, "
                f"{drone.setpoint_y:.2f}, "
                f"{drone.setpoint_z:.2f}) "
                f"| altitude="
                f"{abs(drone.flight_altitude):.1f}m"
            )

    # ========================================================
    # 20 Hz CONTROL LOOP
    # ========================================================

    def control_loop(self):

        for drone in self.drones:

            # ------------------------------------------------
            # OFFBOARD heartbeat
            # ------------------------------------------------

            drone.publish_offboard_mode()

            # ------------------------------------------------
            # Current trajectory setpoint
            # ------------------------------------------------

            drone.publish_setpoint()

        # ----------------------------------------------------
        # Continuously monitor separation.
        #
        # This is currently monitoring only. The deterministic
        # protection comes from the unique altitude layers.
        # ----------------------------------------------------

        self.check_separation()

    # ========================================================
    # STATE MACHINE
    # ========================================================

    def state_machine(self):

        # ====================================================
        # WAIT FOR PX4 STATUS
        # ====================================================

        if self.state == "WAIT_FOR_PX4":

            if self.all_status_received():

                self.get_logger().info(
                    "All 5 PX4 status messages received."
                )

                self.change_state(
                    "WAIT_FOR_POSITION"
                )

            return

        # ====================================================
        # WAIT FOR ODOMETRY
        # ====================================================

        if self.state == "WAIT_FOR_POSITION":

            if self.all_positions_received():

                self.get_logger().info(
                    "All 5 PX4 odometry streams received."
                )

                self.change_state(
                    "ESTABLISH_SETPOINT_STREAM"
                )

            return

        # ====================================================
        # ESTABLISH SETPOINT STREAM
        # ====================================================

        if self.state == "ESTABLISH_SETPOINT_STREAM":

            if self.elapsed() >= 3.0:

                self.get_logger().info(
                    "20 Hz setpoint stream established."
                )

                self.change_state(
                    "REQUEST_OFFBOARD"
                )

            return

        # ====================================================
        # REQUEST OFFBOARD
        # ====================================================

        if self.state == "REQUEST_OFFBOARD":

            self.get_logger().info(
                "Requesting OFFBOARD for all drones..."
            )

            for drone in self.drones:

                drone.request_offboard()

            self.change_state(
                "WAIT_OFFBOARD"
            )

            return

        # ====================================================
        # WAIT FOR OFFBOARD
        # ====================================================

        if self.state == "WAIT_OFFBOARD":

            for drone in self.drones:

                if drone.nav_state != self.NAV_STATE_OFFBOARD:

                    drone.request_offboard()

            if self.all_offboard():

                self.get_logger().info("")
                self.get_logger().info(
                    "############################################"
                )
                self.get_logger().info(
                    "ALL 5 DRONES ARE IN OFFBOARD"
                )
                self.get_logger().info(
                    "############################################"
                )

                self.change_state(
                    "ARM_ALL"
                )

                return

            if self.elapsed() > self.OFFBOARD_TIMEOUT:

                self.get_logger().error(
                    "OFFBOARD REQUEST TIMED OUT."
                )

                self.change_state(
                    "ABORT"
                )

            return

        # ====================================================
        # ARM ALL
        # ====================================================

        if self.state == "ARM_ALL":

            self.get_logger().info(
                "ARMING ALL 5 DRONES..."
            )

            for drone in self.drones:

                drone.request_arm()

            self.change_state(
                "WAIT_ARMED"
            )

            return

        # ====================================================
        # WAIT ARMED
        # ====================================================

        if self.state == "WAIT_ARMED":

            for drone in self.drones:

                if drone.arming_state != self.ARMING_STATE_ARMED:

                    drone.request_arm()

            if self.all_armed():

                self.get_logger().info("")
                self.get_logger().info(
                    "############################################"
                )
                self.get_logger().info(
                    "ALL 5 DRONES ARMED"
                )
                self.get_logger().info(
                    "############################################"
                )

                self.change_state(
                    "TAKEOFF"
                )

                return

            if self.elapsed() > self.ARM_TIMEOUT:

                self.get_logger().error(
                    "ARMING TIMED OUT."
                )

                self.change_state(
                    "ABORT"
                )

            return

        # ====================================================
        # TAKEOFF
        # ====================================================

        if self.state == "TAKEOFF":

            self.get_logger().info(
                "TAKEOFF — UNIQUE ALTITUDE LAYERS"
            )

            for drone in self.drones:

                drone.setpoint_x = drone.home_x
                drone.setpoint_y = drone.home_y

                # ------------------------------------------------
                # CRITICAL:
                # Every drone gets its own altitude.
                # ------------------------------------------------

                drone.setpoint_z = drone.flight_altitude

                drone.setpoint_yaw = 0.0

                self.get_logger().info(
                    f"PX4_{drone.id} "
                    f"→ altitude="
                    f"{abs(drone.flight_altitude):.1f}m "
                    f"(Z={drone.flight_altitude:.1f})"
                )

            self.change_state(
                "WAIT_TAKEOFF"
            )

            return

        # ====================================================
        # WAIT TAKEOFF
        # ====================================================

        if self.state == "WAIT_TAKEOFF":

            if self.all_reached():

                self.get_logger().info(
                    "ALL 5 DRONES REACHED THEIR "
                    "SEPARATED ALTITUDES."
                )

                self.change_state(
                    "ROTATE"
                )

                return

            if self.elapsed() > self.TAKEOFF_TIMEOUT:

                self.get_logger().error(
                    "TAKEOFF TIMED OUT."
                )

                self.change_state(
                    "ABORT"
                )

            return

        # ====================================================
        # ROTATE
        # ====================================================

        if self.state == "ROTATE":

            self.get_logger().info(
                "ROTATING ALL DRONES → 90 DEGREES"
            )

            for drone in self.drones:

                drone.setpoint_yaw = (
                    self.ROTATION_ANGLE
                )

            self.change_state(
                "WAIT_ROTATION"
            )

            return

        # ====================================================
        # WAIT ROTATION
        # ====================================================

        if self.state == "WAIT_ROTATION":

            if self.elapsed() >= 4.0:

                self.get_logger().info(
                    "90° ROTATION COMPLETE."
                )

                self.change_state(
                    "MOVE_TO_ASSIGNED_POINTS"
                )

            return

        # ====================================================
        # MOVE TO ASSIGNED POINTS
        # ====================================================

        if self.state == "MOVE_TO_ASSIGNED_POINTS":

            self.get_logger().info(
                "MOVING TO ASSIGNED COORDINATES "
                "USING SEPARATED ALTITUDE LANES:"
            )

            for drone in self.drones:

                drone.setpoint_x = drone.target_x
                drone.setpoint_y = drone.target_y

                # ------------------------------------------------
                # CRITICAL:
                # Keep each drone on its own altitude.
                # ------------------------------------------------

                drone.setpoint_z = drone.flight_altitude

                drone.setpoint_yaw = (
                    self.ROTATION_ANGLE
                )

                self.get_logger().info(
                    f"  PX4_{drone.id} "
                    f"→ X={drone.target_x:.1f}, "
                    f"Y={drone.target_y:.1f}, "
                    f"Z={drone.flight_altitude:.1f}"
                )

            self.change_state(
                "WAIT_ASSIGNED_POINTS"
            )

            return

        # ====================================================
        # WAIT ASSIGNED POINTS
        # ====================================================

        if self.state == "WAIT_ASSIGNED_POINTS":

            if self.all_reached():

                self.get_logger().info(
                    "ALL 5 DRONES REACHED ASSIGNED POINTS."
                )

                self.change_state(
                    "HOLD_ASSIGNED"
                )

                return

            if self.elapsed() > self.WAYPOINT_TIMEOUT:

                self.get_logger().error(
                    "ASSIGNED WAYPOINT TIMEOUT."
                )

                self.change_state(
                    "ABORT"
                )

            return

        # ====================================================
        # HOLD
        # ====================================================

        if self.state == "HOLD_ASSIGNED":

            if self.elapsed() >= self.HOLD_TIME:

                self.get_logger().info(
                    "5 SECOND HOLD COMPLETE."
                )

                self.change_state(
                    "RETURN_HOME"
                )

            return

        # ====================================================
        # RETURN HOME
        # ====================================================

        if self.state == "RETURN_HOME":

            self.get_logger().info(
                "RETURNING ALL DRONES HOME "
                "ON SEPARATED ALTITUDE LANES."
            )

            for drone in self.drones:

                drone.setpoint_x = drone.home_x
                drone.setpoint_y = drone.home_y

                # ------------------------------------------------
                # Keep vertical separation during return.
                # ------------------------------------------------

                drone.setpoint_z = drone.flight_altitude

                drone.setpoint_yaw = (
                    self.ROTATION_ANGLE
                )

            self.change_state(
                "WAIT_HOME"
            )

            return

        # ====================================================
        # WAIT HOME
        # ====================================================

        if self.state == "WAIT_HOME":

            if self.all_reached():

                self.get_logger().info(
                    "ALL 5 DRONES RETURNED HOME."
                )

                self.change_state(
                    "LAND"
                )

                return

            if self.elapsed() > self.RETURN_TIMEOUT:

                self.get_logger().error(
                    "RETURN HOME TIMED OUT."
                )

                self.change_state(
                    "ABORT"
                )

            return

        # ====================================================
        # LAND
        # ====================================================

        if self.state == "LAND":

            self.get_logger().info(
                "LANDING ALL 5 DRONES..."
            )

            for drone in self.drones:

                if drone.arming_state == self.ARMING_STATE_ARMED:

                    drone.request_land()

            self.change_state(
                "WAIT_LANDING"
            )

            return

        # ====================================================
        # WAIT LANDING
        # ====================================================

        if self.state == "WAIT_LANDING":

            all_disarmed = all(
                drone.arming_state !=
                self.ARMING_STATE_ARMED
                for drone in self.drones
            )

            if all_disarmed:

                self.get_logger().info("")
                self.get_logger().info(
                    "=" * 70
                )
                self.get_logger().info(
                    "             MISSION COMPLETE"
                )
                self.get_logger().info(
                    "=" * 70
                )
                self.get_logger().info(
                    "5 DRONES"
                )
                self.get_logger().info(
                    "SEPARATED TAKEOFF → ROTATE → "
                    "WAYPOINTS → RETURN → LAND"
                )
                self.get_logger().info(
                    "=" * 70
                )

                self.change_state(
                    "DONE"
                )

                return

            for drone in self.drones:

                if drone.arming_state == self.ARMING_STATE_ARMED:

                    drone.request_land()

            return

        # ====================================================
        # ABORT
        # ====================================================

        if self.state == "ABORT":

            self.get_logger().error(
                "MISSION ABORTED."
            )

            armed_drones = [
                drone
                for drone in self.drones
                if drone.arming_state ==
                self.ARMING_STATE_ARMED
            ]

            if not armed_drones:

                self.get_logger().error(
                    "No drones are armed."
                )

                self.change_state(
                    "FAILED"
                )

                return

            self.get_logger().warn(
                f"Sending LAND to "
                f"{len(armed_drones)} armed drone(s)."
            )

            for drone in armed_drones:

                drone.request_land()

            self.change_state(
                "ABORT_LANDING"
            )

            return

        # ====================================================
        # ABORT LANDING
        # ====================================================

        if self.state == "ABORT_LANDING":

            armed_drones = [
                drone
                for drone in self.drones
                if drone.arming_state ==
                self.ARMING_STATE_ARMED
            ]

            if not armed_drones:

                self.get_logger().error(
                    "ABORT LANDING COMPLETE."
                )

                self.change_state(
                    "FAILED"
                )

                return

            for drone in armed_drones:

                drone.request_land()

            return

        # ====================================================
        # DONE
        # ====================================================

        if self.state == "DONE":

            return

        # ====================================================
        # FAILED
        # ====================================================

        if self.state == "FAILED":

            return

    # ========================================================
    # SHUTDOWN
    # ========================================================

    def stop(self):

        self.get_logger().info(
            "Stopping BASIS swarm controller."
        )

        if self.control_timer is not None:
            self.control_timer.cancel()

        if self.state_timer is not None:
            self.state_timer.cancel()

        if self.status_timer is not None:
            self.status_timer.cancel()


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = SwarmController()

        rclpy.spin(node)

    except KeyboardInterrupt:

        if node is not None:

            node.get_logger().warn(
                "CTRL+C received. Stopping controller."
            )

    except Exception as e:

        if node is not None:

            node.get_logger().error(
                f"Controller exception: {e}"
            )

        else:

            print(
                f"Controller exception: {e}"
            )

    finally:

        if node is not None:

            node.stop()
            node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":

    main()
