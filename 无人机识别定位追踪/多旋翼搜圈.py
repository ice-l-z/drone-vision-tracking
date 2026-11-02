import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

TAKEOFF_ALTITUDE = 2.0
START_RADIUS = 3.0
RADIUS_STEP = 1.0
ANGULAR_VEL = 0.4
HEIGHT_THRESHOLD = 0.3
TARGET_FRESH_TIMEOUT = 1.0

LOOP_HZ = 20.0

LOCAL_POS_TOPIC = "/fmu/out/vehicle_local_position_v1"
STATUS_TOPIC = "/fmu/out/vehicle_status"
BEARING_TOPIC = "/object/bearing"
DISTANCE_TOPIC = "/object/distance"
NED_TOPIC = "/object_position_ned"


class MultiRotorCircleSearch(Node):

    def __init__(self):
        super().__init__('multirotor_circle_search')

        pub_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        px4_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        detect_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        callback_group = ReentrantCallbackGroup()

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.traj_ref_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', pub_qos)

        self.create_subscription(
            VehicleLocalPosition, LOCAL_POS_TOPIC,
            self.local_pos_callback, px4_qos, callback_group=callback_group)
        self.create_subscription(
            VehicleStatus, STATUS_TOPIC,
            self.status_callback, px4_qos, callback_group=callback_group)

        self.create_subscription(
            Float32, BEARING_TOPIC,
            self.bearing_callback, detect_qos, callback_group=callback_group)
        self.create_subscription(
            Float32, DISTANCE_TOPIC,
            self.distance_callback, detect_qos, callback_group=callback_group)
        self.create_subscription(
            PointStamped, NED_TOPIC,
            self.ned_callback, detect_qos, callback_group=callback_group)

        self.armed = False
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX

        self.x = None
        self.y = None
        self.z = None
        self.heading = None
        self.pos_valid = False

        self.home_x = None
        self.home_y = None
        self.home_captured = False

        self.bearing_deg = None
        self.distance_m = None
        self.ned_x = None
        self.ned_y = None
        self.last_target_time = None

        self.aircraft_state = 'INIT'
        self.search_angle = 0.0
        self.last_cmd_time = 0.0

        self.timer = self.create_timer(
            1.0 / LOOP_HZ, self.control_loop, callback_group=callback_group)

    def local_pos_callback(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.x = float(msg.x)
            self.y = float(msg.y)
            self.z = float(msg.z)
            self.heading = float(msg.heading)
            self.pos_valid = True
            if not self.home_captured:
                self.home_x = self.x
                self.home_y = self.y
                self.home_captured = True
        else:
            self.pos_valid = False

    def status_callback(self, msg):
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = msg.nav_state

    def bearing_callback(self, msg):
        self.bearing_deg = float(msg.data)
        self.last_target_time = self.get_clock().now()

    def distance_callback(self, msg):
        self.distance_m = float(msg.data)
        self.last_target_time = self.get_clock().now()

    def ned_callback(self, msg):
        self.ned_x = float(msg.point.x)
        self.ned_y = float(msg.point.y)
        self.last_target_time = self.get_clock().now()

    def compute_target(self):
        if self.last_target_time is None:
            return None
        dt = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
        if dt > TARGET_FRESH_TIMEOUT:
            return None

        if self.ned_x is not None and self.ned_y is not None:
            return (self.ned_x, self.ned_y)

        if (self.bearing_deg is not None and self.distance_m is not None
                and self.heading is not None and self.pos_valid):
            bearing_ned = self.heading + math.radians(self.bearing_deg)
            return (self.x + self.distance_m * math.cos(bearing_ned),
                    self.y + self.distance_m * math.sin(bearing_ned))

        return None

    def altitude_reached(self):
        if self.z is None:
            return False
        return -self.z > TAKEOFF_ALTITUDE - HEIGHT_THRESHOLD

    def publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1e3)
        self.offboard_mode_pub.publish(msg)

    def publish_setpoint(self, x, y, z, yaw=float('nan')):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1e3)
        self.traj_ref_pub.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0, param3=0.0,
                     param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 0
        msg.confirmation = 0
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1e3)
        self.command_pub.publish(msg)

    def arm(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def set_offboard_mode(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def control_loop(self):
        self.publish_offboard_heartbeat()
        now = self.get_clock().now().nanoseconds / 1e9

        if self.aircraft_state == 'INIT':
            if self.home_captured and now > 2.0:
                self.aircraft_state = 'ARM'

        elif self.aircraft_state == 'ARM':
            if not self.armed:
                if now - self.last_cmd_time > 1.0:
                    self.arm()
                    self.last_cmd_time = now
            else:
                self.aircraft_state = 'TAKEOFF'

        elif self.aircraft_state == 'TAKEOFF':
            if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                if now - self.last_cmd_time > 1.0:
                    self.set_offboard_mode()
                    self.last_cmd_time = now
                self.publish_setpoint(self.home_x, self.home_y, 0.0)
            else:
                self.publish_setpoint(self.home_x, self.home_y, -TAKEOFF_ALTITUDE)
                if self.altitude_reached():
                    self.aircraft_state = 'CIRCLE'
                    self.search_angle = 0.0

        elif self.aircraft_state == 'CIRCLE':
            target = self.compute_target()
            if target is not None:
                self.aircraft_state = 'GOTO_TARGET'
                return

            self.search_angle += ANGULAR_VEL / LOOP_HZ
            radius = START_RADIUS + (self.search_angle / (2.0 * math.pi)) * RADIUS_STEP
            x = self.home_x + radius * math.cos(self.search_angle)
            y = self.home_y + radius * math.sin(self.search_angle)
            yaw = self.search_angle + math.pi / 2.0
            self.publish_setpoint(x, y, -TAKEOFF_ALTITUDE, yaw=yaw)

        elif self.aircraft_state == 'GOTO_TARGET':
            target = self.compute_target()
            if target is None:
                self.aircraft_state = 'CIRCLE'
                return
            tx, ty = target
            self.publish_setpoint(tx, ty, -TAKEOFF_ALTITUDE)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRotorCircleSearch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
