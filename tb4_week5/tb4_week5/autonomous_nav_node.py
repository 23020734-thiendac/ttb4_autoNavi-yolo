#!/usr/bin/env python3
"""Set AMCL's initial pose and navigate through map goals with Nav2."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from irobot_create_msgs.action import Undock
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class AutonomousNavNode(BasicNavigator):
    """BasicNavigator node that sends goals sequentially without RViz clicks."""

    def __init__(self):
        super().__init__(node_name='week5_autonomous_nav')

        for index in range(1, 4):
            self.declare_parameter(f'goal_{index}', [0.0, 0.0, 0.0])
        self.declare_parameter('goal_count', 3)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('goal_timeout_sec', 180.0)
        self.declare_parameter('wait_after_goal_sec', 2.0)
        self.declare_parameter('initial_pose_wait_sec', 45.0)
        self.declare_parameter('initial_pose_publish_period_sec', 1.0)
        self.declare_parameter('auto_undock', True)
        self.declare_parameter('undock_timeout_sec', 45.0)
        self.declare_parameter('fallback_reverse_sec', 6.0)
        self.declare_parameter('fallback_reverse_speed', 0.08)
        self.declare_parameter('dock_keepout_radius', 1.20)
        self.declare_parameter(
            'dock_bypass_route',
            [3.0, 1.5, 0.0, -1.0, 1.5, 3.14])
        self.declare_parameter('recovery_reverse_sec', 1.5)
        self.declare_parameter('recovery_reverse_speed', 0.08)
        self.declare_parameter('recovery_turn_speed', 0.50)
        self.declare_parameter('recovery_turn_direction', 'left')
        self.declare_parameter('recovery_turn_timeout_sec', 3.5)
        self.declare_parameter('recovery_forward_sec', 1.5)
        self.declare_parameter('recovery_forward_speed', 0.08)

        goal_count = int(self.get_parameter('goal_count').value)
        if goal_count < 1 or goal_count > 3:
            self.get_logger().warning('goal_count phai tu 1 den 3; su dung 3.')
            goal_count = 3
        self.goals = [
            self.get_parameter(f'goal_{index}').value
            for index in range(1, goal_count + 1)
        ]
        self.goal_count = len(self.goals)
        self.goal_timeout = float(self.get_parameter('goal_timeout_sec').value)
        self.wait_after_goal = float(
            self.get_parameter('wait_after_goal_sec').value)
        self.initial_pose_wait = float(
            self.get_parameter('initial_pose_wait_sec').value)
        self.initial_pose_publish_period = max(
            0.5,
            float(self.get_parameter('initial_pose_publish_period_sec').value))
        self.auto_undock = bool(self.get_parameter('auto_undock').value)
        self.undock_timeout = float(
            self.get_parameter('undock_timeout_sec').value)
        self.reverse_duration = float(
            self.get_parameter('fallback_reverse_sec').value)
        self.reverse_speed = abs(float(
            self.get_parameter('fallback_reverse_speed').value))
        self.dock_keepout_radius = float(
            self.get_parameter('dock_keepout_radius').value)
        bypass_values = [float(value) for value in
                         self.get_parameter('dock_bypass_route').value]
        if not bypass_values or len(bypass_values) % 3 != 0:
            self.get_logger().warning(
                'dock_bypass_route phai gom cac bo [x, y, yaw]; '
                'dung hai waypoint mac dinh.')
            bypass_values = [3.0, 1.5, 0.0, -1.0, 1.5, 3.14]
        self.dock_bypass_route = [
            bypass_values[index:index + 3]
            for index in range(0, len(bypass_values), 3)
        ]
        self.recovery_reverse_sec = float(
            self.get_parameter('recovery_reverse_sec').value)
        self.recovery_reverse_speed = abs(float(
            self.get_parameter('recovery_reverse_speed').value))
        self.recovery_turn_speed = abs(float(
            self.get_parameter('recovery_turn_speed').value))
        self.recovery_turn_direction = str(
            self.get_parameter('recovery_turn_direction').value).lower()
        self.recovery_turn_timeout = float(
            self.get_parameter('recovery_turn_timeout_sec').value)
        self.recovery_forward_sec = float(
            self.get_parameter('recovery_forward_sec').value)
        self.recovery_forward_speed = abs(float(
            self.get_parameter('recovery_forward_speed').value))

        initial_x = float(self.get_parameter('initial_x').value)
        initial_y = float(self.get_parameter('initial_y').value)
        initial_yaw = float(self.get_parameter('initial_yaw').value)
        # The stock TurtleBot 4 simulator places the dock collision centre
        # about 0.202 m in front of the initial robot pose.
        self.dock_x = initial_x + 0.202 * math.cos(initial_yaw)
        self.dock_y = initial_y + 0.202 * math.sin(initial_yaw)
        self.get_logger().info(
            f'Nho vi tri dock gan ({self.dock_x:.2f}, {self.dock_y:.2f}); '
            f'vung tranh xa {self.dock_keepout_radius:.2f} m.')
        self.undock_client = ActionClient(self, Undock, 'undock')
        self.safety_override_client = self.create_client(
            SetParameters, '/motion_control/set_parameters')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.bumper_sub = self.create_subscription(
            Contacts, 'bumper_contact', self.bumper_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.latest_odom = None
        self.latest_odom_yaw = None
        self.scan_received = False
        self.dock_collision_detected = False
        self.recovery_active = False
        self.initial_pose = self.make_pose([
            self.get_parameter('initial_x').value,
            self.get_parameter('initial_y').value,
            self.get_parameter('initial_yaw').value,
        ])

    @staticmethod
    def make_pose(values):
        x, y, yaw = [float(value) for value in values]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        _, _, qz, qw = quaternion_from_yaw(yaw)
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def publish_initial_pose_until_ready(self):
        """Publish initial pose at a bounded rate and wait for AMCL output."""
        orientation = self.initial_pose.pose.orientation
        initial_yaw = math.atan2(
            2.0 * orientation.w * orientation.z,
            1.0 - 2.0 * orientation.z ** 2)
        self.get_logger().info(
            'Dang tu dong publish initial pose cho AMCL: '
            f'x={self.initial_pose.pose.position.x:.2f}, '
            f'y={self.initial_pose.pose.position.y:.2f}, '
            f'yaw={initial_yaw:.2f}')

        # AMCL only publishes amcl_pose after processing a laser scan.  Waiting
        # here gives a clear failure instead of leaving RobotModel red forever.
        scan_started = time.monotonic()
        while rclpy.ok() and not self.scan_received:
            if time.monotonic() - scan_started >= self.initial_pose_wait:
                self.get_logger().error(
                    'Khong nhan duoc /scan tu Gazebo. Kiem tra bridge world '
                    'phai la warehouse; khong khoi dong navigation.')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Da nhan /scan; bat dau khoi tao AMCL.')

        started = time.monotonic()
        next_publish = started
        first_publish = True

        while rclpy.ok() and not self.initial_pose_received:
            now = time.monotonic()
            if now - started >= self.initial_pose_wait:
                self.get_logger().error(
                    f'Khong nhan duoc amcl_pose sau {self.initial_pose_wait:.1f}s. '
                    'Dung navigation de tranh gui goal khi TF chua san sang.')
                return False

            if now >= next_publish:
                self.initial_pose.header.stamp = self.get_clock().now().to_msg()
                if first_publish:
                    # Reset BasicNavigator's receive flag exactly once.
                    self.setInitialPose(self.initial_pose)
                    first_publish = False
                else:
                    # Republish without clearing a pose callback that may have
                    # arrived between executor cycles.
                    self._setInitialPose()
                next_publish = now + self.initial_pose_publish_period

            # A fixed, short spin prevents sensor callbacks from turning this
            # loop into an unbounded initial-pose flood.
            rclpy.spin_once(self, timeout_sec=0.1)

        if not rclpy.ok():
            return False

        self.get_logger().info(
            'Da nhan amcl_pose; TF map -> odom da san sang, '
            'khong can click 2D Pose Estimate.')
        return True

    def scan_callback(self, _message):
        self.scan_received = True

    def enable_full_safety_override(self):
        """Enable reversing through the dock without a fragile shell command."""
        started = time.monotonic()
        while rclpy.ok() and not self.safety_override_client.wait_for_service(
                timeout_sec=1.0):
            if time.monotonic() - started >= 15.0:
                self.get_logger().warning(
                    'Service motion_control/set_parameters chua san sang; '
                    'tiep tuc voi safety mac dinh.')
                return False

        request = SetParameters.Request()
        request.parameters = [Parameter(
            name='safety_override',
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING,
                string_value='full'))]
        future = self.safety_override_client.call_async(request)
        while rclpy.ok() and not future.done():
            if time.monotonic() - started >= 20.0:
                self.get_logger().warning(
                    'Dat safety_override qua timeout; dung safety mac dinh.')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        response = future.result()
        if (response is None or not response.results or
                not response.results[0].successful):
            reason = ''
            if response is not None and response.results:
                reason = response.results[0].reason
            self.get_logger().warning(
                f'Khong dat duoc safety_override=full: {reason}')
            return False

        self.get_logger().info('Da dat motion_control safety_override=full.')
        return True

    def distance_to_dock_segment(self, start, goal):
        """Return distance from the dock to the straight goal-to-goal segment."""
        ax, ay = float(start[0]), float(start[1])
        bx, by = float(goal[0]), float(goal[1])
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        if length_sq < 1.0e-9:
            return math.hypot(self.dock_x - ax, self.dock_y - ay)
        projection = ((self.dock_x - ax) * dx +
                      (self.dock_y - ay) * dy) / length_sq
        projection = max(0.0, min(1.0, projection))
        closest_x = ax + projection * dx
        closest_y = ay + projection * dy
        return math.hypot(self.dock_x - closest_x, self.dock_y - closest_y)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def bumper_callback(self, message):
        """Record a physical bumper contact for one-shot recovery."""
        if self.recovery_active or not message.contacts:
            return

        names = []
        for contact in message.contacts:
            names.extend([
                str(contact.collision1.name),
                str(contact.collision2.name),
            ])
        names = [name for name in names if name]
        self.dock_collision_detected = True
        detail = ', '.join(names[:4]) if names else 'bumper_contact'
        self.get_logger().warning(
            f'Phat hien va cham vat ly ({detail}); se thuc hien giai cuu.')

    def publish_motion(self, linear_x, angular_z, duration):
        command = Twist()
        command.linear.x = float(linear_x)
        command.angular.z = float(angular_z)
        started = time.monotonic()
        while rclpy.ok() and time.monotonic() - started < duration:
            self.cmd_vel_pub.publish(command)
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop_motion(self):
        stop = Twist()
        for _ in range(12):
            self.cmd_vel_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)

    def recover_from_collision(self):
        """Back up, turn 90 degrees left/right, then clear the dock area."""
        self.recovery_active = True
        self.dock_collision_detected = False
        try:
            direction = -1.0 if self.recovery_turn_direction == 'right' else 1.0
            self.get_logger().warning(
                'Giai cuu va cham: dung, lui %.1fs, quay 90 do %s, tien %.1fs.' %
                (self.recovery_reverse_sec,
                 'phai' if direction < 0 else 'trai',
                 self.recovery_forward_sec))

            self.stop_motion()
            self.publish_motion(
                -self.recovery_reverse_speed, 0.0, self.recovery_reverse_sec)

            # Use odometry to stop close to 90 degrees; the timeout is a safe
            # fallback when orientation feedback is temporarily unavailable.
            start_yaw = self.latest_odom_yaw
            turn_started = time.monotonic()
            turn_command = Twist()
            turn_command.angular.z = direction * self.recovery_turn_speed
            while (rclpy.ok() and
                   time.monotonic() - turn_started < self.recovery_turn_timeout):
                if start_yaw is not None and self.latest_odom_yaw is not None:
                    turned = abs(self.normalize_angle(
                        self.latest_odom_yaw - start_yaw))
                    if turned >= math.radians(88.0):
                        break
                self.cmd_vel_pub.publish(turn_command)
                rclpy.spin_once(self, timeout_sec=0.05)

            self.stop_motion()
            self.publish_motion(
                self.recovery_forward_speed, 0.0, self.recovery_forward_sec)
            self.stop_motion()
            self.get_logger().warning(
                'Da giai cuu va cham; se gui lai goal dang dang do.')
            return True
        finally:
            self.recovery_active = False

    def route_around_dock(self, start, goal):
        """Insert map-safe bypass poses when a segment approaches the dock."""
        distance = self.distance_to_dock_segment(start, goal)
        if distance >= self.dock_keepout_radius:
            return [goal]

        # Traverse the two house waypoints in whichever direction gives the
        # shorter approach.  This handles both leaving the dock for Goal 1
        # and returning from the right side toward Goal 2.
        forward = list(self.dock_bypass_route)
        backward = list(reversed(self.dock_bypass_route))

        def route_score(route):
            return (math.hypot(float(start[0]) - route[0][0],
                               float(start[1]) - route[0][1]) +
                    math.hypot(float(goal[0]) - route[-1][0],
                               float(goal[1]) - route[-1][1]))

        bypasses = forward if route_score(forward) <= route_score(
            backward) else backward
        route = bypasses + [goal]

        current = start
        for route_index, route_goal in enumerate(route):
            current_dock_distance = math.hypot(
                float(current[0]) - self.dock_x,
                float(current[1]) - self.dock_y)
            next_dock_distance = math.hypot(
                float(route_goal[0]) - self.dock_x,
                float(route_goal[1]) - self.dock_y)
            segment_distance = self.distance_to_dock_segment(
                current, route_goal)

            # Immediately after reversing, the robot can still be inside the
            # keepout circle.  The first segment is valid only when it moves
            # farther away from the dock; all later segments must remain out.
            leaving_keepout = (
                route_index == 0 and
                current_dock_distance < self.dock_keepout_radius and
                next_dock_distance > current_dock_distance)
            if (not leaving_keepout and
                    segment_distance < self.dock_keepout_radius):
                self.get_logger().error(
                    'Duong bypass van cat vung dock; dung navigation de '
                    'tranh va cham.')
                return None
            current = route_goal

        self.get_logger().info(
            f'Doan den ({float(goal[0]):.2f}, {float(goal[1]):.2f}) '
            f'cat vung dock ({distance:.2f} m). '
            f'Tu chen {len(bypasses)} waypoint tranh dock: ' +
            ' -> '.join(
                f'({pose[0]:.2f}, {pose[1]:.2f})' for pose in bypasses))
        return route

    def execute_goal(self, goal, label, allow_collision_recovery=True):
        """Send one pose to Nav2 and wait for its terminal result."""
        self.get_logger().info(
            f'Gui {label}: x={float(goal[0]):.2f}, '
            f'y={float(goal[1]):.2f}, yaw={float(goal[2]):.2f}')
        if not self.goToPose(self.make_pose(goal)):
            return False

        goal_started = time.monotonic()
        # Humble's nav2_simple_commander uses isTaskComplete().
        while rclpy.ok() and not self.isTaskComplete():
            if self.dock_collision_detected:
                if not allow_collision_recovery:
                    self.get_logger().error(
                        f'{label} van va cham sau khi da giai cuu; dung navigation.')
                    self.cancelTask()
                    return False
                self.get_logger().warning(
                    f'Dung {label} de giai cuu va cham; khong doi Nav2 tu day.')
                self.cancelTask()
                cancel_started = time.monotonic()
                while (rclpy.ok() and not self.isTaskComplete() and
                       time.monotonic() - cancel_started < 2.0):
                    rclpy.spin_once(self, timeout_sec=0.05)
                if not self.recover_from_collision():
                    return False
                return self.execute_goal(
                    goal, label, allow_collision_recovery=False)
            if time.monotonic() - goal_started > self.goal_timeout:
                self.get_logger().error(f'{label} qua timeout; huy navigation.')
                self.cancelTask()
                return False
            feedback = self.getFeedback()
            if feedback is not None:
                self.get_logger().debug(
                    f'{label}: distance={feedback.distance_remaining:.2f} m')

        result = self.getResult()
        if result != TaskResult.SUCCEEDED:
            self.get_logger().error(f'{label} that bai: {result}')
            return False

        self.get_logger().info(f'Da den {label}.')
        return True

    def navigate(self):
        self.get_logger().info(
            f'Se tu dong chay {self.goal_count} goal; '
            'khong dung Nav2 Goal tren RViz.')

        if not self.publish_initial_pose_until_ready():
            return False

        # Make sure AMCL has created map -> odom and Nav2 is ready before any
        # undock/reverse motion.  This also guarantees that RobotModel can be
        # transformed into RViz's fixed frame without a manual 2D Pose click.
        self.get_logger().info('Dang cho AMCL va Nav2 active...')
        self.waitUntilNav2Active()
        self.get_logger().info(
            'AMCL/Nav2 active; RobotModel da co chuoi TF map -> odom -> base_link.')

        self.enable_full_safety_override()

        if self.auto_undock and not self.undock_robot():
            self.get_logger().error(
                'Khong the xac nhan robot roi dock; dung de tranh vuong dock.')
            return False

        self.get_logger().info('Robot da roi dock. Bat dau gui goal.')

        if self.latest_odom is not None:
            previous_goal = [
                float(self.latest_odom.x),
                float(self.latest_odom.y),
                float(self.latest_odom_yaw or 0.0),
            ]
        else:
            previous_goal = [
                float(self.get_parameter('initial_x').value),
                float(self.get_parameter('initial_y').value),
                float(self.get_parameter('initial_yaw').value),
            ]

        for index, goal in enumerate(self.goals):
            if len(goal) != 3:
                self.get_logger().error(
                    f'goal_{index + 1} phai co dang [x, y, yaw].')
                return False

            # Also check Goal 1: after reversing, a direct path toward the
            # right side of the house could still cross the physical dock.
            route = self.route_around_dock(previous_goal, goal)
            if route is None:
                return False

            for route_index, route_goal in enumerate(route):
                if route_index < len(route) - 1:
                    label = (
                        f'waypoint tranh dock {route_index + 1}/'
                        f'{len(route) - 1} truoc Goal '
                        f'{index + 1}/{self.goal_count}')
                else:
                    label = f'Goal {index + 1}/{self.goal_count}'
                if not self.execute_goal(route_goal, label):
                    return False

            previous_goal = [float(value) for value in goal]
            if index + 1 < self.goal_count:
                self.get_logger().info(
                    f'Cho {self.wait_after_goal:.1f}s truoc khi gui '
                    f'Goal {index + 2}/{self.goal_count}.')
                time.sleep(self.wait_after_goal)

        self.get_logger().info(
            f'Da hoan thanh {self.goal_count} goal autonomous navigation.')
        return True

    def undock_robot(self):
        """Use TurtleBot 4's official action to leave the charging dock."""
        self.get_logger().info(
            'Dang cho action /undock; Nav2 chua duoc gui goal.')
        started = time.monotonic()
        while rclpy.ok() and not self.undock_client.wait_for_server(
                timeout_sec=1.0):
            if time.monotonic() - started > self.undock_timeout:
                self.get_logger().warning(
                    'Khong tim thay action /undock; chuyen sang buoc lui '
                    'tu dong de roi dock.')
                return self.reverse_out_of_dock()
            self.get_logger().info('Action /undock chua san sang, dang cho...')

        if not rclpy.ok():
            return False

        self.get_logger().info('Gui yeu cau /undock...')
        send_future = self.undock_client.send_goal_async(Undock.Goal())
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning(
                'Action /undock bi tu choi trong mo phong; chuyen sang '
                'buoc lui tu dong de roi dock.')
            return self.reverse_out_of_dock()

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if time.monotonic() - started > self.undock_timeout:
                self.get_logger().warning(
                    'Action /undock qua timeout; chuyen sang buoc lui tu '
                    'dong de roi dock.')
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                return self.reverse_out_of_dock()
            rclpy.spin_once(self, timeout_sec=0.2)

        if not rclpy.ok() or result_future.result() is None:
            return self.reverse_out_of_dock()

        result = result_future.result().result
        if result.is_docked:
            self.get_logger().warning(
                'Action /undock ket thuc nhung robot van bao dang dock; '
                'chuyen sang buoc lui tu dong.')
            return self.reverse_out_of_dock()

        self.get_logger().info(
            'Robot da roi dock. Lui them mot doan thang de tao khoang an '
            'toan truoc khi cho Nav2.')
        return self.reverse_out_of_dock()

    def odom_callback(self, message):
        self.latest_odom = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.latest_odom_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))

    def reverse_out_of_dock(self):
        """Back away from the simulated dock, then verify motion using odom."""
        self.get_logger().info(
            f'Buoc khoi dong: robot tu dong lui {self.reverse_duration:.1f}s '
            f'voi toc do {self.reverse_speed:.2f} m/s.')

        # Wait briefly for odometry so the fallback cannot silently run with
        # a dead controller.
        wait_started = time.monotonic()
        while rclpy.ok() and self.latest_odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - wait_started > 5.0:
                self.get_logger().error(
                    'Khong nhan duoc /odom; khong lui robot de tranh rui ro.')
                return False

        start_x = self.latest_odom.x
        start_y = self.latest_odom.y
        reverse_started = time.monotonic()
        command = Twist()
        command.linear.x = -self.reverse_speed
        command.angular.z = 0.0
        while rclpy.ok() and time.monotonic() - reverse_started < self.reverse_duration:
            self.cmd_vel_pub.publish(command)
            rclpy.spin_once(self, timeout_sec=0.1)

        stop = Twist()
        for _ in range(10):
            self.cmd_vel_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.latest_odom is None:
            return False
        distance = math.hypot(
            self.latest_odom.x - start_x,
            self.latest_odom.y - start_y)
        if distance < 0.05:
            self.get_logger().error(
                f'Robot chi lui duoc {distance:.2f} m; van co the dang mac dock.')
            return False

        self.get_logger().info(
            f'Da lui khoi dock {distance:.2f} m va da dung. '
            'Chuyen sang Nav2.')
        time.sleep(1.0)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavNode()
    try:
        node.navigate()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
