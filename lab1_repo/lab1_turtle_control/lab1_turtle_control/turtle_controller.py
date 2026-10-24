"""goToPose action server + speed-limit parameters + speed-limit service."""
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim.msg import Pose

from lab1_interfaces.action import GoToPose
from lab1_interfaces.srv import SetSpeedLimits
from lab1_turtle_control.controllers import (
    WORLD_MAX, WORLD_MIN, ProportionalController, normalize_angle)

LIN_PARAM = 'max_linear_speed'
ANG_PARAM = 'max_angular_speed'
POS_TOL = 0.05    # units
YAW_TOL = 0.03    # rad
TIMEOUT = 60.0    # s
LOOP_DT = 0.02    # s


def _range_descriptor(text):
    return ParameterDescriptor(
        description=text,
        floating_point_range=[FloatingPointRange(from_value=0.01, to_value=10.0, step=0.0)])


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self._cb_group = ReentrantCallbackGroup()
        self._controller = ProportionalController()  # Strategy
        self._pose = None
        self._busy = False
        self._lock = threading.Lock()

        self.declare_parameter(LIN_PARAM, 2.0, _range_descriptor('Max linear speed (units/s)'))
        self.declare_parameter(ANG_PARAM, 2.0, _range_descriptor('Max angular speed (rad/s)'))

        self._cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self._pose_cb, 10,
                                 callback_group=self._cb_group)

        self._server = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group)

        self.create_service(SetSpeedLimits, 'set_speed_limits', self._set_limits_cb,
                            callback_group=self._cb_group)
        self.get_logger().info('turtle_controller ready (action: go_to_pose, '
                               'service: set_speed_limits)')

    # ---------------- helpers ----------------
    def _pose_cb(self, msg):
        self._pose = msg

    def _limits(self):
        # Read on EVERY call so changes apply live.
        return (self.get_parameter(LIN_PARAM).get_parameter_value().double_value,
                self.get_parameter(ANG_PARAM).get_parameter_value().double_value)

    def _publish(self, lin, ang):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self._cmd_pub.publish(msg)

    def _stop(self):
        self._publish(0.0, 0.0)

    # ---------------- service ----------------
    def _set_limits_cb(self, request, response):
        if request.max_linear <= 0.0 or request.max_angular <= 0.0:
            response.success = False
            response.message = 'Both limits must be > 0'
            return response
        result = self.set_parameters_atomically([
            Parameter(LIN_PARAM, Parameter.Type.DOUBLE, float(request.max_linear)),
            Parameter(ANG_PARAM, Parameter.Type.DOUBLE, float(request.max_angular)),
        ])
        response.success = result.successful
        response.message = ('Limits updated' if result.successful
                            else 'Rejected: ' + result.reason)
        self.get_logger().info(response.message)
        return response

    # ---------------- action ----------------
    def _goal_cb(self, goal_request):
        t = goal_request.target
        if not (WORLD_MIN <= t.x <= WORLD_MAX and WORLD_MIN <= t.y <= WORLD_MAX):
            self.get_logger().warn('Rejected: target outside 0..11')
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                self.get_logger().warn('Rejected: another goal is running')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _finish(self, goal_handle, result, ok, message, cancelled=False):
        self._stop()
        pose = self._pose
        t = goal_handle.request.target
        if pose is not None:
            result.final_distance_error = float(
                ((t.x - pose.x) ** 2 + (t.y - pose.y) ** 2) ** 0.5)
            result.final_yaw_error = float(abs(normalize_angle(t.yaw - pose.theta)))
        result.success = ok
        result.message = message
        if cancelled:
            goal_handle.canceled()
        elif ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _execute(self, goal_handle):
        result = GoToPose.Result()
        feedback = GoToPose.Feedback()
        t = goal_handle.request.target
        try:
            start = time.monotonic()
            # wait for first pose
            while self._pose is None:
                if time.monotonic() - start > 3.0:
                    return self._finish(goal_handle, result, False, 'No pose received')
                time.sleep(LOOP_DT)

            # Phase 1: go to (x, y)
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    return self._finish(goal_handle, result, False, 'Cancelled', True)
                if time.monotonic() - start > TIMEOUT:
                    return self._finish(goal_handle, result, False, 'Timeout')
                p = self._pose
                max_lin, max_ang = self._limits()
                lin, ang, dist = self._controller.drive_command(
                    p.x, p.y, p.theta, t.x, t.y, max_lin, max_ang)
                if dist <= POS_TOL:
                    break
                self._publish(lin, ang)
                feedback.distance_remaining = float(dist)
                feedback.yaw_error = float(normalize_angle(t.yaw - p.theta))
                goal_handle.publish_feedback(feedback)
                time.sleep(LOOP_DT)
            self._stop()

            # Phase 2: rotate to final yaw
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    return self._finish(goal_handle, result, False, 'Cancelled', True)
                if time.monotonic() - start > TIMEOUT:
                    return self._finish(goal_handle, result, False, 'Timeout')
                p = self._pose
                _, max_ang = self._limits()
                ang, err = self._controller.rotate_command(p.theta, t.yaw, max_ang)
                if abs(err) <= YAW_TOL:
                    break
                self._publish(0.0, ang)
                feedback.distance_remaining = 0.0
                feedback.yaw_error = float(err)
                goal_handle.publish_feedback(feedback)
                time.sleep(LOOP_DT)

            return self._finish(goal_handle, result, True, 'Pose reached')
        finally:
            with self._lock:
                self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
