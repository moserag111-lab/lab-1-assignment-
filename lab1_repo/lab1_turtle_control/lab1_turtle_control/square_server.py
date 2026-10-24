"""draw_square action server: reset_turtle service + exactly 4 go_to_pose goals."""
import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from lab1_interfaces.action import DrawSquare, GoToPose
from lab1_interfaces.msg import TargetPose
from lab1_turtle_control.reset_service import START_X, START_Y

DEFAULT_SIDE = 3.0
MAX_SIDE = 11.0 - 5.544445 - 0.2  # keeps the square inside the window


class SquareServer(Node):
    def __init__(self):
        super().__init__('square_server')
        cb = ReentrantCallbackGroup()
        self._reset = self.create_client(Trigger, 'reset_turtle', callback_group=cb)
        self._goto = ActionClient(self, GoToPose, 'go_to_pose', callback_group=cb)
        self._server = ActionServer(
            self, DrawSquare, 'draw_square',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=cb)
        self.get_logger().info('square_server ready (action: draw_square)')

    @staticmethod
    def _wait(future, timeout=None):
        t0 = time.monotonic()
        while not future.done():
            if timeout is not None and time.monotonic() - t0 > timeout:
                return False
            time.sleep(0.01)
        return True

    def _goal_cb(self, goal_request):
        s = goal_request.side_length
        if s < 0.0 or s > MAX_SIDE:
            self.get_logger().warn(f'Rejected side_length={s} (0..{MAX_SIDE:.2f})')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _fail(self, goal_handle, msg, cancelled=False):
        result = DrawSquare.Result(success=False, message=msg)
        if cancelled:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self.get_logger().warn(msg)
        return result

    def _execute(self, goal_handle):
        side = goal_handle.request.side_length or DEFAULT_SIDE

        # 1) reset first -> clean screen + known pose
        if not (self._reset.wait_for_service(timeout_sec=3.0)
                and self._goto.wait_for_server(timeout_sec=3.0)):
            return self._fail(goal_handle, 'reset_turtle service or go_to_pose server missing')
        fut = self._reset.call_async(Trigger.Request())
        if not self._wait(fut, 10.0) or not fut.result().success:
            return self._fail(goal_handle, 'Reset failed')

        # 2) four corners; yaw = heading of the NEXT side
        x0, y0 = START_X, START_Y
        corners = [
            (x0 + side, y0,        math.pi / 2),
            (x0 + side, y0 + side, math.pi),
            (x0,        y0 + side, -math.pi / 2),
            (x0,        y0,        0.0),
        ]
        feedback = DrawSquare.Feedback()
        for i, (x, y, yaw) in enumerate(corners, start=1):
            feedback.current_side = i
            feedback.distance_remaining = 0.0
            goal_handle.publish_feedback(feedback)

            goal = GoToPose.Goal()
            goal.target = TargetPose(x=float(x), y=float(y), yaw=float(yaw))

            def fb_cb(msg, i=i):
                feedback.current_side = i
                feedback.distance_remaining = msg.feedback.distance_remaining
                goal_handle.publish_feedback(feedback)

            send = self._goto.send_goal_async(goal, feedback_callback=fb_cb)
            self._wait(send)
            gh = send.result()
            if not gh.accepted:
                return self._fail(goal_handle, f'Side {i}: go_to_pose rejected the goal')

            res_fut = gh.get_result_async()
            while not res_fut.done():
                if goal_handle.is_cancel_requested:
                    self._wait(gh.cancel_goal_async())
                    return self._fail(goal_handle, 'Square cancelled', True)
                time.sleep(0.02)
            if not res_fut.result().result.success:
                return self._fail(goal_handle, f'Side {i} failed')
            self.get_logger().info(f'Side {i}/4 done')

        goal_handle.succeed()
        return DrawSquare.Result(success=True, message='Square complete')


def main(args=None):
    rclpy.init(args=args)
    node = SquareServer()
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
