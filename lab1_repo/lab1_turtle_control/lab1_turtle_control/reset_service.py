"""reset_turtle service: uses ONLY turtlesim's own services (clear, set_pen, teleport_absolute)."""
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from turtlesim.srv import SetPen, TeleportAbsolute

START_X = 5.544445
START_Y = 5.544445
START_YAW = 0.0


class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        cb = ReentrantCallbackGroup()
        self._clear = self.create_client(Empty, '/clear', callback_group=cb)
        self._pen = self.create_client(SetPen, '/turtle1/set_pen', callback_group=cb)
        self._tp = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute',
                                      callback_group=cb)
        self.create_service(Trigger, 'reset_turtle', self._reset_cb, callback_group=cb)
        self.get_logger().info('reset_service ready (service: reset_turtle)')

    @staticmethod
    def _wait(future, timeout=3.0):
        t0 = time.monotonic()
        while not future.done():
            if time.monotonic() - t0 > timeout:
                return False
            time.sleep(0.01)
        return True

    def _call(self, client, request):
        return self._wait(client.call_async(request))

    def _reset_cb(self, request, response):
        for c in (self._clear, self._pen, self._tp):
            if not c.wait_for_service(timeout_sec=1.0):
                response.success = False
                response.message = f'turtlesim service {c.srv_name} not available'
                return response

        pen_off = SetPen.Request(r=179, g=184, b=255, width=3, off=1)
        pen_on = SetPen.Request(r=179, g=184, b=255, width=3, off=0)
        tp = TeleportAbsolute.Request(x=START_X, y=START_Y, theta=START_YAW)

        ok = (self._call(self._pen, pen_off)          # 1) lift pen (no line while teleporting)
              and self._call(self._tp, tp)            # 2) teleport to start
              and self._call(self._clear, Empty.Request())  # 3) clear screen
              and self._call(self._pen, pen_on))      # 4) pen down again
        response.success = bool(ok)
        response.message = 'Turtle reset' if ok else 'A turtlesim call timed out'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ResetService()
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
