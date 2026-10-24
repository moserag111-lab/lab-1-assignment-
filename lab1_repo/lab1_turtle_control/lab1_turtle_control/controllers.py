"""Motion controllers (Strategy design pattern).

TurtleController depends only on the abstract MotionController, so a different
control law can be plugged in without touching the action-server code.
"""
import math
from abc import ABC, abstractmethod

WORLD_MIN = 0.0
WORLD_MAX = 11.0


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class MotionController(ABC):
    @abstractmethod
    def drive_command(self, x, y, theta, tx, ty, max_lin, max_ang):
        """Return (linear, angular, distance_to_target)."""

    @abstractmethod
    def rotate_command(self, theta, target_yaw, max_ang):
        """Return (angular, yaw_error)."""


class ProportionalController(MotionController):
    def __init__(self, k_lin=1.5, k_ang=6.0, k_yaw=4.0, heading_threshold=0.35):
        self.k_lin = k_lin
        self.k_ang = k_ang
        self.k_yaw = k_yaw
        self.heading_threshold = heading_threshold  # rad: rotate in place above this

    def drive_command(self, x, y, theta, tx, ty, max_lin, max_ang):
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        heading_err = normalize_angle(math.atan2(dy, dx) - theta)
        angular = clamp(self.k_ang * heading_err, -max_ang, max_ang)
        if abs(heading_err) > self.heading_threshold:
            linear = 0.0  # turn first, then drive
        else:
            linear = clamp(self.k_lin * dist, 0.0, max_lin)
        return linear, angular, dist

    def rotate_command(self, theta, target_yaw, max_ang):
        err = normalize_angle(target_yaw - theta)
        return clamp(self.k_yaw * err, -max_ang, max_ang), err
