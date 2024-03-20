from commands2 import Command
from wpimath.geometry import Pose2d, Transform2d, Rotation2d, Translation2d
from wpimath.controller import PIDController
from math import pi, sqrt
from typing import Callable
import time

from subsystems.drive import Drive

import config


class DriveToPose(Command):
    def __init__(
        self,
        target: Pose2d,
        drive: Drive,
        speed: float = config.auto_drive_speed,
        passthrough: bool = False,
    ):
        self.target = target
        self.drive = drive
        self.speed = speed

        self.x_pid = PIDController(5.0, 0, 0)
        self.y_pid = PIDController(5.0, 0, 0)
        self.theta_pid = PIDController(5.0, 0, 0.03)

        self.pos_tolerance = 0.05 if not passthrough else 0.5
        self.theta_tolerance = 0.1

        self.finished = False

        self.slow_time = time.time()

    def initialize(self):
        self.slow_time = time.time()

    def execute(self):
        if self.finished:
            return

        current_pose = self.drive.odometry.pose()

        rot = current_pose.rotation().radians()
        target_rot = self.target.rotation().radians()
        while target_rot > rot + pi:
            target_rot -= 2 * pi
        while target_rot < rot - pi:
            target_rot += 2 * pi

        x_speed = self.x_pid.calculate(
            current_pose.translation().x, self.target.translation().x
        )
        y_speed = self.y_pid.calculate(
            current_pose.translation().y, self.target.translation().y
        )
        omega = self.theta_pid.calculate(rot, target_rot)

        drive_vel = Translation2d(x_speed, y_speed)
        norm = drive_vel.norm()
        if norm > self.speed:
            drive_vel *= self.speed / norm
        if abs(omega) > config.auto_turn_speed:
            omega *= config.auto_turn_speed / abs(omega)

        if drive_vel.norm() < self.pos_tolerance:
            drive_vel = Translation2d()
        if abs(omega) < self.theta_tolerance:
            omega = 0

        drive_input = Transform2d(drive_vel, Rotation2d(omega))
        self.drive.drive(drive_input, True)

        speeds = self.drive.chassis.chassis_speeds()
        is_slow = (
            sqrt(speeds.vx**2 + speeds.vy**2) < 0.05 and abs(speeds.omega) < 0.1
        )
        now = time.time()
        if not is_slow:
            self.slow_time = now

        if now - self.slow_time > 0.5 or (
            drive_vel.norm() < self.pos_tolerance and abs(omega) < self.theta_tolerance
        ):
            self.drive.drive(Transform2d())
            self.finished = True

    def isFinished(self):
        return self.finished
