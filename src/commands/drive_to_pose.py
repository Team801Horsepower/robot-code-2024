from commands2 import Command
from wpimath.geometry import Pose2d, Transform2d, Rotation2d, Translation2d
from wpimath.controller import PIDController
from math import pi
from typing import Callable

from subsystems.drive import Drive

import config


class DriveToPose(Command):
    def __init__(
        self,
        target: Pose2d,
        drive: Drive,
    ):
        self.target = target
        self.drive = drive

        self.x_pid = PIDController(5.0, 0, 0)
        self.y_pid = PIDController(5.0, 0, 0)
        self.theta_pid = PIDController(6.5, 0, 0.01)

        self.pos_tolerance = 0.1
        self.theta_tolerance = 0.1

        self.finished = False

    def initialize(self):
        pass

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
        if norm > config.drive_speed:
            drive_vel *= config.drive_speed / norm
        if abs(omega) > config.turn_speed:
            omega *= config.turn_speed / abs(omega)

        if drive_vel.norm() < self.pos_tolerance:
            drive_vel = Translation2d()
        if abs(omega) < self.theta_tolerance:
            omega = 0

        drive_input = Transform2d(drive_vel, Rotation2d(omega))
        self.drive.drive(drive_input, True)

        # pos_error = (self.target.translation() - current_pose.translation()).norm()
        # theta_error = abs(rot - target_rot)

        # if pos_error < self.pos_tolerance and theta_error < self.theta_tolerance:
        if drive_vel.norm() < self.pos_tolerance and abs(omega) < self.theta_tolerance:
            self.drive.drive(Transform2d())
            self.finished = True

    def isFinished(self):
        return self.finished
