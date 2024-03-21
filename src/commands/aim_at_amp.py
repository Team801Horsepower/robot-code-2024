from wpimath.controller import PIDController
from wpimath.geometry import Transform2d, Translation2d, Rotation2d
from wpimath import units
from commands2 import Command

from wpilib import SmartDashboard

from math import atan, atan2, tan, pi

from subsystems.drive import Drive
from subsystems.vision import Vision
from subsystems.shooter import Shooter

import config


class StrafeToAmp(Command):
    def __init__(self, drive: Drive, vision: Vision):
        self.drive = drive
        self.vision = vision

        self.strafe_pid = PIDController(2.2, 0, 0)
        self.yaw_pid = PIDController(4.6, 0, 0)

        self.atag_pos = None

        self.should_run = False

        if config.is_red():
            self.side_yaw = 3 * pi / 2
        else:
            self.side_yaw = pi / 2

    def initialize(self):
        pass

    def execute(self):
        cur_rot = self.drive.odometry.rotation().radians()
        amp_atag = self.vision.cur_amp_atag()
        if amp_atag is not None:
            atag_pitch, atag_yaw = amp_atag

            cam_dist = (config.amp_tag_height - config.camera_height) / tan(
                atag_pitch + config.camera_angle
            )
            robot_dist = cam_dist + config.camera_center_distance

            cam_yaw_diff = -atag_yaw
            robot_yaw_diff = atan(cam_dist / robot_dist * tan(cam_yaw_diff))

            self.atag_pos = self.drive.odometry.pose().translation() + Translation2d(
                robot_dist, Rotation2d(cur_rot + robot_yaw_diff)
            )

        if not self.should_run:
            return

        if self.atag_pos is not None:
            strafe_power = self.strafe_pid.calculate(
                self.drive.odometry.pose().x, self.atag_pos.x
            )
        else:
            strafe_power = 0

        target_yaw = self.side_yaw
        while target_yaw - cur_rot > pi:
            target_yaw -= 2 * pi
        while target_yaw - cur_rot < -pi:
            target_yaw += 2 * pi

        yaw_power = self.yaw_pid.calculate(cur_rot, target_yaw)
        drive_input = Transform2d(0, strafe_power, yaw_power)
        self.drive.drive(drive_input)

    def isFinished(self):
        return False
