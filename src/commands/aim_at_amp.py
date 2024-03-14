from wpimath.controller import PIDController
from wpimath.geometry import Transform2d
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

        self.strafe_pid = PIDController(0.3, 0, 0)
        self.yaw_pid = PIDController(4.6, 0, 0)

        self.strafe_power = 0

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

            # target_yaw = atan2(config.camera_left_offset, cam_dist)
            cam_yaw_diff = -atag_yaw
            # minus sign jumpscare
            robot_yaw_diff = atan(
                cam_dist
                / (cam_dist + config.camera_center_distance)
                * tan(cam_yaw_diff)
            )
            # self.target_yaw = cur_rot - atag_yaw
            self.target_yaw = cur_rot + robot_yaw_diff

            self.strafe_power = self.strafe_pid.calculate(atag_yaw, 0)

        if not self.should_run:
            return

        yaw_power = self.yaw_pid.calculate(cur_rot, self.side_yaw)
        drive_input = Transform2d(0, self.strafe_power, yaw_power)
        self.drive.drive(drive_input)

    def isFinished(self):
        return False
