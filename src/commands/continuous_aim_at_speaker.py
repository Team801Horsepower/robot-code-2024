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


class ContinuousAimAtSpeaker(Command):
    def __init__(self, drive: Drive, vision: Vision):
        self.drive = drive
        self.vision = vision

        self.yaw_pid = PIDController(4.6, 0, 0)

        self.target_yaw = self.drive.odometry.rotation().radians()

        self.should_run = False

    def initialize(self):
        pass

    def execute(self):
        cur_rot = self.drive.odometry.rotation().radians()
        sp_atag = self.vision.cur_speaker_atag()
        if sp_atag is not None:
            atag_pitch, atag_yaw = sp_atag

            cam_dist = (config.speaker_tag_height - config.camera_height) / tan(
                atag_pitch + config.camera_angle
            )

            # target_yaw = atan2(config.camera_left_offset, cam_dist)
            cam_yaw_diff = -atag_yaw
            robot_yaw_diff = atan(
                cam_dist
                / (cam_dist + config.camera_center_distance)
                * tan(cam_yaw_diff)
            )
            self.target_yaw = cur_rot + robot_yaw_diff

            SmartDashboard.putNumber(
                "robot yaw diff", units.radiansToDegrees(robot_yaw_diff)
            )
        else:
            SmartDashboard.putNumber("robot yaw diff", -1)

        if not self.should_run:
            return

        while self.target_yaw - cur_rot > pi:
            self.target_yaw -= 2 * pi
        while cur_rot - self.target_yaw > pi:
            self.target_yaw += 2 * pi

        yaw_power = self.yaw_pid.calculate(cur_rot, self.target_yaw)
        drive_input = Transform2d(0, 0, yaw_power)
        self.drive.drive(drive_input)

    def isFinished(self):
        return False
