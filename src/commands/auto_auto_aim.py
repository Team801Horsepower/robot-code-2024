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


class AutoAutoAim(Command):
    def __init__(self, drive: Drive, shooter: Shooter, vision: Vision):
        self.drive = drive
        self.shooter = shooter
        self.vision = vision

        self.yaw_pid = PIDController(3.0, 0, 0)

        self.atag_pos = None

        self.yaw_power = None
        self.yaw_power_threshold = 0.1

    def initialize(self):
        self.drive.drive(Transform2d())

    def execute(self):
        # Auto yaw (copy-pasted from continuous aim at speaker)
        cur_rot = self.drive.odometry.rotation().radians()
        sp_atag = self.vision.cur_speaker_atag()
        if sp_atag is not None:
            atag_pitch, atag_yaw = sp_atag

            cam_dist = (config.speaker_tag_height - config.camera_height) / tan(
                atag_pitch + config.camera_angle
            )
            robot_dist = cam_dist + config.camera_center_distance

            cam_yaw_diff = -atag_yaw
            robot_yaw_diff = atan(cam_dist / robot_dist * tan(cam_yaw_diff))

            self.atag_pos = self.drive.odometry.pose().translation() + Translation2d(
                robot_dist, Rotation2d(cur_rot + robot_yaw_diff)
            )

        if self.atag_pos is not None:
            target_yaw = (
                (self.atag_pos - self.drive.odometry.pose().translation())
                .angle()
                .radians()
            )

            while target_yaw - cur_rot > pi:
                target_yaw -= 2 * pi
            while cur_rot - target_yaw > pi:
                target_yaw += 2 * pi

            self.yaw_power = self.yaw_pid.calculate(cur_rot, target_yaw)
            drive_input = Transform2d(0, 0, self.yaw_power)
            self.drive.drive(drive_input)

        # Auto pitch
        target_pitch = self.vision.vision_pitch()
        if target_pitch is not None:
            self.shooter.set_pitch(target_pitch)

        # SmartDashboard.putNumber("aim yaw power", self.yaw_power)
        # SmartDashboard.putNumber("aim pitch ready", self.shooter.pitch_ready())

    def isFinished(self):
        return (
            self.yaw_power is not None
            and abs(self.yaw_power) < self.yaw_power_threshold
            and self.shooter.pitch_ready()
        )

    def end(self, interrupted: bool):
        self.drive.drive(Transform2d())
        self.shooter.stop_pitch()
