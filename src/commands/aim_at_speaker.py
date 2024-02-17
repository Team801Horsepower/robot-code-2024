from wpimath.controller import PIDController
from wpimath.geometry import Transform2d
from wpimath import units
from commands2 import Command

from math import atan2, tan

from subsystems.drive import Drive
from subsystems.vision import Vision
from subsystems.shooter import Shooter

import config


class AimAtSpeaker(Command):
    def __init__(self, drive: Drive, vision: Vision, shooter: Shooter):
        self.drive = drive
        self.vision = vision
        self.shooter = shooter

        self.yaw_pid = PIDController(4.6, 0, 0)
        self.yaw_power_threshold = 0.04

        self.finished = False

    def initialize(self):
        pass

    def execute(self):
        sp_atag = self.vision.cur_speaker_atag()
        if sp_atag is None:
            return
        atag_pitch, atag_yaw = sp_atag
        print("tag pitch:", atag_pitch)

        cam_dist = (config.speaker_tag_height - config.camera_height) / tan(
            atag_pitch + config.camera_angle
        )

        target_yaw = atan2(config.camera_left_offset, cam_dist)

        yaw_power = self.yaw_pid.calculate(atag_yaw, target_yaw)
        drive_input = Transform2d(0, 0, yaw_power)
        self.drive.drive(drive_input)

        shooter_pitch = atan2(
            config.speaker_height - config.shooter_height,
            cam_dist + config.camera_shooter_distance,
        )

        self.shooter.set_pitch(shooter_pitch)
        print("aiming for", units.radiansToDegrees(shooter_pitch))

        if abs(yaw_power) < self.yaw_power_threshold and self.shooter.pitch_ready():
            self.finished = True

    def isFinished(self):
        return self.finished
