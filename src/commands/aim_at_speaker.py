from wpimath.controller import PIDController
from wpimath.geometry import Transform2d
from commands2 import Command

from subsystems.drive import Drive
from subsystems.vision import Vision


class AimAtSpeaker(Command):
    def __init__(self, drive: Drive, vision: Vision):
        self.drive = drive
        self.vision = vision

        self.yaw_pid = PIDController(0.03, 0, 0)
        self.yaw_power_threshold = 0.04

        self.finished = False

    def initialize(self):
        pass

    def execute(self):
        sp_atag = self.vision.cur_speaker_atag()
        if sp_atag is None:
            return
        atag_pitch, atag_yaw = sp_atag

        yaw_power = self.yaw_pid.calculate(atag_yaw, 0)
        drive_input = Transform2d(0, 0, yaw_power)
        self.drive.drive(drive_input)

        if abs(yaw_power) < self.yaw_power_threshold:
            self.finished = True

    def isFinished(self):
        return self.finished
