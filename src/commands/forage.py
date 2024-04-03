from commands2 import Command
from wpimath.geometry import Transform2d
from wpimath.controller import PIDController

from subsystems.drive import Drive
from subsystems.gatherer import Gatherer
from subsystems.note_vision import NoteVision
import config


class Forage(Command):
    def __init__(
        self,
        drive: Drive,
        gatherer: Gatherer,
        note_vision: NoteVision,
        speed: float = config.auto_drive_speed,
    ):
        self.drive = drive
        self.gatherer = gatherer
        self.note_vision = note_vision

        self.speed = speed

        self.finished = False

        self.note_pos = (0, 0)
        self.drive_pid = PIDController(0.1, 0, 0)
        self.turn_pid = PIDController(0.1, 0, 0)

    def initialize(self):
        pass

    def execute(self):
        if self.finished:
            return

        self.note_pos = self.note_vision.get_notes() or self.note_pos

        r, theta = self.note_pos

        if self.gatherer.note_seen():
            self.drive.drive(Transform2d())
            self.finished = True
            return
        else:
            drive_speed = 0.2  # self.drive_pid.calculate(r, 0)
            turn_speed = self.drive_pid.calculate(theta, 0)
            # self.drive.drive(Transform2d(-drive_speed, 0, turn_speed))
            self.drive.drive(Transform2d())
            # self.gatherer.spin_gatherer(0.5)

    def isFinished(self):
        return self.finished
