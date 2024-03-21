from wpimath.controller import PIDController
from wpimath.geometry import Transform2d
from wpimath import units
from commands2 import Command

from subsystems.shooter import Shooter


class AimAtPitch(Command):
    def __init__(self, shooter: Shooter, pitch: float):
        self.shooter = shooter

        self.target = pitch

    def execute(self):
        # print("aiming at pitch", units.radiansToDegrees(self.target))
        self.shooter.set_pitch(self.target)

    def isFinished(self) -> bool:
        return self.shooter.pitch_ready()

    def end(self, interrupted: bool):
        self.shooter.hold_pitch = False
        self.shooter.stop_pitch()
