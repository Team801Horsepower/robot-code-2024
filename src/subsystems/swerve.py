from rev import CANSparkMax, SparkMaxPIDController
from wpilib import AnalogEncoder
from wpimath.geometry import Rotation2d

import config


class Swerve:
    def __init__(
        self, drive: CANSparkMax, turn: CANSparkMax, turn_abs_enc: AnalogEncoder
    ):
        self.drive_motor = drive
        self.drive_pid = self.drive_motor.getPIDController()
        self.drive_encoder = self.drive_motor.getEncoder()

        self.turn_motor = turn
        self.turn_pid = self.turn_motor.getPIDController()
        self.turn_encoder = self.turn_motor.getEncoder()

        self.turn_abs_encoder = turn_abs_enc

        self.update_prevs()

    def rotation(self) -> Rotation2d:
        return Rotation2d(self.turn_encoder.getPosition())

    def update_prevs(self):
        self.prev_drive_enc = self.drive_encoder.getPosition()
        self.prev_rotation = self.rotation()