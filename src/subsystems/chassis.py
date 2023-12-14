from subsystems.swerve import Swerve
import config

import wpimath
import rev
from math import sin, cos, tan, pi


from rev import CANSparkMax, SparkMaxPIDController, SparkMaxRelativeEncoder
from wpilib import AnalogEncoder
from wpimath.geometry import Translation2d, Transform2d, Rotation2d
from itertools import chain

from typing import List, Tuple


# pylint: disable=missing-docstring, too-few-public-methods
class Chassis:
    def __init__(self) -> None:
        def make_swerve(t: Tuple[int, int, int]) -> Swerve:
            drive_id, turn_id, turn_abs_enc_id = t
            return Swerve(
                CANSparkMax(drive_id, CANSparkMax.MotorType.kBrushless),
                CANSparkMax(turn_id, CANSparkMax.MotorType.kBrushless),
                AnalogEncoder(turn_abs_enc_id),
            )

        #                                                    Swerve CAN IDs
        # Old Chassis
        # self.swerves_l = list(map(lambda t: make_swerve(*t), [(12, 11), (20, 19)]))
        # self.swerves_r = list(map(lambda t: make_swerve(*t), [(9, 10), (2, 1)]))
        # New Chassis
        # self.swerves_l = list(map(lambda t: make_swerve(*t), [(8, 14, 2), (4, 7, 1)]))
        # self.swerves_r = list(map(lambda t: make_swerve(*t), [(2, 15, 3), (1, 5, 0)]))
        # Newer Chassis
        self.swerves_l = list(map(make_swerve, [(12, 11, 0), (20, 19, 3)]))
        self.swerves_r = list(map(make_swerve, [(10, 9, 1), (2, 1, 2)]))
        # TODO: Don't hardcode this
        abs_enc_vals = [
            0.8576425,
            0.4142499,
            0.6548459,
            0.2648037,
        ]

        for swerve, abs_enc_val in zip(
            chain(self.swerves_l, self.swerves_r), abs_enc_vals
        ):
            swerve.drive_encoder.setPosition(0.0)
            swerve.turn_motor.setInverted(True)

            # Divide by 0.66 because the function expects 5V but the encoders use 3.3V.
            # TODO: Change this once the encoders move to 5V.
            cur_turn = swerve.turn_abs_encoder.getAbsolutePosition() - abs_enc_val
            swerve.turn_encoder.setPosition(-cur_turn * config.turn_gear_ratio)

            # PIDs to tune
            swerve.drive_pid.setP(0.0001)
            swerve.turn_pid.setP(0.1)

    def set_swerves(self):
        for swerve in chain(self.swerves_l, self.swerves_r):
            cur_position = swerve.turn_encoder.getPosition()
            half_turn = config.turn_gear_ratio / 2.0
            zero_position = 0.0
            while zero_position < cur_position - half_turn:
                zero_position += 2.0 * half_turn
            while zero_position > cur_position + half_turn:
                zero_position -= 2.0 * half_turn
            swerve.turn_pid.setReference(
                zero_position, rev.CANSparkMaxLowLevel.ControlType.kPosition
            )

    """
    `vel`: (forward, leftward, counterclockwise), m/s, rad/s
    """

    def drive(self, vel: Transform2d) -> None:
        # Equality check is fine here since the deadzone handles rounding to 0.
        if vel.rotation().degrees() == 0.0 and vel.translation().norm() == 0.0:
            for swerve in chain(self.swerves_l, self.swerves_r):
                swerve.drive_pid.setReference(
                    0.0, rev.CANSparkMaxLowLevel.ControlType.kVelocity
                )
            return

        swerves = zip(
            chain(self.swerves_l, self.swerves_r),
            [(1, 1), (-1, 1), (1, -1), (-1, -1)],
        )
        for swerve, pos in swerves:
            # Normalizing involves dividing by (radius * 2), but converting from
            # angular velocity to linear velocity means multiplying by radius,
            # leaving only a division by 2.
            rot_vec = config.robot_dimensions / 2.0 * vel.rotation().radians()
            rot_vec = Translation2d(rot_vec.x * pos[0], rot_vec.y * pos[1])
            rot_vec = rot_vec.rotateBy(Rotation2d.fromDegrees(90.0))

            total_vec = rot_vec + vel.translation()

            turn_position = total_vec.angle().degrees() / 360.0 * config.turn_gear_ratio
            # Multiply by 60 for RPM
            drive_speed = (
                total_vec.norm()
                / (pi * config.wheel_diameter)
                * config.drive_gear_ratio
                * 60.0
            )

            cur_position = swerve.turn_encoder.getPosition()
            half_turn = config.turn_gear_ratio / 2.0

            while turn_position < cur_position - half_turn:
                turn_position += 2.0 * half_turn
            while turn_position > cur_position + half_turn:
                turn_position -= 2.0 * half_turn

            # Flip check
            quarter_turn = half_turn / 2.0
            if turn_position < cur_position - quarter_turn:
                turn_position += half_turn
                drive_speed *= -1.0
            elif turn_position > cur_position + quarter_turn:
                turn_position -= half_turn
                drive_speed *= -1.0

            swerve.turn_pid.setReference(
                turn_position, rev.CANSparkMaxLowLevel.ControlType.kPosition
            )
            swerve.drive_pid.setReference(
                -drive_speed, rev.CANSparkMaxLowLevel.ControlType.kVelocity
            )
