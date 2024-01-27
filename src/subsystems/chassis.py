from subsystems.swerve import Swerve
import config

import wpimath
import rev
from math import sin, cos, tan, pi


from rev import CANSparkMax, SparkMaxPIDController, SparkMaxRelativeEncoder
from wpilib import AnalogEncoder
from wpimath.geometry import Translation2d, Transform2d, Rotation2d
from itertools import chain

from wpimath.kinematics import ChassisSpeeds

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

        for swerve, abs_enc_val in zip(
            chain(self.swerves_l, self.swerves_r), config.abs_enc_vals
        ):
            swerve.drive_encoder.setPosition(0.0)
            swerve.turn_motor.setInverted(True)

            print(swerve.turn_abs_encoder.getAbsolutePosition())
            cur_turn = swerve.turn_abs_encoder.getAbsolutePosition() - abs_enc_val

            # Makes turn encoders operate in radians
            swerve.turn_encoder.setPositionConversionFactor(
                2 * pi / config.turn_gear_ratio
            )
            swerve.turn_encoder.setPosition(2 * pi * cur_turn)

            # Makes drive encoders operate in m/s
            swerve.drive_encoder.setVelocityConversionFactor(
                pi * config.wheel_diameter / 60 / config.drive_gear_ratio
            )
            swerve.drive_encoder.setPositionConversionFactor(
                pi * config.wheel_diameter / config.drive_gear_ratio
            )

            # PIDs to tune
            swerve.drive_pid.setP(0.15)
            swerve.turn_pid.setP(0.5)

            swerve.update_prevs()

        self.drive_input = Transform2d()

    def set_swerves(self):
        for swerve, abs_enc_val in zip(
            chain(self.swerves_l, self.swerves_r), config.abs_enc_vals
        ):
            cur_turn = swerve.turn_abs_encoder.getAbsolutePosition() - abs_enc_val
            swerve.turn_encoder.setPosition(cur_turn * config.turn_gear_ratio)
            swerve.turn_encoder.setPosition(2 * pi * cur_turn)
            swerve.turn_pid.setReference(
                0.0, rev.CANSparkLowLevel.ControlType.kPosition
            )

    """
    `vel`: (forward, leftward, counterclockwise), m/s, rad/s
    """

    def drive(self, vel: Transform2d) -> None:
        # Equality check is fine here since the deadzone handles rounding to 0.
        if vel.rotation().degrees() == 0.0 and vel.translation().norm() == 0.0:
            for swerve in chain(self.swerves_l, self.swerves_r):
                swerve.drive_pid.setReference(
                    0.0, rev.CANSparkLowLevel.ControlType.kVelocity
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

            turn_angle = total_vec.angle().radians()
            drive_speed = total_vec.norm()

            cur_position = swerve.turn_encoder.getPosition()

            while turn_angle < cur_position - pi:
                turn_angle += 2.0 * pi
            while turn_angle > cur_position + pi:
                turn_angle -= 2.0 * pi

            # Flip check
            quarter_turn = pi / 2.0
            if turn_angle < cur_position - quarter_turn:
                turn_angle += pi
                drive_speed *= -1.0
            elif turn_angle > cur_position + quarter_turn:
                turn_angle -= pi
                drive_speed *= -1.0

            swerve.turn_pid.setReference(
                turn_angle, rev.CANSparkLowLevel.ControlType.kPosition
            )
            swerve.drive_pid.setReference(
                -drive_speed, rev.CANSparkLowLevel.ControlType.kVelocity
            )

        self.drive_input = vel

    # Robot relative
    def chassis_speeds(self) -> ChassisSpeeds:
        def hadamard(a: Translation2d, b: Tuple[float, float]) -> Translation2d:
            return Translation2d(a.x * b[0], a.y * b[1])

        def cross(a: Translation2d, b: Translation2d) -> float:
            return a.x * b.y - a.y * b.x

        movs = []
        rots = []
        swerves = zip(
            chain(self.swerves_l, self.swerves_r),
            [(1, 1), (-1, 1), (1, -1), (-1, -1)],
        )
        for swerve, pos in swerves:
            v = (
                Translation2d(1, 0).rotateBy(
                    Rotation2d.fromDegrees(swerve.turn_encoder.getPosition() * 180 / pi)
                )
                * swerve.drive_encoder.getVelocity()
            )
            d = config.robot_dimensions
            dnorm = d.norm()

            rot = 2 * cross(v, hadamard(d / dnorm, pos)) / dnorm
            rots.append(rot)

            movs.append(v)

        mov = reduce(Translation2d.__add__, movs) / len(movs)
        rot = sum(rots) / len(rots)

        return ChassisSpeeds(-mov.x, -mov.y, rot)
