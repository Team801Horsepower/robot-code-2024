"""Drive subsystem for the 2022 swerve chassis"""

from swerve import Swerve

import wpimath
import rev
from math import sin, cos, tan, pi


from rev import CANSparkMax, SparkMaxPIDController, SparkMaxRelativeEncoder
from wpimath.geometry import Translation2d, Transform2d, Rotation2d
from itertools import chain

from typing import List

import config
# pylint: disable=missing-docstring, too-few-public-methods
class Drive:
    def __init__(self) -> None:
        def make_swerve(drive_id: int, turn_id: int) -> Swerve:
            return Swerve(
                CANSparkMax(drive_id, CANSparkMax.MotorType.kBrushless),
                CANSparkMax(turn_id, CANSparkMax.MotorType.kBrushless),
            )

        self.swerves_l = list(map(lambda t: make_swerve(*t), [(12, 11), (20, 19)]))
        self.swerves_r = list(map(lambda t: make_swerve(*t), [(9, 10), (2, 1)]))

        for swerve in chain(self.swerves_l, self.swerves_r):
            swerve.drive_encoder.setPosition(0.0)
            swerve.turn_encoder.setPosition(0.0)

            # PIDs to tune
            swerve.drive_pid.setP(0.0001)
            swerve.turn_pid.setP(0.1)

    """
    `vel`: (forward, leftward, counterclockwise), m/s, rad/s
    """

    def drive(self, vel: Transform2d) -> None:
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
            rot_vec = rot_vec.rotateBy(Rotation2d.fromDegrees(90))

            total_vec = rot_vec + vel.translation()

            turn_position = total_vec.angle().degrees() / 360.0 * config.turn_gear_ratio
            drive_speed = total_vec.norm() / (pi * config.wheel_diameter) * config.drive_gear_ratio

            cur_position = swerve.turn_encoder.getPosition()
            half_turn = config.turn_gear_ratio / 2.0
            while turn_position < cur_position - half_turn:
                turn_position += 2 * half_turn
            while turn_position > cur_position + half_turn:
                turn_position -= 2 * half_turn

            swerve.turn_pid.setReference(
                turn_position, rev.CANSparkMaxLowLevel.ControlType.kPosition
            )
            swerve.drive_pid.setReference(
                -drive_speed, rev.CANSparkMaxLowLevel.ControlType.kVelocity
            )
