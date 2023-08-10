"""Drive subsystem for the 2022 swerve chassis"""
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

        def setup_motors(motors: List[CANSparkMax]) -> List[tuple[CANSparkMax, SparkMaxPIDController, SparkMaxRelativeEncoder]]:
            return list(map(lambda motor: (motor, motor.getPIDController(),  motor.getEncoder()), motors))

        self.drive_l = setup_motors([
            CANSparkMax(12, CANSparkMax.MotorType.kBrushless),
            CANSparkMax(20, CANSparkMax.MotorType.kBrushless),
        ])
        self.drive_r = setup_motors([
            CANSparkMax(9, CANSparkMax.MotorType.kBrushless),
            CANSparkMax(2, CANSparkMax.MotorType.kBrushless),
        ])

        self.turn_l = setup_motors([
            CANSparkMax(11, CANSparkMax.MotorType.kBrushless),
            CANSparkMax(19, CANSparkMax.MotorType.kBrushless),
        ])
        self.turn_r = setup_motors([
            CANSparkMax(10, CANSparkMax.MotorType.kBrushless),
            CANSparkMax(1, CANSparkMax.MotorType.kBrushless),
        ])

        for _, _, encoder in chain(self.drive_l, self.drive_r):
            encoder.setPosition(0.0)
        for _, _, encoder in chain(self.turn_l, self.turn_r):
            # encoder.setPosition(0.25 * self.turn_gear_ratio)
            encoder.setPosition(0.0)

        # PIDs to tune
        for _, pid, _ in chain(self.drive_l, self.drive_r):
            pid.setP(0.0001)
        for _, pid, _ in chain(self.turn_l, self.turn_r):
            pid.setP(0.1)

        # for i in self.turn_motors:
        #     i.setIdleMode(CANSparkMax.IdleMode.kBrake)
        #     i.set(0)

    # def drive(self, power_l: float, power_r: float):
    #     for i in self.drive_l:
    #         i.set(power_l)

    #     for j in self.drive_r:
    #         j.set(power_r)

    """
    `vel`: (forward, leftward, counterclockwise), m/s, rad/s
    """

    def drive(self, vel: Transform2d) -> None:
        motors = zip(
            chain(self.drive_l, self.drive_r),
            chain(self.turn_l, self.turn_r),
            [(1, 1), (-1, 1), (1, -1), (-1, -1)],
        )
        for (_, drive_pid, _), (_, turn_pid, turn_enc), pos in motors:
            # Normalizing involves dividing by (radius * 2), but converting from
            # angular velocity to linear velocity means multiplying by radius,
            # leaving only a division by 2.
            rot_vec = config.robot_dimensions / 2.0 * vel.rotation().radians()
            rot_vec = Translation2d(rot_vec.x * pos[0], rot_vec.y * pos[1])
            rot_vec = rot_vec.rotateBy(Rotation2d.fromDegrees(90))

            total_vec = rot_vec + vel.translation()

            turn_position = total_vec.angle().degrees() / 360.0 * config.turn_gear_ratio
            # # TODO: Divide by wheel circumference
            # drive_speed = total_vec.norm() * self.drive_gear_ratio / 2.0 / pi * 60.0
            drive_speed = total_vec.norm() / (pi * config.wheel_diameter) * config.drive_gear_ratio

            # if pos == (1, 1):
            #     print(total_vec, turn_position, drive_speed, turn_enc.getPosition())

            cur_position = turn_enc.getPosition()
            half_turn = config.turn_gear_ratio / 2.0
            while turn_position < cur_position - half_turn:
                turn_position += 2 * half_turn
            while turn_position > cur_position + half_turn:
                turn_position -= 2 * half_turn

            turn_pid.setReference(
                turn_position, rev.CANSparkMaxLowLevel.ControlType.kPosition
            )
            drive_pid.setReference(
                -drive_speed, rev.CANSparkMaxLowLevel.ControlType.kVelocity
            )
