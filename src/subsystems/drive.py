import wpimath
import rev
import math
from math import sin, cos, tan


from rev import CANSparkMax
from wpimath.geometry import Translation2d, Transform2d, Rotation2d
from itertools import chain

from typing import List


# 12 9 2 20
class Drive:
    def __init__(self) -> None:
        # TODO: replace with actually measured `dimensions`
        # meters, (length, width)
        self.dimensions = Translation2d(0.7, 0.4)
        self.radius = self.dimensions.norm() / 2.0

        # TODO: replace with actual gear ratios
        self.drive_gear_ratio = 1.0
        self.turn_gear_ratio = 1.0

        self.drive_l: List[CANSparkMax] = [
            CANSparkMax(12, CANSparkMax.MotorType.kBrushless),
            CANSparkMax(20, CANSparkMax.MotorType.kBrushless),
        ]
        self.drive_r: List[CANSparkMax] = [
            CANSparkMax(9, CANSparkMax.MotorType.kBrushless),
            CANSparkMax(2, CANSparkMax.MotorType.kBrushless),
        ]

        self.turn_l: List[CANSparkMax] = [
            CANSparkMax(11, CANSparkMax.MotorType.kBrushless),
            CANSparkMax(19, CANSparkMax.MotorType.kBrushless),
        ]
        self.turn_r: List[CANSparkMax] = [
            CANSparkMax(10, CANSparkMax.MotorType.kBrushless),
            CANSparkMax(1, CANSparkMax.MotorType.kBrushless),
        ]

        # PIDs to tune
        for motor in chain(self.drive_l, self.drive_r):
            motor.getPIDController().setP(0.5)
        for motor in chain(self.turn_l, self.turn_r):
            motor.getPIDController().setP(0.5)

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
        for drive_motor, turn_motor, pos in motors:
            # Normalizing involves dividing by (radius * 2), but converting from
            # angular velocity to linear velocity means multiplying by radius,
            # leaving only a division by 2.
            rot_vec = self.dimensions / 2.0 * vel.rotation().radians()
            rot_vec = Translation2d(rot_vec.x * pos[0], rot_vec.y * pos[1])
            rot_vec = rot_vec.rotateBy(
                Rotation2d(cos(math.pi / 2.0), sin(math.pi / 2.0))
            )

            total_vec = rot_vec + vel.translation()

            turn_position = total_vec.angle() * self.turn_gear_ratio
            drive_speed = total_vec.norm() * self.drive_gear_ratio

            turn_motor.getPIDController().setReference(
                turn_position.radians(), rev.CANSparkMaxLowLevel.ControlType.kPosition
            )
            drive_motor.getPIDController().setReference(
                drive_speed, rev.CANSparkMaxLowLevel.ControlType.kVelocity
            )
