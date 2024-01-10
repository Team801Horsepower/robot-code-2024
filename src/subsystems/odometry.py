from navx import AHRS
from wpilib import SPI
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from itertools import chain
from math import pi

from subsystems.chassis import Chassis
import config


class Odometry:
    def __init__(self):
        self.ahrs = AHRS(SPI.Port.kMXP)
        self.translation = Translation2d()

    def rotation(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.ahrs.getAngle())

    def pose(self) -> Pose2d:
        return Pose2d(self.translation.x, self.translation.y, self.rotation())

    # Outputs garbage values
    # def navx_position(self) -> Translation2d:
    #     return Translation2d(self.ahrs.getDisplacementX(), self.ahrs.getDisplacementY())

    def reset(self):
        self.translation = Translation2d()
        self.ahrs.setAngleAdjustment(0)
        self.ahrs.setAngleAdjustment(-self.rotation().degrees())
        self.ahrs.resetDisplacement()

    def update(self, chassis: Chassis):
        count = 0
        total = Translation2d()

        for swerve in chain(chassis.swerves_l, chassis.swerves_r):
            delta = (
                Translation2d(1.0, 0.0)
                .rotateBy(swerve.rotation())
                .rotateBy(self.rotation())
                * (swerve.drive_encoder.getPosition() - swerve.prev_drive_enc)
                / config.drive_gear_ratio
                * config.wheel_diameter
                * pi
            )

            total += delta
            count += 1

            swerve.update_prevs()

        avg = total / count
        self.translation += avg
