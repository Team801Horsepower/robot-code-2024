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
        return Rotation2d.fromDegrees(-self.ahrs.getAngle())

    def pose(self) -> Pose2d:
        return Pose2d(self.translation.x, self.translation.y, self.rotation())

    # Outputs garbage values
    # def navx_position(self) -> Translation2d:
    #     return Translation2d(self.ahrs.getDisplacementX(), self.ahrs.getDisplacementY())

    def reset(self, pose: Pose2d = Pose2d()):
        self.translation = pose.translation()
        self.ahrs.setAngleAdjustment(0)
        self.ahrs.setAngleAdjustment(
            self.rotation().degrees() - pose.rotation().degrees()
        )
        self.ahrs.resetDisplacement()

    def update(self, chassis: Chassis):
        count = 0
        total = Translation2d()

        for swerve in chassis.swerves:
            # Grab previous values and update immediately to prevent any loss of delta
            prev_drive = swerve.prev_drive_enc
            prev_turn = swerve.prev_rotation
            swerve.update_prevs()

            delta = Translation2d(
                prev_drive - swerve.drive_encoder.getPosition(), 0
            ).rotateBy(swerve.rotation() + self.rotation())

            total += delta
            count += 1

        avg = total / count
        self.translation += avg
