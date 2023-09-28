from navx import AHRS
from wpilib import SPI
from wpimath.geometry import Rotation2d


class Odometry:
    def __init__(self):
        self.ahrs = AHRS(SPI.Port.kMXP)

    def rotation(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.ahrs.getAngle())
