from navx import AHRS
from wpilib import SPI


class Odometry:
    def __init__(self):
        self.ahrs = AHRS(SPI.Port.kMXP)

    def rotation(self):
        return self.ahrs.getAngle()
