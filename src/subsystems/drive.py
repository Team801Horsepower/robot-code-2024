from subsystems.chassis import Chassis
from subsystems.odometry import Odometry

from wpimath.geometry import Transform2d


class Drive:
    def __init__(self):
        self.chassis = Chassis()
        self.odometry = Odometry()

    def drive(self, vel: Transform2d):
        self.chassis.drive(vel)
