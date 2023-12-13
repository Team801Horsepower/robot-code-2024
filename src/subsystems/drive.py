from subsystems.chassis import Chassis
from subsystems.odometry import Odometry

from wpimath.geometry import Transform2d


class Drive:
    def __init__(self):
        self.chassis = Chassis()
        self.odometry = Odometry()

    def drive(self, vel: Transform2d, field_oriented: bool = False):
        if field_oriented:
            translation = vel.translation().rotateBy(self.odometry.rotation())
            vel = Transform2d(translation, vel.rotation())
        self.chassis.drive(vel)
        self.odometry.update(self.chassis)
