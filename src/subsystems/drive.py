from subsystems.chassis import Chassis

from wpimath.geometry import Transform2d


class Drive:
    def __init__(self):
        self.chassis = Chassis()

    def drive(self, vel: Transform2d):
        self.chassis.drive(vel)
