from subsystems.chassis import Chassis
from subsystems.odometry import Odometry
import config

from wpimath.geometry import Transform2d, Rotation2d
from commands2 import CommandScheduler, Subsystem

from math import pi


class Drive(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        self.chassis = Chassis()
        self.odometry = Odometry()

        scheduler.registerSubsystem(self)

    def periodic(self):
        self.odometry.update(self.chassis)

    def drive(self, vel: Transform2d, field_oriented: bool = False):
        if field_oriented:
            translation = (
                vel.translation().rotateBy(-self.odometry.rotation())
                # .rotateBy(Rotation2d(pi if config.is_red() else 0))
            )
            vel = Transform2d(translation, vel.rotation())
        self.chassis.drive(vel)
