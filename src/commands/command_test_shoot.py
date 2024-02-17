from commands2 import Command, CommandScheduler
from wpimath.geometry import Pose2d
from math import atan2

from typing import Callable


class Shoot(Command):
    def __init__(self, waypoint_pos):
        self.finished = False
        self.waypoint_pos = waypoint_pos
        self.scheduler = CommandScheduler()

    def initialize(self):
        flip_angle = 1
        if self.waypoint_pos[1] < 5.5:
            flip_angle = -1

        target_angle = (
            90 - atan2((self.waypoint_pos[1] - 5.5), (self.waypoint_pos[0] - 0.2))
        ) * flip_angle

        dtp = DriveToPose(
            Pose2d(self.waypoint_pos[0], self.waypoint_pos[1], target_angle),
            self.drive.odometry.pose,
            self.drive.drive,
        )

        self.scheduler.schedule(dtp)

    def execute(self):
        if self.finished:
            return

        self.scheduler.run()

        self.scheduler.onCommandFinish(eval("self.finished = True"))
        # self.scheduler.onCommandFinish(lambda: self.finished = True)

    def isFinished(self):
        print("done shooting")
        return self.finished
