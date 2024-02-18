from commands2 import Command, CommandScheduler
from wpimath.geometry import Pose2d
from math import atan2
from subsystems.drive import Drive
from commands.drive_to_pose import DriveToPose
from typing import Callable


class Shoot(Command):
    def __init__(self, drive: Drive, waypoint_pos):
        self.drive = drive
        self.finished = False
        self.waypoint_pos = waypoint_pos
        flip_angle = 1
        if self.waypoint_pos.y < 5.5:
            flip_angle = -1

        target_angle = (
            90 - atan2((self.waypoint_pos.y - 5.5), (self.waypoint_pos.x - 0.2))
        ) * flip_angle

        self.dtp = DriveToPose(
            Pose2d(self.waypoint_pos.x, self.waypoint_pos.y, target_angle),
            self.drive.odometry.pose,
            self.drive.drive,
        )

    def initialize(self):
        pass

    def execute(self):
        if self.finished:
            return

        CommandScheduler.initCommand(self.dtp)

        self.scheduler.onCommandFinish(eval("self.finished = True"))
        # self.scheduler.onCommandFinish(lambda: self.finished = True)

    def isFinished(self):
        print("done shooting")
        return self.finished
