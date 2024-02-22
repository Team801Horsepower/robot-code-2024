from commands2 import Command
from subsystems.shooter import Shooter
import time

import config


class Shoot(Command):
    def __init__(self, shooter: Shooter):
        self.shooter = shooter
        # self.drive = drive
        # self.finished = False
        # self.waypoint_pos = waypoint_pos
        # flip_angle = 1
        # if self.waypoint_pos.y < 5.5:
        #     flip_angle = -1

        # target_angle = (
        #     90 - atan2((self.waypoint_pos.y - 5.5), (self.waypoint_pos.x - 0.2))
        # ) * flip_angle

        # self.dtp = DriveToPose(
        #     Pose2d(self.waypoint_pos.x, self.waypoint_pos.y, target_angle),
        #     self.drive.odometry.pose,
        #     self.drive.drive,
        # )
        pass

    def initialize(self):
        self.start_time = time.time()

    def execute(self):
        self.shooter.run_shooter(config.shooter_speed)

    def isFinished(self):
        return time.time() - self.start_time > 1.25

    def end(self, interrupted: bool):
        self.shooter.run_shooter(0)
