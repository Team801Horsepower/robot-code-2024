from commands2 import Command
from wpimath.geometry import Pose2d, Transform2d, Rotation2d, Translation2d
from math import atan2

from commands.drive_to_pose import DriveToPose
from subsystems.drive import Drive
from utils.graph import Graph

import config


class GraphPathfind(Command):
    def __init__(self, target: Translation2d, graph: Graph, drive: Drive):
        self.target = target
        self.graph = graph
        self.drive = drive
        self.path = []
        self.dtp = None
        self.target_rot = None

    def initialize(self):
        self.path = self.graph.create_path(
            self.drive.odometry.pose().translation(), self.target
        )
        self.target_rot = (self.path[-1] - self.path[-2]).angle()

    def execute(self):
        if not self.path:
            return
        cur_pos = self.drive.odometry.pose().translation()
        if self.dtp is None:
            passthrough = 0.1 if len(self.path) == 1 else 0.5
            target_pose = Pose2d(self.path[0], self.target_rot)
            self.dtp = DriveToPose(target_pose, self.drive, passthrough=passthrough)
            self.dtp.initialize()
        self.dtp.execute()

        if self.dtp.isFinished():
            self.path.pop(0)
            self.dtp = None

    def isFinished(self):
        return len(self.path) == 0
