from commands2 import Command
from wpimath.geometry import Pose2d, Transform2d, Rotation2d, Translation2d
from math import atan2

from commands.drive_to_pose import DriveToPose
from commands import chase_note
from subsystems.drive import Drive
from subsystems.note_vision import NoteVision
from utils.graph import Graph

import config


class GraphPathfind(Command):
    def __init__(
        self,
        target: Translation2d,
        graph: Graph,
        drive: Drive,
        note_vision: NoteVision,
        chase_note: bool = False,
        target_rot_override: Translation2d = None,
    ):
        self.target = target
        self.graph = graph
        self.drive = drive
        self.note_vision = note_vision
        self.path = None
        self.dtp = None
        self.target_rot = None
        self.chase_note = chase_note
        self.target_rot_override = target_rot_override

    def initialize(self):
        self.path = self.graph.create_path(
            self.drive.odometry.pose().translation(), self.target
        )
        self.target_rot = self.target_rot_override or (
            (self.path[-1] - self.path[-2]).angle() + Rotation2d.fromDegrees(180)
        )

    def execute(self):
        # print("path:", self.path)
        if not self.path:
            return
        cur_pos = self.drive.odometry.pose().translation()
        if self.dtp is None:
            passthrough = 0.1 if len(self.path) == 1 else 0.4
            target_pose = Pose2d(self.path[0], self.target_rot)
            dtp = DriveToPose(target_pose, self.drive, passthrough=passthrough)
            if self.chase_note and len(self.path) == 1:
                dtp = chase_note.from_dtp(dtp, self.note_vision)
            self.dtp = dtp
            self.dtp.initialize()
        self.dtp.execute()

        if self.dtp.isFinished():
            self.path.pop(0)
            self.dtp = None

    def isFinished(self):
        # return self.path is not None and len(self.path) <= (1 if self.end_early else 0)
        return self.path is not None and len(self.path) == 0

    def end(self, interrupted: bool):
        self.drive.drive(Transform2d(), True)
