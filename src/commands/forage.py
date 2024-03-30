from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from subsystems.drive import Drive
from commands.drive_to_pose import DriveToPose

import config

class Forage(DriveToPose):
    def __init__(
        self,
        target,
        drive: Drive,
        speed: float = config.auto_drive_speed,
    ):

        self.drive = drive

        super().__init__(target, drive, speed, passthrough=0)
        self.finished = False
        self.note_delta = None

    def initialize(self):
        super().initialize()
        # self.dtp = DriveToPose(, self.drive, speed=self.speed, passthrough=False)

    def get_note(self):
        r = 2
        theta = 0
        return (r, theta)

    def execute(self):
        if self.finished:
            return

        current_pose = self.drive.odometry.pose()
        self.note_delta = self.get_note() or self.note_delta
        if self.note_delta is None:
            return

        note_translation = current_pose.translation() + Translation2d(self.note_delta[0], self.note_delta[1]).rotateBy(current_pose.rotation())

        note_pos = Pose2d(note_translation, Rotation2d(0))

        super().target = note_pos

        super().execute()


    def isFinished(self):
        return self.finished
