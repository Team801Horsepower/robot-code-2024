from subsystems.note_vision import NoteVision
from subsystems.drive import Drive
from commands.drive_to_pose import DriveToPose
from wpimath.geometry import Translation2d, Pose2d, Rotation2d

import config


class ChaseNote(DriveToPose):
    def __init__(
        self,
        target: Pose2d,
        note_vision: NoteVision,
        drive: Drive,
        speed: float = config.auto_drive_speed,
        passthrough: float = 0,
    ):
        self.vision = note_vision
        super().__init__(target, drive, speed, passthrough)

    def update_note_pos(self):
        cur_pose = self.drive.odometry.pose()

        # TODO: Make this more robust
        cur_error = (cur_pose.translation() - self.target.translation()).norm()
        if cur_error < 0.5:
            return

        relative_note_pos: Translation2d = self.vision.robot_space_note_pos()
        if relative_note_pos is not None:
            field_relative_note_pos = relative_note_pos.rotateBy(cur_pose.rotation())
            rotation = field_relative_note_pos.angle().rotateBy(
                Rotation2d.fromDegrees(180)
            )
            new_target = Pose2d(
                cur_pose.translation() + field_relative_note_pos, rotation
            )
            self.target = new_target

    def execute(self):
        self.update_note_pos()
        super().execute()


def from_dtp(dtp: DriveToPose, note_vision: NoteVision) -> ChaseNote:
    return ChaseNote(dtp.target, note_vision, dtp.drive, dtp.speed, dtp.passthrough)
