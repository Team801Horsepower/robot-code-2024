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

        self.use_bounds = False

    def update_note_pos(self):
        cur_pose = self.drive.odometry.pose()

        # If we're close enough to the last note location, we're
        # probably gathering it and can't see it, so we don't want
        # to update the targeted note location from a different note.
        # TODO: Make this more robust
        cur_error = (cur_pose.translation() - self.target.translation()).norm()
        if cur_error < 0.5:
            return

        relative_note_pos: Translation2d = self.vision.robot_space_note_pos()
        if relative_note_pos is not None:
            relative_note_pos = Translation2d(
                relative_note_pos.x - 0.2, relative_note_pos.y
            )
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
        if self.use_bounds:
            x = max(self.x_min, min(self.x_max, self.target.x))
            y = max(self.y_min, min(self.y_max, self.target.y))
            self.target = Pose2d(Translation2d(x, y), self.target.rotation())
        super().execute()

    def bound(self, x_min: float, x_max: float, y_min: float, y_max: float):
        self.use_bounds = True
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max


def from_dtp(dtp: DriveToPose, note_vision: NoteVision) -> ChaseNote:
    return ChaseNote(dtp.target, note_vision, dtp.drive, dtp.speed, dtp.passthrough)
