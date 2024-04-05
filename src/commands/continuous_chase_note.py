from subsystems.note_vision import NoteVision
from subsystems.drive import Drive
from commands.chase_note import ChaseNote
from wpimath.geometry import Translation2d, Pose2d, Rotation2d

import config


class ContinuousChaseNote(ChaseNote):
    def __init__(
        self,
        target: Pose2d,
        note_vision: NoteVision,
        drive: Drive,
        speed: float = config.auto_drive_speed,
    ):
        self.should_run = False
        super().__init__(target, note_vision, drive, speed, 0)

    def execute(self):
        if self.should_run:
            super().execute()
        else:
            # Only run in else branch because it
            # gets run anyway in super().execute()
            self.update_note_pos()

    def isFinished(self) -> bool:
        return False
