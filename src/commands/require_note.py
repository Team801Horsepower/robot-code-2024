from commands2 import Command
from subsystems.gatherer import Gatherer


class RequireNote(Command):
    def __init__(self, gatherer: Gatherer):
        self.gatherer = gatherer
        self.should_end = False

    def initialize(self):
        self.should_end = not self.gatherer.auto_note_seen

    def isFinished(self) -> bool:
        return self.should_end
