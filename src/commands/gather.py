from commands2 import Command
from subsystems.gatherer import Gatherer


class Gather(Command):
    def __init__(self, gatherer: Gatherer, short: bool = False):
        self.gatherer = gatherer
        self.short = short

        self.short_finished = False

    def initialize(self):
        pass

    def execute(self):
        self.gatherer.spin_gatherer(1)
        self.short_finished |= self.short and self.gatherer.note_seen()

        self.gatherer.auto_note_seen |= (
            self.gatherer.note_seen() or self.gatherer.note_present()
        )

    def isFinished(self) -> bool:
        return self.gatherer.note_present() or self.short_finished

    def end(self, interrupted):
        self.gatherer.spin_gatherer(0)
