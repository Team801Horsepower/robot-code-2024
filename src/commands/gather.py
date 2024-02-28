from commands2 import Command
from subsystems.gatherer import Gatherer

from typing import Callable


class Gather(Command):
    def __init__(self, gatherer: Gatherer):
        self.gatherer = gatherer

    def initialize(self):
        pass

    def execute(self):
        self.gatherer.spin_gatherer(1)

    def isFinished(self):
        return self.gatherer.note_present()

    def end(self, interrupted):
        self.gatherer.spin_gatherer(0)
