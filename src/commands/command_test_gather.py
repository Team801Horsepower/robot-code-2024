from commands2 import Command
from subsystems import gatherer

from typing import Callable


class Gather(Command):
    def __init__(self, next_dtp):
        self.finished = False

    def initialize(self):
        pass

    def execute(self):
        if self.finished:
            return
        Gatherer(0.7)

    def isFinished(self):
        return self.finished
