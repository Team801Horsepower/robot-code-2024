from commands2 import Command
import time


class MeasureTime(Command):
    def __init__(self):
        self.start_time = 0

    def initialize(self):
        self.start_time = time.time()

    def execute(self):
        pass

    def end(self, interrupted: bool):
        took = time.time() - self.start_time
        print("took", took)

    def isFinished(self) -> bool:
        return False
