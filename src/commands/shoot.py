from commands2 import Command
from subsystems.shooter import Shooter
import time

import config


class Shoot(Command):
    def __init__(self, shooter: Shooter):
        self.shooter = shooter

    def initialize(self):
        self.start_time = time.time()

    def execute(self):
        self.shooter.run_shooter(config.flywheel_setpoint)

    def isFinished(self):
        return time.time() - self.start_time > 1.5

    def end(self, interrupted: bool):
        self.shooter.run_shooter(0)
