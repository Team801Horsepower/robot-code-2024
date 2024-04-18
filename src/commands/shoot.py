from commands2 import Command
from subsystems.shooter import Shooter
from subsystems.gatherer import Gatherer
import time

import config


class Shoot(Command):
    def __init__(self, shooter: Shooter, gatherer: Gatherer, keep_spin: bool = False):
        self.shooter = shooter
        self.gatherer = gatherer
        self.keep_spin = keep_spin
        self.note_was_present = False
        self.shot_time = None

    def initialize(self):
        self.start_time = time.time()
        self.shooter.set_feed_override(False)

    def execute(self):
        self.note_was_present |= self.gatherer.note_present()
        self.shooter.run_shooter(config.flywheel_setpoint)
        if (
            self.shot_time is None
            and self.note_was_present
            and not self.gatherer.note_present()
        ):
            self.shot_time = time.time()

    def isFinished(self):
        now = time.time()
        cond = now - self.start_time > 1.5
        if self.shot_time is not None:
            cond |= now - self.shot_time > 0.15
        return cond

    def end(self, interrupted: bool):
        self.gatherer.auto_note_seen = False
        if self.keep_spin:
            self.shooter.set_feed_override(True)
        else:
            self.shooter.run_shooter(0)
            self.shooter.set_feed_override(False)
