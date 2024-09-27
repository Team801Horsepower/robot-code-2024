from commands2 import Command
from wpimath.geometry import Translation2d
from wpimath import units

import time
from typing import List

from utils.graph import Graph
from commands.graph_pathfind import GraphPathfind
from commands.gather import Gather
from commands.auto_auto_aim import AutoAutoAim
from commands.shoot import Shoot
from commands.aim_at_pitch import AimAtPitch
from subsystems.drive import Drive
from subsystems.gatherer import Gatherer
from subsystems.shooter import Shooter
from subsystems.vision import Vision
from subsystems.note_vision import NoteVision
import config
from config import flip_red


GATHERING = 0
SHOOTING = 1
FINISHED = 2


class GraphAuto(Command):
    def __init__(
        self,
        graph_path: str,
        notes: List[Translation2d],
        drive: Drive,
        gatherer: Gatherer,
        shooter: Shooter,
        vision: Vision,
        note_vision: NoteVision,
    ):
        self.graph = Graph(graph_path)
        self.notes = notes

        self.drive = drive
        self.gatherer = gatherer
        self.shooter = shooter
        self.vision = vision
        self.note_vision = note_vision

        self.state = None
        self.cmd = None

        self.last_transition = time.time()

    def initialize(self):
        print("BEGIN AUTO")
        if self.gatherer.note_present():
            self.transition_shooting()
        else:
            self.transition_gathering()

    def execute(self):
        if self.state == FINISHED:
            return
        elif self.state == GATHERING and (
            self.cmd.isFinished() or self.gatherer.note_present()
        ):
            self.transition_shooting()
        elif self.state == SHOOTING and (
            self.cmd.isFinished()
            or (
                not self.gatherer.note_present()
                and time.time() - self.last_transition > 1
            )
        ):
            self.transition_gathering()
        self.cmd.execute()

    def transition_gathering(self):
        print("NOW GATHERING")
        if not self.notes:
            if self.cmd is not None:
                self.cmd.end(True)
            self.state = FINISHED
            self.last_transition = time.time()
            return
        note = self.notes.pop(0)
        if self.cmd is not None:
            self.cmd.end(True)
        self.cmd = (
            GraphPathfind(
                note, self.graph, self.drive, self.note_vision, chase_note=True
            )
            .deadlineWith(AimAtPitch(self.shooter, units.degreesToRadians(20)))
            .raceWith(Gather(self.gatherer, True))
        )
        self.cmd.initialize()
        self.state = GATHERING
        self.last_transition = time.time()

    def transition_shooting(self):
        print("NOW SHOOTING")
        shoot_pos_s = [self.graph.nodes[i] for i in self.graph.shoot_idxs]
        cur_pos = self.drive.odometry.pose().translation()
        nearest_shoot_pos = min(shoot_pos_s, key=lambda pos: (pos - cur_pos).norm())
        speaker_pos = Translation2d(flip_red(0.0), 5.54)
        # speaker_pos = Translation2d(2.2606, 3.7178)
        shoot_rot = (speaker_pos - nearest_shoot_pos).angle()
        if self.cmd is not None:
            self.cmd.end(True)
        self.cmd = (
            GraphPathfind(
                nearest_shoot_pos,
                self.graph,
                self.drive,
                self.note_vision,
                target_rot_override=shoot_rot,
            )
            .deadlineWith(AimAtPitch(self.shooter, units.degreesToRadians(20)))
            .deadlineWith(Gather(self.gatherer))
            .andThen(AutoAutoAim(self.drive, self.shooter, self.vision))
            .andThen(Shoot(self.shooter, self.gatherer, True))
        )
        self.cmd.initialize()
        self.state = SHOOTING
        self.last_transition = time.time()

    def isFinished(self) -> bool:
        return self.state == FINISHED

    def end(self, interrupted: bool):
        if self.cmd is not None:
            self.cmd.end(interrupted)
