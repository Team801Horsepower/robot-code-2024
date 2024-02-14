#!/usr/bin/env python3

import wpilib
import wpimath
from subsystems import chassis, drive
from commands.drive_to_pose import DriveToPose
from utils.read_auto import read_auto

from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.config import (
    HolonomicPathFollowerConfig,
    ReplanningConfig,
    PIDConstants,
)
from wpilib import DriverStation
from wpimath.geometry import Transform2d, Pose2d, Rotation2d
from commands2 import CommandScheduler, Command
from functools import reduce

from math import pi

import config


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # pylint: disable=attribute-defined-outside-init
        self.driver_controller = wpilib.XboxController(0)

        self.drive = drive.Drive()

        self.field_oriented_drive = True

        self.scheduler = CommandScheduler()

    def robotPeriodic(self):
        self.drive.odometry.update(self.drive.chassis)

    def autonomousInit(self):
        self.drive.odometry.reset(Pose2d(2,7,0))
        
        pose_list = read_auto("/home/lvuser/py/autos/Biangle.json")
        dtps = []
        for pose in pose_list:
            dtp = DriveToPose(
                pose,
                self.drive.odometry.pose,
                self.drive.drive,
            )
            dtps.append(dtp)
        
        self.scheduler.schedule(reduce(Command.andThen, dtps))

        
    def autonomousPeriodic(self):
        self.scheduler.run()

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        def deadzone(activation: float) -> float:
            if abs(activation) < 0.14:
                return 0.0
            return activation

        drive_input = wpimath.geometry.Transform2d(
            config.drive_speed * deadzone(-self.driver_controller.getLeftY()),
            config.drive_speed * deadzone(-self.driver_controller.getLeftX()),
            config.turn_speed * deadzone(-self.driver_controller.getRightX()),
        )
        self.drive.drive(drive_input, self.field_oriented_drive)

        # Set swerves button
        if self.driver_controller.getAButton():
            self.drive.chassis.set_swerves()
        if self.driver_controller.getBButtonPressed():
            self.field_oriented_drive ^= True
        if self.driver_controller.getXButtonPressed():
            self.drive.odometry.reset()

    def testInit(self):
        pass

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
