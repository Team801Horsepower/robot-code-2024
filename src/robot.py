#!/usr/bin/env python3

import wpilib
import wpimath
from subsystems import chassis, drive

from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.config import (
    HolonomicPathFollowerConfig,
    ReplanningConfig,
    PIDConstants,
)
from wpilib import DriverStation
from wpimath.geometry import Transform2d
from commands2 import CommandScheduler

import config


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # pylint: disable=attribute-defined-outside-init
        self.driver_controller = wpilib.XboxController(0)

        self.drive = drive.Drive()

        self.field_oriented_drive = True

        AutoBuilder.configureHolonomic(
            self.drive.odometry.pose,
            self.drive.odometry.reset,
            self.drive.chassis.chassis_speeds,
            lambda cs: self.drive.chassis.drive(Transform2d(cs.vx, cs.vy, cs.omega)),
            HolonomicPathFollowerConfig(
                PIDConstants(0.4, 0, 0),
                PIDConstants(0.4, 0, 0),
                2.5,
                16.97056275,
                ReplanningConfig(),
            ),
            lambda: False,
            self.drive,
        )

        self.auto_command = PathPlannerAuto("Circle Auto")

    def autonomousInit(self):
        self.auto_command.initialize()

    def autonomousPeriodic(self):
        if not self.auto_command.isFinished():
            self.auto_command.execute()

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        def deadzone(activation: float) -> float:
            if abs(activation) < 0.1:
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
