#!/usr/bin/env python3

import wpilib
import wpimath
from subsystems import chassis, drive, shooter
from commands.drive_to_pose import DriveToPose

from wpilib import DriverStation
from wpimath.geometry import Transform2d, Pose2d, Rotation2d
from commands2 import CommandScheduler

from math import pi

import config


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # pylint: disable=attribute-defined-outside-init
        self.driver_controller = wpilib.XboxController(0)

        self.drive = drive.Drive()

        self.field_oriented_drive = True

        self.scheduler = CommandScheduler()

        self.shooter = shooter.Shooter([14, 7], 13, 12)

    def robotPeriodic(self):
        self.drive.odometry.update(self.drive.chassis)

    def autonomousInit(self):
        self.drive.odometry.reset()
        dtp = DriveToPose(
            Pose2d(1, 0, 3 * pi / 2),
            self.drive.odometry.pose,
            self.drive.drive,
        )
        self.scheduler.schedule(dtp)

    def autonomousPeriodic(self):
        self.scheduler.run()

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        print(self.shooter.get_pitch())
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

        self.shooter.set_pitch(0.3 * self.driver_controller.getRightY() + 0.3)
        # self.shooter.set_pitch(0.3)

        # Set swerves button
        if self.driver_controller.getAButtonPressed():
            self.drive.chassis.set_swerves()
        if self.driver_controller.getBButtonPressed():
            self.field_oriented_drive ^= True
        if self.driver_controller.getXButtonPressed():
            self.drive.odometry.reset()
        if self.driver_controller.getYButtonPressed():
            self.shooter.set_flywheels([-1.0, 1.0])
            self.shooter.feed()

    def testInit(self):
        pass

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
