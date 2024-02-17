#!/usr/bin/env python3

import wpilib
import wpimath
from subsystems import chassis, drive, vision, gatherer, feeder, shooter
from commands.drive_to_pose import DriveToPose
from commands.aim_at_speaker import AimAtSpeaker

from wpilib import DriverStation
from wpimath.geometry import Transform2d, Pose2d, Rotation2d
from commands2 import CommandScheduler

from math import pi

import config


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # pylint: disable=attribute-defined-outside-init
        self.driver_controller = wpilib.XboxController(0)

        self.scheduler = CommandScheduler()

        self.drive = drive.Drive(self.scheduler)
        self.vision = vision.Vision(self.scheduler)
        self.gatherer = gatherer.Gatherer(1)
        self.feeder = feeder.Feeder(13)
        self.shooter = shooter.Shooter([14, 7], 12)

        self.field_oriented_drive = True

        aas = AimAtSpeaker(self.vision, self.drive)
        self.scheduler.schedule(aas)

    def robotPeriodic(self):
        self.scheduler.run()

    def autonomousInit(self):
        self.drive.odometry.reset()
        dtp = DriveToPose(
            Pose2d(1, 0, 3 * pi / 2),
            self.drive.odometry.pose,
            self.drive.drive,
        )
        self.scheduler.schedule(dtp)

    def autonomousPeriodic(self):
        pass

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
        if self.driver_controller.getAButtonPressed():
            self.drive.chassis.set_swerves()
        if self.driver_controller.getBButtonPressed():
            self.field_oriented_drive ^= True
        if self.driver_controller.getXButtonPressed():
            self.drive.odometry.reset()
        if self.driver_controller.getRightBumper():
            # 0.5 filler value for pitch. Pitching is currently disabled.
            self.shooter.run_shooter(0.5, 1)
        else:
            self.shooter.run_shooter(0.5, 0)

        gather_power = 0.5 * (
            self.driver_controller.getRightTriggerAxis()
            - self.driver_controller.getLeftTriggerAxis()
        )
        self.gatherer.spin_gatherer(gather_power)

        feed_power = max(self.gatherer.feed_power(), self.shooter.feed_power())
        self.feeder.run(feed_power)

    def testInit(self):
        pass

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
