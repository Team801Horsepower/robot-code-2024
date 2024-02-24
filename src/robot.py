#!/usr/bin/env python3

import wpilib
import wpimath
from subsystems import chassis, drive, vision, gatherer, feeder, shooter
from commands.drive_to_pose import DriveToPose
from commands.aim_at_speaker import AimAtSpeaker
from wpilib.event import EventLoop

from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Transform2d, Pose2d, Rotation2d
from wpimath import units
from commands2 import CommandScheduler

from math import pi, sqrt

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
        self.shooter = shooter.Shooter([14, 7], 12, 5, 16)

        self.field_oriented_drive = True

        SmartDashboard.putNumber("speaker distance", -1)
        SmartDashboard.putNumber("shooter pitch", -1)

    def robotPeriodic(self):
        self.scheduler.run()

        SmartDashboard.putNumber("speaker distance", self.vision.speaker_dist())
        SmartDashboard.putNumber(
            "shooter pitch", units.radiansToDegrees(self.shooter.get_pitch())
        )

    def autonomousInit(self):
        self.drive.odometry.reset()
        # dtp = DriveToPose(
        #     Pose2d(1, 0, 3 * pi / 2),
        #     self.drive.odometry.pose,
        #     self.drive.drive,
        # )
        # self.scheduler.schedule(dtp)

        aas = AimAtSpeaker(self.drive, self.vision, self.shooter)
        self.scheduler.schedule(aas)

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        def deadzone(activation: float) -> float:
            if abs(activation) < 0.14:
                return 0.0
            return activation

        def input_curve(input: float) -> float:
            a = 0.2
            return ((input * a) + input**3) / (1 + a)

        drive_input = wpimath.geometry.Transform2d(
            config.drive_speed
            * input_curve(deadzone(-self.driver_controller.getLeftY())),
            config.drive_speed
            * input_curve(deadzone(-self.driver_controller.getLeftX())),
            config.turn_speed
            * input_curve(deadzone(-self.driver_controller.getRightX())),
        )
        self.drive.drive(drive_input, self.field_oriented_drive)

        # Set swerves button
        if self.driver_controller.getAButtonPressed():
            self.drive.chassis.set_swerves()
        if self.driver_controller.getBButtonPressed():
            self.field_oriented_drive ^= True
        if self.driver_controller.getXButtonPressed():
            self.drive.odometry.reset()
        print(self.shooter.get_pitch())
        if self.driver_controller.getRightBumper():
            self.shooter.run_shooter(config.flywheel_setpoint)
        else:
            self.shooter.run_shooter(0)
        dpad = self.driver_controller.getPOV()
        speed = self.drive.chassis.chassis_speeds()
        if sqrt(speed.vx**2 + speed.vy**2) < 1.0:
            if dpad in [315, 0, 45]:
                self.shooter.pitch_up()
            elif dpad in [135, 180, 225]:
                self.shooter.pitch_down()
            else:
                self.shooter.stop_pitch()
        else:
            self.shooter.set_pitch(0.4, speed=1)

        if self.driver_controller.getStartButton():
            self.shooter.amp_scorer.is_up = True
            self.shooter.set_pitch(config.amp_shooter_pitch)
        elif self.driver_controller.getBackButton():
            self.shooter.amp_scorer.is_up = False
        self.shooter.amp_scorer.update()

        gather_power = (
            self.driver_controller.getRightTriggerAxis()
            - self.driver_controller.getLeftTriggerAxis()
        )
        should_rumble = self.gatherer.spin_gatherer(gather_power)
        rumble = 0.3 if should_rumble else 0
        self.driver_controller.setRumble(
            wpilib.interfaces.GenericHID.RumbleType.kRightRumble, rumble
        )

        feed_power = max(self.gatherer.feed_power(), self.shooter.feed_power(), key=abs)
        self.feeder.run(feed_power)

    def testInit(self):
        pass

    def testPeriodic(self):
        print(self.gatherer.note_present())


if __name__ == "__main__":
    wpilib.run(MyRobot)
