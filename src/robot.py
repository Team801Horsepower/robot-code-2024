#!/usr/bin/env python3

import wpilib
import wpimath
from subsystems import chassis, drive, vision, gatherer, feeder, shooter
from commands.drive_to_pose import DriveToPose
from commands.aim_at_speaker import AimAtSpeaker
from wpilib.event import EventLoop
from commands.command_test_gather import Gather
from commands.command_test_shoot import Shoot
from utils.read_auto import read_auto, read_cmds

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

        self.scheduler = CommandScheduler()

        self.drive = drive.Drive(self.scheduler)
        self.vision = vision.Vision(self.scheduler)
        self.gatherer = gatherer.Gatherer(1)
        self.feeder = feeder.Feeder(13)
        self.shooter = shooter.Shooter([14, 7], 12)

        self.field_oriented_drive = True

    def robotPeriodic(self):
        self.scheduler.run()

    def autonomousInit(self):
        # self.drive.odometry.reset()
        # # dtp = DriveToPose(
        # #     Pose2d(1, 0, 3 * pi / 2),
        # #     self.drive.odometry.pose,
        # #     self.drive.drive,
        # # )
        # # self.scheduler.schedule(dtp)

        # aas = AimAtSpeaker(self.drive, self.vision, self.shooter)
        # self.scheduler.schedule(aas)
        self.drive.odometry.reset(Pose2d(2, 7, 0))
        file_path = "/home/lvuser/py/autos/Biangle.json"
        dtps = []
        auto_cmds = []
        pose_list = read_auto(file_path)
        cmd_list = read_cmds(file_path)

        for pose in pose_list:
            dtp = DriveToPose(
                pose,
                self.drive.odometry.pose,
                self.drive.drive,
            )
            dtps.append(dtp)

        for i in range(len(dtps)):
            gather = False
            if bool(cmd_list[i]) == True:
                for cmd in cmd_list[i]:
                    if cmd == "g":
                        gather = True
                        # auto_cmds.append(eval("dtps[i] Command.deadlineWith Gather()"))
                        auto_cmds.append(dtps[i].deadlineWith(Gather(self.gatherer)))
                    elif cmd == "s":
                        auto_cmds.append(Shoot(self.drive, pose_list[i]))
            if gather == True:
                pass
            else:
                auto_cmds.append(dtps[i])

        self.scheduler.schedule(reduce(Command.andThen, auto_cmds))

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
        if self.driver_controller.getRightBumper():
            self.shooter.run_shooter(5600)
        else:
            self.shooter.run_shooter(0)
        dpad = self.driver_controller.getPOV()
        if dpad in [315, 0, 45]:
            self.shooter.pitch_up()
        elif dpad in [135, 180, 225]:
            self.shooter.pitch_down()
        else:
            self.shooter.stop_pitch()

        # self.driver_controller.leftBumper(
        #     EventLoop().bind(
        #         AimAtSpeaker(self.drive, self.vision, self.shooter).execute
        #     )
        # )

        self.driver_controller.button

        gather_power = 0.5 * (
            self.driver_controller.getRightTriggerAxis()
            - self.driver_controller.getLeftTriggerAxis()
        )
        self.gatherer.spin_gatherer(gather_power)

        feed_power = max(self.gatherer.feed_power(), self.shooter.feed_power())
        self.feeder.run(feed_power)

        # if self.shooter.flywheels_ready():
        #     print([encoder.getVelocity() for encoder in self.shooter.flywheel_encoders])
        # print([encoder.getVelocity() for encoder in self.shooter.flywheel_encoders])

    def testInit(self):
        pass

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
