#!/usr/bin/env python3

import wpilib
import wpimath
from subsystems import chassis, drive, vision, gatherer, feeder, shooter
from commands.drive_to_pose import DriveToPose
from commands.aim_at_speaker import AimAtSpeaker
from commands.aim_at_pitch import AimAtPitch
from wpilib.event import EventLoop
from commands.gather import Gather
from commands.shoot import Shoot
from utils.read_auto import read_auto, read_cmds

from wpilib import DriverStation
from wpimath.geometry import Transform2d, Pose2d, Rotation2d, Translation2d
from wpimath import units
from commands2 import CommandScheduler, Command, SequentialCommandGroup
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
        # print(self.gatherer.color_sensor.getProximity())

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
        # self.drive.odometry.reset(Pose2d(2, 7, 0))
        file_path = "/home/lvuser/py/autos/Gollum'sEvenBetterQuest.json"
        dtps = []
        # auto_cmds = []
        new_cmds = []
        pose_list = read_auto(file_path)
        cmd_list = read_cmds(file_path)
        new_new_cmds = []

        self.drive.odometry.reset(pose_list[0][0])

        for pose, pitch in pose_list:
            # dtp = DriveToPose(
            #     pose,
            #     self.drive.odometry.pose,
            #     self.drive.drive,
            # )
            dtp = DriveToPose(
                pose,
                self.drive,
            )
            dtps.append(dtp)
            # new_cmds.append(dtp.alongWith(AimAtPitch(self.shooter, pitch)))
            new_cmds.append((dtp, AimAtPitch(self.shooter, pitch)))

        # for i in range(len(dtps)):
        #     gather = False
        #     if bool(cmd_list[i]) == True:
        #         for cmd in cmd_list[i]:
        #             if cmd == "g":
        #                 gather = True
        #                 # auto_cmds.append(eval("dtps[i] Command.deadlineWith Gather()"))
        #                 auto_cmds.append(dtps[i].deadlineWith(Gather(self.gatherer)))
        #             elif cmd == "s":
        #                 auto_cmds.append(Shoot(self.drive, pose_list[i]))
        #     if gather == True:
        #         pass
        #     else:
        #         auto_cmds.append(dtps[i])
        for i in range(len(new_cmds)):
            cmd, aap = new_cmds[i]
            target_pose = cmd.target
            cmd = cmd.alongWith(aap)
            for cmd_s in cmd_list[i]:
                if cmd_s == "g":
                    cmd = cmd.deadlineWith(Gather(self.gatherer))
                elif cmd_s == "s":
                    # cmd = cmd.andThen(Shoot(self.shooter))
                    # speaker_pos = Translation2d(0, 5.5)
                    speaker_pos = Translation2d(0.5, 5.5)
                    aim_rotation = (speaker_pos - target_pose.translation()).angle()
                    aim_dtp = DriveToPose(
                        Pose2d(target_pose.translation(), aim_rotation), self.drive
                    )
                    # TODO: Use AimAtSpeaker
                    cmd = (
                        cmd.andThen(
                            aim_dtp.deadlineWith(Gather(self.gatherer))
                            # We might want to remove this deadline with because of that "sg" vs "gs" issue
                            # ).andThen(Shoot(self.shooter).deadlineWith(Gather(self.gatherer)))
                        )
                        # .andThen(AimAtSpeaker(self.drive, self.vision, self.shooter))
                        .andThen(Shoot(self.shooter))
                    )
            # new_cmds[i] = cmd
            new_new_cmds.append(cmd)

        # self.scheduler.schedule(reduce(Command.andThen, auto_cmds))
        # self.scheduler.schedule(reduce(Command.andThen, new_cmds))
        self.scheduler.schedule(reduce(Command.andThen, new_new_cmds))

    def autonomousPeriodic(self):
        feed_power = max(self.gatherer.feed_power(), self.shooter.feed_power())
        self.feeder.run(feed_power)

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
            self.drive.odometry.reset(Pose2d(0, 0, pi))
        if self.driver_controller.getRightBumper():
            self.shooter.run_shooter(config.shooter_speed)
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

        print(units.radiansToDegrees(self.shooter.get_pitch()))

    def testInit(self):
        pass

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
