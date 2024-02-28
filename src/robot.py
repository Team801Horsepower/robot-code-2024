#!/usr/bin/env python3

import wpilib
import wpimath
from subsystems import chassis, drive, vision, gatherer, feeder, shooter, climber
from commands.drive_to_pose import DriveToPose
from commands.aim_at_speaker import AimAtSpeaker
from commands.aim_at_pitch import AimAtPitch
from wpilib.event import EventLoop
from commands.gather import Gather
from commands.shoot import Shoot
from utils.read_auto import read_auto, read_cmds

from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Transform2d, Pose2d, Rotation2d, Translation2d
from wpimath import units
from commands2 import CommandScheduler, Command, SequentialCommandGroup
from functools import reduce

from math import pi, sqrt, copysign

import config


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # pylint: disable=attribute-defined-outside-init
        self.driver_controller = wpilib.XboxController(0)
        self.manip_controller = wpilib.XboxController(1)

        self.scheduler = CommandScheduler()

        self.drive = drive.Drive(self.scheduler)
        self.vision = vision.Vision(self.scheduler)
        self.gatherer = gatherer.Gatherer(1)
        self.feeder = feeder.Feeder(13)
        self.shooter = shooter.Shooter([14, 7], 12, 5, 16)
        self.climber = climber.Climber(6, 15)

        self.field_oriented_drive = True
        self.drive.odometry.reset()
        # With this enabled, the position of the turn joystick directly translates to robot heading
        self.special_turning = False
        self.use_yaw_setpoint = False
        self.yaw_setpoint = 0

        self.drive.chassis.set_swerves()
        for swerve in self.drive.chassis.swerves:
            swerve.turn_motor.set(0)

        SmartDashboard.putNumber("speaker distance", -1)
        SmartDashboard.putNumber("shooter pitch", -1)

    def robotPeriodic(self):
        self.scheduler.run()

        SmartDashboard.putNumber("speaker distance", self.vision.speaker_dist())
        SmartDashboard.putNumber(
            "shooter pitch", units.radiansToDegrees(self.shooter.get_pitch())
        )

    def autonomousInit(self):
        self.drive.chassis.set_swerves()

        is_red = DriverStation.getAlliance() == DriverStation.Alliance.kRed
        # TODO: Enable auto selection
        if is_red:
            file_path = "/home/lvuser/py/autos/Gollum'sMiddleEarthQuest.json"
        else:
            file_path = "/home/lvuser/py/autos/Gollum'sReverseEarthQuest.json"
        new_cmds = []
        pose_list = read_auto(file_path)
        cmd_list = read_cmds(file_path)
        new_new_cmds = []

        self.drive.odometry.reset(pose_list[0][0])

        for pose, pitch in pose_list:
            dtp = DriveToPose(
                pose,
                self.drive,
            )
            new_cmds.append((dtp, AimAtPitch(self.shooter, pitch)))

        for (cmd, aap), loc_cmds in zip(new_cmds, cmd_list):
            target_pose = cmd.target
            cmd = cmd.alongWith(aap)
            for cmd_s in loc_cmds:
                if cmd_s == "g":
                    cmd = cmd.deadlineWith(Gather(self.gatherer))
                elif cmd_s == "G":
                    cmd = cmd.raceWith(Gather(self.gatherer))
                elif cmd_s == "s" or cmd_s == "S":
                    keep_spin = cmd_s == "S"
                    # speaker_pos = Translation2d(0.5, 5.5)

                    # aim_rotation = (speaker_pos - target_pose.translation()).angle()
                    # aim_dtp = DriveToPose(
                    #     Pose2d(target_pose.translation(), aim_rotation), self.drive
                    # )
                    # cmd = cmd.andThen(
                    #     aim_dtp.deadlineWith(Gather(self.gatherer))
                    # ).andThen(Shoot(self.shooter, self.gatherer, keep_spin))

                    # This shoots without auto aiming
                    cmd = cmd.andThen(Shoot(self.shooter, self.gatherer, keep_spin))
            new_new_cmds.append(cmd)

        self.scheduler.schedule(reduce(Command.andThen, new_new_cmds))

    def autonomousPeriodic(self):
        feed_power = max(self.gatherer.feed_power(), self.shooter.feed_power())
        self.feeder.run(feed_power)

    def teleopInit(self):
        self.drive.chassis.set_swerves()
        self.shooter.set_feed_override(False)

    def teleopPeriodic(self):
        def deadzone(activation: float) -> float:
            # if abs(activation) < 0.14:
            if abs(activation) < 0.01:
                return 0.0
            return activation

        # def input_curve(input: float) -> float:
        #     a = 0.2
        #     return ((input * a) + input**3) / (1 + a)
        def input_curve(input: float) -> float:
            return input

        if self.driver_controller.getRightStickButtonPressed():
            self.special_turning ^= True

        if self.special_turning and self.field_oriented_drive:
            stick_inp = Translation2d(
                -self.driver_controller.getRightY(), -self.driver_controller.getRightX()
            )
            if stick_inp.norm() >= 0.8:
                self.yaw_setpoint = stick_inp.angle().radians()
            cur_angle = self.drive.odometry.pose().rotation().radians()
            while self.yaw_setpoint - cur_angle > pi:
                self.yaw_setpoint -= 2 * pi
            while cur_angle - self.yaw_setpoint > pi:
                self.yaw_setpoint += 2 * pi
            diff = self.yaw_setpoint - cur_angle
            turn_speed = min(5 * diff, copysign(config.turn_speed, diff), key=abs)
        else:
            turn_speed = config.turn_speed * input_curve(
                deadzone(-self.driver_controller.getRightX())
            )

        drive_input = wpimath.geometry.Transform2d(
            config.drive_speed
            * input_curve(deadzone(-self.driver_controller.getLeftY())),
            config.drive_speed
            * input_curve(deadzone(-self.driver_controller.getLeftX())),
            turn_speed,
        )
        self.drive.drive(drive_input, self.field_oriented_drive)

        # Set swerves button
        if self.driver_controller.getBackButtonPressed():
            self.drive.chassis.zero_swerves()
        if self.driver_controller.getLeftStickButtonPressed():
            self.field_oriented_drive ^= True
        if self.driver_controller.getStartButtonPressed():
            self.drive.odometry.reset()
            self.yaw_setpoint = 0

        if self.driver_controller.getRightBumper():
            self.shooter.run_shooter(config.flywheel_setpoint)
        else:
            self.shooter.run_shooter(0)
        dpad = self.manip_controller.getPOV()
        speed = self.drive.chassis.chassis_speeds()
        # if sqrt(speed.vx**2 + speed.vy**2) < 1.0:
        #     if dpad in [315, 0, 45]:
        #         self.shooter.pitch_up()
        #     elif dpad in [135, 180, 225]:
        #         self.shooter.pitch_down()
        #     else:
        #         self.shooter.stop_pitch()
        # else:
        #     self.shooter.set_pitch(0.4, speed=1)
        # if sqrt(speed.vx**2 + speed.vy**2) >= 1.0:
        #     self.shooter.set_pitch(0.4, speed=1)
        # elif self.manip_controller.getStartButton():
        manip_rumble = 0
        if self.manip_controller.getStartButton():
            self.shooter.amp_scorer.is_up = True
            self.shooter.set_pitch(config.amp_shooter_pitch)
        else:
            if self.manip_controller.getBackButton():
                self.shooter.amp_scorer.is_up = False

            if self.manip_controller.getXButton():
                self.shooter.set_pitch(0.4, speed=1)
            elif dpad in [315, 0, 45]:
                self.shooter.pitch_up()
            elif dpad in [135, 180, 225]:
                self.shooter.pitch_down()
            elif self.manip_controller.getYButton():
                self.shooter.set_pitch(0.9, speed=1)
            elif self.manip_controller.getBButton():
                self.shooter.set_pitch(0.78, speed=1)
            elif self.manip_controller.getAButton():
                self.shooter.set_pitch(0.69, speed=1)
            elif self.manip_controller.getLeftBumper():
                pitch = self.vision.vision_pitch()
                if pitch is not None:
                    self.shooter.set_pitch(pitch)
            else:
                self.shooter.stop_pitch()
                manip_rumble = -1

        if self.shooter.pitch_ready() and manip_rumble != -1:
            manip_rumble = 1.0
        manip_rumble = max(0, manip_rumble)

        self.shooter.amp_scorer.update()
        self.manip_controller.setRumble(
            wpilib.interfaces.GenericHID.RumbleType.kLeftRumble, manip_rumble
        )

        gather_power = (
            self.driver_controller.getRightTriggerAxis()
            - self.driver_controller.getLeftTriggerAxis()
        )
        should_rumble = self.gatherer.spin_gatherer(gather_power)
        rumble = 1.0 if should_rumble else 0
        self.driver_controller.setRumble(
            wpilib.interfaces.GenericHID.RumbleType.kRightRumble, rumble
        )

        feed_power = max(self.gatherer.feed_power(), self.shooter.feed_power(), key=abs)
        self.feeder.run(feed_power)

        if self.manip_controller.getRightBumper():
            self.climber.motor_right.set(-1)
            self.climber.motor_left.set(-1)
        else:
            self.climber.run(
                (
                    self.manip_controller.getLeftTriggerAxis(),
                    self.manip_controller.getRightTriggerAxis(),
                )
            )

    def testInit(self):
        pass

    def testPeriodic(self):
        print(self.gatherer.note_present())


if __name__ == "__main__":
    wpilib.run(MyRobot)
