#!/usr/bin/env python3

import wpilib
import wpimath
from subsystems import (
    chassis,
    drive,
    vision,
    gatherer,
    feeder,
    shooter,
    climber,
    led,
    note_vision,
)
from commands.drive_to_pose import DriveToPose
from commands.aim_at_speaker import AimAtSpeaker
from commands.continuous_aim_at_speaker import ContinuousAimAtSpeaker
from commands.aim_at_pitch import AimAtPitch
from commands.auto_auto_aim import AutoAutoAim
from commands.chase_note import ChaseNote
from commands import chase_note
from commands.continuous_chase_note import ContinuousChaseNote
from wpilib.event import EventLoop
from commands.gather import Gather
from commands.shoot import Shoot
from utils.read_auto import read_auto, read_cmds

from wpilib import SmartDashboard, SendableChooser
from wpimath.geometry import Transform2d, Pose2d, Rotation2d, Translation2d
from wpimath import units
from commands2 import CommandScheduler, Command, SequentialCommandGroup
from functools import reduce

from math import pi, sqrt, copysign

import config


class Gollum(wpilib.TimedRobot):
    def robotInit(self):
        # pylint: disable=attribute-defined-outside-init
        self.driver_controller = wpilib.XboxController(0)
        self.manip_controller = wpilib.XboxController(1)

        self.scheduler = CommandScheduler()

        self.drive = drive.Drive(self.scheduler)
        self.vision = vision.Vision(self.scheduler)
        self.gatherer = gatherer.Gatherer(1)
        self.feeder = feeder.Feeder(13)
        self.shooter = shooter.Shooter(self.scheduler, [14, 7], 12, 5, 16)
        self.climber = climber.Climber(6, 15)
        self.led = led.Led(0)
        self.note_vision = note_vision.NoteVision()

        self.field_oriented_drive = True
        self.drive.odometry.reset()
        # With this enabled, the position of the turn joystick directly translates to robot heading
        self.special_turning = False
        self.use_yaw_setpoint = False
        self.yaw_setpoint = 0

        self.drive.chassis.set_swerves()
        for swerve in self.drive.chassis.swerves:
            swerve.turn_motor.set(0)

        self.aas_command = ContinuousAimAtSpeaker(self.drive, self.vision)
        self.cn_command = ContinuousChaseNote(
            self.drive.odometry.pose(), self.note_vision, self.drive
        )

        SmartDashboard.putNumber("speaker distance", -1)
        SmartDashboard.putNumber("shooter pitch", -1)
        SmartDashboard.putNumber("shooter abs enc", -1)
        SmartDashboard.putNumber("shooter abs enc abs", -1)
        SmartDashboard.putNumber("color value", -1)

        self.auto_chooser = SendableChooser()
        self.auto_chooser.setDefaultOption("4 note", 0)
        self.auto_chooser.addOption("1 note", 1)
        self.auto_chooser.addOption("amp side 1 note", 2)
        self.auto_chooser.addOption("legacy 4 note", 3)
        self.auto_chooser.addOption("blue only note theft", 4)
        self.auto_chooser.addOption("far note", 5)
        self.auto_chooser.addOption("test auto", 6)
        SmartDashboard.putData("auto select", self.auto_chooser)

    def robotPeriodic(self):
        self.scheduler.run()

        SmartDashboard.putNumber("speaker distance", self.vision.speaker_dist())
        SmartDashboard.putNumber(
            "shooter pitch", units.radiansToDegrees(self.shooter.get_pitch())
        )
        SmartDashboard.putNumber("shooter abs enc", self.shooter.pitch_encoder.get())
        SmartDashboard.putNumber(
            "shooter abs enc abs", self.shooter.pitch_encoder.getAbsolutePosition()
        )

    def autonomousInit(self):
        self.drive.chassis.set_swerves()

        autos_dir = "/home/lvuser/py/autos/"
        auto_i = self.auto_chooser.getSelected()
        red_autos = [
            "Gollum'sMiddleEarthQuest.json",
            "Gollum'sSideQuest.json",
            "Gollum'sUltraSideQuest.json",
            "Gollum'sEvenBetterQuest.json",
            "Gollum'sUltraSideQuest.json",  # Filler; TODO: Create red version of PettyTheft
            "Gollum'sRedstensiveQuest.json",
            "Gollum'sPracticeQuest.json",
        ]
        blue_autos = [
            "Gollum'sReverseEarthQuest.json",
            "Gollum'sBlueSideQuest.json",
            "Gollum'sUltraBlueSideQuest.json",
            "Gollum'sEvenRedderQuest.json",
            "PettyTheft.json",
            "Gollum'sExtensiveQuest.json",
            "Gollum'sPracticeQuest.json",
        ]
        if config.is_red():
            auto_name = red_autos[auto_i]
        else:
            auto_name = blue_autos[auto_i]
        file_path = autos_dir + auto_name

        new_cmds = []
        pose_list = read_auto(file_path)
        cmd_list = read_cmds(file_path)
        new_new_cmds = []

        self.drive.odometry.reset(pose_list[0][0])

        for pose, pitch, speed, passthrough in pose_list:
            dtp = DriveToPose(pose, self.drive, passthrough=passthrough, speed=speed)
            new_cmds.append((dtp, AimAtPitch(self.shooter, pitch)))

        for (cmd, aap), loc_cmds in zip(new_cmds, cmd_list):
            # target_pose = cmd.target
            if "6" in loc_cmds:
                cmd = chase_note.from_dtp(cmd, self.note_vision)
            cmd = cmd.alongWith(aap)
            for cmd_s in loc_cmds:
                if cmd_s == "g":
                    cmd = cmd.deadlineWith(Gather(self.gatherer))
                elif cmd_s in "G6":
                    cmd = cmd.raceWith(Gather(self.gatherer, True))
                elif cmd_s == "s" or cmd_s == "S":
                    # keep_spin = cmd_s == "S"
                    # speaker_pos = Translation2d(0.5, 5.5)

                    # aim_rotation = (speaker_pos - target_pose.translation()).angle()
                    # aim_dtp = DriveToPose(
                    #     Pose2d(target_pose.translation(), aim_rotation), self.drive
                    # )
                    # cmd = cmd.andThen(
                    #     aim_dtp.deadlineWith(Gather(self.gatherer))
                    # ).andThen(Shoot(self.shooter, self.gatherer, keep_spin))

                    # Vision-based auto aim
                    if cmd_s == "S":
                        cmd = cmd.andThen(
                            AutoAutoAim(self.drive, self.shooter, self.vision)
                        )
                    cmd = cmd.andThen(Shoot(self.shooter, self.gatherer, True))
            new_new_cmds.append(cmd)

        self.scheduler.schedule(reduce(Command.andThen, new_new_cmds))

    def autonomousPeriodic(self):
        feed_power = max(self.gatherer.feed_power(), self.shooter.feed_power())
        self.feeder.run(feed_power)

    def teleopInit(self):
        self.drive.chassis.set_swerves()
        self.shooter.set_feed_override(False)
        self.shooter.hold_pitch = False
        self.shooter.stop_pitch()
        self.use_yaw_setpoint = False
        self.aas_command.initialize()

    def teleopPeriodic(self):
        # SmartDashboard.putNumber("amp abs enc val", self.shooter.amp_scorer.flipper_encoder.getAbsolutePosition())
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

        cur_angle = self.drive.odometry.pose().rotation().radians()

        if self.special_turning and self.field_oriented_drive:
            stick_inp = Translation2d(
                -self.driver_controller.getRightY(), -self.driver_controller.getRightX()
            )
            if stick_inp.norm() >= 0.8:
                self.yaw_setpoint = stick_inp.angle().radians()

        # if self.driver_controller.getAButtonPressed():
        #     self.yaw_setpoint = pi
        #     self.use_yaw_setpoint = True
        elif self.driver_controller.getBButtonPressed():
            self.yaw_setpoint = 3 * pi / 2
            self.use_yaw_setpoint = True
        elif self.driver_controller.getXButtonPressed():
            self.yaw_setpoint = pi / 2
            self.use_yaw_setpoint = True
        elif self.driver_controller.getYButtonPressed():
            self.yaw_setpoint = 0
            self.use_yaw_setpoint = True

        turn_input = deadzone(-self.driver_controller.getRightX())
        self.use_yaw_setpoint &= turn_input == 0

        if self.use_yaw_setpoint or (
            self.special_turning and self.field_oriented_drive
        ):
            while self.yaw_setpoint - cur_angle > pi:
                self.yaw_setpoint -= 2 * pi
            while cur_angle - self.yaw_setpoint > pi:
                self.yaw_setpoint += 2 * pi
            diff = self.yaw_setpoint - cur_angle
            turn_speed = min(5 * diff, copysign(config.turn_speed, diff), key=abs)
        else:
            turn_speed = config.turn_speed * input_curve(turn_input)

        self.aas_command.should_run = (
            self.driver_controller.getLeftBumper() and self.gatherer.note_present()
        )
        self.aas_command.execute()

        self.cn_command.should_run = (
            self.driver_controller.getLeftBumper() and not self.gatherer.note_present()
        )
        self.cn_command.execute()

        if not self.aas_command.should_run and not self.cn_command.should_run:
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
        # if self.driver_controller.getLeftStickButtonPressed():
        if self.driver_controller.getAButtonPressed():
            self.field_oriented_drive ^= True
        if self.driver_controller.getStartButtonPressed():
            self.drive.odometry.reset()
            self.yaw_setpoint = 0

        should_shoot = self.driver_controller.getRightBumper()
        if should_shoot:
            self.shooter.set_feed_override(False)
            self.shooter.run_shooter(config.flywheel_setpoint)
        elif not self.shooter.feed_override:
            self.shooter.run_shooter(0)

        dpad = self.manip_controller.getPOV()
        pitch_stick = -self.manip_controller.getLeftY()
        # speed = self.drive.chassis.chassis_speeds()
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
            self.shooter.set_feed_override(False)
        else:
            if self.manip_controller.getBackButton():
                self.shooter.amp_scorer.is_up = False

            # TODO: Turn it back on once testing is done
            # prespin = not should_shoot and not self.shooter.amp_scorer.is_up
            prespin = False

            if abs(pitch_stick) > 0.01:
                self.shooter.manual_pitch(pitch_stick * 0.5)
                manip_rumble = -1
                prespin = False
            elif dpad in [315, 0, 45]:
                self.shooter.pitch_up()
                prespin = False
            elif dpad in [135, 180, 225]:
                self.shooter.pitch_down()
                prespin = False
            elif self.manip_controller.getYButton():
                self.shooter.set_pitch(0.9)
            elif self.manip_controller.getBButton():
                self.shooter.set_pitch(0.78)
            elif self.manip_controller.getAButton():
                self.shooter.set_pitch(0.69)
            elif self.manip_controller.getXButton():
                self.shooter.set_pitch(0.4)
                prespin = False
            elif self.manip_controller.getLeftBumper():
                pitch = self.vision.vision_pitch()
                if pitch is not None:
                    self.shooter.set_pitch(pitch)
            else:
                self.shooter.stop_pitch()
                prespin = False
                manip_rumble = -1

            self.shooter.set_feed_override(prespin)
            if prespin:
                self.shooter.run_shooter(config.flywheel_setpoint)

        if self.shooter.pitch_ready() and manip_rumble != -1:
            manip_rumble = 1.0
        manip_rumble = max(0, manip_rumble)

        self.shooter.amp_scorer.update()
        self.manip_controller.setRumble(
            wpilib.interfaces.GenericHID.RumbleType.kLeftRumble, manip_rumble
        )

        gather_power = (
            (
                self.driver_controller.getRightTriggerAxis()
                - self.driver_controller.getLeftTriggerAxis()
            )
            if not self.cn_command.should_run
            else 1
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

        if (
            self.shooter.pitch_ready()
            and self.aas_command.is_ready()
            and self.aas_command.should_run
        ):
            self.led.blue_solid()
        elif self.gatherer.note_seen() or self.gatherer.note_present():
            self.led.blue_blink()
        else:
            self.led.idle()

    def teleopExit(self):
        self.aas_command.end(True)
        self.cn_command.end(True)

    def testInit(self):
        pass

    def testPeriodic(self):
        feed_power = max(self.gatherer.feed_power(), self.shooter.feed_power())
        self.feeder.run(feed_power)
        test = self.note_vision.robot_space_note_pos()
        print(test)


if __name__ == "__main__":
    wpilib.run(Gollum)
