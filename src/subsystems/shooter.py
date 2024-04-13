from math import pi
from typing import List
from functools import reduce

from rev import CANSparkMax, CANSparkFlex, SparkPIDController
from wpilib import DutyCycleEncoder, SmartDashboard
from wpimath import units
from wpimath.controller import PIDController
from commands2 import CommandScheduler, Subsystem

from subsystems.amp_scorer import AmpScorer

import time


import config

# pylint: disable=too-many-instance-attributes


class Shooter(Subsystem):
    def __init__(
        self,
        scheduler: CommandScheduler,
        flywheel_motors: List[int],
        pitch_motor: int,
        amp_flipper: int,
        amp_scorer: int,
    ):
        self.pitch_motor = CANSparkMax(pitch_motor, CANSparkMax.MotorType.kBrushless)
        self.pitch_motor.setInverted(True)
        self.pitch_encoder = DutyCycleEncoder(0)
        self.link_pivot_encoder = DutyCycleEncoder(3)
        self.pitch_target = 0.0
        self.hold_pitch = False

        self.pitch_min = units.degreesToRadians(12)
        self.pitch_max = units.degreesToRadians(59)

        self.pitch_pid = PIDController(2.8, 0, 0.05)

        self.should_feed = False
        self.feed_override = False

        self.flywheel_motors = [
            CANSparkFlex(id, CANSparkMax.MotorType.kBrushless) for id in flywheel_motors
        ]

        self.flywheel_pids: List[SparkPIDController] = [
            motor.getPIDController() for motor in self.flywheel_motors
        ]
        for flywheel_pid in self.flywheel_pids:
            flywheel_pid.setP(0.0009)
            flywheel_pid.setI(0)
            flywheel_pid.setD(0.01)
        self.flywheel_encoders = [motor.getEncoder() for motor in self.flywheel_motors]
        self.flywheel_targets = [0.0 for flywheel in self.flywheel_pids]

        self.amp_scorer = AmpScorer(amp_flipper, amp_scorer)

        self.flywheels_ready_time = time.time()

        scheduler.registerSubsystem(self)

    def periodic(self):
        SmartDashboard.putNumber(
            "pitch setpoint", units.radiansToDegrees(self.pitch_target)
        )
        if self.hold_pitch:
            self.stop_pitch()

    def get_pitch(self) -> float:
        angle_offset = 0.565696
        angle = self.pitch_encoder.get() * 2.0 * pi - angle_offset

        while angle > pi:
            angle -= 2.0 * pi

        while angle < -pi:
            angle += 2.0 * pi

        return angle

    def set_pitch(self, pitch: float, max_power: float = 1):
        self.hold_pitch = False
        pitch = min(self.pitch_max, max(self.pitch_min, pitch))
        self.pitch_target = pitch
        current_pitch = self.get_pitch()
        pid_power = self.pitch_pid.calculate(current_pitch, self.pitch_target)
        power = min(max_power, max(-max_power, pid_power))
        # TODO: Allow this to be overridden for climbing (dpad down?)
        if pid_power < 0:
            pid_power *= 0.3

        link_pivot_pos = self.link_pivot_encoder.getAbsolutePosition()
        while link_pivot_pos > 0.8:
            link_pivot_pos -= 1
        # SmartDashboard.putNumber("link pivot pos", link_pivot_pos)

        if power < 0:
            power *= 1.5

        if link_pivot_pos > 0.71:
            power = max(0, power)
        if link_pivot_pos < -0.08:
            power = min(0, power)
        self.pitch_motor.set(power)

    def stow(self):
        if self.link_pivot_encoder.getAbsolutePosition() < 0.71:
            self.pitch_down()
        else:
            self.stop_pitch()

    def pitch_up(self):
        self.set_pitch(self.get_pitch() + 1)

    def pitch_down(self):
        self.set_pitch(self.get_pitch() - 1)

    def manual_pitch(self, diff: float):
        self.set_pitch(self.get_pitch() + diff)

    def stop_pitch(self):
        if not self.hold_pitch:
            self.set_pitch(self.get_pitch())
        else:
            self.set_pitch(self.pitch_target)
        self.hold_pitch = True

    def pitch_ready(self) -> bool:
        pitch_ok_threshold = 0.025
        return abs(self.get_pitch() - self.pitch_target) < pitch_ok_threshold

    def feed_power(self) -> float:
        # return 1.0 if self.should_feed else 0
        if self.should_feed and not self.feed_override:
            if self.amp_scorer.is_up:
                return 0.3
            else:
                return 1.0
        else:
            return 0

    def set_flywheels(self, speeds: List[float]):
        self.flywheel_targets = speeds
        # for motor, target in zip(self.flywheel_motors, self.flywheel_targets):
        #     motor.set(target)
        if min(self.flywheel_targets) == 0:
            for motor in self.flywheel_motors:
                motor.set(0)
        else:
            for pid, target in zip(self.flywheel_pids, self.flywheel_targets):
                pid.setReference(target, CANSparkMax.ControlType.kVelocity)

    def flywheels_ready(self) -> bool:
        # return (
        #     min(map(lambda e: abs(e.getVelocity()), self.flywheel_encoders))
        #     >= config.flywheel_min_speed
        # )
        ready = reduce(
            bool.__and__,
            map(
                lambda e: abs(abs(e.getVelocity()) - config.flywheel_speed) < 100,
                self.flywheel_encoders,
            ),
        )
        now = time.time()
        if not ready:
            self.flywheels_ready_time = now
        return now - self.flywheels_ready_time > 0.1

    def run_shooter(self, velocity: float, differential: float = 0):
        if self.amp_scorer.is_up:
            if velocity > 0:
                self.should_feed = True
                for whl, mul in zip(self.flywheel_motors, [-1, 1]):
                    whl.set(0.1 * mul)
                self.amp_scorer.set_scorer(0.5)
            else:
                self.should_feed = False
                for whl in self.flywheel_motors:
                    whl.set(0)
                self.amp_scorer.set_scorer(0)
            return
        self.amp_scorer.set_scorer(0)
        flywheel_speeds = [-(velocity + differential), velocity - differential]
        self.set_flywheels(flywheel_speeds)

        self.should_feed = abs(velocity) > 0 and (
            self.flywheels_ready() or self.should_feed
        )

        dbg = list(map(lambda e: abs(e.getVelocity()), self.flywheel_encoders))
        SmartDashboard.putNumberArray("flywheel speeds", dbg)

    def set_feed_override(self, override: bool):
        self.feed_override = override
