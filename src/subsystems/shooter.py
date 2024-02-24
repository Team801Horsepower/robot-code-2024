from math import pi
from typing import List
from functools import reduce

from rev import CANSparkMax, SparkPIDController, ColorSensorV3
from wpilib import DutyCycleEncoder, SmartDashboard
from wpimath import units

import time

import config

# pylint: disable=too-many-instance-attributes


class Shooter:
    def __init__(self, flywheel_motors: List[int], pitch_motor: int):
        self.pitch_motor = CANSparkMax(pitch_motor, CANSparkMax.MotorType.kBrushless)
        self.pitch_motor.setInverted(True)
        self.pitch_encoder = DutyCycleEncoder(0)
        self.pitch_target = 0.0

        self.pitch_min = units.degreesToRadians(25.7)
        self.pitch_max = units.degreesToRadians(57.8)

        self.should_feed = False

        self.flywheel_motors = [
            CANSparkMax(id, CANSparkMax.MotorType.kBrushless) for id in flywheel_motors
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

        self.flywheels_ready_time = time.time()

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

    def get_pitch(self) -> float:
        angle_offset = 4.163813417
        angle = self.pitch_encoder.get() * 2.0 * pi - angle_offset

        while angle > pi:
            angle -= 2.0 * pi

        while angle < -pi:
            angle += 2.0 * pi

        return angle

    def set_pitch(self, pitch: float, speed: float = 0.5):
        self.pitch_target = pitch
        current_pitch = self.get_pitch()
        # print("shooter at", units.radiansToDegrees(current_pitch))
        if abs(current_pitch - self.pitch_target) < 0.05:
            self.pitch_motor.set(0)
        elif current_pitch > self.pitch_target and current_pitch > self.pitch_min:
            self.pitch_motor.set(-speed)
        elif current_pitch < self.pitch_target and current_pitch < self.pitch_max:
            self.pitch_motor.set(speed)
        else:
            self.pitch_motor.set(0)

    def pitch_up(self):
        self.set_pitch(self.get_pitch() + 0.2)

    def pitch_down(self):
        self.set_pitch(self.get_pitch() - 0.2)

    def stop_pitch(self):
        self.set_pitch(self.get_pitch())

    def feed_power(self) -> float:
        return 1.0 if self.should_feed else 0

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

    def pitch_ready(self) -> bool:
        pitch_ok_threshold = 0.1
        return abs(self.get_pitch() - self.pitch_target) < pitch_ok_threshold

    def run_shooter(self, velocity: float, differential: float = 0):
        flywheel_speeds = [-(velocity + differential), velocity - differential]
        self.set_flywheels(flywheel_speeds)
        # TODO: incorporate self.pitch_ready()
        self.should_feed = abs(velocity) > 0 and (
            self.flywheels_ready() or self.should_feed
        )
        dbg = list(map(lambda e: abs(e.getVelocity()), self.flywheel_encoders))
        SmartDashboard.putNumberArray("flywheel speeds", dbg)
