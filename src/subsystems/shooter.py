from math import pi
from typing import List

from rev import CANSparkMax, SparkPIDController, ColorSensorV3
from wpilib import DutyCycleEncoder

import config

# pylint: disable=too-many-instance-attributes


class Shooter:
    def __init__(self, flywheel_motors: List[int], pitch_motor: int):
        self.pitch_motor = CANSparkMax(pitch_motor, CANSparkMax.MotorType.kBrushless)
        self.pitch_encoder = DutyCycleEncoder(0)
        self.pitch_target = 0.0

        self.should_feed = False

        self.flywheel_motors = [
            CANSparkMax(id, CANSparkMax.MotorType.kBrushless) for id in flywheel_motors
        ]

        self.flywheel_pids: List[SparkPIDController] = [
            motor.getPIDController() for motor in self.flywheel_motors
        ]
        for flywheel_pid in self.flywheel_pids:
            flywheel_pid.setP(0.5)
        self.flywheel_encoders = [motor.getEncoder() for motor in self.flywheel_motors]
        self.flywheel_targets = [0.0 for flywheel in self.flywheel_pids]

    def set_flywheels(self, speeds: List[float]):
        self.flywheel_targets = speeds
        for motor, target in zip(self.flywheel_motors, self.flywheel_targets):
            motor.set(target)

    def get_pitch(self) -> float:
        angle_offset = 0.718247042956176
        angle = (self.pitch_encoder.get() - angle_offset) * 2.0 * pi

        while angle > pi:
            angle -= 2.0 * pi

        while angle < -pi:
            angle += 2.0 * pi

        return angle

    def set_pitch(self, pitch: float):
        self.pitch_target = pitch
        current_pitch = self.get_pitch()
        if abs(current_pitch - self.pitch_target) < 0.05:
            self.pitch_motor.set(0)
        elif current_pitch > self.pitch_target or current_pitch > 0.66:
            self.pitch_motor.set(1.0)
        elif current_pitch < self.pitch_target or current_pitch < 0.04:
            self.pitch_motor.set(-1.0)

    def pitch_up(self):
        self.set_pitch(self.get_pitch() + 0.06)

    def pitch_down(self):
        self.set_pitch(self.get_pitch() - 0.06)

    def stop_pitch(self):
        self.set_pitch(self.get_pitch())

    def feed_power(self) -> float:
        return 0.5 if self.should_feed else 0

    def flywheels_ready(self) -> bool:
        return (
            min(map(lambda e: abs(e.getVelocity()), self.flywheel_encoders))
            >= config.flywheel_min_speed
        )

    def pitch_ready(self) -> bool:
        pitch_ok_threshold = 0.1
        return abs(self.get_pitch() - self.pitch_target) < pitch_ok_threshold

    def note_present(self) -> bool:
        return self.color_sensor.getProximity() >= 512

    def run_shooter(self, pitch: float, velocity: float, differential: float = 0):
        flywheel_speeds = [-(velocity + differential), velocity - differential]
        self.set_flywheels(flywheel_speeds)
        # self.set_pitch(pitch)
        # TODO: incorporate self.pitch_ready()
        self.should_feed = self.flywheels_ready() and abs(velocity) > 0
