from math import pi
from typing import List

from rev import CANSparkMax, SparkPIDController, ColorSensorV3
from wpilib import DutyCycleEncoder

# pylint: disable=too-many-instance-attributes


class Shooter:
    def __init__(self, flywheel_motors: List[int], feeder_motor: int, pitch_motor: int):
        self.pitch_motor = CANSparkMax(pitch_motor, CANSparkMax.MotorType.kBrushless)
        self.pitch_encoder = DutyCycleEncoder(9)
        self.pitch_target = 0.0

        self.feeder_motor = CANSparkMax(feeder_motor, CANSparkMax.MotorType.kBrushless)
        self.feeder_pid = self.feeder_motor.getPIDController()
        self.feeder_pid.setP(0.5)
        self.feeder_target = False

        # self.color_sensor = ColorSensorV3(9999)

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

    def set_flywheels(self, speeds: List[float]) -> None:
        self.flywheel_targets = speeds
        # for pid, target in zip(self.flywheel_pids, self.flywheel_targets):
        #     pid.setReference(target, CANSparkMax.ControlType.kVelocity)
        for motor, target in zip(self.flywheel_motors, self.flywheel_targets):
            motor.set(target)

    def get_pitch(self) -> float:
        angle_offset = 0
        angle = self.pitch_encoder.get() * 2.0 * pi + angle_offset

        while angle > pi:
            angle -= 2.0 * pi

        while angle < -pi:
            angle += 2.0 * pi

        return angle

    def set_pitch(self, pitch: float) -> None:
        self.pitch_target = pitch
        current_pitch = self.get_pitch()
        if abs(current_pitch - self.pitch_target) < 0.05:
            self.pitch_motor.set(0)
        elif current_pitch > self.pitch_target:
            self.pitch_motor.set(0.1)
        elif current_pitch < self.pitch_target:
            self.pitch_motor.set(-0.1)

    def feed(self) -> None:
        self.feeder_target = True
        # self.feeder_pid.setReference(0.5, CANSparkMax.ControlType.kVelocity)
        self.feeder_motor.set(0.5)

    def stop_feed(self) -> None:
        self.feeder_target = False
        self.feeder_motor.set(0)
        # self.feeder_pid.setReference(0, CANSparkMax.ControlType.kVelocity)

    def flywheels_ready(self) -> bool:
        flywheel_ok_threshold = 0.1
        return (
            sum(
                abs(self.flywheel_targets[i] - self.flywheel_encoders[i].getVelocity())
                for i in range(len(self.flywheel_targets))
            )
            / len(self.flywheel_targets)
            <= flywheel_ok_threshold
        )

    def pitch_ready(self) -> bool:
        pitch_ok_threshold = 0.1
        return abs(self.get_pitch() - self.pitch_target) < pitch_ok_threshold

    def note_present(self) -> bool:
        return self.color_sensor.getProximity() >= 512

    def run_shooter(self, pitch: float, velocity: float, differential: float = 0):
        flywheel_speeds = [velocity + differential, velocity - differential]
        self.set_flywheels(flywheel_speeds)
        self.set_pitch(pitch)
        if self.flywheels_ready() and self.pitch_ready() and self.note_present():
            self.feed()
        else:
            self.stop_feed()
