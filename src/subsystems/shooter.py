from typing import List
from rev import CANSparkMax, SparkPIDController
from wpilib import DutyCycleEncoder
from math import pi


class Shooter:
    def __init__(self, flywheel_motors: List[int], feeder_motor: int, pitch_motor: int):
        self.pitch_motor = CANSparkMax(pitch_motor, CANSparkMax.MotorType.kBrushless)
        self.pitch_encoder = DutyCycleEncoder(9)

        self.feeder_motor = CANSparkMax(feeder_motor, CANSparkMax.MotorType.kBrushless)
        self.feeder_pid = self.feeder_motor.getPIDController()
        self.feeder_pid.setP(0.5)

        self.flywheel_motors: List[CANSparkMax] = list(
            map(
                lambda id: CANSparkMax(id, CANSparkMax.MotorType.kBrushless),
                flywheel_motors,
            )
        )
        self.flywheel_pids: List[SparkPIDController] = [
            motor.getPIDController() for motor in self.flywheel_motors
        ]
        for flywheel_pid in self.flywheel_pids:
            flywheel_pid.setP(0.5)

    def set_flywheels(self, speeds: List[float]) -> None:
        for pair in zip(self.flywheel_pids, speeds):
            pair[0].setReference(pair[1], CANSparkMax.ControlType.kVelocity)

    def get_pitch(self) -> float:
        angle_offset = 0
        angle = self.pitch_encoder.get() * 2.0 * pi + angle_offset

        while angle > pi:
            angle -= 2.0 * pi

        while angle < pi:
            angle += 2.0 * pi

        return angle

    def seek_pitch(self, target_pitch: float) -> None:
        current_pitch = self.get_pitch()
        if abs(current_pitch - target_pitch) < 0.05:
            self.pitch_motor.set(0)
        elif current_pitch > target_pitch:
            self.pitch_motor.set(0.1)
        elif current_pitch < target_pitch:
            self.pitch_motor.set(-0.1)

    def feed(self) -> None:
        self.feeder_pid.setReference(0.5, CANSparkMax.ControlType.kVelocity)

    def stop_feed(self) -> None:
        self.feeder_pid.setReference(0, CANSparkMax.ControlType.kVelocity)
