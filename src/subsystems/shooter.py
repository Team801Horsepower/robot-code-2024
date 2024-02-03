from typing import List
from rev import CANSparkMax, SparkPIDController
from wpilib import DutyCycleEncoder


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

    def start_flywheels(self, speed: float) -> None:
        self.flywheel_pids[0].setReference(speed, CANSparkMax.ControlType.kVelocity)
        self.flywheel_pids[1].setReference(-speed, CANSparkMax.ControlType.kVelocity)
