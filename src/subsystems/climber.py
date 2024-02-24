from rev import CANSparkMax, SparkMaxPIDController
from typing import Tuple


class Climber:
    def __init__(self, motor_id_left: int, motor_id_right: int):
        self.motor_left = CANSparkMax(motor_id_left, CANSparkMax.MotorType.kBrushless)
        self.motor_left_pid = self.motor_left.getPIDController()
        self.motor_left_pid.setP(1)
        self.motor_left_pid.setI(0)
        self.motor_left_pid.setD(0)

        self.motor_right = CANSparkMax(motor_id_right, CANSparkMax.MotorType.kBrushless)
        self.motor_right_pid = self.motor_right.getPIDController()
        self.motor_right_pid.setP(1)
        self.motor_right_pid.setI(0)
        self.motor_right_pid.setD(0)

    def target_setpoint(self, setpoint: Tuple[float, float]) -> None:
        self.motor_right_pid.setReference(
            setpoint[0], CANSparkMax.ControlType.kPosition
        )
        self.motor_left_pid.setReference(setpoint[1], CANSparkMax.ControlType.kPosition)

    def run(self, speeds: Tuple[float, float]):
        self.motor_left.set(speeds[0])
        self.motor_right.set(speeds[1])
