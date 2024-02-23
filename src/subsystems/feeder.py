from rev import CANSparkMax


class Feeder:
    def __init__(self, motor_id: int):
        self.motor = CANSparkMax(motor_id, CANSparkMax.MotorType.kBrushless)

        self.motor.setIdleMode(CANSparkMax.IdleMode.kBrake)

    def run(self, power: float):
        self.motor.set(power)

    def stop(self):
        self.motor.set(0)
