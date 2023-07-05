import rev


class Drive:
    def __init__(self, id: int):
        self.motor = rev.CANSparkMax(id, rev.CANSparkMax.MotorType.kBrushless)

    def drive(self, power: float):
        self.motor.set(power)
