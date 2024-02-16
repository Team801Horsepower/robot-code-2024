from rev import CANSparkMax, SparkMaxPIDController


class Gatherer:
    def __init__(self, motor_id: int):
        self.motor = CANSparkMax(motor_id, CANSparkMax.MotorType.kBrushless)
        self.motor.setInverted(True)

    def spin_gatherer(self, spin_speed):
        # Deadzone for controller triggers/setting gather speed
        if abs(spin_speed) < 0.1:
            self.motor.set(0.0)
        else:
            self.motor.set(spin_speed)
