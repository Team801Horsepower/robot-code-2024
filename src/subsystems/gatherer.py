from rev import CANSparkMax, SparkMaxPIDController, ColorSensorV3
from wpilib import I2C

import config


class Gatherer:
    def __init__(self, motor_id: int):
        self.motor = CANSparkMax(motor_id, CANSparkMax.MotorType.kBrushless)
        self.motor.setInverted(True)

        self.color_sensor = ColorSensorV3(I2C.Port.kOnboard)

        self._should_feed = False

    def spin_gatherer(self, spin_speed):
        # Deadzone for controller triggers/setting gather speed
        if abs(spin_speed) < 0.1 or self.note_present():
            self.motor.set(0.0)
            self._should_feed = False
        else:
            self.motor.set(spin_speed)
            self._should_feed = True

    def should_feed(self) -> bool:
        return self._should_feed and not self.note_present()

    def note_present(self) -> bool:
        return self.color_sensor.getProximity() > config.note_proximity_threshold
