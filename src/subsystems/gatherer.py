from rev import CANSparkMax, SparkMaxPIDController, ColorSensorV3
from wpilib import I2C, DigitalInput

import config


class Gatherer:
    def __init__(self, motor_id: int):
        self.motor = CANSparkMax(motor_id, CANSparkMax.MotorType.kBrushless)
        self.motor.setInverted(True)

        # self.color_sensor = ColorSensorV3(I2C.Port.kOnboard)
        self.beam_break_top = DigitalInput(2)
        self.beam_break_bottom = DigitalInput(1)

        self.should_feed = False

    def spin_gatherer(self, spin_speed):
        # Deadzone for controller triggers/setting gather speed
        if abs(spin_speed) < 0.1 or self.note_present():
            self.motor.set(0.0)
            self.should_feed = False
        else:
            self.motor.set(spin_speed)
            self.should_feed = True

    def feed_power(self) -> float:
        return 0.1 if self.should_feed else 0

    def note_present(self) -> bool:
        # return self.color_sensor.getProximity() > config.note_proximity_threshold
        # return not (self.beam_break_top.get() and self.beam_break_bottom.get())
        return not self.beam_break_top.get()
