from rev import CANSparkMax
from wpilib import DigitalInput

from math import copysign


class Gatherer:
    def __init__(self, motor_id: int):
        self.motor = CANSparkMax(motor_id, CANSparkMax.MotorType.kBrushless)
        self.motor.setInverted(True)

        self.beam_break_top = DigitalInput(2)
        self.beam_break_bottom = DigitalInput(1)

        self.should_feed = False
        self.feed_val = 0

    # Returns whether the controller should rumble
    def spin_gatherer(self, spin_speed) -> bool:
        spin_speed *= 0.75
        # Deadzone for controller triggers/setting gather speed
        deadzone = 0.1
        should_rumble = False
        if abs(spin_speed) < deadzone:
            self.motor.set(0)
            self.feed_val = 0
            should_rumble = False
        else:
            if spin_speed < -deadzone or not self.note_present():
                self.motor.set(spin_speed)
                should_rumble = False
            else:
                self.motor.set(0)
                should_rumble = True
            if self.note_present():
                self.feed_val = 0
            else:
                self.feed_val = copysign(1, spin_speed)
        return should_rumble

    def feed_power(self) -> float:
        return 0.2 * self.feed_val

    def note_present(self) -> bool:
        return not self.beam_break_top.get()

    def note_seen(self) -> bool:
        return not self.beam_break_bottom.get()
