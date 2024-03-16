from wpilib import Spark
import time

class Led:
    def __init__(self, led_id: int):
        self.led_driver = Spark(led_id)
        self.start_blink = -1

    def set_leds(self, test_value):
        self.led_driver.set(test_value)
        self.start_blink = time.time()

    def stop_blink(self):
        if time.time() - self.start_blink >= 3:
            self.set_leds(0.99)
