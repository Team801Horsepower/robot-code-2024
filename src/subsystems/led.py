from wpilib import Spark


class Led:
    def __init__(self, led_id: int):
        self.led_driver = Spark(led_id)

    def blue_solid(self):
        self.led_driver.set(0.87)

    def blue_blink(self):
        self.led_driver.set(-0.09)

    def red_solid(self):
        self.led_driver.set(-0.17)

    def idle(self):
        # self.led_driver.set(0.69)
        self.led_driver.set(0.75)

    def off(self):
        self.led_driver.set(0.99)
