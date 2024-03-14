import wpilib

class Led:
    def __init__(self, led_id: int):
    #     self.led = wpilib.AddressableLED(999)
    #     self.kLEDBuffer = 60
    #     self.ledData = [wpilib.AddressableLED.LEDData() for _ in range(self.kLEDBuffer)]
    #     self.led.setLength(self.kLEDBuffer)
    #     self.led.setData(self.ledData)
    #     self.led.start()
        self.led = wpilib.AddressableLED(led_id)
        self.kLEDBuffer = 60
        self.led.setLength(self.kLEDBuffer)
        self.led.start()

    def set_leds(self, red: int, green: int, blue: int):
        for i in range(self.kLEDBuffer):
            self.led.LEDData[i].setRGB(red, green, blue)