from rev import CANSparkMax, CANSparkFlex

import time

import config


class AmpScorer:
    def __init__(self, flipper: int, scorer: int):
        self.flipper = CANSparkMax(flipper, CANSparkMax.MotorType.kBrushless)
        self.flipper_encoder = self.flipper.getEncoder()
        self.flipper.setInverted(True)
        self.flipper.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.flipper_encoder.setPosition(0)

        self.scorer = CANSparkFlex(scorer, CANSparkMax.MotorType.kBrushless)
        self.scorer.setInverted(True)

        self.is_up = False
        self.upper_limit = None
        self.at_bottom = False

        self.limit_set = False
        self.limit_cycles = 0

        self.flip_power = 0.5

    def set_flip_power(self, power: float):
        self.flipper.set(power)

    def test(self):
        print(self.flipper.getOutputCurrent())

    def update(self):
        if self.is_up:
            self.flip_up()
        else:
            self.flip_down()

    def flip_down(self):
        if not self.limit_set or self.flipper_encoder.getPosition() > 0.5:
            self.flipper.set(-self.flip_power)
            cur = self.flipper.getOutputCurrent()
            if cur > 20:
                self.limit_cycles += 1
                if self.limit_cycles >= 10:
                    self.flipper_encoder.setPosition(0)
                    self.limit_set = True
            else:
                self.limit_cycles = 0
        else:
            self.flipper.set(0)

    def flip_up(self):
        if not self.limit_set:
            self.flip_down()
        else:
            power = 0.5 * (
                config.amp_flipper_up_value - self.flipper_encoder.getPosition()
            )
            self.flipper.set(min(self.flip_power, max(0, power)))

    def set_scorer(self, power: float):
        self.scorer.set(power)
