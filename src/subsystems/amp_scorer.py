from rev import CANSparkMax, CANSparkFlex
from wpilib import DutyCycleEncoder

import time

import config


class AmpScorer:
    def __init__(self, flipper: int, scorer: int):
        self.flipper = CANSparkMax(flipper, CANSparkMax.MotorType.kBrushless)
        self.flipper.setInverted(True)
        self.flipper.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.flipper_encoder = DutyCycleEncoder(4)

        self.scorer = CANSparkFlex(scorer, CANSparkMax.MotorType.kBrushless)
        self.scorer.setInverted(True)

        self.is_up = False

        self.flip_power = 0.5

    def set_flip_power(self, power: float):
        self.flipper.set(power)

    def update(self):
        if self.is_up:
            self.flip_up()
        else:
            self.flip_down()

    def flip_down(self):
        if self.flipper_encoder.get() < config.amp_abs_enc_down:
            self.flipper.set(-self.flip_power)
        else:
            self.flipper.set(0)

    def flip_up(self):
        power = 0.5 * (
            config.amp_abs_enc_up - self.flipper_encoder.getAbsolutePosition()
        )
        self.flipper.set(min(self.flip_power, max(0, power)))

    def set_scorer(self, power: float):
        self.scorer.set(power)
