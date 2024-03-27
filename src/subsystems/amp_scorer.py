from rev import CANSparkMax, CANSparkFlex
from wpilib import DutyCycleEncoder


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

        self.flip_power = 1

    def set_flip_power(self, power: float):
        self.flipper.set(power)

    def update(self):
        if self.is_up:
            self.flip_up()
        else:
            self.flip_down()

    def flip_down(self):
        if self.flipper_encoder.get() < config.amp_abs_enc_down:
            self.flipper.set(-0.5 * self.flip_power)
        else:
            self.flipper.set(0)

    def flip_up(self):
        power = 3 * (self.flipper_encoder.getAbsolutePosition() - config.amp_abs_enc_up)
        self.flipper.set(max(min(power, self.flip_power), 0))

    def set_scorer(self, power: float):
        self.scorer.set(power)
