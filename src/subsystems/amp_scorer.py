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

    def set_flip_power(self, power: float):
        self.flipper.set(power)

    def test(self):
        print(self.flipper.getOutputCurrent())

    # def update(self):
    #     pos = self.flipper_encoder.getPosition()
    #     print(pos)
    #     if self.is_up:
    #         self.at_bottom = False
    #         if self.upper_limit is None:
    #             # print("finding upper limit")
    #             self.flipper.set(0.15)
    #             if self.flipper.getOutputCurrent() > 30:
    #                 self.upper_limit = pos
    #                 self.flipper.set(0)
    #         else:
    #             # print("seeking upper limit")
    #             power = (self.upper_limit - pos) * 0.5
    #             self.flipper.set(power)
    #     else:
    #         self.upper_limit = None
    #         if not self.at_bottom:
    #             # print("lowering")
    #             self.flipper.set(-0.15)
    #             if self.flipper.getOutputCurrent() > 30:
    #                 self.at_bottom = True
    #                 self.flipper.set(0)
    #         else:
    #             # print("resting")
    #             self.flipper.set(0)
    #     if self.at_bottom:
    #         self.flipper_encoder.setPosition(0)
    def update(self):
        if self.is_up:
            self.flip_up()
        else:
            self.flip_down()

    def flip_down(self):
        # print(self.flipper_encoder.getPosition())
        if not self.limit_set or self.flipper_encoder.getPosition() > 0.5:
            # print("flip_down: seeking limit")
            self.flipper.set(-0.15)
            cur = self.flipper.getOutputCurrent()
            # print(cur)
            if cur > 20:
                self.limit_cycles += 1
                if self.limit_cycles >= 10:
                    self.flipper_encoder.setPosition(0)
                    self.limit_set = True
            else:
                self.limit_cycles = 0
        else:
            # print("flip_down: resting")
            self.flipper.set(0)

    def flip_up(self):
        if not self.limit_set:
            # print("flip_up: flipping down")
            self.flip_down()
        else:
            # print("flip_up: seeking limit")
            power = 0.5 * (
                config.amp_flipper_up_value - self.flipper_encoder.getPosition()
            )
            self.flipper.set(min(0.15, max(0, power)))

    def set_scorer(self, power: float):
        self.scorer.set(power)
