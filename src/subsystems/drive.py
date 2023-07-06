"""Drive subsystem for the 2022 swerve chassis"""

import rev

# pylint: disable=missing-docstring, too-few-public-methods


class Drive:
    def __init__(self):
        self.drive_l = [
            rev.CANSparkMax(12, rev.CANSparkMax.MotorType.kBrushless),
            rev.CANSparkMax(20, rev.CANSparkMax.MotorType.kBrushless),
        ]
        self.drive_r = [
            rev.CANSparkMax(9, rev.CANSparkMax.MotorType.kBrushless),
            rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless),
        ]

        self.turn_motors = [
            rev.CANSparkMax(11, rev.CANSparkMax.MotorType.kBrushless),
            rev.CANSparkMax(10, rev.CANSparkMax.MotorType.kBrushless),
            rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless),
            rev.CANSparkMax(19, rev.CANSparkMax.MotorType.kBrushless),
        ]

        for i in self.turn_motors:
            i.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
            i.set(0)

    def drive(self, power_l: float, power_r: float):
        for i in self.drive_l:
            i.set(power_l)

        for j in self.drive_r:
            j.set(power_r)
