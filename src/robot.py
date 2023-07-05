#!/usr/bin/env python3

import wpilib
import rev
from subsystems.drive import Drive


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.motor = rev.CANSparkMax(11, rev.CANSparkMax.MotorType.kBrushless)

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        # print("Hello, World!")
        # Open pynetconsole ("python3 -m netconsole 10.8.1.2") to view

        self.motor.set(0.5)

    def testInit(self):
        pass

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
