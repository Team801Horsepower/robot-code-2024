#!/usr/bin/env python3

import wpilib


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        pass

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        print("Hello, World!")
        # Open pynetconsole ("python3 -m netconsole 10.8.1.2") to view

    def testInit(self):
        pass

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
