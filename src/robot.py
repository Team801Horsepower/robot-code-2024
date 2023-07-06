#!/usr/bin/env python3

import wpilib
import wpimath
import rev
from subsystems import drive


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.driver_controller = wpilib.XboxController(0)

        self.drive = drive.Drive()

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        # print("Hello, World!")
        # Open pynetconsole ("python3 -m netconsole 10.8.1.2") to view

        # self.motor.set(0.5)

        # TODO: Enable driving after resolving the gear ratios and robot dimensions
        # self.drive.drive(
        #     self.driver_controller.getLeftY(),
        #     self.driver_controller.getRightY(),
        # )
        pass

    def testInit(self):
        pass

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
