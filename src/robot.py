#!/usr/bin/env python3

import wpilib
import wpimath
from subsystems import chassis, drive

import config


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # pylint: disable=attribute-defined-outside-init
        self.driver_controller = wpilib.XboxController(0)

        self.drive = drive.Drive()

        self.field_oriented_drive = True

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

        def deadzone(activation: float) -> float:
            if abs(activation) < 0.1:
                return 0.0
            return activation

        drive_input = wpimath.geometry.Transform2d(
            config.drive_speed * deadzone(-self.driver_controller.getLeftY()),
            config.drive_speed * deadzone(-self.driver_controller.getLeftX()),
            config.turn_speed * deadzone(-self.driver_controller.getRightX()),
        )
        self.drive.drive(drive_input, self.field_oriented_drive)

        # Set swerves button
        if self.driver_controller.getAButton():
            self.drive.chassis.set_swerves()
        if self.driver_controller.getBButtonPressed():
            self.field_oriented_drive ^= True

    def testInit(self):
        pass

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
