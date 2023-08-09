"""Contains interfaces related to driving the robot."""

from __future__ import annotations

from abc import ABC, abstractmethod

from wpimath.kinematics import ChassisSpeeds  # type: ignore


class IDrive(ABC):
    """Provides a drive agnostic interface for robot drive bases."""

    @abstractmethod
    def get_chassis_speeds(self) -> ChassisSpeeds:
        """Returns the current frame speed of the robot.

        Returns
        -------
        ChassisSpeeds
            The current frame speed of the robot."""
        raise NotImplementedError()

    def set_chassis_speeds(self, speeds: ChassisSpeeds) -> None:
        """Sets the desired frame speed of the robot.

        Parameters
        ----------
        speeds : ChassisSpeeds
            The desired frame speed of the robot."""
        raise NotImplementedError()
