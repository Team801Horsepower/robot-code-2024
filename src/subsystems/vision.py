from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import (
    PhotonPoseEstimator,
    AprilTagFieldLayout,
    PoseStrategy,
)
from wpimath.geometry import Transform3d, Rotation3d
from wpimath import units
from commands2 import Subsystem, CommandScheduler
from typing import Tuple

from math import tan

import config


class Vision(Subsystem):
    def __init__(self, scheduler: CommandScheduler, camera_name="Camera_Module_v1"):
        self.camera = PhotonCamera(camera_name)

        layout = AprilTagFieldLayout("/home/lvuser/py/crescendo-apriltags.json")
        strat = PoseStrategy.LOWEST_AMBIGUITY
        robot_to_cam = Transform3d(
            units.inchesToMeters(1.5),
            units.inchesToMeters(12),
            units.inchesToMeters(18.5),
            Rotation3d.fromDegrees(0, 20, 0),
        )
        self.estimator = PhotonPoseEstimator(layout, strat, self.camera, robot_to_cam)

        self.current_pose = "periodic hasn't been called yet"

        scheduler.registerSubsystem(self)

    def periodic(self):
        result = self.camera.getLatestResult()
        self.current_pose = self.estimator.update(result)
        # print(self.current_pose)

    def test(self):
        # result = self.camera.getLatestResult()
        # for target in result.getTargets():
        #     target.getPitch()
        # print(result.getTargets())
        pass

    def cur_speaker_atag(self) -> Tuple[float, float] | None:
        result = self.camera.getLatestResult()
        for target in result.getTargets():
            # TODO: Use is_red check rather than allowing both
            if target.fiducialId in [4, 7]:
                return (
                    units.degreesToRadians(target.getPitch()),
                    units.degreesToRadians(target.getYaw()),
                )
        return None

    def cur_amp_atag(self) -> Tuple[float, float] | None:
        result = self.camera.getLatestResult()
        for target in result.getTargets():
            if target.fiducialId in [5, 6]:  # red, blue
                return (
                    units.degreesToRadians(target.getPitch()),
                    units.degreesToRadians(target.getYaw()),
                )
        return None

    def speaker_dist(self) -> float:
        sp_atag = self.cur_speaker_atag()
        if sp_atag is None:
            return -1
        atag_pitch, atag_yaw = sp_atag

        return (config.speaker_tag_height - config.camera_height) / tan(
            atag_pitch + config.camera_angle
        )

    def vision_pitch(self) -> float | None:
        x = self.speaker_dist()
        if x == -1:
            return None
        return units.degreesToRadians(
            # Polynomial interpolation of four lookup table values
            # Actual lookup table in Google Sheet
            5.47632 * x * x * x
            - 24.3666 * x * x
            + 22.5556 * x
            + 49.0865
        )
