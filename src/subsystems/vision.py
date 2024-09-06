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

        layout = AprilTagFieldLayout(config.code_path + "crescendo-apriltags.json")
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

    def cur_atag(self, red_id: int, blue_id: int) -> Tuple[float, float] | None:
        atag_id = red_id if config.is_red() else blue_id
        result = self.camera.getLatestResult()
        for target in result.getTargets():
            if target.fiducialId == atag_id:
                return (
                    units.degreesToRadians(target.getPitch()),
                    units.degreesToRadians(target.getYaw()),
                )
        return None

    def cur_speaker_atag(self) -> Tuple[float, float] | None:
        return self.cur_atag(4, 7)

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
        pitch_table = [(r[0], r[1]) for r in config.shooter_lookup_table]
        return config.lookup(pitch_table, x)
