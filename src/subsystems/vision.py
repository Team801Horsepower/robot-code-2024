from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
from photonlibpy.photonPoseEstimator import (
    PhotonPoseEstimator,
    AprilTagFieldLayout,
    PoseStrategy,
)
from wpimath.geometry import Transform3d, Rotation3d, Pose2d, Translation2d
from wpimath import units
from commands2 import Subsystem, CommandScheduler
from typing import Tuple, Optional, List

from math import tan, sin, cos

import config


class Vision(Subsystem):
    def __init__(self, scheduler: CommandScheduler, camera_name="Camera_Module_v1"):
        self.camera = PhotonCamera(camera_name)

        self.layout = AprilTagFieldLayout(
            # config.code_path
            # + "crescendo-apriltags.json"
            config.code_path
            + "firehouse-apriltags.json"
        )

        scheduler.registerSubsystem(self)

    def periodic(self):
        pass

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

    # TODO: Incorporate camera pose relative to robot center, then use multiple cameras
    def estimate_multitag_pose(self, robot_angle: float) -> List[Pose2d]:
        result = self.camera.getLatestResult()
        tags = result.getTargets()
        poses = []
        for i in range(len(tags)):
            for j in range(i + 1, len(tags)):
                poses.append(self.estimate_2tag_pose(robot_angle, tags[i], tags[j]))
        return poses

    def estimate_2tag_pose(
        self,
        robot_angle: float,
        tag1: PhotonTrackedTarget,
        tag2: PhotonTrackedTarget,
    ) -> Pose2d:
        p1 = self.layout.getTagPose(tag1.getFiducialId()).toPose2d().translation()
        p2 = self.layout.getTagPose(tag2.getFiducialId()).toPose2d().translation()

        th1 = units.degreesToRadians(-tag1.getYaw()) + robot_angle
        th2 = units.degreesToRadians(-tag2.getYaw()) + robot_angle

        a1, b1 = -sin(th1), cos(th1)
        a2, b2 = -sin(th2), cos(th2)

        c1 = -a1 * p1.x - b1 * p1.y
        c2 = -a2 * p2.x - b2 * p2.y

        x = (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1)
        y = (c1 * a2 - c2 * a1) / (a1 * b2 - a2 * b1)

        return Pose2d(x, y, robot_angle)
