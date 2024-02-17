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

    def cur_speaker_atag(self) -> Tuple[float, float]:
        result = self.camera.getLatestResult()
        for target in result.getTargets():
            if target.fiducialId in [4, 7]:
                return (target.getPitch(), target.getYaw())
        return None
