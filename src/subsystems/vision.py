from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import (
    PhotonPoseEstimator,
    AprilTagFieldLayout,
    PoseStrategy,
)
from wpimath.geometry import Transform3d, Rotation3d
from wpimath import units


class Vision:
    def __init__(self, camera_name="Camera_Module_v1"):
        self.camera = PhotonCamera(camera_name)
        layout = AprilTagFieldLayout("crescendo-apriltags.fmap")
        strat = PoseStrategy.LOWEST_AMBIGUITY
        robot_to_cam = Transform3d(
            units.inchesToMeters(1.5),
            units.inchesToMeters(12),
            units.inchesToMeters(18.5),
            Rotation3d.fromDegrees(0, 20, 0),
        )
        self.estimator = PhotonPoseEstimator(layout, strat, self.camera, robot_to_cam)

    def test(self):
        result = self.camera.getLatestResult()
        # self.estimator.update(result)
        print(result.getTargets())
