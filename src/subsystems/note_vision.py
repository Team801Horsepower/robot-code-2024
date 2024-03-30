import math
from operator import itemgetter
from photonlibpy.photonCamera import PhotonCamera
from wpilib import SmartDashboard

from commands2 import Subsystem, CommandScheduler


from config import rear_camera_angle_offset, second_camera_height


class Vision(Subsystem):
    def __init__(self, scheduler: CommandScheduler, camera_name="Deeby Server"):
        self.camera = PhotonCamera(camera_name)
        scheduler.registerSubsystem(self)
        # SmartDashboard.putNumber("note angle", -1)
    def periodic(self):
        print(self.get_distance_degree())
        # if self.get_distance_degree() is None:
        #     SmartDashboard.putNumber("note angle", -1)
        # else:
        #     SmartDashboard.putNumber("note angle", self.get_distance_degree())

    def get_distance_degree(self) -> dict[float, float] | None:
        result = self.camera.getLatestResult()
        # note = {"dist": value, "angle": value}
        all_distances = []
        if len(result.getTargets()) >= 1:
            for target in result.getTargets():
                target_to_cam_angle = target.getPitch()
                if target_to_cam_angle < 0:
                    distance = second_camera_height / (
                        math.tan(math.radians(target_to_cam_angle))
                    )
                    angle = target.getYaw()
                    final_angle = angle + rear_camera_angle_offset
                    all_distances.append({"dist": distance, "angle": final_angle})
                else:
                    angle = target.getYaw()
                    final_angle = angle + rear_camera_angle_offset
                    all_distances.append({"dist": -1, "angle": final_angle})

            final_distances = sorted(all_distances, key=itemgetter("dist"))
            closest_note = final_distances[0]
            return closest_note

        else:
            return None
