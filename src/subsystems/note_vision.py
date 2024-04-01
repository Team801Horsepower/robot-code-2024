import math
from operator import itemgetter
from photonlibpy.photonCamera import PhotonCamera
from wpilib import SmartDashboard

from commands2 import Subsystem, CommandScheduler


from config import rear_camera_angle_offset, second_camera_height


class NoteVision():
    def __init__(self, camera_name="Deeby Server"):
        self.camera = PhotonCamera(camera_name)
        # SmartDashboard.putNumber("note angle", -1)

    def get_notes(self):
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
                    all_distances.append((distance, final_angle))

            if len(all_distances) > 0:
                    final_distances = sorted(all_distances, key=lambda x: x[0])
                    closest_note = final_distances[0]
                    return closest_note
