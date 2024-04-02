import math
import config
from math import sqrt, sin, cos, asin, pi
from operator import itemgetter
from photonlibpy.photonCamera import PhotonCamera
from wpilib import SmartDashboard

from commands2 import Subsystem, CommandScheduler


from config import rear_camera_angle_offset, second_camera_height


class NoteVision():
    def __init__(self, camera_name="Deeby Server"):
        self.camera = PhotonCamera(camera_name)
        # SmartDashboard.putNumber("note angle", -1)

    # def __init__(self, camera):
    #     self.camera = camera

    def get_notes(self):
        result = self.camera.getLatestResult()
        # note = {"dist": value, "angle": value}
        all_distances = []
        if len(result.getTargets()) >= 1:
            for target in result.getTargets():
                target_to_cam_angle = target.getPitch()
                if target_to_cam_angle < 0:
                    distance = second_camera_height / (
                        -math.tan(math.radians(target_to_cam_angle))
                    )
                    angle = target.getYaw()
                    final_angle = angle + rear_camera_angle_offset
                    all_distances.append((distance, final_angle))

            if len(all_distances) > 0:
                    final_distances = sorted(all_distances, key=lambda x: x[0])
                    closest_note = final_distances[0]
                    return self.camera_offsetify(*closest_note)

    def camera_offsetify(self, r, theta):
        theta = math.radians(theta)
        f = config.second_camera_horiz_offset
        l = sqrt(f ** 2 + r ** 2 - (2 * f * r * cos((pi / 2) + theta)))
        phi = asin(r * sin(theta) / l)
        phi = math.degrees(phi)

        print(f"rltp: {r:.4f}\t{l:.4f}\t{theta:.4f}\t{phi:.4f}")
        return (l, phi)
