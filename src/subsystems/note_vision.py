import math
import config
from math import sqrt, sin, cos, asin, pi, acos, tan
from operator import itemgetter
from photonlibpy.photonCamera import PhotonCamera
from wpilib import SmartDashboard
from wpimath.geometry import Translation2d, Rotation2d
from wpimath import units
from commands2 import Subsystem, CommandScheduler
from typing import Tuple


from config import rear_camera_angle_offset, second_camera_height


class NoteVision:
    def __init__(self, camera_name="Deeby Server"):
        self.camera = PhotonCamera(camera_name)

    def robot_space_note_pos(self) -> Translation2d | None:
        res = self.camera.getLatestResult()
        if not res.hasTargets():
            return None

        pos_s = []

        for target in res.getTargets():
            pn = units.degreesToRadians(target.getPitch())
            yn = units.degreesToRadians(-target.getYaw())

            pc = config.second_camera_pitch
            yc = config.second_camera_yaw
            hc = config.second_camera_height

            dc = -hc / tan(pn + pc)

            if dc < 0:
                continue

            dist_from_cam = dc * cos(yn)
            yaw_from_robot = yn + yc + pi

            pos_from_cam = Translation2d(dist_from_cam, Rotation2d(yaw_from_robot))
            pos_from_robot = Translation2d(
                pos_from_cam.x, pos_from_cam.y + config.second_camera_horiz_offset
            )

            # 3.5 meter maximum note distance
            if pos_from_robot.norm() < 3.5:
                pos_s.append(pos_from_robot)

        if not pos_s:
            return None

        return min(pos_s, key=Translation2d.norm)
