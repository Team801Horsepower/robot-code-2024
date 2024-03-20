import json
from wpimath import units
from wpimath.geometry import Pose2d

from typing import List, Tuple


def read_auto(path: str) -> List[Tuple[Pose2d, float]]:
    with open(path) as f:
        data = json.load(f)

    waypoints = []
    for waypoint in data["waypoints"]:
        # pose = (i["waypoints"]["x"], i["waypoints"]["y"], i["waypoints"]["r"])
        pose = Pose2d(
            waypoint["x"], waypoint["y"], units.degreesToRadians(waypoint["r"])
        )
        pitch = units.degreesToRadians(waypoint["p"])
        waypoints.append((pose, pitch))

    return waypoints


def read_cmds(path: str) -> List[str]:
    with open(path) as f:
        data = json.load(f)

    cmds = []

    for wp in data["waypoints"]:
        cmds.append(wp["c"])

    return cmds
