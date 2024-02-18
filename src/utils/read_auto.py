import json
from wpimath import units
from wpimath.geometry import Pose2d

from typing import List


def read_auto(path: str) -> List[Pose2d]:
    with open(path) as f:
        data = json.load(f)

    waypoints = []
    for waypoint in data["waypoints"]:
        # pose = (i["waypoints"]["x"], i["waypoints"]["y"], i["waypoints"]["r"])
        pose = Pose2d(waypoint["x"], waypoint["y"], units.degreesToRadians(waypoint["r"]))
        waypoints.append(pose)

    return waypoints


def read_cmds(path: str) -> List[str]:
    with open(path) as f:
        data = json.load(f)

    cmds = []
    for cmd in data["commands"]:
        cmds.append(cmd)

    return cmds
