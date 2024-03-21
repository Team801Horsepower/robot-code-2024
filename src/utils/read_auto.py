import json
from wpimath import units
from wpimath.geometry import Pose2d

from typing import List, Tuple


def read_auto(path: str) -> List[Tuple[Pose2d, float]]:
    with open(path) as f:
        contents = ''.join(line.split('#')[0] + ('\n' if '#' in line else '') for line in f.readlines())
        data = json.loads(contents)

    waypoints = []
    for waypoint in data["waypoints"]:
        # pose = (i["waypoints"]["x"], i["waypoints"]["y"], i["waypoints"]["r"])
        pose = Pose2d(
            waypoint["x"], waypoint["y"], units.degreesToRadians(waypoint["r"])
        )
        pitch = units.degreesToRadians(waypoint["p"])
        speed = waypoint["v"]
        passthrough = waypoint["passthrough"]
        waypoints.append((pose, pitch, speed, passthrough))

    return waypoints


def read_cmds(path: str) -> List[str]:
    with open(path) as f:
        contents = ''.join(line.split('#')[0] + ('\n' if '#' in line else '') for line in f.readlines())
        data = json.loads(contents)

    cmds = []

    for wp in data["waypoints"]:
        cmds.append(wp["c"])

    return cmds
