import json
from wpimath import units
from wpimath.geometry import Pose2d

def read_auto(path):
    with open(path) as f:
        data = json.load(f)
    
    waypoints = []
    for i in data["waypoints"]:
        # pose = (i["waypoints"]["x"], i["waypoints"]["y"], i["waypoints"]["r"])
        pose = Pose2d(i["x"], i["y"], units.degreesToRadians(i["r"]))
        waypoints.append(pose)

    return waypoints
             
