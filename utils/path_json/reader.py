import json


class Reader:
    def __init__(self, path):
        with open(path) as f:
            self.data = json.load(f)
        
        self.waypoints = []
        for i in self.data["waypoints"]:
            # pose = (i["waypoints"]["x"], i["waypoints"]["y"], i["waypoints"]["r"])
            pose = (i["x"], i["y"], i["r"])
            self.waypoints.append(pose)
            

reader = Reader("ReadPath.json")
print(reader.waypoints)
