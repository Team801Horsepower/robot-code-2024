import json


class Reader:
    def __init__(self, path):
        self.path = path
        with open(path) as f:
            self.data = json.load(f)

    def print_anchor(self):
        waypoint_list = []
        waypoint_num = 1
        for i in self.data["waypoints"]:
            rotation = input("input rotation at " + str(i["anchor"]) + " waypoint " + str(waypoint_num) + ": ")
            waypoint_list.append({"x": i["anchor"]["x"], "y": i["anchor"]["y"], "r": int(rotation)}) 
            waypoint_num += 1
 
        data = {
            "version": 1.18,
            "waypoints":  waypoint_list
            
        }

        file_name = ""
        l = 0
        while self.path[l] != ".":
            file_name += self.path[l]
            l += 1
        file_path = file_name + ".json"

        # Write data to JSON file
        with open(file_path, "w") as json_file:
            json.dump(data, json_file, indent=4)
            

reader = Reader(input("File Name: "))
reader.print_anchor()

