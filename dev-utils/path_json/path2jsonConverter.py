import json


class Converter:
    def __init__(self, path):
        self.path = path
        with open(path) as f:
            self.data = json.load(f)

    def path_to_json(self):
        waypoint_list = []
        command_run_list = []
        waypoint_num = 1
        for i in self.data["waypoints"]:
            rotation = input(
                "input rotation at "
                + str(i["anchor"])
                + " waypoint "
                + str(waypoint_num)
                + ": "
            )
            print(rotation)
            command_run = input(
                "What cmds you want here anyway (gs) leave blank for none: "
            )
            waypoint_list.append(
                {"x": i["anchor"]["x"], "y": i["anchor"]["y"], "r": int(rotation)}
            )
            command_run_list.append(command_run)
            waypoint_num += 1

        data = {
            "version": 1.18,
            "waypoints": waypoint_list,
            "commands": command_run_list,
        }

        file_path = self.path.split(".")[0] + ".json"

        # Write data to JSON file
        with open(file_path, "w") as json_file:
            json.dump(data, json_file, indent=4)


converter = Converter(input("File Name: "))
converter.path_to_json()
