import wpimath
from wpimath import units
from wpimath.geometry import Translation2d
import tomllib
import sys

try:
    with open("/config.toml", "rb") as f:
        robot_data = tomllib.load(f)["robot"]
except FileNotFoundError:
    raise Exception(
        "/config.toml is not present; please refer to src/config/README.md for instructions on the placement of the config file"
    )


# (length, width)
# robot_dimensions = Translation2d(0.631825, 0.53975)

# Old Chassis
# drive_gear_ratio = 16.0 / 3.0
# turn_gear_ratio = 60.0
# New Chassis
# drive_gear_ratio = 6.0
# turn_gear_ratio = 25.5

# wheel_diameter = units.inchesToMeters(4.0)

drive_speed = 150.0
turn_speed = 150.0
