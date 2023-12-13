import wpimath
from wpimath import units
from wpimath.geometry import Translation2d


# lengths: meters

# (length, width)
# Old Chassis
# robot_dimensions = Translation2d(0.631825, 0.53975)
# New Chassis
# robot_dimensions = Translation2d(units.inchesToMeters(25.0), units.inchesToMeters(25.0))
# Newer Chassis
robot_dimensions = Translation2d(units.inchesToMeters(24.0), units.inchesToMeters(24.0))

# Old Chassis
# drive_gear_ratio = 16.0 / 3.0
# turn_gear_ratio = 60.0
# New Chassis

# drive_gear_ratio = 6.0
# turn_gear_ratio = 25.5

drive_gear_ratio = 6.12
turn_gear_ratio = 150.0 / 7.0

# wheel_diameter = units.inchesToMeters(4.0)
wheel_diameter = units.inchesToMeters(4.5)

drive_speed = 2.5
turn_speed = 4.0
