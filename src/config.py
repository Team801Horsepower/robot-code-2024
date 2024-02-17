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
robot_dimensions = Translation2d(
    units.inchesToMeters(23.25), units.inchesToMeters(23.25)
)

# Old Chassis
# drive_gear_ratio = 16.0 / 3.0
# turn_gear_ratio = 60.0
# New Chassis

# drive_gear_ratio = 6.0
# turn_gear_ratio = 25.5

drive_gear_ratio = 6.12
turn_gear_ratio = 150.0 / 7.0

# wheel_diameter = units.inchesToMeters(4.0)
# wheel_diameter = units.inchesToMeters(4.5)
wheel_diameter = units.inchesToMeters(3.965)

drive_speed = 0.8
turn_speed = 1.0

# (drive ID, turn ID, absolute encoder ID, absolute encoder offset)
# All absolute encoder values are measured with the swerve wheel facing
# RADIALLY OUTWARD with the gear of the wheel on the LEFT SIDE of the wheel.
swerves = [
    (2, 3, 0.7892499),
    (8, 9, 0.5298459),
    (18, 19, 0.9826425),
    (10, 11, 0.8898037),
]

# front left, back left, front right, back right
swerve_ids = [0, 1, 2, 3]

note_proximity_threshold = 1600
