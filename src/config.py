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

# note_proximity_threshold = 1600
note_proximity_threshold = 600
flywheel_min_speed = 4000

# Height of the *pivot* of the shooter
shooter_height = units.inchesToMeters(9.061)

# Height of the camera
camera_height = units.inchesToMeters(7.712)

camera_angle = units.degreesToRadians(20)

# Forward distance between camera and shooter *pivot*,
# with positive for shooter behind camera
camera_shooter_distance = units.inchesToMeters(23.06)

# Distance of the camera left of the center of the robot
camera_left_offset = units.inchesToMeters(-7.917)

# Speaker height to aim at
# speaker_height = units.inchesToMeters(81)
# speaker_height = units.inchesToMeters(90)
speaker_height = units.inchesToMeters(100)

# Height of the center of a speaker april tag
speaker_tag_height = units.inchesToMeters(57.125)

shooter_lookup_table = [ # position to angle; entries every three feet, starting from 6
    units.degreesToRadians(48),
    units.degreesToRadians(42),
    units.degreesToRadians(0),
    units.degreesToRadians(0),
    units.degreesToRadians(0),
    units.degreesToRadians(0),
    units.degreesToRadians(0),
    units.degreesToRadians(0),
    units.degreesToRadians(0),
    units.degreesToRadians(0),
    
]
