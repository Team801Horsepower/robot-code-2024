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

drive_speed = 4
turn_speed = 4
auto_drive_speed = 1.8
auto_turn_speed = 3

# (drive ID, turn ID, absolute encoder ID, absolute encoder offset)
# All absolute encoder values are measured with the swerve wheel facing
# RADIALLY OUTWARD with the gear of the wheel on the LEFT SIDE of the wheel.
swerves = [
    (2, 3, 0.7892499),
    (8, 9, 0.5298459),
    (18, 19, 0.9826425),
    (10, 11, 0.0611273),
]

# front left, back left, front right, back right
swerve_ids = [0, 1, 2, 3]

# note_proximity_threshold = 1600

note_proximity_threshold = 600
flywheel_speed = 4200
# Weird PID offset thing (we haven't figured out why we have to do this)
flywheel_setpoint = flywheel_speed + 700

# Height of the *pivot* of the shooter
shooter_height = units.inchesToMeters(9.061)

# Height of the camera
camera_height = units.inchesToMeters(7.633)

camera_angle = units.degreesToRadians(30)

# Forward distance between camera and shooter *pivot*,
# with positive for shooter behind camera
camera_shooter_distance = units.inchesToMeters(22.719)

camera_center_distance = units.inchesToMeters(13)

# Distance of the camera left of the center of the robot
camera_left_offset = units.inchesToMeters(0)

# Speaker height to aim at
# speaker_height = units.inchesToMeters(81)
# speaker_height = units.inchesToMeters(90)
speaker_height = units.inchesToMeters(100)

# Height of the center of a speaker april tag
speaker_tag_height = units.inchesToMeters(57.125)

shooter_lookup_table = [  # position to angle; entries every three feet, starting from 6
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

amp_flipper_up_value = 5.28
# amp_shooter_pitch = 0.77
amp_shooter_pitch = 0.8223598776
