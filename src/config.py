from wpimath import units
from wpimath.geometry import Translation2d
from wpilib import DriverStation

from typing import List, Tuple


def is_red() -> bool:
    return DriverStation.getAlliance() == DriverStation.Alliance.kRed


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

drive_speed = 6
turn_speed = 6
auto_drive_speed = 0.8
auto_turn_speed = 1

# (drive ID, turn ID, absolute encoder ID, absolute encoder offset)
# All absolute encoder values are measured with the swerve wheel facing
# RADIALLY OUTWARD with the gear of the wheel on the LEFT SIDE of the wheel.
swerves = [
    (3, 2, 0.985),
    (8, 9, 0.5298459),
    (18, 19, 0.0104102),
    (11, 10, 0.0611273),
]

# front left, back left, front right, back right
swerve_ids = [0, 1, 2, 3]

# note_proximity_threshold = 1600

note_proximity_threshold = 600
flywheel_speed = 6200
# Weird PID offset thing (we haven't figured out why we have to do this)
flywheel_setpoint = flywheel_speed + 1030

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

# (distance, pitch, yaw difference)
shooter_lookup_table = [
    (1.40, units.degreesToRadians(42.3), units.degreesToRadians(5)),
    (1.97, units.degreesToRadians(37.5), units.degreesToRadians(6)),
    (2.46, units.degreesToRadians(29.7), units.degreesToRadians(3)),
    (2.93, units.degreesToRadians(29.0), units.degreesToRadians(4)),
    (
        3.29,
        units.degreesToRadians(26.5),
        units.degreesToRadians(5),
    ),  # guesstimated for corner shots (works basically fine)
    (4.00, units.degreesToRadians(24.1), units.degreesToRadians(2)),
    (4.00, units.degreesToRadians(00.0), units.degreesToRadians(6)),
]


def lookup(table: List[Tuple[float, float]], x: float) -> float:
    x = max(table[0][0], min(table[-1][0], x))
    idx = max(0, [r[0] >= x for r in table].index(True) - 1)
    pt0, pt1 = table[idx], table[idx + 1]
    ratio = (x - pt0[0]) / (pt1[0] - pt0[0])
    return pt0[1] + ratio * (pt1[1] - pt0[1])


amp_flipper_up_value = 5.28
amp_shooter_pitch = 0.8223598776

link_pivot_setpoint_down = 0.71
link_pivot_setpoint_up = 0.04

amp_abs_enc_up = 0.800
amp_abs_enc_down = 1.2799

rear_camera_angle_offset = 0
second_camera_height = units.inchesToMeters(11.5)
second_camera_horiz_offset = units.inchesToMeters(-5.125)
second_camera_pitch = units.degreesToRadians(-5)
second_camera_yaw = units.degreesToRadians(-5)
