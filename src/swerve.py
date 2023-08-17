from rev import CANSparkMax, SparkMaxPIDController, SparkMaxRelativeEncoder

class Swerve:
    def __init__(self, drive: CANSparkMax, turn: CANSparkMax):
        self.drive_motor = drive
        self.drive_pid = self.drive_motor.getPIDController()
        self.drive_encoder = self.drive_motor.getEncoder()

        self.turn_motor = turn
        self.turn_pid = self.turn_motor.getPIDController()
        self.turn_encoder = self.turn_motor.getEncoder()
