
import wpilib
import wpimath
from ctre import TalonFX, ControlMode
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d, Translation2d

from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.trajectory import TrapezoidProfile

from wpilib import Encoder
from wpilib.interfaces import MotorController
from wpilib import PWMSparkMax

import math

class SwerveModule:
    def __init__(self, driveMotorChannel : int, turningMotorChannel : int, \
        driveEncoderChannelA : int, driveEncoderChannelB : int, \
        turningEncoderChannelA : int, turningEncoderChannelB : int) -> None:
        self.WHEEL_RADIUS = 0.0508
        self.ENCODER_RESOLUTION = 4096
        self.MODULE_MAX_ANGULAR_VELOCITY = math.pi # 1/2 rotation per second. Note this must == SwerveDriveTrain.MAX_ANGULAR_SPEED
        # Does this constant mean we're using radians?
        self.MODULE_MAX_ANGULAR_ACCELERATION = 2 * math.pi

        self.driveMotor = TalonFX(driveMotorChannel)
        self.turningMotor = TalonFX(turningMotorChannel)
        
        self.driveEncoder = wpilib.Encoder(driveEncoderChannelA, driveEncoderChannelB)
        self.turningEncoder = wpilib.Encoder(turningEncoderChannelA, turningEncoderChannelB)

        # Set the distance per pulse for the drive encoder. This can be
        # calculated by dividing the circumference of the wheel by the
        # resolution of the encoder.
        self.driveEncoder.setDistancePerPulse(2 * math.pi * self.WHEEL_RADIUS / self.ENCODER_RESOLUTION)

        # Set the distance per pulse for the turning encoder. This is a 
        # complete 'turn' (360 degrees / 2Pi radians) divided by the resolution of the
        # encoder.
        self.turningEncoder.setDistancePerPulse(2 * math.pi / self.ENCODER_RESOLUTION)

        # Gains are for example purpose only -- must be determined for physical robot!
        # applies to: PIDControllers and feedforward gains
        # Limit the PID controller's input range between -pi (== -180 degrees) and pi (== 180 degrees)
        # and set the input to be continuous
        self.turningPIDController = ProfiledPIDController(1, 0, 0, \
            TrapezoidProfile.Constraints(self.MODULE_MAX_ANGULAR_VELOCITY, self.MODULE_MAX_ANGULAR_ACCELERATION))
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

        self.drivePIDController = PIDController(1, 0, 0)
        self.driveFeedForward = SimpleMotorFeedforwardMeters(1, 3)
        self.turnFeedForward = SimpleMotorFeedforwardMeters(1, 0.5)

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.driveEncoder.getRate(), \
            Rotation2d(self.turningEncoder.getDistance()))

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.driveEncoder.getDistance(), \
            Rotation2d(self.turningEncoder.getDistance()))

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        state = SwerveModuleState.optimize(desiredState, Rotation2d(self.turningEncoder.getDistance()))

        # Calculate the drive output from the drive PID controller
        driveOutput = self.drivePIDController.calculate(self.driveEncoder.getRate(), state.speed)

        driveFeedForward = self.driveFeedForward.calculate(state.speed)

        # Calculate the turning motor output from the turning PID controller
        turnOutput = self.turningPIDController.calculate(self.turningEncoder.getDistance(), state.angle.radians())

        turnFeedForward = self.turnFeedForward.calculate(self.turningPIDController.getSetpoint().velocity)

        # Set the motor outputs
        self.driveMotor.set(ControlMode.Current, driveOutput + driveFeedForward)
        self.turningMotor.set(ControlMode.Current, turnOutput + turnFeedForward)

    def resetEncoders(self) -> None:
        self.driveEncoder.reset()
        self.turningEncoder.reset()