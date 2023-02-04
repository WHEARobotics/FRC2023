
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

class SwerveModule:                                                                 #UPDATE: 2/3/2023
    def __init__(self, driveMotorChannel : int, turningMotorChannel : int, \        #When it talks about channels, those are the numbers we assign to
        driveEncoderChannelA : int, driveEncoderChannelB : int, \                   #   each module/item on the CANbus. It looks like each encoder has two channels
        turningEncoderChannelA : int, turningEncoderChannelB : int) -> None:        #   we need to assign to them to make them function
            
            #The above portion of the method, conceptually, could be changed to hold four parameters, since the ctre CANcoder method only needs one channel
            #This means we need to go into SwerveDrivetrain.py and change the SwerveModules to hold the four parameters we cahnge the code above to.
            
        self.WHEEL_RADIUS = 0.0508                                                  #Q: What's ours?
        self.ENCODER_RESOLUTION = 4096                                              #This is true for the CTRE CANcoders (see manual linked in SLACK).
        self.MODULE_MAX_ANGULAR_VELOCITY = math.pi # 1/2 rotation per second. Note this must == SwerveDriveTrain.MAX_ANGULAR_SPEED
        # Does this constant mean we're using radians?                              #A: 2/3/2023- YES!!  Let's do it!!!
        self.MODULE_MAX_ANGULAR_ACCELERATION = 2 * math.pi

        self.driveMotor = TalonFX(driveMotorChannel)                 #"Channel" is ID on CAN bus
        self.turningMotor = TalonFX(turningMotorChannel)             #i.e. TalonFX(1) is ID#1 in phoenix Tuner/on CAN bus.
        
        #   2/3/2023- USE THIS METHOD:ctre.sensors.CANCoder(deviceNumber: int)
        #EG- self.driveEncoder = ctre.sensors.CANCoder(deviceNumber: int)
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
