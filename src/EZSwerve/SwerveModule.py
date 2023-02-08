
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
        #EG- self.driveEncoder = ctre.sensors.CANCoder(deviceNumber: int)              # Rod: Good, except probably the turning encoder.
        self.driveEncoder = wpilib.Encoder(driveEncoderChannelA, driveEncoderChannelB)
        self.turningEncoder = wpilib.Encoder(turningEncoderChannelA, turningEncoderChannelB)

        # Set the distance per pulse for the drive encoder. This can be
        # calculated by dividing the circumference of the wheel by the
        # resolution of the encoder.

        # Rod: Instead of setting this parameter, which doesn't exist on the Falcon, 
        # let's define helper functions/methods that convert back and forth between  
        # Falcon internal sensor units and meters of wheel rim travel.
        self.driveEncoder.setDistancePerPulse(2 * math.pi * self.WHEEL_RADIUS / self.ENCODER_RESOLUTION)

        # Set the distance per pulse for the turning encoder. This is a 
        # complete 'turn' (360 degrees / 2Pi radians) divided by the resolution of the
        # encoder.

        # Rod: See CANCoder example for configuring it to return radians.
        # Note in the next section that various parts of the code are expecting -pi to +pi,
        # so also configure the CANCoder to produce that range (rather than 0 to 2pi).
        self.turningEncoder.setDistancePerPulse(2 * math.pi / self.ENCODER_RESOLUTION)


        # Rod: I think it makes sense to use the Falcon's internal velocity control loop for
        # the drive motor, and therefore not use the software PID controller below for it.
        # It haven't decided whether the software PID like below or the Falcon reading CANCoder
        # angle feedback loop is better for turning.  Let's try software PID first.

        # Gains are for example purpose only -- must be determined for physical robot!
        # applies to: PIDControllers and feedforward gains
        # Limit the PID controller's input range between -pi (== -180 degrees) and pi (== 180 degrees)
        # and set the input to be continuous
        self.turningPIDController = ProfiledPIDController(1, 0, 0, \
            TrapezoidProfile.Constraints(self.MODULE_MAX_ANGULAR_VELOCITY, self.MODULE_MAX_ANGULAR_ACCELERATION))
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

        self.drivePIDController = PIDController(1, 0, 0)             # Rod: We won't need this for drive, instead we'll set up a bunch of internal PID parameters.
        self.driveFeedForward = SimpleMotorFeedforwardMeters(1, 3)   # Rod: same as above.
        self.turnFeedForward = SimpleMotorFeedforwardMeters(1, 0.5)  # Rod: We probably want this for software PID, but the parameter values may be different.

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.driveEncoder.getRate(), \  
            Rotation2d(self.turningEncoder.getDistance()))           # Rod: needs a rate in meters/sec and turning angle in radians.

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.driveEncoder.getDistance(), \
            Rotation2d(self.turningEncoder.getDistance()))           # Rod: needs the distance the wheel has driven (meters), and the turning angle in radians

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        '''This method is does all the work.  Pass it a desired SwerveModuleState (that is, wheel rim velocity and
        turning direction), and it sets the feedback loops to achieve that.'''
        state = SwerveModuleState.optimize(desiredState, Rotation2d(self.turningEncoder.getDistance()))

        # Rod: instead of this, calculate the target velocity in Falcon velocity units.
        # Calculate the drive output from the drive PID controller
        driveOutput = self.drivePIDController.calculate(self.driveEncoder.getRate(), state.speed)

        driveFeedForward = self.driveFeedForward.calculate(state.speed)

        # Rod: This one might stay the same if we are keeping software PID.
        # Calculate the turning motor output from the turning PID controller
        turnOutput = self.turningPIDController.calculate(self.turningEncoder.getDistance(), state.angle.radians())

        turnFeedForward = self.turnFeedForward.calculate(self.turningPIDController.getSetpoint().velocity)

        # Set the motor outputs
        self.driveMotor.set(ControlMode.Current, driveOutput + driveFeedForward) # Rod: Replace with setting ctre.TalonFXControlMode.Velocity and the target velocity for the internal PID.
        self.turningMotor.set(ControlMode.Current, turnOutput + turnFeedForward) # Rod: 

    # Rod: I don't think that this method is ever called, so we may not need to implement it.
    # Also, I'm not sure we want to reset the absolute encoder used for turning control.
    def resetEncoders(self) -> None:
        self.driveEncoder.reset()
        self.turningEncoder.reset()
