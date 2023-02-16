
import wpilib
import wpimath
from ctre import TalonFX, ControlMode, DemandType
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
        
        # Fill in the appropriate physical parameters for our module:
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
        # See empty functions at bottom of file.
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
    
        # Rod's template for the rest of the initialization process.
        # ----------------------------------------------------------

        # Configure the CANCoder with its magnet offset (different for each module).

        # Configure the drive motor's brake, inversion, and internal PID loop parameters for velocity control.

        # Configure the steering motor's brake, inversion, and internal PID loop parameters for position control.

        # Set the steering motor's internal encoder to match the absolute position reported
        # by the CANCoder.  After this we can use the internal encoder for angle feedback.
        # Uncomment and edit to match the actual object names defined above.
        # abs_position_deg = self.turning_encoder.getAbsolutePosition()
        # abs_position_counts = self.degrees_to_falcon(abs_position_deg)
        # self.turningMotor.setSelectedSensorPosition(abs_position_counts)



    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.driveEncoder.getRate(), \  
            Rotation2d(self.turningEncoder.getDistance()))           # Rod: needs a rate in meters/sec and turning angle in radians.

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.driveEncoder.getDistance(), \
            Rotation2d(self.turningEncoder.getDistance()))           # Rod: needs the distance the wheel has driven (meters), and the turning angle in radians

    # New version based on Team364/EWall code, rather than EZSwerve
    def setDesiredState(self, desiredState: SwerveModuleState, open_loop: bool) -> None:
        '''This method does all the work.  Pass it a desired SwerveModuleState (that is, wheel rim velocity and
           turning direction), and it sets the feedback loops to achieve that.
           Set open_loop to True to run drive motors with percent output rather than velocity control.
        '''
        # state = SwerveModuleState.optimize(desiredState, Rotation2d(self.turningEncoder.getDistance()))

        # # Rod: instead of this, calculate the target velocity in Falcon velocity units.
        # # Calculate the drive output from the drive PID controller
        # driveOutput = self.drivePIDController.calculate(self.driveEncoder.getRate(), state.speed)

        # driveFeedForward = self.driveFeedForward.calculate(state.speed)

        # # Calculate the turning motor output from the turning PID controller
        # turnOutput = self.turningPIDController.calculate(self.turningEncoder.getDistance(), state.angle.radians())

        # turnFeedForward = self.turnFeedForward.calculate(self.turningPIDController.getSetpoint().velocity)

        # # Set the motor outputs
        # self.driveMotor.set(ControlMode.Current, driveOutput + driveFeedForward) # Rod: Replace with setting ctre.TalonFXControlMode.Velocity and the target velocity for the internal PID.
        # self.turningMotor.set(ControlMode.Current, turnOutput + turnFeedForward) # Rod: 

        # Revised template for our hardware
        present_degrees = self.falcon_to_degrees(self.turningMotor.getSelectedSensorPosition())
        present_rotation = Rotation2d.fromDegrees(present_degrees)
        state = self.optimize(desiredState, present_rotation)

        if open_loop:
            percent_output = state.speed / self.MAX_SPEED  # TODO: define the max speed in meters/second
            self.driveMotor.set(ControlMode.PercentOutput, percent_output)
        else:
            velocity = self.mps_to_falcon(state.speed)
            self.driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, self.driveFeedForward.calculate(state.speed))

        angle = self.degrees_to_falcon(state.angle.degrees())
        self.turningMotor.set(ControlMode.Position, angle)


    # Rod: I don't think that this method is ever called, so we may not need to implement it.
    # Also, I'm not sure we want to reset the absolute encoder used for turning control.
    def resetEncoders(self) -> None:
        self.driveEncoder.reset()
        self.turningEncoder.reset()

    # Helper functions to optimize the steering angle.  Ported from Team364's CTREModuleState class,
    # https://github.com/Team364/BaseFalconSwerve/blob/main/src/main/java/frc/lib/util/CTREModuleState.java,
    # And also looking at EWall25's swervelib/mod.py: https://github.com/EWall25/swervelib/blob/master/swervelib/mod.py,
    # which adjusts to use Python's implementation of the modulo operator for negative dividends (different than Java).

    def optimize(self, desired_state: SwerveModuleState, current_angle: Rotation2d) -> SwerveModuleState:
        '''Our own optimize method (instead of SwerveModuleState.optimize())
            There are two ways for a swerve module to reach its goal:
            1) Rotate to its intended steering angle and drive at its intended speed.
            2) Rotate to the mirrored steering angle (subtract 180) and drive at the opposite of its intended speed.
            Optimizing finds the option that requires the smallest rotation by the module.
        '''
        # This method is customized from WPILib's version to include placing in appropriate scope for 
        # CTRE Falcon's onobard control.
        target_angle = self.place_in_proper_0_to_360_scope(current_angle.degrees(), desired_state.angle.degrees())
        target_speed = desired_state.speed
        delta = target_angle - current_angle.degrees()

        if abs(delta) > 90:
            target_speed *= -1
            if delta > 90:
                # delta positive
                target_angle -= 180
            else:
                target_angle += 180

        return SwerveModuleState(target_speed, Rotation2d.fromDegrees(target_angle))


    def placeInAppropriate0To360Scope(scope_reference: float, new_angle: float) -> float:
        '''Place the new_angle in the range that is a multiple of [0,360] that is closest
           to the scope_reference.
        '''
        lower_offset = scope_reference % 360         # Modulo (remainder) is always positive when divisor (360) is positive.
        lower_bound = scope_reference - lower_offset
        upper_bound = lower_bound + 360

        # Adjust the new_angle to fit between the bounds.
        while new_angle < lower_bound:
            new_angle += 360
        while new_angle > upper_bound:
            new_angle -= 360

        # Adjust new_angle more to make sure it is within 180 degrees of the reference.
        if new_angle - scope_reference > 180:
            new_angle -= 360
        elif new_angle - scope_reference < -180:
            new_angle += 360

        return new_angle


    # Helper functions to convert between Falcon encoder counts and degrees/radians or meters,
    # depending on which encoder we use.
    # See Team 364's Java helper class Conversions: https://github.com/Team364/BaseFalconSwerve/blob/main/src/main/java/frc/lib/math/Conversions.java

    def falcon_to_degrees(self, counts: float) -> float:
        '''Fill in the function's documentation string.'''
        pass # Fill me in!

    def degrees_to_falcon(self, degrees: float) -> float:
        '''Fill in the function's documentation string.'''
        pass # Fill me in!

    def falcon_to_meters(self, counts: float) -> float:
        '''Fill in the function's documentation string.'''
        pass # Fill me in!

    def meters_to_falcon(self, meters: float) -> float:
        '''Fill in the function's documentation string.'''
        pass # Fill me in!

    def falcon_to_mps(self, velocity_counts: float) -> float:
        '''Convert Falcon velocity counts per 100 msec to meters/sec.'''
        pass # Fill me in!

    def mps_to_falcon(self, velocity_mps: float) -> float:
        '''Convert meters per second to Falcon velocity counts per 100 msec.'''
        pass # Fill me in!
