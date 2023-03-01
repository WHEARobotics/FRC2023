
import wpilib
import wpimath
import ctre
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
import time # Temporary for diagnostics

class SwerveModule:                                                                 #UPDATE: 2/3/2023

    
    WHEELDIAMETER = 4 * 0.0254 #4 inch diameter for the wheel times the conversion to meters
    TRACKWIDTH = 22.25 * 0.0254 #both back and front have the module center to other mod center as 22 and 1/4th
    WHEELBASE = 22.75 * 0.0254 #left side of front mod to back mod #right is 26 7/8
    TURNING_GEAR_RATIO = 150.0/7.0 # Docs call it a 150/7:1
    DRIVE_GEAR_RATIO = 6.75
    #WHEELCIRCUMFERENCE = WHEELDIAMETER * pi


    #encoder motor values for the absolute position CANCoders in degrees
    ABSOLUTEPOS_3: 32.959   # Back Right
    ABSOLUTEPOS_4: 106.424  # Front Right
    ABSOLUTEPOS_2: 206.455  # Back Left
    ABSOLUTEPOS_1: 296.543  # Front Left

    #turn angle invert: yes
    #drive invert: no

    kSlotIdx = 0 

    kPIDLoopIdx = 0

    def __init__(self, driveMotorChannel : int, turningMotorChannel : int, absoluteEncoderChannel : int, absEncOffset : float) -> None:        #   we need to assign to them to make them function
        #The above portion of the method, conceptually, could be changed to hold four parameters, since the ctre CANcoder method only needs one channel
        #This means we need to go into SwerveDrivetrain.py and change the SwerveModules to hold the four parameters we cahnge the code above to.

        self.driveMotor = TalonFX(driveMotorChannel)                 #"Channel" is ID on CAN bus
        self.turningMotor = TalonFX(turningMotorChannel)
        self.absEnc = ctre.sensors.CANCoder(absoluteEncoderChannel)

        #self.absEnc.configMagnetOffset(absEncOffset)#we used cancoder configuration class when we were supposed to just use cancoder class remember that mistake

        self.driveMotor.setInverted(False)
        self.turningMotor.setInverted(True)

        self.driveMotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.turningMotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)

        #self.targetPos = 0                       #target pos is used in teleop periodic, we would set it to the joystick rotation, and set the motor pos to the target pos

        #We'll need to set these up for the drive motor
        self.turningMotor.configNominalOutputForward(0)
        self.turningMotor.configNominalOutputReverse(0)
        self.turningMotor.configPeakOutputForward(1)
        self.turningMotor.configPeakOutputReverse(-1)
        
        self.driveMotor.configNominalOutputForward(0)
        self.driveMotor.configNominalOutputReverse(0)
        self.driveMotor.configPeakOutputForward(1)
        self.driveMotor.configPeakOutputReverse(-1)

        self.turningMotor.configAllowableClosedloopError(0, self.kPIDLoopIdx)      #kind of like a dead band- the higher the value, the more deadband-- NEED THIS?

        self.turningMotor.selectProfileSlot(self.kSlotIdx, self.kPIDLoopIdx)       #kind of like a dead band- the higher the value, the more deadband-- NEED THIS?
        self.turningMotor.config_kF(self.kSlotIdx, 0)              #a constant that, if we know how much to correct already, this can help us correct faster (Nascar vs. DriverPermit)
        self.turningMotor.config_kP(self.kSlotIdx, 0.3)            #proportion of how far off the current value is from the input. DOUBLE CHECK-READ 364 INFO: WE MIGHT NEED TO ADJUST THIS
        self.turningMotor.config_kI(self.kSlotIdx, 0)              #This takes how much error there is over time and uses it to help correct (high value = SUPERIMMEDIATECORRECT!!!
                                                                   #... lower value = LET'S CORRECT, BUT WE'LL BE A LITTLE MORE CAUTIOUS)
        self.turningMotor.config_kD(self.kSlotIdx, 0)              #This senses how MUCH the error is off, and helps correct based on how much the error is



<<<<<<< HEAD
        #absolutePos = self.absEnc.getAbsolutePosition()
=======
        absolutePos = self.absEnc.getAbsolutePosition()
        print(absolutePos) # Print the value as a diagnostic.
>>>>>>> 0107c0b8697a387f833546a3fe7a06449481a52a

        #initPos = self.DegToTurnCount(absolutePos) #COMMENTED OUT MONDAY AFTERNOON AFTER SETTING INIT TO ZERO

        print(self.turningMotor.setSelectedSensorPosition(0))
        #print(self.turningMotor.setSelectedSensorPosition(initPos))

        self.MODULE_MAX_ANGULAR_VELOCITY = math.pi # 1/2 rotation per second. Note this must == SwerveDriveTrain.MAX_ANGULAR_SPEED
        # Does this constant mean we're using radians?                              #A: 2/3/2023- YES!!  Let's do it!!!
        self.MODULE_MAX_ANGULAR_ACCELERATION = 2 * math.pi

        self.MAX_SPEED = 3     #6380 rpm * (4in * 0.0254 * 3.14) / 6.75 / 60 = 5.02 METERS PER SECOND unweighted

        
        #   2/3/2023- USE THIS METHOD:ctre.sensors.CANCoder(deviceNumber: int)
        #EG- self.driveEncoder = ctre.sensors.CANCoder(deviceNumber: int)              # Rod: Good, except probably the turning encoder.
        # Set the distance per pulse for the drive encoder. This can be
        # calculated by dividing the circumference of the wheel by the
        # resolution of the encoder.

        # Rod: Instead of setting these parameters, which don't exist on the Falcon, 
        # let's define helper functions/methods that convert back and forth between  
        # Falcon internal sensor units and meters of wheel rim travel.


        # Rod: I think it makes sense to use the Falcon's internal velocity control loop for
        # the drive motor, and therefore not use the software PID controller below for it.
        # It haven't decided whether the software PID like below or the Falcon reading CANCoder
        # angle feedback loop is better for turning.  Let's try software PID first.

        # Gains are for example purpose only -- must be determined for physical robot!
        # applies to: PIDControllers and feedforward gains
        # Limit the PID controller's input range between -pi (== -180 degrees) and pi (== 180 degrees)
        # and set the input to be continuous
        '''
        self.turningPIDController = ProfiledPIDController(1, 0, 0, \
            TrapezoidProfile.Constraints(self.MODULE_MAX_ANGULAR_VELOCITY, self.MODULE_MAX_ANGULAR_ACCELERATION))
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

        self.drivePIDController = PIDController(1, 0, 0)             # Rod: We won't need this for drive, instead we'll set up a bunch of internal PID parameters.
        self.driveFeedForward = SimpleMotorFeedforwardMeters(1, 3)   # Rod: same as above.
        self.turnFeedForward = SimpleMotorFeedforwardMeters(1, 0.5)  # Rod: We probably want this for software PID, but the parameter values may be different.
        '''
    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.driveVelocitytToMPS(self.driveMotor.getSelectedSensorVelocity()), Rotation2d.fromDegrees(self.TurnCountToDeg(self.turningMotor.getSelectedSensorPosition())))           # Rod: needs a rate in meters/sec and turning angle in radians.

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.driveMotor.getSelectedSensorPosition(), Rotation2d.fromDegrees(self.TurnCountToDeg(self.turningMotor.getSelectedSensorPosition())))           # Rod: needs the distance the wheel has driven (meters), and the turning angle in radians

    def setDesiredState(self, desiredState: SwerveModuleState, open_loop: bool) -> None:
        '''This method is does all the work.  Pass it a desired SwerveModuleState (that is, wheel rim velocity and
        turning direction), and it sets the feedback loops to achieve that.'''
        """
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
        """
        present_degrees = self.TurnCountToDeg(self.turningMotor.getSelectedSensorPosition()) #Soren here, I think instead of tuning motor selected sensor, we might have to use absolute encoder
        present_rotation = Rotation2d.fromDegrees(present_degrees)
        state = self.optimize(desiredState, present_rotation)

        if open_loop:
            percent_output = state.speed / self.MAX_SPEED  # TODO: define the max speed in meters/second #DONE
            self.driveMotor.set(ControlMode.PercentOutput, percent_output)
        else:
            velocity = self.MPSToDriveVelocity(state.speed)
            self.driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, self.driveFeedForward.calculate(state.speed))

        angle = self.DegToTurnCount(state.angle.degrees())
        self.turningMotor.set(ControlMode.Position, angle)

    # Rod: I don't think that this method is ever called, so we may not need to implement it.
    # Also, I'm not sure we want to reset the absolute encoder used for turning control.
    def resetEncoders(self) -> None:
        self.driveEncoder.reset()
        self.turningEncoder.reset()
    
    def DegToTurnCount(self, deg):
        return deg * (2048/360) * self.TURNING_GEAR_RATIO #150/7 : 1
    
    def TurnCountToDeg(self, count):
        return count * (360/2048) / self.TURNING_GEAR_RATIO
    
    def driveCountToMeters(self, x):
        return (x/2048) * (self.WHEELDIAMETER * math.pi) / self.DRIVE_GEAR_RATIO #6.75 : 1
    
    def metersToDriveCount(self, x):
        return (2048/x) / (self.WHEELDIAMETER * math.pi) * self.DRIVE_GEAR_RATIO
    
    def driveVelocitytToMPS(self, x):
        return self.driveCountToMeters(x * 10)     #.getSelectedSensorVelocity measures counts per 1/10 of a second, rather than per second

    def MPSToDriveVelocity(self, x):
        pass

    def optimize(self, desired_state: SwerveModuleState, current_angle: Rotation2d) -> SwerveModuleState:
        '''Our own optimize method (instead of SwerveModuleState.optimize())
            There are two ways for a swerve module to reach its goal:
            1) Rotate to its intended steering angle and drive at its intended speed.
            2) Rotate to the mirrored steering angle (subtract 180) and drive at the opposite of its intended speed.
            Optimizing finds the option that requires the smallest rotation by the module.
        '''
        # This method is customized from WPILib's version to include placing in appropriate scope for 
        # CTRE Falcon's onobard control.
        target_angle = self.placeInAppropriate0To360Scope(current_angle.degrees(), desired_state.angle.degrees())
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


    def placeInAppropriate0To360Scope(self, scope_reference: float, new_angle: float) -> float:
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
    
    def resetSteering(self):
        """Call this when the steering arrow is pointed straight forward, aligned with the others."""
        self.turningMotor.setSelectedSensorPosition(0.0)

    def steeringDiagnostic(self):
        """Call this at the end of RobotInit() to check on absolute position calls.
           The idea is to wait until all the other traffic on the bus has hopefully died down,
           and then to ask for absolute position several times and see what it returns. 
           Are all 10 the same?  Are they the same as the absolute position returned during
           the module's __init__()?
        """
        print('steeringDiag')
        # Get the absolute position 10 times and print it, spaced by 0.1 second.
        for x in range(10):
            print(self.absEnc.getAbsolutePosition())
            time.sleep(0.1) # In general, using time.sleep() is not a good idea in a robot program; doing this for diagnostics only.

