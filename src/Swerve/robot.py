import wpilib
import wpilib.drive
import wpimath
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
import ctre
import time
#import cv2
from SwerveDrivetrain import SwerveDrivetrain
from SwerveModule import SwerveModule          #we use functions in SwerveModule to make calculations in autonomus
#import 
import utilities
from Manipulator import Manipulator;

class Myrobot(wpilib.TimedRobot):

    kPIDLoopIdx = 0

    kTimeoutMs = 10

    kSlotIdx  = 0


    #encoder motor values for the absolute position CANCoders in degrees

    def robotInit(self):
        

        #the wrist and arm motor commands
        self.Man = Manipulator()
        self.wrist = self.Man.wristmotor
        self.arm = self.Man.armmotor
        self.arm2 = self.Man.armmotor2
        self.stator_Passed = False
        
        #Create a manipulator object and call it "grabber".
        

        self.AUTO_SCORING_RIGHT = 'Cube Scoring Right'

        self.AUTO_SCORING_LEFT = 'Cube Scoring Left'

        self.AUTO_DOCKING = 'Docking'

        self.AUTO_SCORE_LOW = 'low score'

        self.DO_NOTHING = 'do nothing'

        self.CONE_SCORING = 'high cone'

        wpilib.SmartDashboard.putStringArray('Auto List', [self.AUTO_SCORING_LEFT, self.AUTO_SCORING_RIGHT, self.AUTO_DOCKING, self.AUTO_SCORE_LOW, self.DO_NOTHING, self.CONE_SCORING])


        self.wiggleTimer = wpilib.Timer()

        self.halfSpeed = False

        self.swerve = SwerveDrivetrain()

        self.invert_request_state = 0
        
        #changes the limit of rate of change in the input value. the smaller the number the longer it takes to reach the destination if slew rate = 1 and input = 1 it would take 1 secnd to accelerate to full speed
        #if input = 0.5 it would take 0.5 seconds to accelerate to desired speed.
        self.xSpeedLimiter = SlewRateLimiter(3)
        self.ySpeedLimiter = SlewRateLimiter(3)
        self.rotLimiter = SlewRateLimiter(3)
    
        self.xboxD = wpilib.XboxController(1)#Driver xbox
        self.xboxO = wpilib.XboxController(0)#Operator xbox

         #ARM AND WRIST CONSTANTS


        # INITS BELOW TAKEN FROM WIGGLETON ROBOT.PY

        #Initializing the arm wrist motors from SwerveDrivetrain file and claw motors
        
        
        self.claw = ctre.TalonFX(12)

        #setting the motors to brake
        self.claw.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        

        #launching the camera in driver station
        wpilib.CameraServer.launch()
        
        self.state = 0 #initializing the state for the arm to activate the state machines



        #this is the wrist PID loop that can controll the motor deceleration and acceleration bettween certain points and with handling weight and pull better
        self.wrist.configNominalOutputForward(0, self.kTimeoutMs)
        self. wrist.configNominalOutputReverse(0, self.kTimeoutMs)
        self.wrist.configPeakOutputForward(1, self.kTimeoutMs)
        self.wrist.configPeakOutputReverse(-1, self.kTimeoutMs)#MAKE SURE THE PEAK AND NOMINAL OUTPUTS ARE NOT BOTH FORWARD OR BOTH REVERSE
        
        self.wrist.selectProfileSlot(self.kSlotIdx, self.kPIDLoopIdx)
        self.wrist.config_kF(0, 0, self.kTimeoutMs)
        self.wrist.config_kP(0, 0.3, self.kTimeoutMs)
        self.wrist.config_kI(0, 0, self.kTimeoutMs)
        self.wrist.config_kD(0, 0, self.kTimeoutMs)

        self.wrist.configMotionCruiseVelocity(self.Man.wrist_Range_Counts / 5, self.kTimeoutMs)
        self.wrist.configMotionAcceleration(self.Man.wrist_Range_Counts / 3.5, self.kTimeoutMs)

        self.wrist.setSelectedSensorPosition(utilities.wristDegrees_to_counts(self.Man.WRIST_START), self.kPIDLoopIdx, self.kTimeoutMs) #MONDAY NIGHT- made it so that when it's in the wrist sensor initializes to 56deg.
        # self.wrist.setSelectedSensorPosition(utilities.wristDegrees_to_counts(self.WRIST_START)) #CHANGED MONDAY NIGHT- this was set at wrist inner, but we need it to initialize at wrist resting position (56deg) 

        #arm PID loops
        self.arm.configNominalOutputForward(0, self.kTimeoutMs)
        self.arm.configNominalOutputReverse(0, self.kTimeoutMs)
        self.arm.configPeakOutputForward(1, self.kTimeoutMs)
        self.arm.configPeakOutputReverse(-1, self.kTimeoutMs)#MAKE SURE THE PEAK AND NOMINAL OUTPUTS ARE NOT BOTH FORWARD OR BOTH REVERSE
        
        
        self.arm.selectProfileSlot(self.kSlotIdx, self.kPIDLoopIdx)
        self.arm.config_kF(0, 0, self.kTimeoutMs)
        self.arm.config_kP(0, 0.3, self.kTimeoutMs)
        self.arm.config_kI(0, 0, self.kTimeoutMs)
        self.arm.config_kD(0, 0, self.kTimeoutMs)

        self.arm.configMotionCruiseVelocity(self.Man.arm_Range_Counts / 10, self.kTimeoutMs)
        self.arm.configMotionAcceleration(self.Man.arm_Range_Counts / 10, self.kTimeoutMs)

        #we need to have the arm on the lowest position when turned on
        self.arm.setSelectedSensorPosition(utilities.armDegrees_to_counts(self.Man.ARM_MIN))   
        
        start = time.time()
        self.swerve.gyro.calibrate()#8/3/2023 changed gyro reset to calibrate to possibly stop it from drifting
        #calibrate takes five seconds so cant be put in autonomous init that runs 20ms before auto periodic
        end = time.time()
        self.elapsed = start - end
        print (f" took :{self.elapsed=} seconds")
        wpilib.SmartDashboard.putString('DB/String 0',(str(self.elapsed)))
        wpilib.SmartDashboard.putNumber("gyro", 0)        
        
        


    def disabledInit(self):
        self.autoPlan = self.DO_NOTHING

        
        self.wiggleTimer.reset()
        self.wiggleTimer.start()

        #setting all motors to coast to move them around in disabled
        self.wrist.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        self.arm.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        self.arm2.setNeutralMode(ctre._ctre.NeutralMode.Coast)


        self.swerve.frontRight.driveMotor.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        self.swerve.frontLeft.driveMotor.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        self.swerve.backRight.driveMotor.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        self.swerve.backLeft.driveMotor.setNeutralMode(ctre._ctre.NeutralMode.Coast)

        self.swerve.frontRight.turningMotor.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        self.swerve.frontLeft.turningMotor.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        self.swerve.backRight.turningMotor.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        self.swerve.backLeft.turningMotor.setNeutralMode(ctre._ctre.NeutralMode.Coast)


    def disabledPeriodic(self):

        position = self.swerve.get_pose()
        moduleFRState = self.swerve.frontRight.getState()
        moduleFLState = self.swerve.frontLeft.getState()
        moduleBRState = self.swerve.backRight.getState()
        moduleBLState = self.swerve.backLeft.getState()
        modFRCAN = self.swerve.frontRight.absEnc.getAbsolutePosition()
        modFLCAN = self.swerve.frontLeft.absEnc.getAbsolutePosition()
        modBRCAN = self.swerve.backRight.absEnc.getAbsolutePosition()
        modBLCAN = self.swerve.backLeft.absEnc.getAbsolutePosition()

        motorPos = self.arm.getSelectedSensorPosition()
        wristPos = self.wrist.getSelectedSensorPosition()

        Wrist_Angle_Deg = utilities.wristCounts_to_degrees(wristPos)
        Arm_Angle_Deg = utilities.armCounts_to_degrees(motorPos)

        wpilib.SmartDashboard.putString('DB/String 0',"FR: {:4.1f}  {:4.1f}".format(moduleFRState.angle.degrees() % 360, modFRCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 1',"FL: {:4.1f}  {:4.1f}".format(moduleFLState.angle.degrees() % 360, modFLCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 2',"BR: {:4.1f}  {:4.1f}".format(moduleBRState.angle.degrees() % 360, modBRCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 3',"BL: {:4.1f}  {:4.1f}".format(moduleBLState.angle.degrees() % 360, modBLCAN % 360))

        wpilib.SmartDashboard.putString('DB/String 4',"Xposition: {:4.2f}".format(position.X()))
        wpilib.SmartDashboard.putString('DB/String 5',"Yposition: {:4.2f}".format(position.Y()))

        #wpilib.SmartDashboard.putString('DB/String 5',"FR: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[1].angle.degrees() % 360, moduleFRState.speed))
        #wpilib.SmartDashboard.putString('DB/String 6',"FL: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[0].angle.degrees() % 360, moduleFLState.speed))
        #wpilib.SmartDashboard.putString('DB/String 7',"BR: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[3].angle.degrees() % 360, moduleBRState.speed))
        #wpilib.SmartDashboard.putString('DB/String 8',"BL: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[2].angle.degrees() % 360, moduleBLState.speed))

        # wpilib.SmartDashboard.putString('DB/String 9',"robot angle: {:4.2f}".format(self.swerve.get_heading().degrees() % 360))
        wpilib.SmartDashboard.putString('DB/String 9',"Wrist_Pos_Deg: {:4.2f}".format(Wrist_Angle_Deg))
        wpilib.SmartDashboard.putString('DB/String 8',"Arm_Pos_Deg: {:4.2f}".format(Arm_Angle_Deg))

        if wpilib.SmartDashboard.getBoolean('DB/Button 0', True):
            self.autoPlan = 1
        elif wpilib.SmartDashboard.getBoolean('DB/Button 1', True):
            self.autoPlan = 2
        elif wpilib.SmartDashboard.getBoolean('DB/Button 2', True):
            self.autoPlan = 3

        



        

    def disabledExit(self):

        self.wiggleTimer.stop()

        wpilib.SmartDashboard.putString('DB/String 0',"")
        wpilib.SmartDashboard.putString('DB/String 1',"")
        wpilib.SmartDashboard.putString('DB/String 2',"")
        wpilib.SmartDashboard.putString('DB/String 3',"")
        wpilib.SmartDashboard.putString('DB/String 4',"")
        wpilib.SmartDashboard.putString('DB/String 5',"")
        wpilib.SmartDashboard.putString('DB/String 6',"")
        wpilib.SmartDashboard.putString('DB/String 7',"")
        wpilib.SmartDashboard.putString('DB/String 8',"")
        wpilib.SmartDashboard.putString('DB/String 9',"")

        #setting motors back to brake before enabling
        self.wrist.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.arm.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.arm2.setNeutralMode(ctre._ctre.NeutralMode.Brake)


        self.swerve.frontRight.driveMotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.swerve.frontLeft.driveMotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.swerve.backRight.driveMotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.swerve.backLeft.driveMotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)

        self.swerve.frontRight.turningMotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.swerve.frontLeft.turningMotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.swerve.backRight.turningMotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.swerve.backLeft.turningMotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)


    def autonomousInit(self):

        self.wiggleTimer.reset()
        self.wiggleTimer.start()
        #self.wiggleTimer.restart()
        
        self.autoState = 0
        GYRO_OFFSET = 180

        self.swerve.gyro.reset()

        #Example for the buttons
        # self.foo = wpilib.SmartDashboard.getBoolean('DB/Button 0', True) #May have to adjust the text of the "key" (DB/Button 0)

        #Example to get floating point numbers from the sliders 
        # self.user_value = wpilib.SmartDashboard.getNumber('DB/Slider 1', 0.0) #0.0 is the similar default value.

        self.autoPlan = wpilib.SmartDashboard.getString('Auto Selector', 'did not find it')
        print(self.autoPlan)  


    def autonomousPeriodic(self):

        self.fieldRelative = False

        self.swerve.periodic()
     
        self.motorPos2 = self.arm.getSelectedSensorPosition()
        self.Arm_Angle_Deg2 = utilities.armCounts_to_degrees(self.motorPos2)
        self.autopos = self.swerve.get_pose()
        self.wristPos = self.wrist.getSelectedSensorPosition()
        
        # meters = self.swerve.frontRight.driveCountToMeters(self.swerve.frontRight.driveMotor.getSelectedSensorPosition)
        # meters = SwerveModule.driveCountToMeters(self.swerve.backLeft.driveMotor.getSelectedSensorPosition)
        # meters = self.swerve.get_pose()

        # if wpilib.SmartDashboard.give me the DB button

        #The if statement below sets the what autonomous mode we want
        if self.autoPlan == self.AUTO_SCORING_LEFT or self.autoPlan == self.AUTO_SCORING_RIGHT:
            self.autoScore()
        elif self.autoPlan == self.AUTO.DOCKING:
            self.autoDocking()
        elif self.autoPlan == self.DO_NOTHING:
            self.autoDoNothing()
        elif self.autoPlan == self.AUTO_SCORE_LOW:
            self.autoScoreLow()
        elif self.autoPlan == self.CONE_SCORING:
            self.autoConeScoreHigh()
        else:
            pass


    def autoDoNothing(self):

        self.AUTOSTATE_LIFTARM = 0 
        self.AUTOSTATE_POSITION_WRIST = 1
        self.AUTOSTATE_OUTTAKE = 2
        self.AUTOSTATE_DRIVE_BACK = 3
        self.AUTOSTATE_DRIVE_SIDEWAYS = 4
        self.AUTOSTATE_ESCAPE_COMMUNITY = 5
        self.AUTO_STOPPING = 6
        self.AUTO_METER = -1.0

        self.UPPER_POSITION = utilities.armDegrees_to_counts(11)  
        self.ARM_UPPER_THRESHOLD = self.highCube
        self.ARM_GROUND_POSITION = self.groundLevel

        self.WRIST_OUT_POSITION = self.WRIST_MIN
        self.WRIST_OUT_THRESHOLD = self.WRIST_MID
        
        #THIS IS DEADBAND FOR WRIST 
        self.AUTO_DEADBAND_IN_DEGREES = 2.0

        if self.autoState == 0:
            pass
        pass



    def autoScore(self):

        # Enumerate our new auto states
        self.AUTOSTATE_LIFTARM = 0 
        self.AUTOSTATE_POSITION_WRIST = 1
        self.AUTOSTATE_OUTTAKE = 2
        self.AUTOSTATE_DRIVE_BACK = 3
        self.AUTOSTATE_DRIVE_SIDEWAYS = 4
        self.AUTOSTATE_ESCAPE_COMMUNITY = 5
        self.AUTO_STOPPING = 6
        self.AUTO_METER = -1.0

        self.UPPER_POSITION = utilities.armDegrees_to_counts(11)  
        self.ARM_UPPER_THRESHOLD = self.Man.highCube
        self.ARM_GROUND_POSITION = self.Man.groundLevel
        # ARM_LOWER_THRESHOLD = self.Man.groundLevel



        #high cube is set to 10 and highest position is set to 11 and we set the motor to 
        #move to 11 degrees but because its not exact it will always be under that amount
        # so the threshold is set to 10 to stop it. 
        self.WRIST_OUT_POSITION = -89
        self.WRIST_OUT_THRESHOLD = self.Man.WRIST_MID
        
        

        #THIS IS DEADBAND FOR WRIST 
        self.AUTO_DEADBAND_IN_DEGREES = 2.0

        # wpilib.SmartDashboard.putData('DB/String 0', f"{motorPos2}")
        #wpilib.SmartDashboard.putString('DB/String 8',"Wrist_Pos_Deg: {:4.2f}".format(autopos.angle()))
        #wpilib.SmartDashboard.putString('DB/String 2',"Xposition: {:4.2f}".format(AUTOSTATE_PARK_WRIST_AND_ARM))
        wpilib.SmartDashboard.putString('DB/String 0',"Xposition: {:4.2f}".format(self.autopos.X()))
        wpilib.SmartDashboard.putString('DB/String 1',"Yposition: {:4.2f}".format(self.autopos.Y()))


        if self.autoState == self.AUTOSTATE_LIFTARM:
            print("autostate = AUTOSTATE_LIFTARM")
            self.wristDesiredPos2 = 15
            xSpeed = 0
            ySpeed = 0
            rot = 0
            #if abs(motorPos2 - ARM_THRESHOLD) > AUTO_DEADBAND_IN_DEGREES:
            print(f"{self.motorPos2}-{self.ARM_UPPER_THRESHOLD} = {self.motorPos2 - self.ARM_UPPER_THRESHOLD}")
            if self.motorPos2 <= self.ARM_UPPER_THRESHOLD:
                armDesiredPosition = self.UPPER_POSITION
            else:
                armDesiredPosition = self.ARM_UPPER_THRESHOLD
                self.autoState = self.AUTOSTATE_POSITION_WRIST
            
        elif self.autoState == self.AUTOSTATE_POSITION_WRIST:
            print(abs(self.motorPos2 - self.ARM_UPPER_THRESHOLD))
            print("autostate = 1")
            xSpeed = 0
            ySpeed = 0
            rot = 0

            self.wristDesiredPos2 = self.WRIST_OUT_POSITION #position not threshold because after that we call for the threshold in this state
            armDesiredPosition = self.ARM_UPPER_THRESHOLD
            #self.wrist.set(ctre._ctre.ControlMode.MotionMagic, utilities.wristDegrees_to_counts(WRIST_MAX_POSITION))

            wristPosition = self.wrist.getSelectedSensorPosition()
            wristPositionDegrees = utilities.wristCounts_to_degrees(wristPosition)
            print(f"{wristPositionDegrees} - {self.WRIST_OUT_THRESHOLD} = {wristPositionDegrees - self.WRIST_OUT_THRESHOLD}")
            if abs(wristPositionDegrees - self.WRIST_OUT_THRESHOLD) < self.AUTO_DEADBAND_IN_DEGREES or self.wiggleTimer.advanceIfElapsed(3):
                #self.wrist.set(ctre._ctre.ControlMode.PercentOutput, 0)
                # if self.wiggleTimer.advanceIfElapsed(4.0):
                self.autoState = self.AUTOSTATE_OUTTAKE
                self.wiggleTimer.reset() 

        elif self.autoState == self.AUTOSTATE_OUTTAKE:
            print("autostate = AUTOSTATE_OUTTAKE")
            self.wristDesiredPos2 = self.WRIST_OUT_THRESHOLD
            armDesiredPosition = self.ARM_UPPER_THRESHOLD
            xSpeed = 0
            ySpeed = 0
            rot = 0
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, -0.5)
            # self.wiggleTimer.reset() 
            if self.wiggleTimer.advanceIfElapsed(1.0):
                self.autoState = self.AUTOSTATE_DRIVE_BACK

        elif self.autoState == self.AUTOSTATE_DRIVE_BACK:
            print("autostate = AUTOSTATE_DRIVE_BACK")
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
            self.wristDesiredPos2 = self.Man.WRIST_MAX  
            armDesiredPosition = self.Man.groundLevel
            xSpeed = -0.5
            ySpeed = 0
            rot = 0
            if self.autopos.X() <= self.AUTO_METER:
                self.autoState = self.AUTOSTATE_DRIVE_SIDEWAYS
                print ("passed")

    
        elif self.autoState == self.AUTOSTATE_DRIVE_SIDEWAYS:
                
            print("autostate = AUTOSTATE_PARK_WRIST_AND_ARM")
            xSpeed = 0
            rot = 0
            self.wristDesiredPos2 = self.Man.WRIST_MAX
            armDesiredPosition = self.Man.groundLevel
            if self.autoPlan == self.AUTO_SCORING_LEFT:
                ySpeed = -0.5
                if self.autopos.Y() <= -1.0:
                    self.autoState = self.AUTOSTATE_ESCAPE_COMMUNITY
            else:
                ySpeed = 0.5
                if self.autopos.Y() >= 1.0:
                    self.autoState = self.AUTOSTATE_ESCAPE_COMMUNITY
        elif self.autoState == self.AUTOSTATE_ESCAPE_COMMUNITY:
            self.wristDesiredPos2 = self.Man.WRIST_MAX  
            armDesiredPosition = self.Man.groundLevel
            xSpeed = -0.5
            ySpeed = 0
            rot = 0
            if self.autopos.X() <= -5.0:
                self.autoState = self.AUTO_STOPPING
        else: #including auto stopping
            self.wristDesiredPos2 = self.Man.WRIST_MAX  
            armDesiredPosition = self.Man.groundLevel
            xSpeed = 0
            ySpeed = 0
            rot = 0



        
        Wrist_Convertion2 = self.wristDesiredPos2 -(63 + self.Arm_Angle_Deg2) #takes the desired position (in or out) and compensates for the angle ofthe arm
        self.wrist.set(ctre._ctre.ControlMode.MotionMagic, utilities.wristDegrees_to_counts(Wrist_Convertion2))
        self.arm.set(ctre._ctre.ControlMode.MotionMagic, armDesiredPosition)

        self.swerve.drive(xSpeed, ySpeed, rot, self.fieldRelative)



    def autoScoreHigh(self):
    # Enumerate our new auto states
        wristDesiredPos = 15
        AUTOSTATE_LIFTARM = 0 
        AUTOSTATE_POSITION_WRIST = 1
        AUTOSTATE_OUTTAKE = 2
        AUTOSTATE_DRIVE_BACK = 3
        AUTOSTATE_DRIVE_SIDEWAYS = 4
        AUTOSTATE_ESCAPE_COMMUNITY = 5
        AUTO_STOPPING = 6
        AUTO_METER = -1.0

        self.UPPER_POSITION = self.Man.shootingCone + 1
        self.ARM_UPPER_THRESHOLD = self.Man.shootingCone
        self.ARM_GROUND_POSITION = self.Man.groundLevel
        # ARM_LOWER_THRESHOLD = self.groundLevel

        #high cube is set to 10 and highest position is set to 11 and we set the motor to 
        #move to 11 degrees but because its not exact it will always be under that amount
        # so the threshold is set to 10 to stop it. 
        self.WRIST_OUT_POSITION = -79
        self.WRIST_OUT_THRESHOLD = self.Man.WRIST_SHOOTING

        if self.autoState == AUTOSTATE_LIFTARM:
            print("autostate = AUTOSTATE_LIFTARM")
            self.wristDesiredPos2 = 15
            xSpeed = 0
            ySpeed = 0
            rot = 0
            #if abs(motorPos2 - ARM_THRESHOLD) > AUTO_DEADBAND_IN_DEGREES:
            print(f"{self.motorPos2}-{self.ARM_UPPER_THRESHOLD} = {self.motorPos2 - self.ARM_UPPER_THRESHOLD}")
            if self.motorPos2 <= self.ARM_UPPER_THRESHOLD:
                armDesiredPosition = self.UPPER_POSITION
            else:
                armDesiredPosition = self.ARM_UPPER_THRESHOLD
                self.autoState = self.AUTOSTATE_POSITION_WRIST
            
        elif self.autoState == self.AUTOSTATE_POSITION_WRIST:
            print(abs(self.motorPos2 - self.ARM_UPPER_THRESHOLD))
            print("autostate = 1")
            xSpeed = 0
            ySpeed = 0
            rot = 0
            if self.wristPos <= self.ARM_UPPER_THRESHOLD:
                wristDesiredPos = self.WRIST_OUT_POSITION
            else:
                wristDesiredPos = self.WRIST_OUT_THRESHOLD
                self.autoState = self.AUTO_STOPPING
        elif self.state == AUTO_STOPPING:
            wristDesiredPos = self.Man.WRIST_MAX
            armDesiredPosition = self.Man.groundLevel
            xSpeed = 0
            ySpeed = 0
            rot = 0


        Wrist_Convertion = wristDesiredPos -(63 + self.Arm_Angle_Deg2) #takes the desired position (in or out) and compensates for the angle ofthe arm
        self.wrist.set(ctre._ctre.ControlMode.MotionMagic, utilities.wristDegrees_to_counts(Wrist_Convertion))
        self.arm.set(ctre._ctre.ControlMode.MotionMagic, armDesiredPosition)

        self.swerve.drive(xSpeed, ySpeed, rot, self.fieldRelative)
        
        



    def autoDocking(self):
    
        if self.autoState == 0:
            print("autostate = 0")
            self.xSpeed = 0.5
            self.ySpeed = 0
            self.rot = 0
            if self.wiggleTimer.advanceIfElapsed(1):
                print ("timer has passed")
                self.autoState = 1
        elif self.autoState == 1:
            print("autostate = 1")
            self.xSpeed = 0
            self.ySpeed = 0
            rot = 0
            if self.wiggleTimer.advanceIfElapsed(0.5):
                print ("timer has passed")
                self.autoState = 2
        elif self.autoState == 2:
            print("autostate = 2")
            self.xSpeed = -0.5
            self.ySpeed = 0
            self.rot = 0
            if self.wiggleTimer.advanceIfElapsed(1):
                print ("timer has passed")
                self.autoState = 3
        elif self.autoState == 3:
            print("autostate = 3")
            self.xSpeed = 0
            self.ySpeed = 0
            self.rot = 0
            if self.wiggleTimer.advanceIfElapsed(0.5):
                self.autoState = 4
        elif self.autoState == 4:
            print("autostate = 4")
            self.xSpeed = 1
            self.ySpeed = 1
            self.rot = 0
            if self.wiggleTimer.advanceIfElapsed(1):
                self.autoState = 5
        elif self.autoState == 5:
            self.xSpeed = 0
            self.ySpeed = 0
            self.rot = 0
            self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.highCube)
            self.wristDesiredPos2 = self.WRIST_MID
            if self.wiggleTimer.advanceIfElapsed(4):
                print ("timer has passed")
                self.autoState = 6
        elif self.autoState == 6:
            self.xSpeed = 0
            self.ySpeed = 0
            self.rot = 0
            self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.groundLevel)
            self.wristDesiredPos2 = self.WRIST_MAX
            if self.wiggleTimer.advanceIfElapsed(4):
                print ("timer has passed")
                self.autoState = 7
        elif self.autoState == 7:
            print("autostate = 7")
            self.xSpeed = 0
            self.ySpeed = 0
            self.rot = 1
            if self.wiggleTimer.advanceIfElapsed(1):
                print ("timer has passed")
                self.autoState = 8
        elif self.autoState == 8:
            print("autostate = 8")
            self.xSpeed = 0
            self.ySpeed = 0
            self.rot = 0



    def autoScoreLow(self):

        AUTO_WRIST_OUT = 0
        AUTO_OUTTAKE = 1
        AUTO_WRIST_IN = 2
        AUTO_DRIVE_OUT = 3
        AUTO_STOP = 4

        WRIST_OUT_POSITION = self.Man.WRIST_MIN
        WRIST_OUT_THRESHOLD = self.Man.WRIST_MID
        WRIST_IN_POSITION = 31
        WRIST_IN_THRESHOLD = self.Man.WRIST_MAX#wrist max is 30





        if self.autoState == AUTO_WRIST_OUT:
            xSpeed = 0
            ySpeed = 0
            rot = 0
            self.wristDesiredPos2 = WRIST_OUT_POSITION
            wristPositionDegrees = utilities.wristCounts_to_degrees(wristPosition)
            if abs(wristPositionDegrees - self.WRIST_OUT_THRESHOLD) < self.AUTO_DEADBAND_IN_DEGREES or self.wiggleTimer.advanceIfElapsed(3):
                self.autoState = AUTO_OUTTAKE

        elif self.autoState == AUTO_OUTTAKE:
            xSpeed = 0
            ySpeed = 0
            rot = 0
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, -0.5)
            self.wiggleTimer.reset() 
            if self.wiggleTimer.advanceIfElapsed(1.0):
                self.autoState = AUTO_WRIST_IN
        elif self.autoState == AUTO_WRIST_IN:
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
            xSpeed = 0
            ySpeed = 0
            rot = 0
            if self.wristPosition <= WRIST_IN_THRESHOLD:
                self.wristDesiredPos2 = WRIST_IN_POSITION
            else:
                self.wrist.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
                self.autoState = AUTO_DRIVE_OUT
        elif self.autoState == AUTO_DRIVE_OUT:
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)  
            xSpeed = -0.5
            ySpeed = 0
            rot = 0
            if self.autopos.X() <= self.AUTO_METER:
                self.autoState = AUTO_STOP
        elif self.autoSate == AUTO_STOP:
            self.wristDesiredPos2 = self.Man.WRIST_MAX  
            xSpeed = 0
            ySpeed = 0
            rot = 0
        else:
            self.wristDesiredPos2 = self.Man.WRIST_MAX  
            xSpeed = 0
            ySpeed = 0
            rot = 0
        

        wristPosition = self.wrist.getSelectedSensorPosition()
        Wrist_Convertion2 = self.wristDesiredPos2 -(63 + self.Arm_Angle_Deg2) #takes the desired position (in or out) and compensates for the angle ofthe arm
        self.wrist.set(ctre._ctre.ControlMode.MotionMagic, utilities.wristDegrees_to_counts(Wrist_Convertion2))

        self.swerve.drive(xSpeed, ySpeed, rot, self.fieldRelative)


    def autonomousExit(self):
        
        self.wiggleTimer.stop()

        wpilib.SmartDashboard.putString('DB/String 0',"")
        wpilib.SmartDashboard.putString('DB/String 1',"")
        wpilib.SmartDashboard.putString('DB/String 2',"")
        wpilib.SmartDashboard.putString('DB/String 3',"")
        wpilib.SmartDashboard.putString('DB/String 4',"")
        wpilib.SmartDashboard.putString('DB/String 5',"")
        wpilib.SmartDashboard.putString('DB/String 6',"")
        wpilib.SmartDashboard.putString('DB/String 7',"")
        wpilib.SmartDashboard.putString('DB/String 8',"")
        wpilib.SmartDashboard.putString('DB/String 9',"")

    def teleopInit(self):

        
        self.wiggleTimer.reset()
        self.wiggleTimer.start()
  

        # These three need to be removed for competition

        self.state = 0
        self.invert_request_state = 0

        self.intake_state = 0 
        self.intake_cooldown = 0
        
        # For teleop testing purposes, align the wheels before enabling.
        # For the match, move the following statement to automomousInit().
        # self.swerve.resetSteering()
        self.fieldRelative = True

    def teleopPeriodic(self):
        '''
        self.grabber.set_arm_position(Manipulator.ARM_TUCKED_IN)

        #Uses the "class attribute/variable/constant" as a parameter.
        '''

        #these two lines are lifted from SwerveRobot.py, as is the driveWithJoystick function
        self.driveWithJoystick(True)
        self.swerve.periodic()

        #Brock asked if he could reset in in the middle of a match.
        #these button inputs are obviously subject to change
        if self.xboxD.getRightBumper() and self.xboxD.getLeftBumper():
            self.swerve.gyro.reset()

        #setting variables for modules to rint to dashboard
        moduleFRState = self.swerve.frontRight.getState()
        moduleFLState = self.swerve.frontLeft.getState()
        moduleBRState = self.swerve.backRight.getState()
        moduleBLState = self.swerve.backLeft.getState()
        modFRCAN = self.swerve.frontRight.absEnc.getAbsolutePosition()
        modFLCAN = self.swerve.frontLeft.absEnc.getAbsolutePosition()
        modBRCAN = self.swerve.backRight.absEnc.getAbsolutePosition()
        modBLCAN = self.swerve.backLeft.absEnc.getAbsolutePosition()

        gyro_heading = self.swerve.gyro.getAngle()

        #printing module turning angles to dahboard and a few other things
        wpilib.SmartDashboard.putString('DB/String 0',"FR: {:4.1f}  {:4.1f}".format(moduleFRState.angle.degrees() % 360, modFRCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 1',"FL: {:4.1f}  {:4.1f}".format(moduleFLState.angle.degrees() % 360, modFLCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 2',"BR: {:4.1f}  {:4.1f}".format(moduleBRState.angle.degrees() % 360, modBRCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 3',"BL: {:4.1f}  {:4.1f}".format(moduleBLState.angle.degrees() % 360, modBLCAN % 360))

        wpilib.SmartDashboard.putString('DB/String 5',"FR: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[1].angle.degrees() % 360, moduleFRState.speed))
        wpilib.SmartDashboard.putString('DB/String 6',"FL: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[0].angle.degrees() % 360, moduleFLState.speed))
        wpilib.SmartDashboard.putString('DB/String 7',"BR: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[3].angle.degrees() % 360, moduleBRState.speed))
        #wpilib.SmartDashboard.putString('DB/String 8',"BL: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[2].angle.degrees() % 360, moduleBLState.speed))

        wpilib.SmartDashboard.putNumber('gyro', gyro_heading)

        wpilib.SmartDashboard.putNumber('claw/motoroutputpercent', self.claw.getMotorOutputPercent())
        wpilib.SmartDashboard.putNumber('claw/motoroutputvoltage', self.claw.getMotorOutputVoltage())
        wpilib.SmartDashboard.putNumber('claw/statorcurrent', self.claw.getStatorCurrent())
        wpilib.SmartDashboard.putNumber('claw/supplycurrent', self.claw.getSupplyCurrent())
        #wpilib.SmartDashboard.putString('DB/String 9',"robot angle: {:4.2f}".format(self.swerve.get_heading().degrees() % 360))
        #wpilib.SmartDashboard.putString('DB/String 4',"Mod4TurnEnc: {:4.2f}".format(self.mod4turnEnc % 360))
        

        wpilib.SmartDashboard.putString('DB/String 6',"State:  ".format(self.state))


        # EVERYTHING BELOW IS TAKEN FROM WIGGLETON ROBOT.PY


        # getting arm and wrist motor positions for dashboard
        motorPos = self.arm.getSelectedSensorPosition()

        wristPos = self.wrist.getSelectedSensorPosition()

        Arm_Angle_Deg = utilities.armCounts_to_degrees(motorPos)

        Wrist_Angle_Deg = utilities.wristCounts_to_degrees(wristPos)

        stator_current = self.claw.getStatorCurrent()

 
        # wpilib.SmartDashboard.putString('DB/String 9',"Arm_Pos_Degrees: {:4.2f}".format(self.xboxD.getLeftY()))
        # wpilib.SmartDashboard.putString('DB/String 8',"Time: {:3.2f}".format(self.xboxD.getLeftX()))
        wpilib.SmartDashboard.putString('DB/String 8',"Wrist_Pos_Deg: {:4.2f}".format(Wrist_Angle_Deg))
        
       
       
        AButton = self.xboxO.getAButton()
        BButton = self.xboxO.getBButton()
        XButton = self.xboxO.getXButton()
        YButton = self.xboxO.getYButton()
        lBumper = self.xboxO.getLeftBumper()
        rBumper = self.xboxO.getRightBumper()
        lTrigger = self.xboxO.getLeftTriggerAxis()
        rTrigger = self.xboxO.getRightTriggerAxis()
        start = self.xboxO.getStartButton() 
        lStickButton = self.xboxO.getLeftStickButton()
        rStickButton = self.xboxO.getRightStickButton()
        back = self.xboxO.getBackButton()
        driver_AButton = self.xboxD.getAButton()
        driver_BButton = self.xboxD.getBButton



        IDLE = 0
        MID_CUBE = 1
        MID_CONE = 2
        CONE_FEEDER = 3
        ARM_DOWN = 4
        ORIGINAL_POS = 5
        WRIST_IN = 6
        WRIST_OUT = 7
        HIGH_CUBE = 8
        CUBE_INTAKE = 9
        CUBE_OUTTAKE = 10
        CONE_INTAKE = 11
        CONE_OUTTAKE = 12
        ARM_DOWN_WRIST_IN = 13
        SHOOTING = 14



        # ARM / WRIST STATE MACHINE
        #these are the buttons that trigger the states
        if lBumper: #mid cube
            self.state = MID_CUBE
        elif rBumper: #midcone 
            self.state = MID_CONE
        elif lTrigger > 0.1: #high cube
            self.state = CONE_FEEDER
        elif start: #ground level wrist down
            self.state = SHOOTING
        elif back: #ground level wrist in
            self.state = ARM_DOWN_WRIST_IN
        elif lStickButton: # wrist in
            self.state = WRIST_IN
        elif rStickButton: # wrist down
            self.state = WRIST_OUT
        elif rTrigger > 0.1:
            self.state = HIGH_CUBE
        elif AButton and self.stator_Passed == False:
            self.state = CUBE_INTAKE
        elif BButton:
            self.state = CUBE_OUTTAKE
        elif XButton and self.stator_Passed == False:
            self.state = CONE_INTAKE
        elif YButton:
            self.state = CONE_OUTTAKE
        
                

        # these are the actions dealing with the states
        
        if self.state == IDLE:
            self.wrist.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
            
        else:
            if self.state == MID_CUBE:

                self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.Man.midCube) #sets the arm motor to desired height
                self.Man.wristDesiredPos = self.Man.WRIST_MIN

            elif self.state == SHOOTING:
                self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.Man.shootingCone)
                self.Man.wristDesiredPos = self.Man.WRIST_SHOOTING 

            elif self.state == MID_CONE:

                self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.Man.midCone)
                self.Man.wristDesiredPos = self.Man.WRIST_MIN 

            elif self.state == CONE_FEEDER:
                self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.Man.feederstation)
                self.Man.wristDesiredPos = self.Man.WRIST_MIN 
                

            elif self.state == ARM_DOWN:

                self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.Man.groundLevel)
                self.Man.wristDesiredPos = self.Man.WRIST_MIN 

            elif self.state == ORIGINAL_POS:
                if Arm_Angle_Deg < 0:
              
                    self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.Man.groundLevel)
                    self.Man.wristDesiredPos = self.Man.WRIST_MAX
                    self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)

                    if not (AButton or XButton) and self.stator_Passed == True:
                        self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
                        self.stator_Passed = False

                elif Arm_Angle_Deg > 0:
                    self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)

                    if not (AButton or XButton) and self.stator_Passed == True:
                        self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
                        self.stator_Passed = False

            elif self.state == ARM_DOWN_WRIST_IN:
                    self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.Man.groundLevel)
                    self.Man.wristDesiredPos = self.Man.WRIST_MAX


            elif self.state == WRIST_IN:
                self.Man.wristDesiredPos = self.Man.WRIST_MAX

            elif self.state == WRIST_OUT:
                self.Man.wristDesiredPos = self.Man.WRIST_MIN

            elif self.state == HIGH_CUBE:
                self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.Man.highCube)
                self.Man.wristDesiredPos = self.Man.WRIST_MID

            #elif (self.state == CUBE_INTAKE or CUBE_OUTTAKE or CONE_INTAKE or CUBE_OUTTAKE):
            elif not (AButton or BButton or XButton or YButton) and Arm_Angle_Deg < 0:
                self.state = ORIGINAL_POS
            elif not (AButton or BButton or XButton or YButton) and Arm_Angle_Deg > 0:
                self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)

            elif self.state == CUBE_INTAKE and Arm_Angle_Deg < 0:
                self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.4)
                self.Man.wristDesiredPos = self.Man.WRIST_MIN
                if stator_current > 70:
                    print ("STATOR PASSED CUBE")
                    self.stator_Passed = True
                    self.state = ORIGINAL_POS
            elif self.state == CUBE_INTAKE and Arm_Angle_Deg > 0:
                self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.4)
                if stator_current > 70:
                    print ("STATOR PASSED CUBE")
                    self.stator_Passed = True
                    self.state = ORIGINAL_POS

            elif self.state == CUBE_OUTTAKE:
                self.claw.set(ctre._ctre.ControlMode.PercentOutput, -0.5)

            elif self.state == CONE_INTAKE and Arm_Angle_Deg < 0:
                self.claw.set(ctre._ctre.ControlMode.PercentOutput, -0.75)
                self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.Man.groundCone)
                self.Man.wristDesiredPos = self.Man.WRIST_MIN
                if stator_current > 40:
                    print ("STATOR PASSED CONE")
                    self.stator_Passed = True
                    self.state = ORIGINAL_POS
            elif self.state == CONE_INTAKE and Arm_Angle_Deg > 0:
                self.claw.set(ctre._ctre.ControlMode.PercentOutput, -0.75)
                if stator_current > 40:
                    print ("STATOR PASSED CONE")
                    self.stator_Passed = True
                    self.state = ORIGINAL_POS

            elif self.state == CONE_OUTTAKE:
                self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.75)


            else:
                self.state = IDLE

        

            #DO NOT set 63 in wrist convertion to Wrist_Min it ruins the positions 
            Wrist_Convertion = self.Man.wristDesiredPos -(63 + Arm_Angle_Deg) #takes the desired position (in or out) and compensates for the angle ofthe arm
            self.wrist.set(ctre._ctre.ControlMode.MotionMagic, utilities.wristDegrees_to_counts(Wrist_Convertion))
            #each time the code moves through the "else" states, it moves the arm to the desired position within that state, then passes the value of the wrist's desired position 
            # to the function "self.wrist.set()" at the end.  Thus, unlike the arm being set in each state, the wrist's VALUE is set in each state, and that value
            #is used to set the wrist position each time the "else" runs.
            


        # CUBE_OUTTAKE = 1

        # #CLAW CODE DEALING WITH STATES AND STATOR SPIKES

        # #intake state machine that take actions from the controller and trigger the states
        # stator_current = self.claw.getStatorCurrent()

        # if AButton and self.intake_state == 0 and time.time() > self.intake_cooldown: #cube INTAKE
        #     self.intake_state = CUBE_OUTTAKE
        # elif stator_current > 70: #CUBE stator current value
        #     self.intake_state = 2
        # elif XButton and self.intake_state == 0 and time.time() > self.intake_cooldown: #cone OUTAKE
        #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, -0.75)
        #     self.intake_state = 3
        # elif stator_current > 40: #CONE stator current value
        #     self.intake_state = 4
        # elif BButton: #cube OUTTAKE
        #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, -0.5) 
        # elif YButton: #cone OUTTAKE
        #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.75)
        # elif self.intake_state == 0:
        #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
        # elif AButton:
        #     self.wrist.set(ctre._ctre.ControlMode.MotionMagic, self.Man.wristGroudLevel)
        # elif XButton:
        #     if Arm_Angle_Deg < 0:
        #         self.wrist.set(ctre._ctre.ControlMode.MotionMagic, self.Man.wristCone)
        #         self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.Man.groundCone)
        #     elif Arm_Angle_Deg > 0:
        #         if XButton: #cone OUTAKE
        #             self.claw.set(ctre._ctre.ControlMode.PercentOutput, -0.75)
        # else: 
        #     if not (BButton or YButton or AButton or XButton) :
        #         self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
        #         self.wrist.set(ctre._ctre.ControlMode.MotionMagic, self.Man.wristInnerPos)
        #         self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.Man.groundLevel)


        #     # Big if..elif..else that looks at each possible state and handles state transitions between them.
        # if self.intake_state == IDLE:
        #     # Logic to check inputs and change to new states.  Notice no self.claw.set() here.
        #     if AButton:
        #         self.intake_state = CUBE_INTAKE
        #     elif BButton:
        #         self.intake_state = CUBE_OUTTAKE
        #     ...
        # elif self.intake_state == CUBE_INTAKE:
        #     if not AButton and stator_current > 70:
        #         self.intake_state = COOLING_OFF         # A special cooling off state to let the driver release the button.
        #         self.intake_cooldown = time.time() + 1.0 # Capture time for the cooling off interval.
        # elif self.intake_state == COOLING_OFF:
        #     if time.time() > self.intake_cooldown:
        #         self.intake_state = IDLE
        
        # # code to handle cube outake and cone
        # else:
        #     # In case there is a syntax error and the state gets set to an invalid value, set back to idle.
        #     self.intake_state = IDLE

        # # Another big if..elif to handle actions based on what state we are in.
        # # This ensures that self.claw.set() is called exactly once per teleopPeriodic()
        # # Also note that there are NO state transitions here.
        # if self.intake_state == IDLE or self.intake_state == COOLING_OFF:
        #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
        # elif self.intake_state == CUBE_INTAKE:
        #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.4)
        # elif self.intake_state == CUBE_OUTTAKE:
        #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, -0.5)
        
        # # other cases
        
        # else:
        #     # A default case just in case the state is set to an invalid value.
        #     # Note that we could omit the IDLE and COOLDOWN above and just include them in the default
        #     # because I've decided that the default is zero.  But this way is clearer.
        #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
        

        
        # Intake state machine for cubes - based on the stator current that determins the percent output 

        # if self.intake_state == 0:
        #     # Claw is idle
        #     pass
        # if self.intake_state == 1: 
        #     # Check if A button released
        #     if not AButton:
        #         self.intake_state = 0
        #     # Get current stator
        #     stator_current = self.claw.getStatorCurrent()
        # if self.intake_state == 2:
        #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
        #     self.intake_state = 0
        #     self.intake_cooldown = time.time() + 1.0
        

        # # Cone intake state machine

        # if self.intake_state == 0:
        #     # Claw is idle
        #     pass
        # if self.intake_state == 3:
        #     # Check if Y button released
        #     if not XButton:
        #         self.intake_state = 0
        #     # Get current stator
        # if self.intake_state == 4:
        #     print("Intake shutdown")
        #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
        #     self.intake_state = 0
        #     self.intake_cooldown = time.time() + 2.0   

    def teleopExit(self):

        self.wiggleTimer.stop()

        #self.webcam.release()
        wpilib.SmartDashboard.putString('DB/String 0',"")
        wpilib.SmartDashboard.putString('DB/String 1',"")
        wpilib.SmartDashboard.putString('DB/String 2',"")
        wpilib.SmartDashboard.putString('DB/String 3',"")
        wpilib.SmartDashboard.putString('DB/String 4',"")
        wpilib.SmartDashboard.putString('DB/String 5',"")
        wpilib.SmartDashboard.putString('DB/String 6',"")
        wpilib.SmartDashboard.putString('DB/String 7',"")
        wpilib.SmartDashboard.putString('DB/String 8',"")
        wpilib.SmartDashboard.putString('DB/String 9',"")

    def driveWithJoystick(self, fieldRelativeParam: bool) -> None:
        '''
        Inversion code.
        This is a two-state state machine because this function is called many times per second, so we have to ignore multiple "button presses". 
        If the machine is in state 0, it checks for the start button.
        If the start button is hit, it inverts and sets state to 1 and sets a timer to the current time.
        In the second state, the current time is read. If the current time is greater than 1 second since the inversion, it resets
        the state machine to state 0
        '''
        INVERT_STATE_WAITING_FOR_BUTTON = 0
        INVERT_STATE_DELAY = 1
        INVERT_DELAY_TIME_IN_SECONDS = 1.0
        if self.invert_request_state == INVERT_STATE_WAITING_FOR_BUTTON:
            if self.xboxD.getStartButton() :
                self.swerve.toggleDriveMotorsInverted()
                self.invert_request_state = INVERT_STATE_DELAY
                self.last_invert_time = time.time()
        if self.invert_request_state == INVERT_STATE_DELAY:
                if time.time() > self.last_invert_time + INVERT_DELAY_TIME_IN_SECONDS:
                    self.invert_request_state = INVERT_STATE_WAITING_FOR_BUTTON

        self.joystick_x = -self.xboxD.getLeftX()
        self.joystick_y = -self.xboxD.getLeftY()
        self.joystick_x = applyDeadband(self.joystick_x , 0.1)
        self.joystick_y = applyDeadband(self.joystick_y , 0.1)

        rot = -self.xboxD.getRightX()
        rot = applyDeadband(rot, 0.05)
        

        

        if self.xboxD.getRightTriggerAxis() > 0.9 and self.xboxD.getAButton():
            self.halfSpeed = True
        elif self.xboxD.getLeftTriggerAxis() > 0.9 and self.xboxD.getAButton():
            self.halfSpeed = False

        if self.halfSpeed == True:
            joystick_y = utilities.joystickscaling(self.joystick_y )
            xSpeed = self.xSpeedLimiter.calculate(joystick_y) * SwerveDrivetrain.getMaxSpeed() / 6

            joystick_x = utilities.joystickscaling(self.joystick_x )
            ySpeed = self.ySpeedLimiter.calculate(joystick_x) * SwerveDrivetrain.MAX_SPEED / 6

            rot = utilities.joystickscaling(rot)
            rot = self.rotLimiter.calculate(rot) * SwerveDrivetrain.MAX_ANGULAR_SPEED / 3

            self.swerve.drive(xSpeed, ySpeed, rot, fieldRelativeParam)

        else:
            # Get the x speed. We are inverting this because Xbox controllers return
            # negative values when we push forward.
            joystick_y = utilities.joystickscaling(self.joystick_y )
            xSpeed = self.xSpeedLimiter.calculate(joystick_y) * SwerveDrivetrain.getMaxSpeed()

            # Get the y speed. We are inverting this because Xbox controllers return
            # negative values when we push to the left.
            joystick_x = utilities.joystickscaling(self.joystick_x)
            ySpeed = self.ySpeedLimiter.calculate(joystick_x) * SwerveDrivetrain.MAX_SPEED

            rot = utilities.joystickscaling(rot)
            rot = self.rotLimiter.calculate(rot) * SwerveDrivetrain.MAX_ANGULAR_SPEED

            self.swerve.drive(xSpeed, ySpeed, rot, fieldRelativeParam)
    
        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.



if __name__ == '__main__':
    wpilib.run(Myrobot)
