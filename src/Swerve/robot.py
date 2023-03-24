# !/usr/bin/env python3

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


class Myrobot(wpilib.TimedRobot):

    kPIDLoopIdx = 0

    kTimeoutMs = 10

    kSlotIdx  = 0


    #encoder motor values for the absolute position CANCoders in degrees

    def robotInit(self):
        self.AUTO_SCORING_RIGHT = 'Cube Scoring Right'

        self.AUTO_SCORING_LEFT = 'Cube Scoring Left'

        self.AUTO_DOCKING = 'Docking'

        self.AUTO_SCORE_LOW = 'low score'

        self.DO_NOTHING = 'do nothing'

        wpilib.SmartDashboard.putStringArray('Auto List', [self.AUTO_SCORING_LEFT, self.AUTO_SCORING_RIGHT, self.AUTO_DOCKING, self.AUTO_SCORE_LOW, self.DO_NOTHING])


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

        #Initializing the arm wrist and claw motors
        self.wristmotor = ctre.TalonFX(11)
        self.armmotor = ctre.TalonFX(2)
        self.armmotor2 = ctre.TalonFX(3)
        self.claw = ctre.TalonFX(12)

        #setting the motors to coast or brake
        self.claw.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.wristmotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.armmotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.armmotor2.setNeutralMode(ctre._ctre.NeutralMode.Brake)

        #this makes the armmotor2 follow the other one but the oppose master makes it go the opposite direction from the motor it follows
        self.armmotor2.follow(self.armmotor)
        self.armmotor2.setInverted(ctre._ctre.InvertType.OpposeMaster)

        #launching the camera in driver station
        wpilib.CameraServer.launch()

        self.ARM_GEAR_RATIO = 35.7 * 4      #to get the gear ratio, we had 48 teeth from the sprocket and divided it from the output sahft which is 12, then we multiplied that from the gear ratio which is 12.75
        self.ARM_MIN = -63.0
        self.ARM_HORIZONTAL = 0.0
        self.ARM_MAX = 26.0 #might change to 13.0
        self.PEAK_ARM_FF = 0.07 #this is the amount of output we need in the motor to hold itself when it is in its desired position
        self.WRIST_GEAR_RATIO = 80
        self.WRIST_START = 30  # 56 degrees from the "tucked in position"
        self.WRIST_MAX = 30 # We're calling the "tucked in position" 0 degrees
        self.WRIST_MIN = -121 # Degrees, wrist dropped in collecting position at the ground level.
        self.WRIST_MID = -90
        self.wristDesiredPos = 15
        self.state = 0 #initializing the state for the arm to activate the state machines

        wrist_Range_Counts = self.wristDegrees_to_counts(self.WRIST_MAX - self.WRIST_MIN)
        arm_Range_Counts = self.armDegrees_to_counts(self.ARM_MAX - self.ARM_MIN)

        #these are all the positions that will be used for the arm
        self.groundLevel = self.armDegrees_to_counts(-63) 
        self.feederStation = self.armDegrees_to_counts(17) 
        self.midCube = self.armDegrees_to_counts(2) 
        self.midCone = self.armDegrees_to_counts(7)
        self.highCube = self.armDegrees_to_counts(10)
        self.highestpoint = self.armDegrees_to_counts(24)
         
        #positons for wrist
        self.wristGroudLevel = self.wristDegrees_to_counts(self.WRIST_MIN)
        self.wristInnerPos = self.wristDegrees_to_counts(self.WRIST_MAX)

        #this is the wrist PID loop that can controll the motor deceleration and acceleration bettween certain points and with handling weight and pull better
        self.wristmotor.configNominalOutputForward(0, self.kTimeoutMs)
        self.wristmotor.configNominalOutputReverse(0, self.kTimeoutMs)
        self.wristmotor.configPeakOutputForward(1, self.kTimeoutMs)
        self.wristmotor.configPeakOutputReverse(-1, self.kTimeoutMs)#MAKE SURE THE PEAK AND NOMINAL OUTPUTS ARE NOT BOTH FORWARD OR BOTH REVERSE
        
        self.wristmotor.selectProfileSlot(self.kSlotIdx, self.kPIDLoopIdx)
        self.wristmotor.config_kF(0, 0, self.kTimeoutMs)
        self.wristmotor.config_kP(0, 0.3, self.kTimeoutMs)
        self.wristmotor.config_kI(0, 0, self.kTimeoutMs)
        self.wristmotor.config_kD(0, 0, self.kTimeoutMs)

        self.wristmotor.configMotionCruiseVelocity(wrist_Range_Counts / 5, self.kTimeoutMs)
        self.wristmotor.configMotionAcceleration(wrist_Range_Counts / 3.5, self.kTimeoutMs)

        self.wristmotor.setSelectedSensorPosition(self.wristDegrees_to_counts(self.WRIST_START), self.kPIDLoopIdx, self.kTimeoutMs) #MONDAY NIGHT- made it so that when it's in the wrist sensor initializes to 56deg.
        # self.wristmotor.setSelectedSensorPosition(self.wristDegrees_to_counts(self.WRIST_START)) #CHANGED MONDAY NIGHT- this was set at wrist inner, but we need it to initialize at wrist resting position (56deg) 

        #arm PID loops
        self.armmotor.configNominalOutputForward(0, self.kTimeoutMs)
        self.armmotor.configNominalOutputReverse(0, self.kTimeoutMs)
        self.armmotor.configPeakOutputForward(1, self.kTimeoutMs)
        self.armmotor.configPeakOutputReverse(-1, self.kTimeoutMs)#MAKE SURE THE PEAK AND NOMINAL OUTPUTS ARE NOT BOTH FORWARD OR BOTH REVERSE
        
        
        self.armmotor.selectProfileSlot(self.kSlotIdx, self.kPIDLoopIdx)
        self.armmotor.config_kF(0, 0, self.kTimeoutMs)
        self.armmotor.config_kP(0, 0.3, self.kTimeoutMs)
        self.armmotor.config_kI(0, 0, self.kTimeoutMs)
        self.armmotor.config_kD(0, 0, self.kTimeoutMs)

        self.armmotor.configMotionCruiseVelocity(arm_Range_Counts / 10, self.kTimeoutMs)
        self.armmotor.configMotionAcceleration(arm_Range_Counts / 10, self.kTimeoutMs)

        #we need to have the arm on the lowest position when turned on
        self.armmotor.setSelectedSensorPosition(self.armDegrees_to_counts(self.ARM_MIN))   
        
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
        self.wristmotor.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        self.armmotor.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        self.armmotor2.setNeutralMode(ctre._ctre.NeutralMode.Coast)


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

        wristPos = self.wristmotor.getSelectedSensorPosition()
        Wrist_Angle_Deg = self.wristCounts_to_degrees(wristPos)

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
        self.wristmotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.armmotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.armmotor2.setNeutralMode(ctre._ctre.NeutralMode.Brake)


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

        fieldRelative = False

        self.swerve.periodic()
     
        motorPos2 = self.armmotor.getSelectedSensorPosition()
        Arm_Angle_Deg2 = self.armCounts_to_degrees(motorPos2)
        autopos = self.swerve.get_pose()
        
        # meters = self.swerve.frontRight.driveCountToMeters(self.swerve.frontRight.driveMotor.getSelectedSensorPosition)
        # meters = SwerveModule.driveCountToMeters(self.swerve.backLeft.driveMotor.getSelectedSensorPosition)
        # meters = self.swerve.get_pose()

        # if wpilib.SmartDashboard.give me the DB button
        if self.autoPlan == self.DO_NOTHING:
            AUTOSTATE_LIFTARM = 0 
            AUTOSTATE_POSITION_WRIST = 1
            AUTOSTATE_OUTTAKE = 2
            AUTOSTATE_DRIVE_BACK = 3
            AUTOSTATE_DRIVE_SIDEWAYS = 4
            AUTOSTATE_ESCAPE_COMMUNITY = 5
            AUTO_STOPPING = 6
            AUTO_METER = -0.4

            UPPER_POSITION = self.armDegrees_to_counts(11)  
            ARM_UPPER_THRESHOLD = self.highCube
            ARM_GROUND_POSITION = self.groundLevel

            WRIST_OUT_POSITION = self.WRIST_MIN
            WRIST_OUT_THRESHOLD = self.WRIST_MID
            
            #THIS IS DEADBAND FOR WRIST 
            AUTO_DEADBAND_IN_DEGREES = 2.0

            if self.autoState == 0:
                pass
            pass
        elif self.autoPlan == self.AUTO_SCORING_LEFT or self.autoPlan == self.AUTO_SCORING_RIGHT:
             # Enumerate our new auto states
            AUTOSTATE_LIFTARM = 0 
            AUTOSTATE_POSITION_WRIST = 1
            AUTOSTATE_OUTTAKE = 2
            AUTOSTATE_DRIVE_BACK = 3
            AUTOSTATE_DRIVE_SIDEWAYS = 4
            AUTOSTATE_ESCAPE_COMMUNITY = 5
            AUTO_STOPPING = 6
            AUTO_METER = -0.4

            UPPER_POSITION = self.armDegrees_to_counts(11)  
            ARM_UPPER_THRESHOLD = self.highCube
            ARM_GROUND_POSITION = self.groundLevel
            # ARM_LOWER_THRESHOLD = self.groundLevel



            #high cube is set to 10 and highest position is set to 11 and we set the motor to 
            #move to 11 degrees but because its not exact it will always be under that amount
            # so the threshold is set to 10 to stop it. 

            WRIST_OUT_POSITION = -89
            WRIST_OUT_THRESHOLD = self.WRIST_MID
            
            

            #THIS IS DEADBAND FOR WRIST 
            AUTO_DEADBAND_IN_DEGREES = 2.0

            # wpilib.SmartDashboard.putData('DB/String 0', f"{motorPos2}")
            #wpilib.SmartDashboard.putString('DB/String 8',"Wrist_Pos_Deg: {:4.2f}".format(autopos.angle()))
            #wpilib.SmartDashboard.putString('DB/String 2',"Xposition: {:4.2f}".format(AUTOSTATE_PARK_WRIST_AND_ARM))
            wpilib.SmartDashboard.putString('DB/String 0',"Xposition: {:4.2f}".format(autopos.X()))
            wpilib.SmartDashboard.putString('DB/String 1',"Yposition: {:4.2f}".format(autopos.Y()))


            if self.autoState == AUTOSTATE_LIFTARM:
                print("autostate = AUTOSTATE_LIFTARM")
                self.wristDesiredPos2 = 15
                xSpeed = 0
                ySpeed = 0
                rot = 0
                #if abs(motorPos2 - ARM_THRESHOLD) > AUTO_DEADBAND_IN_DEGREES:
                print(f"{motorPos2}-{ARM_UPPER_THRESHOLD} = {motorPos2 - ARM_UPPER_THRESHOLD}")
                if motorPos2 <= ARM_UPPER_THRESHOLD:
                    armDesiredPosition = UPPER_POSITION
                else:
                    armDesiredPosition = ARM_UPPER_THRESHOLD
                    self.autoState = AUTOSTATE_POSITION_WRIST
                
            elif self.autoState == AUTOSTATE_POSITION_WRIST:
                print(abs(motorPos2 - ARM_UPPER_THRESHOLD))
                print("autostate = 1")
                xSpeed = 0
                ySpeed = 0
                rot = 0

                self.wristDesiredPos2 = WRIST_OUT_POSITION #position not threshold because after that we call for the threshold in this state
                armDesiredPosition = ARM_UPPER_THRESHOLD
                #self.wristmotor.set(ctre._ctre.ControlMode.MotionMagic, self.wristDegrees_to_counts(WRIST_MAX_POSITION))

                wristPosition = self.wristmotor.getSelectedSensorPosition()
                wristPositionDegrees = self.wristCounts_to_degrees(wristPosition)
                print(f"{wristPositionDegrees} - {WRIST_OUT_THRESHOLD} = {wristPositionDegrees - WRIST_OUT_THRESHOLD}")
                if abs(wristPositionDegrees - WRIST_OUT_THRESHOLD) < AUTO_DEADBAND_IN_DEGREES:
                    #self.wristmotor.set(ctre._ctre.ControlMode.PercentOutput, 0)
                    # if self.wiggleTimer.advanceIfElapsed(4.0):
                    self.autoState = AUTOSTATE_OUTTAKE
                    self.wiggleTimer.reset() 

            elif self.autoState == AUTOSTATE_OUTTAKE:
                print("autostate = AUTOSTATE_OUTTAKE")
                self.wristDesiredPos2 = WRIST_OUT_THRESHOLD
                armDesiredPosition = ARM_UPPER_THRESHOLD
                xSpeed = 0
                ySpeed = 0
                rot = 0
                self.claw.set(ctre._ctre.ControlMode.PercentOutput, -0.5)
                # self.wiggleTimer.reset() 
                if self.wiggleTimer.advanceIfElapsed(1.0):
                    self.autoState = AUTOSTATE_DRIVE_BACK

            elif self.autoState == AUTOSTATE_DRIVE_BACK:
                print("autostate = AUTOSTATE_DRIVE_BACK")
                self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0)
                self.wristDesiredPos2 = self.WRIST_MAX  
                armDesiredPosition = self.groundLevel
                xSpeed = -0.5
                ySpeed = 0
                rot = 0
                if autopos.X() <= AUTO_METER:
                    self.autoState = AUTOSTATE_DRIVE_SIDEWAYS
                    print ("passed")

        
            elif self.autoState == AUTOSTATE_DRIVE_SIDEWAYS:
                    
                print("autostate = AUTOSTATE_PARK_WRIST_AND_ARM")
                xSpeed = 0
                rot = 0
                self.wristDesiredPos2 = self.WRIST_MAX
                armDesiredPosition = self.groundLevel
                if self.autoPlan == self.AUTO_SCORING_LEFT:
                    ySpeed = -0.5
                    if autopos.Y() <= -0.4:
                        self.autoState = AUTOSTATE_ESCAPE_COMMUNITY
                else:
                    ySpeed = 0.5
                    if autopos.Y() >= 0.4:
                        self.autoState = AUTOSTATE_ESCAPE_COMMUNITY
            elif self.autoState == AUTOSTATE_ESCAPE_COMMUNITY:
                self.wristDesiredPos2 = self.WRIST_MAX  
                armDesiredPosition = self.groundLevel
                xSpeed = -0.5
                ySpeed = 0
                rot = 0
                if autopos.X() <= -4.0:
                    self.autoState = AUTO_STOPPING
            else: #including auto stopping
                self.wristDesiredPos2 = self.WRIST_MAX  
                armDesiredPosition = self.groundLevel
                xSpeed = 0
                ySpeed = 0
                rot = 0



            
            Wrist_Convertion2 = self.wristDesiredPos2 -(63 + Arm_Angle_Deg2) #takes the desired position (in or out) and compensates for the angle ofthe arm
            self.wristmotor.set(ctre._ctre.ControlMode.MotionMagic, self.wristDegrees_to_counts(Wrist_Convertion2))
            self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, armDesiredPosition)

            self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative)




        elif self.autoPlan == self.AUTO_DOCKING:
            if self.autoState == 0:
                print("autostate = 0")
                xSpeed = 0.5
                ySpeed = 0
                rot = 0
                if self.wiggleTimer.advanceIfElapsed(1):
                    print ("timer has passed")
                    self.autoState = 1
            elif self.autoState == 1:
                print("autostate = 1")
                xSpeed = 0
                ySpeed = 0
                rot = 0
                if self.wiggleTimer.advanceIfElapsed(0.5):
                    print ("timer has passed")
                    self.autoState = 2
            elif self.autoState == 2:
                print("autostate = 2")
                xSpeed = -0.5
                ySpeed = 0
                rot = 0
                if self.wiggleTimer.advanceIfElapsed(1):
                    print ("timer has passed")
                    self.autoState = 3
            elif self.autoState == 3:
                print("autostate = 3")
                xSpeed = 0
                ySpeed = 0
                rot = 0
                if self.wiggleTimer.advanceIfElapsed(0.5):
                    self.autoState = 4
            elif self.autoState == 4:
                print("autostate = 4")
                xSpeed = 1
                ySpeed = 1
                rot = 0
                if self.wiggleTimer.advanceIfElapsed(1):
                    self.autoState = 5
            elif self.autoState == 5:
                xSpeed = 0
                ySpeed = 0
                rot = 0
                self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.highCube)
                self.wristDesiredPos2 = self.WRIST_MID
                if self.wiggleTimer.advanceIfElapsed(4):
                    print ("timer has passed")
                    self.autoState = 6
            elif self.autoState == 6:
                xSpeed = 0
                ySpeed = 0
                rot = 0
                self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.groundLevel)
                self.wristDesiredPos2 = self.WRIST_MAX
                if self.wiggleTimer.advanceIfElapsed(4):
                    print ("timer has passed")
                    self.autoState = 7
            elif self.autoState == 7:
                print("autostate = 7")
                xSpeed = 0
                ySpeed = 0
                rot = 1
                if self.wiggleTimer.advanceIfElapsed(1):
                    print ("timer has passed")
                    self.autoState = 8
            elif self.autoState == 8:
                print("autostate = 8")
                xSpeed = 0
                ySpeed = 0
                rot = 0

        elif self.autoPlan == self.AUTO_SCORE_LOW:
            AUTO_WRIST_OUT = 0
            AUTO_OUTTAKE = 1
            AUTO_WRIST_IN = 2
            AUTO_DRIVE_OUT = 3
            AUTO_STOP = 4

            WRIST_OUT = -122
            WRIST_OUT_THRESHOLD = self.WRIST_MIN#wrist min is -121
            WRIST_IN = 31
            WRIST_IN_THRESHOLD = self.WRIST_MAX#wrist max is 30
            METER_DISTANCE = -5





            if self.autoState == AUTO_WRIST_OUT:
                xSpeed = 0
                ySpeed = 0
                rot = 0
                print(f"{wristPosition}-{ARM_THRESHOLD} = {motorPos2 - ARM_THRESHOLD}")
                if wristPosition <= WRIST_OUT_THRESHOLD:
                    self.wristmotor.set(ctre._ctre.ControlMode.MotionMagic, WRIST_OUT)
                else:
                    self.wristmotor.set(ctre._ctre.ControlMode.MotionMagic, WRIST_OUT_THRESHOLD)
                    self.autoState = AUTO_OUTTAKE
                    self.wiggleTimer.reset()

            elif self.autoState == AUTO_OUTTAKE:
                xSpeed = 0
                ySpeed = 0
                rot = 0
                self.claw.set(ctre._ctre.ControlMode.PercentOutput, -0.5)
                if self.wiggleTimer.advancedIfElapsed(1):
                    self.autoState = AUTO_WRIST_IN
            elif self.autoState == AUTO_WRIST_IN:
                self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
                xSpeed = 0
                ySpeed = 0
                rot = 0
                print(f"{wristPosition}-{ARM_THRESHOLD} = {motorPos2 - ARM_THRESHOLD}")
                if wristPosition <= WRIST_IN_THRESHOLD:
                    self.wristmotor.set(ctre._ctre.ControlMode.MotionMagic, WRIST_IN)
                else:
                    self.wristmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
                    self.autoState = AUTO_DRIVE_OUT
            elif self.autoState == AUTO_DRIVE_OUT:
                xSpeed = -0.5
                ySpeed = 0
                rot = 0
                if autopos.X() <= METER_DISTANCE:
                    self.autoState = AUTO_STOP
            elif self.autoState == AUTO_STOP:
                xSpeed = 0
                ySpeed = 0
                rot = 0







            self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative)


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
        

    def teleopPeriodic(self):

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
        


        # EVERYTHING BELOW IS TAKEN FROM WIGGLETON ROBOT.PY


        # getting arm and wrist motor positions for dashboard
        motorPos = self.armmotor.getSelectedSensorPosition()

        wristPos = self.wristmotor.getSelectedSensorPosition()

        Arm_Angle_Deg = self.armCounts_to_degrees(motorPos)

        Wrist_Angle_Deg = self.wristCounts_to_degrees(wristPos)

 
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


        # ARM / WRIST STATE MACHINE
        #these are the buttons that trigger the states
        if lBumper: #mid cube
            self.state = 1
        elif rBumper: #midcone 
            self.state = 2
        elif lTrigger > 0.1: #high cube
            self.state = 3
        elif start: #ground level wrist down
            self.state = 4
        elif back: #ground level wrist in
            self.state = 5
        elif lStickButton: # wrist in
            self.state = 6
        elif rStickButton: # wrist down
            self.state = 7
        elif rTrigger > 0.1:
            self.state = 8
                

        # these are the actions dealing with the states
        
        if self.state == 0:
            self.armmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
            self.wristmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
        else:
            if self.state == 1:
                self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.midCube) #sets the arm motor to desired height
                self.wristDesiredPos = self.WRIST_MIN
            elif self.state == 2:
                self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.midCone)
                self.wristDesiredPos = self.WRIST_MIN 
            elif self.state == 3:
                self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.highestpoint)
                self.wristDesiredPos = self.WRIST_MIN 
            elif self.state == 4:
                self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.groundLevel)
                self.wristDesiredPos = self.WRIST_MIN 
            elif self.state == 5:
                self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.groundLevel)
                self.wristDesiredPos = self.WRIST_MAX
            elif self.state == 6:
                self.wristDesiredPos = self.WRIST_MAX
            elif self.state == 7:
                self.wristDesiredPos = self.WRIST_MIN
            elif self.state == 8:
                self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.highCube)
                self.wristDesiredPos = self.WRIST_MID
            else:
                self.state = 0

            #DO NOT set 63 in wrist convertion to Wrist_Min it ruins the positions 
            Wrist_Convertion = self.wristDesiredPos -(63 + Arm_Angle_Deg) #takes the desired position (in or out) and compensates for the angle ofthe arm
            self.wristmotor.set(ctre._ctre.ControlMode.MotionMagic, self.wristDegrees_to_counts(Wrist_Convertion))
            #each time the code moves through the "else" states, it moves the arm to the desired position within that state, then passes the value of the wrist's desired position 
            # to the function "self.wristmotor.set()" at the end.  Thus, unlike the arm being set in each state, the wrist's VALUE is set in each state, and that value
            #is used to set the wrist position each time the "else" runs.
            



        #CLAW CODE DEALING WITH STATES AND STATOR SPIKES

        #intake state machine that take actions from the controller and trigger the states
        stator_current = self.claw.getStatorCurrent()

        if AButton and self.intake_state == 0 and time.time() > self.intake_cooldown: #cube INTAKE
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.4)
            self.intake_state = 1 
        elif stator_current > 70: #CUBE stator current value
            self.intake_state = 2
        elif XButton and self.intake_state == 0 and time.time() > self.intake_cooldown: #cone OUTAKE
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, -0.75)
            self.intake_state = 3
        elif stator_current > 100: #CONE stator current value
            self.intake_state = 4
        elif BButton: #cube OUTTAKE
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, -0.5) 
        elif YButton: #cone OUTTAKE
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.75)
        elif self.intake_state == 0:
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0) 
        else: 
            if not (AButton or BButton or XButton or YButton) :
                self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)

        
        # Intake state machine for cubes - based on the stator current that determins the percent output 

        if self.intake_state == 0:
            # Claw is idle
            pass
        if self.intake_state == 1:
            # Check if A button released
            if not AButton:
                self.intake_state = 0
            # Get current stator
            stator_current = self.claw.getStatorCurrent()
        if self.intake_state == 2:
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
            self.intake_state = 0
            self.intake_cooldown = time.time() + 1.0
        

        # Cone intake state machine

        if self.intake_state == 0:
            # Claw is idle
            pass
        if self.intake_state == 3:
            # Check if Y button released
            if not XButton:
                self.intake_state = 0
            # Get current stator
        if self.intake_state == 4:
            print("Intake shutdown")
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
            self.intake_state = 0
            self.intake_cooldown = time.time() + 2.0   

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

    def driveWithJoystick(self, fieldRelative: bool) -> None:
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

        self.rot = self.xboxD.getRightX()
        self.rot = applyDeadband(self.rot, 0.05)
        



        if self.xboxD.getRightTriggerAxis() > 0.1:
            self.halfSpeed = True
        elif self.xboxD.getLeftTriggerAxis() > 0.1:
            self.halfSpeed = False

        if self.halfSpeed == True:
            joystick_y = self.joystickscaling(self.joystick_y / 2)
            xSpeed = self.xSpeedLimiter.calculate(joystick_y) * SwerveDrivetrain.getMaxSpeed()

            joystick_x = self.joystickscaling(self.joystick_x / 2)
            ySpeed = self.ySpeedLimiter.calculate(joystick_x) * SwerveDrivetrain.MAX_SPEED

            rot = self.joystickscaling(self.rot)
            rot = self.rotLimiter.calculate(rot) * SwerveDrivetrain.MAX_ANGULAR_SPEED

            self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative)

        else:

            # Get the x speed. We are inverting this because Xbox controllers return
            # negative values when we push forward.
            joystick_y = self.joystickscaling(self.joystick_y)
            xSpeed = self.xSpeedLimiter.calculate(joystick_y) * SwerveDrivetrain.getMaxSpeed()

            # Get the y speed. We are inverting this because Xbox controllers return
            # negative values when we push to the left.
            joystick_x = self.joystickscaling(self.joystick_x)
            ySpeed = self.ySpeedLimiter.calculate(joystick_x) * SwerveDrivetrain.MAX_SPEED

            rot = self.joystickscaling(self.rot)
            rot = self.rotLimiter.calculate(rot) * SwerveDrivetrain.MAX_ANGULAR_SPEED

            self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative)
    
        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.

    
    def armCounts_to_degrees(self, counts):

        degrees = (counts * (360/2048)) / self.ARM_GEAR_RATIO
        return degrees

    
    def armDegrees_to_counts(self, degrees):

        counts = (degrees * (2048/360)) * self.ARM_GEAR_RATIO
        return counts
    
    def wristCounts_to_degrees(self, counts):

        degrees = (counts * (360/2048)) / self.WRIST_GEAR_RATIO
        return degrees

    
    def wristDegrees_to_counts(self, degrees):

        counts = (degrees * (2048/360)) * self.WRIST_GEAR_RATIO
        return counts
    
    def joystickscaling(self, input): #this function helps bring an exponential curve in the joystick value and near the zero value it uses less value and is more flat
        a = 1
        output = a * input * input * input + (1 - a) * input
        return output
    
if __name__ == '__main__':
    wpilib.run(Myrobot)