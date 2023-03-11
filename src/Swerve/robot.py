# !/usr/bin/env python3

import wpilib
import wpilib.drive
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
import ctre
import time

#import cv2

from SwerveModule import SwerveModule
from SwerveDrivetrain import SwerveDrivetrain


class Myrobot(wpilib.TimedRobot):

    kPIDLoopIdx = 0

    kTimeoutMs = 10

    kSlotIdx  = 0
    WHEELDIAMETER = 4 * 0.0254 #4 inch diameter for the wheel times the conversion to meters
    TRACKWIDTH = 22.25 * 0.0254 #both back and front have the module center to other mod center as 22 and 1/4th
    WHEELBASE = 22.75 * 0.0254 #left side of front mod to back mod #right is 26 7/8
    TURNING_GEAR_RATIO = 150.0 / 7.0 # Docs call it a 150/7:1
    DRIVE_GEAR_RATIO = 6.75
    #WHEELCIRCUMFERENCE = WHEELDIAMETER * pi


    #encoder motor values for the absolute position CANCoders in degrees

    def robotInit(self):

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

        """inits below taken from Wiggleton robot.py"""
        self.ARM_GEAR_RATIO = 35.7 * 4      #to get the gear ratio, we had 48 teeth from the sprocket and divided it from the output sahft which is 12, then we multiplied that from the gear ratio which is 12.75
        
        self.ARM_MIN = -63.0

        self.ARM_HORIZONTAL = 0.0

        self.ARM_MAX = 26.0 #might change to 13.0

        self.PEAK_ARM_FF = 0.07 #this is the amount of output we need in the motor to hold itself when it is in its desired position

        self.WRIST_GEAR_RATIO = 80

        self.WRIST_START = 56  # 56 degrees from the "tucked in position"

        self.WRIST_MAX = 30 # We're calling the "tucked in position" 0 degrees

        self.WRIST_MIN = -115 # Degrees, wrist dropped in collecting position at the ground level.

        self.wristDesiredPos = self.WRIST_START

        #need to set a 

        self.state = 0 #initializing the state for the arm

        self.wristmotor = ctre.TalonFX(11)
        self.armmotor = ctre.TalonFX(2)
        self.armmotor2 = ctre.TalonFX(3)

        self.claw = ctre.TalonFX(11)

        self.claw.setNeutralMode(ctre._ctre.NeutralMode.Brake)

        
        self.wristmotor.setInverted(False)

    
        self.wristmotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        
        self.armmotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.armmotor2.setNeutralMode(ctre._ctre.NeutralMode.Brake)

        wpilib.CameraServer.launch()

        this makes the armmotor2 follow the other one but the oppose master makes it go the opposite direction from the motor it follows
        self.armmotor2.follow(self.armmotor)
        self.armmotor2.setInverted(ctre._ctre.InvertType.OpposeMaster)
        
        #these are all the positions that will be used for the arm
        
        self.groundLevel = self.armDegrees_to_counts(-63) 

        self.feederStation = self.armDegrees_to_counts(17) 

        self.midCube = self.armDegrees_to_counts(7) 

        self.midCone = self.armDegrees_to_counts(13)

        self.highCube = self.armDegrees_to_counts(17)
        
         
        #35.7 then multiply by 4 for sprockets

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

        wrist_Range_Counts = self.wristDegrees_to_counts(self.WRIST_MAX - self.WRIST_MIN)

        self.wristmotor.configMotionCruiseVelocity(wrist_Range_Counts / 7.5, self.kTimeoutMs)
        self.wristmotor.configMotionAcceleration(wrist_Range_Counts / 5, self.kTimeoutMs)

        self.wristmotor.setSelectedSensorPosition(self.wristDegrees_to_counts(self.WRIST_START), self.kPIDLoopIdx, self.kTimeoutMs) #MONDAY NIGHT- made it so that when it's in the wrist sensor initializes to 56deg.
        

        self.armmotor.configNominalOutputForward(0, self.kTimeoutMs)
        self.armmotor.configNominalOutputReverse(0, self.kTimeoutMs)
        self.armmotor.configPeakOutputForward(1, self.kTimeoutMs)
        self.armmotor.configPeakOutputReverse(-1, self.kTimeoutMs)#MAKE SURE THE PEAK AND NOMINAL OUTPUTS ARE NOT BOTH FORWARD OR BOTH REVERSE
        
        
        self.armmotor.selectProfileSlot(self.kSlotIdx, self.kPIDLoopIdx)
        self.armmotor.config_kF(0, 0, self.kTimeoutMs)
        self.armmotor.config_kP(0, 0.3, self.kTimeoutMs)
        self.armmotor.config_kI(0, 0, self.kTimeoutMs)
        self.armmotor.config_kD(0, 0, self.kTimeoutMs)

        arm_Range_Counts = self.armDegrees_to_counts(self.ARM_MAX - self.ARM_MIN)

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
        '''
        self.gyro_heading is the current reading from self.swerve.gyro.getAngle()
        It is measured in degrees and ranges from -180 to 180.
        Clockwise is positive (maybe confirm this!)
        '''
        self.gyro_heading = 0
        self.intake_state = 0 
        self.intake_cooldown = 0




        self.wristmotor.setSelectedSensorPosition(self.wristDegrees_to_counts(self.WRIST_START)) #CHANGED MONDAY NIGHT- this was set at wrist inner, but we need it to initialize at wrist resting position (56deg) 
        
        
        # # Temporary diagnostic to check on CANCoder communication.
        # # print('CANCoder FR')
        # # self.swerve.frontRight.steeringDiagnostic()
        # # print('CANCoder FL')
        # # self.swerve.frontLeft.steeringDiagnostic()
        # # print('CANCoder BR')
        # # self.swerve.backRight.steeringDiagnostic()
        # # print('CANCoder BL')
        # # self.swerve.backLeft.steeringDiagnostic()

    def disabledInit(self):
        
        self.wiggleTimer.reset()
        self.wiggleTimer.start()
        

    def disabledPeriodic(self):

        self.driveWithJoystick(True)

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

        #wpilib.SmartDashboard.putString('DB/String 0',"FR: {:4.1f}  {:4.1f}".format(moduleFRState.angle.degrees() % 360, modFRCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 1',"FL: {:4.1f}  {:4.1f}".format(moduleFLState.angle.degrees() % 360, modFLCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 2',"BR: {:4.1f}  {:4.1f}".format(moduleBRState.angle.degrees() % 360, modBRCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 3',"BL: {:4.1f}  {:4.1f}".format(moduleBLState.angle.degrees() % 360, modBLCAN % 360))

        #wpilib.SmartDashboard.putString('DB/String 5',"FR: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[1].angle.degrees() % 360, moduleFRState.speed))
        #wpilib.SmartDashboard.putString('DB/String 6',"FL: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[0].angle.degrees() % 360, moduleFLState.speed))
        #wpilib.SmartDashboard.putString('DB/String 7',"BR: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[3].angle.degrees() % 360, moduleBRState.speed))
        #wpilib.SmartDashboard.putString('DB/String 8',"BL: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[2].angle.degrees() % 360, moduleBLState.speed))

        # wpilib.SmartDashboard.putString('DB/String 9',"robot angle: {:4.2f}".format(self.swerve.get_heading().degrees() % 360))
        # wpilib.SmartDashboard.putString('DB/String 9',"Wrist_Pos_Deg: {:4.2f}".format(Wrist_Angle_Deg))




        

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


    def autonomousInit(self):

        self.wiggleTimer.reset()
        self.wiggleTimer.start()
        #self.wiggleTimer.restart()
        
        self.autoState = 0

        self.swerve.gyro.reset()


    def autonomousPeriodic(self):
    
        
        fieldRelative = True

        self.tick += 0.02
        self.smart_dashboard.putNumber("tick", math.cos(self.tick))


        # if wpilib.SmartDashboard.give me the DB button
        # if self.autoState == 0:
        #     xSpeed = 0
        #     ySpeed = 0
        #     rot = 0
        #     if self.wiggletimer.advanceIfElapsed(3):
        #         self.autoState = 1
        if  self.autoState == 0:
            xSpeed = 0.5
            ySpeed = 0 
            rot = 0
            if self.wiggleTimer.advanceIfElapsed(7.5):
                self.autoState = 1
        elif self.autoState == 1:
            xSpeed = 0
            ySpeed = 0 
            rot = 0
            self.wiggleTimer.advanceIfElapsed(3)
   

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
        

        # For teleop testing purposes, align the wheels before enabling.
        # For the match, move the following statement to automomousInit().
        # self.swerve.resetSteering()
        

    def teleopPeriodic(self):

        #these two lines are lifted from SwerveRobot.py, as is the driveWithJoystick function
        self.driveWithJoystick(True)
        self.swerve.periodic()

        #Brock asked if he could reset in in the middle of a match.
        #these button inputs are obviously subject to change
        if self.xboxD.getAButton() and self.xboxD.getBButton() and self.xboxD.getXButton():
            self.swerve.gyro.reset()


        moduleFRState = self.swerve.frontRight.getState()
        moduleFLState = self.swerve.frontLeft.getState()
        moduleBRState = self.swerve.backRight.getState()
        moduleBLState = self.swerve.backLeft.getState()
        modFRCAN = self.swerve.frontRight.absEnc.getAbsolutePosition()
        modFLCAN = self.swerve.frontLeft.absEnc.getAbsolutePosition()
        modBRCAN = self.swerve.backRight.absEnc.getAbsolutePosition()
        modBLCAN = self.swerve.backLeft.absEnc.getAbsolutePosition()

        self.gyro_heading = self.swerve.gyro.getAngle()

        wpilib.SmartDashboard.putString('DB/String 0',"FR: {:4.1f}  {:4.1f}".format(moduleFRState.angle.degrees() % 360, modFRCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 1',"FL: {:4.1f}  {:4.1f}".format(moduleFLState.angle.degrees() % 360, modFLCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 2',"BR: {:4.1f}  {:4.1f}".format(moduleBRState.angle.degrees() % 360, modBRCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 3',"BL: {:4.1f}  {:4.1f}".format(moduleBLState.angle.degrees() % 360, modBLCAN % 360))

        wpilib.SmartDashboard.putString('DB/String 5',"FR: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[1].angle.degrees() % 360, moduleFRState.speed))
        wpilib.SmartDashboard.putString('DB/String 6',"FL: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[0].angle.degrees() % 360, moduleFLState.speed))
        wpilib.SmartDashboard.putString('DB/String 7',"BR: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[3].angle.degrees() % 360, moduleBRState.speed))
        wpilib.SmartDashboard.putString('DB/String 8',"BL: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[2].angle.degrees() % 360, moduleBLState.speed))

        wpilib.SmartDashboard.putNumber('gyro', self.gyro_heading)

        wpilib.SmartDashboard.putNumber('claw/motoroutputpercent', self.claw.getMotorOutputPercent())
        wpilib.SmartDashboard.putNumber('claw/motoroutputvoltage', self.claw.getMotorOutputVoltage())
        wpilib.SmartDashboard.putNumber('claw/statorcurrent', self.claw.getStatorCurrent())
        wpilib.SmartDashboard.putNumber('claw/supplycurrent', self.claw.getSupplyCurrent())
        #wpilib.SmartDashboard.putString('DB/String 9',"robot angle: {:4.2f}".format(self.swerve.get_heading().degrees() % 360))
        # wpilib.SmartDashboard.putString('DB/String 4',"Mod4TurnEnc: {:4.2f}".format(self.mod4turnEnc % 360))


        """everything below is teleop periodic taken from Wiggleton robot.py"""

        # motorPos = self.armmotor.getSelectedSensorPosition()

        # wristPos = self.wristmotor.getSelectedSensorPosition()

        # Arm_Angle_Deg = self.armCounts_to_degrees(motorPos)

        # Wrist_Angle_Deg = self.wristCounts_to_degrees(wristPos)

        



        
        # wpilib.SmartDashboard.putString('DB/String 4',"Arm_Pos_Degrees: {:4.2f}".format(Arm_Angle_Deg))
        # #wpilib.SmartDashboard.putString('DB/String 4',"Time: {:3.2f}".format(self.wiggleTimer.get()))
        # wpilib.SmartDashboard.putString('DB/String 9',"Wrist_Pos_Deg: {:4.2f}".format(Wrist_Angle_Deg))
       


        #rTrigger = self.xbox.getRightTriggerAxis()
        AButton = self.xboxO.getAButton()
        BButton = self.xboxO.getBButton()
        XButton = self.xboxO.getXButton()
        YButton = self.xboxO.getYButton()
        lBumper = self.xboxO.getLeftBumper()
        rBumper = self.xboxO.getRightBumper()
        lTrigger = self.xboxO.getLeftTriggerAxis()
        start = self.xboxO.getStartButton() 
        lStickButton = self.xboxO.getLeftStickButton()
        rStickButton = self.xboxO.getRightStickButton()
        back = self.xboxO.getBackButton()



j

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
                self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.highCube)
                self.wristDesiredPos = self.WRIST_MIN 
            elif self.state == 4:
                self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.groundLevel)
                self.wristDesiredPos = self.WRIST_MIN 
            elif self.state == 5:
                self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.groundLevel)
                self.wristDesiredPos = self.WRIST_MAX
            elif self.state == 6:
                self.wristmotor.set(ctre._ctre.ControlMode.MotionMagic, self.wristInnerPos) 
                self.wristDesiredPos = self.WRIST_MAX
            elif self.state == 7:
                self.wristmotor.set(ctre._ctre.ControlMode.MotionMagic, self.wristGroudLevel)
                self.wristDesiredPos = self.WRIST_MIN
            else:
                self.state = 0

            Wrist_Convertion = self.wristDesiredPos -(63 + Arm_Angle_Deg) #takes the desired position (in or out) and compensates for the angle ofthe arm
            self.wristmotor.set(ctre._ctre.ControlMode.MotionMagic, self.wristDegrees_to_counts(Wrist_Convertion))
            #each time the code moves through the "else" states, it moves the arm to the desired position within that state, then passes the value of the wrist's desired position 
            # to the function "self.wristmotor.set()" at the end.  Thus, unlike the arm being set in each state, the wrist's VALUE is set in each state, and that value
            #is used to set the wrist position each time the "else" runs.
            
            # if AButton and BButton:
            #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
            # elif XButton and YButton:
            #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
            # elif AButton and YButton:
            #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
            # elif XButton and BButton:
            #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
            # elif AButton and XButton:
            #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
            

        if AButton and self.intake_state == 0 and time.time() > self.intake_cooldown: #cube INTAKE
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.3)
            self.intake_state = 1 
        elif BButton: #cube OUTTAKE
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, -0.25) 
        elif XButton: #cone INTAKE
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.5) 
        elif YButton and self.intake_state == 0 and time.time() > self.intake_cooldown: #cone OUTAKE
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, -0.75) 
            self.intake_state = 3
        elif self.intake_state == 0:
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0) 
        else: 
            if not (AButton or BButton or XButton or YButton) :
                self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)

        # Intake state machine for current spike
        if self.intake_state == 0:
            # Claw is idle
            pass
        if self.intake_state == 1:
            # Check if A button released
            if not AButton:
                self.intake_state = 0
            # Get current stator
            stator_current = self.claw.getStatorCurrent()
            if stator_current > 79:
                self.intake_state = 2 
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
            if not YButton:
                self.intake_state = 0
            # Get current stator
            stator_current = self.claw.getStatorCurrent()
            if stator_current > 140:
                self.intake_state = 5 
        if self.intake_state == 5:
            print("Intake shutdown")
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
            self.intake_state = 0
            self.intake_cooldown = time.time() + 1.0   

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
        

        if self.xboxD.getRightTriggerAxis():
            self.halfSpeed = True
        elif self.xboxD.getLeftTriggerAxis():
            self.halfSpeed = False



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
        
        if self.halfSpeed == True:
            joystick_y = -self.xboxD.getLeftY() / 4
            joystick_y = applyDeadband(joystick_y, 0.02)
            xSpeed = self.xSpeedLimiter.calculate(joystick_y) * SwerveDrivetrain.getMaxSpeed()

            joystick_x = -self.xboxD.getLeftX() / 4
            joystick_x = applyDeadband(joystick_x, 0.02)
            ySpeed = self.ySpeedLimiter.calculate(joystick_x) * SwerveDrivetrain.MAX_SPEED
        
        else:

            # Get the x speed. We are inverting this because Xbox controllers return
            # negative values when we push forward.
            joystick_y = -self.xboxD.getLeftY() / 2
            joystick_y = applyDeadband(joystick_y, 0.02)
            xSpeed = self.xSpeedLimiter.calculate(joystick_y) * SwerveDrivetrain.getMaxSpeed()

            # Get the y speed. We are inverting this because Xbox controllers return
            # negative values when we push to the left.
            joystick_x = -self.xboxD.getLeftX() / 2
            joystick_x = applyDeadband(joystick_x, 0.02)
            ySpeed = self.ySpeedLimiter.calculate(joystick_x) * SwerveDrivetrain.MAX_SPEED



            
        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        
        
        rot = -self.xboxD.getRightX()
        rot = applyDeadband(rot, 0.02)
        rot = self.rotLimiter.calculate(rot) * SwerveDrivetrain.MAX_ANGULAR_SPEED

        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative)

    def degrees_to_counts(self, degrees):
        counts = degrees * (2048/360) * self.TURNING_GEAR_RATIO
        return counts
    
    def counts_to_degrees(self, counts):
        degrees = counts * (360/2048) / self.TURNING_GEAR_RATIO
        return degrees
    
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
        a = 0.8
        output = a * input * input * input + (1 - a) * input
        return output
    
if __name__ == '__main__':
    wpilib.run(Myrobot)
