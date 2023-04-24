import wpilib
import wpilib.drive
import wpimath
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
import ctre
import time
from SwerveDrivetrain import SwerveDrivetrain

# Don't need to import, since it is encapsulated in SwerveDrivetrain
# from SwerveModule import SwerveModule          #we use functions in SwerveModule to make calculations in autonomus

from elevator import Elevator

class Myrobot(wpilib.TimedRobot):
    """Example robot with elevator.  Code initially based on Swerve code.
       Rod replaced arm, wrist, and claw code with the example elevator class in autonomous,
       but ran out of time to modify the teleop, so it still has arm code.
       Rod also rearranged robotInit to group related items.
    """

    def robotInit(self):
        
        # Set up autonomous scoring options and send to dashboard selector
        self.AUTO_SCORING_RIGHT = 'Cube Scoring Right'
        self.AUTO_SCORING_LEFT = 'Cube Scoring Left'
        self.AUTO_DOCKING = 'Docking'
        self.AUTO_SCORE_LOW = 'low score'
        self.DO_NOTHING = 'do nothing'
        wpilib.SmartDashboard.putStringArray('Auto List', [self.AUTO_SCORING_LEFT, self.AUTO_SCORING_RIGHT, self.AUTO_DOCKING, self.AUTO_SCORE_LOW, self.DO_NOTHING])

        # A general-purpose timer.
        self.wiggleTimer = wpilib.Timer()

        # Set up some general state variables.
        self.halfSpeed = False
        self.invert_request_state = 0
        self.state = 0 #TODO: do we need this? initializing the state for the arm to activate the state machines
        

        # Driver input and slew rate limiters for them.
        self.xboxD = wpilib.XboxController(1)#Driver xbox
        self.xboxO = wpilib.XboxController(0)#Operator xbox
        #changes the limit of rate of change in the input value. the smaller the number the longer it takes to reach the destination if slew rate = 1 and input = 1 it would take 1 secnd to accelerate to full speed
        #if input = 0.5 it would take 0.5 seconds to accelerate to desired speed.
        self.xSpeedLimiter = SlewRateLimiter(3)
        self.ySpeedLimiter = SlewRateLimiter(3)
        self.rotLimiter = SlewRateLimiter(3)
    
        # Create the game piece manipulator.
        self.manipulator = Elevator(2, 11, 12)

        # Create the swerve drive annd calibrate its gyro
        self.swerve = SwerveDrivetrain()
        # Time the calibration because we are curious.
        start = time.time()
        self.swerve.gyro.calibrate()#8/3/2023 changed gyro reset to calibrate to possibly stop it from drifting
        #calibrate takes five seconds so cant be put in autonomous init that runs 20ms before auto periodic
        end = time.time()
        self.elapsed = start - end
        print (f" took :{self.elapsed=} seconds")
        wpilib.SmartDashboard.putString('DB/String 0',(str(self.elapsed)))
        wpilib.SmartDashboard.putNumber("gyro", 0)        
        
        #launching the camera in driver station
        wpilib.CameraServer.launch()        


    def disabledInit(self):
        self.autoPlan = self.DO_NOTHING

        
        self.wiggleTimer.reset()
        self.wiggleTimer.start()

        #setting all motors to coast to move them around in disabled
        self.manipulator.set_coast()


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

        wpilib.SmartDashboard.putString('DB/String 0',"FR: {:4.1f}  {:4.1f}".format(moduleFRState.angle.degrees() % 360, modFRCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 1',"FL: {:4.1f}  {:4.1f}".format(moduleFLState.angle.degrees() % 360, modFLCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 2',"BR: {:4.1f}  {:4.1f}".format(moduleBRState.angle.degrees() % 360, modBRCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 3',"BL: {:4.1f}  {:4.1f}".format(moduleBLState.angle.degrees() % 360, modBLCAN % 360))

        wpilib.SmartDashboard.putString('DB/String 4',"Xposition: {:4.2f}".format(position.X()))
        wpilib.SmartDashboard.putString('DB/String 5',"Yposition: {:4.2f}".format(position.Y()))

        # Get elevator height and display it.
        wpilib.SmartDashboard.putString('DB/String 9',"Elev Pos: {:4.2f}".format(self.manipulator.get_actual_height()))

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
        self.manipulator.set_brake()


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

        # Call these functions once per autonomousPeriodic()
        self.manipulator.periodic()
        self.swerve.periodic()

        self.fieldRelative = False
        self.autopos = self.swerve.get_pose()
        
        #The if statement below sets the what autonomous mode we want
        if self.autoPlan == self.AUTO_SCORING_LEFT or self.autoPlan == self.AUTO_SCORING_RIGHT:
            self.autoScore()
        elif self.autoPlan == self.AUTO.DOCKING:
            self.autoDocking()
        elif self.autoPlan == self.DO_NOTHING:
            self.autoDoNothing()
        elif self.autoPlan == self.AUTO_SCORE_LOW:
            self.autoScoreLow()
        else:
            pass


    def autoDoNothing(self):
        """Just sit idle"""
        # Deleted for clarity in this example.
        pass


    def autoScore(self):

        # Enumerate our new auto states
        self.AUTOSTATE_LIFTARM = 0 
        # self.AUTOSTATE_POSITION_WRIST = 1
        self.AUTOSTATE_OUTTAKE = 2
        self.AUTOSTATE_DRIVE_BACK = 3
        self.AUTOSTATE_DRIVE_SIDEWAYS = 4
        self.AUTOSTATE_ESCAPE_COMMUNITY = 5
        self.AUTO_STOPPING = 6
        self.AUTO_METER = -1.0


        if self.autoState == self.AUTOSTATE_LIFTARM:
            # Lift the mechanism to spit 
            self.manipulator.set_position(Elevator.POS_CMD_HIGH_REACH)
            xSpeed = 0
            ySpeed = 0
            rot = 0
            if self.manipulator.is_in_position():
                self.autoState = self.AUTOSTATE_OUTTAKE
                # And reset the timer.
                self.wiggleTimer.reset()
            
        elif self.autoState == self.AUTOSTATE_OUTTAKE:
            # Keep the mechanism high and spit out the cube for 1 second.
            self.manipulator.set_position(Elevator.POS_CMD_HIGH_REACH)
            self.manipulator.set_intake(Elevator.INTAKE_CMD_OUT)
            xSpeed = 0
            ySpeed = 0
            rot = 0
            if self.wiggleTimer.advanceIfElapsed(1.0):
                self.autoState = self.AUTOSTATE_DRIVE_BACK

        elif self.autoState == self.AUTOSTATE_DRIVE_BACK:
            # Retract the mechanism and drive away a set distance.
            self.manipulator.set_position(Elevator.POS_CMD_START)
            self.manipulator.set_intake(Elevator.INTAKE_CMD_STOP)
            xSpeed = -0.5
            ySpeed = 0
            rot = 0
            if self.autopos.X() <= self.AUTO_METER:
                self.autoState = self.AUTOSTATE_DRIVE_SIDEWAYS
                print ("passed")

    
        elif self.autoState == self.AUTOSTATE_DRIVE_SIDEWAYS:
            # Manipulator should be in and stopped by now no need to call.
            xSpeed = 0
            rot = 0
            if self.autoPlan == self.AUTO_SCORING_LEFT:
                ySpeed = -0.5
                if self.autopos.Y() <= -1.0:
                    self.autoState = self.AUTOSTATE_ESCAPE_COMMUNITY
            else:
                ySpeed = 0.5
                if self.autopos.Y() >= 1.0:
                    self.autoState = self.AUTOSTATE_ESCAPE_COMMUNITY

        elif self.autoState == self.AUTOSTATE_ESCAPE_COMMUNITY:
            xSpeed = -0.5
            ySpeed = 0
            rot = 0
            if self.autopos.X() <= -5.0:
                self.autoState = self.AUTO_STOPPING
        else: #including auto stopping
            xSpeed = 0
            ySpeed = 0
            rot = 0

        self.swerve.drive(xSpeed, ySpeed, rot, self.fieldRelative)


    def autoDocking(self):
        """Dock"""
        # Deleted for clarity.
        pass

    def autoScoreLow(self):
        """Score in the low part of the grid"""
        # Deleted for clarity.
        pass


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
        


        # EVERYTHING BELOW IS TAKEN FROM WIGGLETON ROBOT.PY


        # getting arm and wrist motor positions for dashboard
        motorPos = self.arm.getSelectedSensorPosition()

        wristPos = self.wrist.getSelectedSensorPosition()

        Arm_Angle_Deg = utilities.armCounts_to_degrees(motorPos)

        Wrist_Angle_Deg = utilities.wristCounts_to_degrees(wristPos)

 
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
            self.arm.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
            self.wrist.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
        else:
            if self.state == 1:
                self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.Man.midCube) #sets the arm motor to desired height
                self.wristDesiredPos = self.Man.WRIST_MIN
            elif self.state == 2:
                self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.Man.midCone)
                self.wristDesiredPos = self.Man.WRIST_MIN 
            elif self.state == 3:
                self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.Man.highestpoint)
                self.wristDesiredPos = self.Man.WRIST_MIN 
            elif self.state == 4:
                self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.Man.groundLevel)
                self.wristDesiredPos = self.Man.WRIST_MIN 
            elif self.state == 5:
                self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.Man.groundLevel)
                self.wristDesiredPos = self.Man.WRIST_MAX
            elif self.state == 6:
                self.wristDesiredPos = self.Man.WRIST_MAX
            elif self.state == 7:
                self.wristDesiredPos = self.Man.WRIST_MIN
            elif self.state == 8:
                self.arm.set(ctre._ctre.ControlMode.MotionMagic, self.Man.highCube)
                self.wristDesiredPos = self.Man.WRIST_MID
            else:
                self.state = 0

            #DO NOT set 63 in wrist convertion to Wrist_Min it ruins the positions 
            Wrist_Convertion = self.wristDesiredPos -(63 + Arm_Angle_Deg) #takes the desired position (in or out) and compensates for the angle ofthe arm
            self.wrist.set(ctre._ctre.ControlMode.MotionMagic, utilities.wristDegrees_to_counts(Wrist_Convertion))
            #each time the code moves through the "else" states, it moves the arm to the desired position within that state, then passes the value of the wrist's desired position 
            # to the function "self.wrist.set()" at the end.  Thus, unlike the arm being set in each state, the wrist's VALUE is set in each state, and that value
            #is used to set the wrist position each time the "else" runs.
            



        #CLAW CODE DEALING WITH STATES AND STATOR SPIKES

        #intake state machine that take actions from the controller and trigger the states
        stator_current = self.claw.getStatorCurrent()

        if AButton and self.intake_state == 0 and time.time() > self.intake_cooldown: #cube INTAKE
            self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.4)
            self.wristDesiredPos = self.Man.WRIST_MID
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
                
            # elif not AButton:
            #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
            #     self.wristDesiredPos = self.Man.WRIST_MAX
        if AButton:
            self.wrist.set(ctre._ctre.ControlMode.MotionMagic, self.Man.wristGroudLevel)
        else:
            self.wrist.set(ctre._ctre.ControlMode.MotionMagic, self.Man.wristInnerPos)

        
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
        

        

        if self.xboxD.getRightTriggerAxis() > 0.1:
            self.halfSpeed = True
        elif self.xboxD.getLeftTriggerAxis() > 0.1:
            self.halfSpeed = False

        if self.halfSpeed == True:
            joystick_y = utilities.joystickscaling(self.joystick_y / 2)
            xSpeed = self.xSpeedLimiter.calculate(joystick_y) * SwerveDrivetrain.getMaxSpeed()

            joystick_x = utilities.joystickscaling(self.joystick_x / 2)
            ySpeed = self.ySpeedLimiter.calculate(joystick_x) * SwerveDrivetrain.MAX_SPEED

            rot = utilities.joystickscaling(rot / 2)
            rot = self.rotLimiter.calculate(rot) * SwerveDrivetrain.MAX_ANGULAR_SPEED

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