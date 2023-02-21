# !/usr/bin/env python3

#ERRORS? 1 CHECK THE FIRMWARE AND RUN THE MOTOR THROUGH THERE 2 CHECK IF YOU CAN SPIN THE MOTOR MANUALY (WITH HAND)
 
import wpilib
import ctre
import wpilib.drive

#import cv2

class Myrobot(wpilib.TimedRobot):

    kPIDLoopIdx = 0

    kTimeoutMs = 10

    kSlotIdx  = 0

    def robotInit(self):

        self.ARM_GEAR_RATIO = 51

        self.wiggleTimer = wpilib.Timer()

        # self.joystickR = wpilib.Joystick(0) #//Check the steps commented below for what to start putting back in for testing but this will be taken out
        # self.joystickL = wpilib.Joystick(1)
    
        self.xbox = wpilib.XboxController(1) #you can choose between using the xbox code or joystick but xbox is what well be sticking with


        # self.frontLeft = ctre.TalonFX(8)
        # self.frontRight = ctre.TalonFX(6)
        # self.backLeft = ctre.TalonFX(9) 
        # self.backRight = ctre.TalonFX(5)

        #//self.testmotor = ctre.TalonFX(7)
        self.armmotor = ctre.TalonFX(3)
        self.armmotor2 = ctre.TalonFX(2)

        # self.wristjoint = ctre.TalonFX(7)
        # self.claw = ctre.TalonFX(4)

        # self.frontLeft.setInverted(False) # 
        
        # self.frontRight.setInverted(True) # 
        # self.backLeft.setInverted(False) # 
        # self.backRight.setInverted(True) # 
        
        #//self.testmotor.setInverted(False)

        # self.frontLeft.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        
        # self.frontRight.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        # self.backLeft.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        # self.backRight.setNeutralMode(ctre._ctre.NeutralMode.Coast)

        #//self.testmotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)

        self.armmotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        #self.wristjoint.setNeutralMode(ctre._ctre.NeutralMode.Brake)

        #Gear ratio is 10.71 for Falcon motor and 2048 is the motors counts in one revolution
        #self.armMPos = 2048 * 10.71 * 0.1
        #Lower Position for arm motor
        #self.armLowerPos = 0

        #self.armUpperPos = (2048 * 80) * (115/360)

        self.targetPos = (2048 * 80) * (115/360)
        self.armUpperPos = 2048 * 51
        
        
        #this is the wrist PID loop that can controll the motor deceleration and acceleration bettween certain points and with handling weight and pull better
        '''
        self.testmotor.configNominalOutputForward(0, self.kTimeoutMs)
        self.testmotor.configNominalOutputReverse(0, self.kTimeoutMs)
        self.testmotor.configPeakOutputForward(1, self.kTimeoutMs)
        self.testmotor.configPeakOutputReverse(-1, self.kTimeoutMs)#MAKE SURE THE PEAK AND NOMINAL OUTPUTS ARE NOT BOTH FORWARD OR BOTH REVERSE

        
        self.testmotor.selectProfileSlot(self.kSlotIdx, self.kPIDLoopIdx)
        self.testmotor.config_kF(0, 0, self.kTimeoutMs)
        self.testmotor.config_kP(0, 0.3, self.kTimeoutMs)
        self.testmotor.config_kI(0, 0, self.kTimeoutMs)
        self.testmotor.config_kD(0, 0, self.kTimeoutMs)

        self.testmotor.configMotionCruiseVelocity(self.targetPos / 15, self.kTimeoutMs)
        self.testmotor.configMotionAcceleration(self.targetPos / 30, self.kTimeoutMs)

        self.testmotor.setSelectedSensorPosition(0, self.kPIDLoopIdx, self.kTimeoutMs)
        '''

        self.armmotor.configNominalOutputForward(0, self.kTimeoutMs)
        self.armmotor.configNominalOutputReverse(0, self.kTimeoutMs)
        self.armmotor.configPeakOutputForward(1, self.kTimeoutMs)
        self.armmotor.configPeakOutputReverse(-1, self.kTimeoutMs)#MAKE SURE THE PEAK AND NOMINAL OUTPUTS ARE NOT BOTH FORWARD OR BOTH REVERSE

        
        self.armmotor.selectProfileSlot(self.kSlotIdx, self.kPIDLoopIdx)
        self.armmotor.config_kF(0, 0, self.kTimeoutMs)
        self.armmotor.config_kP(0, 0.3, self.kTimeoutMs)
        self.armmotor.config_kI(0, 0, self.kTimeoutMs)
        self.armmotor.config_kD(0, 0, self.kTimeoutMs)

        self.armmotor.configMotionCruiseVelocity(self.armUpperPos / 15, self.kTimeoutMs)
        self.armmotor.configMotionAcceleration(self.armUpperPos / 30, self.kTimeoutMs)

        self.armmotor.setSelectedSensorPosition(0, self.kPIDLoopIdx, self.kTimeoutMs)


        

                                                  


        # self.wristjointLowerPos = 0

        # self.wristjointUpperPos = 2048 * 12  * 0.2

    
        #vision subsystem
        #wpilib.CameraServer.launch("vision.py:main")
        #https://roborio-3881-frc.local:1181
        #https://10.38.81.101:1181/
        
        #video_capture_device_index=0
        # Java Type is "edu.wpi.cscore.CvSink"
        #self.webcam = wpilib.CameraServer.getVideo()
        #print(f"Webcam is : {self.webcam}")  


    #    self.drivetrain = wpilib.drive.MecanumDrive(self.frontLeft, self.backLeft, self.frontRight, self.backLeft)
    
    #// take out comments for setting motors to coast + for mecanum drive function


        # #makes the back motors follow the front motor movement
        # self.backLeft.follow(self.frontLeft)     
        #self.backRight.follow(self.frontRight)
        #self.backRight.setInverted(ctre._ctre.InvertType.FollowMaster)

        self.armmotor2.follow(self.armmotor)
        self.armmotor2.setInverted(ctre._ctre.InvertType.OpposeMaster)


    def disabledInit(self): #fixed type 1/24/2022
        self.wiggleTimer.start()

        #wpilib.SmartDashboard.putString('DB/String 0',"system test")
        #wpilib.SmartDashboard.putString('DB/String 1',"system test")
        #wpilib.SmartDashboard.putString('DB/String 2',"system test")
        #wpilib.SmartDashboard.putString('DB/String 3',"system test")
        wpilib.SmartDashboard.putString('DB/String 4',"Time: {:3.2f}".format(self.wiggleTimer.get()))

        #wpilib.SmartDashboard.putNumber('DB/number 5',113)
        #wpilib.SmartDashboard.putNumber('DB/number 6',113)

    def disabledPeriodic(self):
        pass

    def autonomousInit(self):
        #if self.wiggleTimer.hasElapsed(1)
        #if self.wiggleTimer.hasPeriodPassed(1)
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        self.wiggleTimer.reset()
        self.wiggleTimer.start()

        self.armmotor.setSelectedSensorPosition(0.0)
        # self.wristjoint.setSelectedSensorPosition(0.0)

        self.state = 0
         # self.wstate = 0
       
        

    def teleopPeriodic(self):

       

        #rTrigger = self.xbox.getRightTriggerAxis()
        #AButton = self.xbox.getAButton()

        motorPos = self.armmotor.getSelectedSensorPosition()

        motorDegrees = self.armCounts_to_degrees(motorPos)


        
        wpilib.SmartDashboard.putString('DB/String 7',"Pos_Degrees: {:4.2f}".format(motorDegrees))
        wpilib.SmartDashboard.putString('DB/String 4',"Time: {:3.2f}".format(self.wiggleTimer.get()))

       


        #//rTrigger = self.xbox.getRightTriggerAxis()
        #AButton = self.xbox.getAButton()
        #BButton = self.xbox.getBButton()
        lBumper = self.xbox.getLeftBumper()
        rBumper = self.xbox.getRightBumper()
        lTrigger = self.xbox.getLeftTriggerAxis()


        #wpilib.SmartDashboard.putString('DB/String 8',"Position: {:4.2f}".format(rTrigger))
        
        '''
        if AButton:
            self.testmotor.set(ctre._ctre.ControlMode.MotionMagic, self.targetPos)
        elif BButton:
            self.testmotor.set(ctre._ctre.ControlMode.MotionMagic, 0)
        else:
            self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, rTrigger)
        '''


        if lBumper:
            self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.armUpperPos)
        elif rBumper:
            self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, 0)
        else:
            self.armmotor.set(ctre._ctre.ControlMode.PercentOutput, lTrigger)

        wpilib.SmartDashboard.putString('DB/String 6',"Trigger: {:4.2f}".format(lTrigger))

        self.armmotor.set(ctre._ctre.ControlMode.PercentOutput, lTrigger)



        '''
        if AButton:
                self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, self.targetPos)
            elif self.xbox.getBButton():
                self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, self.targetPos / 2)
            else:
                self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, rTrigger) 
        '''
        
            
  







        
        

        '''  #joystick code
        move_y = self.joystick.getY()   
        move_x = self.joystick.getX()
        move_z = self.joystick.getZ()
        '''

        #xbox code
        #move_y = -self.xbox.getRightY()
        #move_x = self.xbox.getRightX()
        #move_z = self.xbox.getLeftX()



        #Get joystick values 
        # left_command = self.joystickL.getY()
        #right_command = self.joystickR.getY()

        # #Get xbox values
        # right_command = -self.xbox.getRightY()
        # left_command = -self.xbox.getLeftY()

        # #arm motor
        # #value of the arm = getthebutton
        # #arm_command1 x= self.xbox.getBButton()
        # #value of the arm = getanothterbutton
        # #arm_command2 = -self.xbox.getAButton()

        # #get the output of the motors in percentage of joystick values
        # self.frontLeft.set(ctre._ctre.ControlMode.PercentOutput, left_command)  #// delete b,l + b,r motor output and delete comments from the follow code in
        # self.frontRight.set(ctre._ctre.ControlMode.PercentOutput, right_command) #// line 39-40 if follow command does not work, bring back motor percentage #// 
        # self.backLeft.set(ctre._ctre.ControlMode.PercentOutput, left_command)  
        # self.backRight.set(ctre._ctre.ControlMode.PercentOutput, right_command) #// also change values (the 0.2) to the joystick values
        # self.frontRight.set(ctre._ctre.ControlMode.PercentOutput, right_command) #//you can delete this, but replace numbers with right_command

        #get the output of the motors for button values
        # if self.xbox.getBButton():
        #     self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, -0.1)
        # elif self.xbox.getAButton():
        #     self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.1)
        # else:
        #     self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
        


        '''# Drive cartesian allows the robot to drive in the cartesian cordinates by specifying the desired speed for each axis
        self.drivetrain.driveCartesian(move_y / 2, move_x  /2, move_z / 2)
        #self.drivetrain.driveCartesian(move_y / 4, move_x / 4, move_z / 4, -self.gyro.getAngle())
        '''
        #wpilib.SmartDashboard.putString('DB/String 4',"Time: {:3.2f}".format(self.wiggleTimer.get()))
        #position = self.testmotor.getSelectedSensorPosition()
        # wposition = self.wristjoint.getSelectedSensorPosition()
        #wpilib.SmartDashboard.putString('DB/String 7',"Position: {:4.2f}".format(position / self.armUpperPos))
        
        # this is a block saying if B button on the xbox is pressed then it will move positively by .1 or else it wont move
        # if self.testmotor.getSelectedSensorPosition() < self.armUpperPos:
        #     if self.xbox.getBButton():
        #         self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.1)
        #     else: 
        #         self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
        # else:
        #     self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.0)


        #if the left bumper is pressed then the motor will move up and stay at the set position until the right bumper is pressed and the motor will move back to 0, its lowest position
        #if the left bumper is pressed and the motor position is lower than the "2048 * 10.71" position (you wont be able to press right bumper as the value is lower than its "2048 * 10.71"" position) the motor will move up to its "2048 *10.71:
        #if the right bumper is pressed and the motor position is above the "0" position (you wont be able to press the right bumper as the value is its "0" position) the motor will move down to its "0"
        #if the motor is already at its "0 position" then it wont move at all

        
        # henry
        # if self.xbox.getAButton()
        #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.1)
        # elif self.xbox.getBButton()
        #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, -0.1) 
        # else:
        #     self.claw.set(ctre._ctre.ControlMode.PercentOutput, 0.0)

        '''
        if self.state == 0:
            if self.xbox.getRightBumper():
                if position < self.armMPos: 
                    self.state = 1
                elif position >= self.armMPos and position < self.armUpperPos: 
                    self.state = 3
            elif self.xbox.getLeftBumper():
                if position > self.armMPos: 
                       self.state = 2
                elif position <= self.armMPos and position > self.armLowerPos: 
                    self.state = 4
#            elif self.xbox.getAButton() and position <= self.armMPos: #for now we are using buttons A and B but plan to change it later
#                self.state = 1
                #if 
#            elif self.xbox.getBButton() and position >= self.armMPos:
#                self.state = 2
        '''
        #jay, zack, james
        '''
        elif self.state == 1:
            if self.testmotor.getSelectedSensorPosition() >= self.armMPos: 
                self.state = 0
        elif self.state == 3: 
            if self.testmotor.getSelectedSensorPosition() >= self.armUpperPos:
               self.state = 0
        elif self.state == 2: 
            if self.testmotor.getSelectedSensorPosition() <= self.armMPos:
                self.state = 0
        elif self.state == 4:
            if self.testmotor.getSelectedSensorPosition() <= self.armLowerPos:
                self.state = 0
 


        if self.state == 0:
            self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
        elif self.state == 1:
            self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.25 )
        elif self.state == 2: 
            self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, -0.25 )
        elif self.state == 3:
            self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.25 )
        elif self.state == 4: 
            self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, -0.25 )
        '''
        
        # #tam

        # if self.wstate == 0:
        #     if self.xbox.getAButton() and wposition <= self.wristjointUpperPos: 
        #         self.wstate = 1
        #     elif self.xbox.getBButton() and wposition >= self.wristjointLowerPos:
        #         self.wstate = 2 
        # elif self.wstate == 1:
        #     if self.wristjoint.getSelectedSensorPosition() >= self.wristjointUpperPos:
        #         self.wstate = 0
        # else: 
        #     if self.wristjoint.getSelectedSensorPosition() <= self.wristjointLowerPos:
        #         self.wstate = 0

        # if self.wstate == 0:
        #     self.wristjoint.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
        # elif self.wstate == 1:
        #     self.wristjoint.set(ctre._ctre.ControlMode.PercentOutput, 0.1)
        # elif self.wstate == 2:
        #     self.wristjoint.set(ctre._ctre.ControlMode.PercentOutput, -0.1)
        # else:
        #     self.wristjoint.set(ctre._ctre.ControlMode.PercentOutput, 0.0)

    

    #def teleopExit(self):
        #self.webcam.release()
       
    def armCounts_to_degrees(self, counts):

        degrees = (counts * (360/2048)) / self.ARM_GEAR_RATIO
        return degrees

    
    def armDegrees_to_counts(self, degrees):

        counts = (degrees * (2048/360)) * self.ARM_GEAR_RATIO
        return counts

           
            

        

if __name__ == '__main__':
    wpilib.run(Myrobot)
