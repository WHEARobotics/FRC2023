# !/usr/bin/env python3

import wpilib
import ctre
import wpilib.drive
#import cv2

class Myrobot(wpilib.TimedRobot):

    def robotInit(self):

        self.wiggleTimer = wpilib.Timer()

        # self.joystickR = wpilib.Joystick(0) #//Check the steps commented below for what to start putting back in for testing but this will be taken out
        # self.joystickL = wpilib.Joystick(1)
    
        self.xbox = wpilib.XboxController(1) #you can choose between using the xbox code or joystick but xbox is what well be sticking with


        self.frontLeft = ctre.TalonFX(8)
        self.frontRight = ctre.TalonFX(6)
        self.backLeft = ctre.TalonFX(9) 
        self.backRight = ctre.TalonFX(5)

        self.testmotor = ctre.TalonFX(4)

        self.armjoint = ctre.TalonFX(7)
        self.claw = ctre.TalonFX(4)

        self.frontLeft.setInverted(False) # 
        
        self.frontRight.setInverted(True) # 
        self.backLeft.setInverted(False) # 
        self.backRight.setInverted(True) # 


        self.frontLeft.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        
        self.frontRight.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        self.backLeft.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        self.backRight.setNeutralMode(ctre._ctre.NeutralMode.Coast)

        self.testmotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)

        #Gear ratio is 10.71 for Falcon motor and 2048 is the motors counts in one revolution
        self.armUpperPos = 2048 * 10.71
        #Lower Position for arm motor
        self.armLowerPos = 0

    
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
        # self.backLeft.follow(self.frontLeft)      #// take out comments for follow command and test to see if it works if not keep back motors on 86-87
        #self.backRight.follow(self.frontRight)
        #self.backRight.setInverted(ctre._ctre.InvertType.FollowMaster)

    def disabledInit(self): #fixed type 1/24/2022
        self.wiggleTimer.start()

        wpilib.SmartDashboard.putString('DB/String 0',"system test")
        wpilib.SmartDashboard.putString('DB/String 1',"system test")
        wpilib.SmartDashboard.putString('DB/String 2',"system test")
        wpilib.SmartDashboard.putString('DB/String 3',"system test")
        wpilib.SmartDashboard.putString('DB/String 4',"Time: {:3.2f}".format(self.wiggleTimer.get()))

        wpilib.SmartDashboard.putNumber('DB/number 5',113)
        wpilib.SmartDashboard.putNumber('DB/number 6',113)

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

        self.testmotor.setSelectedSensorPosition(0.0)

        self.state = 0

        

    def teleopPeriodic(self):
        
        

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

        #Get xbox values
        right_command = -self.xbox.getRightY()
        left_command = -self.xbox.getLeftY()

        #arm motor
        #value of the arm = getthebutton
        #arm_command1 x= self.xbox.getBButton()
        #value of the arm = getanothterbutton
        #arm_command2 = -self.xbox.getAButton()

        #get the output of the motors in percentage of joystick values
        self.frontLeft.set(ctre._ctre.ControlMode.PercentOutput, left_command)  #// delete b,l + b,r motor output and delete comments from the follow code in
        self.frontRight.set(ctre._ctre.ControlMode.PercentOutput, right_command) #// line 39-40 if follow command does not work, bring back motor percentage #// 
        self.backLeft.set(ctre._ctre.ControlMode.PercentOutput, left_command)  
        self.backRight.set(ctre._ctre.ControlMode.PercentOutput, right_command) #// also change values (the 0.2) to the joystick values
        self.frontRight.set(ctre._ctre.ControlMode.PercentOutput, right_command) #//you can delete this, but replace numbers with right_command

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
        wpilib.SmartDashboard.putString('DB/String 4',"Time: {:3.2f}".format(self.wiggleTimer.get()))
        position = self.testmotor.getSelectedSensorPosition()
        wpilib.SmartDashboard.putString('DB/String 7',"Position: {:4.2f}".format(position))
        
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
        
        if self.state == 0:
            if self.xbox.getLeftBumper() and position <= self.armUpperPos: 
                self.state = 1
            elif self.xbox.getRightBumper() and position >= self.armLowerPos:
                self.state = 2 
        elif self.state == 1:
            if self.testmotor.getSelectedSensorPosition() >= self.armUpperPos:
                self.state = 0
        else: 
            if self.testmotor.getSelectedSensorPosition() <= self.armLowerPos:
                self.state = 0

        if self.state == 0:
            self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
        elif self.state == 1:
            self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.1)
        else: 
            self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, -0.1)
        
    
    #def teleopExit(self):
        #self.webcam.release()



if __name__ == '__main__':
    wpilib.run(Myrobot)
