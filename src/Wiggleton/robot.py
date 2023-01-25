# !/usr/bin/env python3

import wpilib
import ctre
import wpilib.drive

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


        self.frontLeft.setInverted(False) # 
        
        self.frontRight.setInverted(True) # 
        self.backLeft.setInverted(False) # 
        self.backRight.setInverted(True) # 


        self.frontLeft.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        
        self.frontRight.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        self.backLeft.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        self.backRight.setNeutralMode(ctre._ctre.NeutralMode.Coast)
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

        #get the output of the motors in percentage of joystick values
        self.frontLeft.set(ctre._ctre.ControlMode.PercentOutput, left_command)  #// delete b,l + b,r motor output and delete comments from the follow code in
        self.frontRight.set(ctre._ctre.ControlMode.PercentOutput, right_command) #// line 39-40 if follow command does not work, bring back motor percentage #// 
        self.backLeft.set(ctre._ctre.ControlMode.PercentOutput, left_command)  
        self.backRight.set(ctre._ctre.ControlMode.PercentOutput, right_command) #// also change values (the 0.2) to the joystick values

        # self.frontRight.set(ctre._ctre.ControlMode.PercentOutput, right_command) //you can delete this, but replace numbers with right_command

        '''# Drive cartesian allows the robot to drive in the cartesian cordinates by specifying the desired speed for each axis
        self.drivetrain.driveCartesian(move_y / 2, move_x  /2, move_z / 2)
        #self.drivetrain.driveCartesian(move_y / 4, move_x / 4, move_z / 4, -self.gyro.getAngle())
        '''
        wpilib.SmartDashboard.putString('DB/String 4',"Time: {:3.2f}".format(self.wiggleTimer.get()))



if __name__ == '__main__':
    wpilib.run(Myrobot)
