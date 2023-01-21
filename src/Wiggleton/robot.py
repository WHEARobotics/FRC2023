# !/usr/bin/env python3

import wpilib
import ctre
import wpilib.drive

class Myrobot(wpilib.TimedRobot):

    def robotInit(self):

        self.wiggleTimer = wpilib.Timer()

        # self.joystickR = wpilib.Joystick(0)
        # self.joystickL = wpilib.Joystick(1)

        self.frontLeft = ctre.TalonFX(9)
        
        self.frontRight = ctre.TalonFX(6)
        self.backLeft = ctre.TalonFX(8) 
        self.backRight = ctre.TalonFX(5)


        self.frontLeft.setInverted(False) # 
        
        self.frontRight.setInverted(True) # 
        self.backLeft.setInverted(False) # 
        self.backRight.setInverted(True) # 


        self.frontLeft.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        
        # self.frontRight.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        # self.backLeft.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        # self.backRight.setNeutralMode(ctre._ctre.NeutralMode.Coast)
    #    self.drivetrain = wpilib.drive.MecanumDrive(self.frontLeft, self.backLeft, self.frontRight, self.backLeft)


        # #makes the back motors follow the front motor movement
        # self.backLeft.follow(self.frontLeft)
        # self.backRight.follow(self.frontRight)


    def diabledInit(self):
        self.wiggleTimer.start()

        wpilib.SmartDashboard.putString('DB/String 0',"system test")
        wpilib.SmartDashboard.putString('DB/String 1',"system test")
        wpilib.SmartDashboard.putString('DB/String 2',"system test")
        wpilib.SmartDashboard.putString('DB/String 3',"system test")
        wpilib.SmartDashboard.putString('DB/String 4',"Time: {:3.2f}".format(self.wiggleTimer.get()))

        wpilib.SmartDashboard.putNumber('DB/string 5',113)
        wpilib.SmartDashboard.putString('DB/string 6',113)

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
        '''
        move_y = self.joystick.getY()
        move_x = self.joystick.getX()
        move_z = self.joystick.getZ()
        '''

        #Get joystick values 
        # left_command = self.joystickL.getY()
        # right_command = self.joystickR.getY()

        #get the output of the motors in percentage of joystick values
        self.frontLeft.set(ctre._ctre.ControlMode.PercentOutput, 0.2)
        self.frontRight.set(ctre._ctre.ControlMode.PercentOutput, 0.2)
        self.backLeft.set(ctre._ctre.ControlMode.PercentOutput, 0.2)
        self.backRight.set(ctre._ctre.ControlMode.PercentOutput, 0.2)

        # self.frontRight.set(ctre._ctre.ControlMode.PercentOutput, right_command)

        '''
        self.drivetrain.driveCartesian(move_y / 2, move_x  /2, move_z / 2)
        #self.drivetrain.driveCartesian(move_y / 4, move_x / 4, move_z / 4, -self.gyro.getAngle())
        '''
        wpilib.SmartDashboard.putString('DB/String 4',"Time: {:3.2f}".format(self.wiggleTimer.get()))


if __name__ == '__main__':
    wpilib.run(Myrobot)