# !/usr/bin/env python3

import wpilib
import wpilib.drive
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
import ctre

#import cv2

from SwerveModule import SwerveModule
from SwerveDrivetrain import SwerveDrivetrain


class Myrobot(wpilib.TimedRobot):

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

    def robotInit(self):

        self.halfSpeed = False

        self.swerve = SwerveDrivetrain()

        self.xSpeedLimiter = SlewRateLimiter(3)
        self.ySpeedLimiter = SlewRateLimiter(3)
        self.rotLimiter = SlewRateLimiter(3)

        """
        these inits are done in SwerveDrivetrain, which is called above, these inits are only used to print values to smartDashboard
        """
        # self.moduleFR = SwerveModule(1, 10, 4, 106.424)
        # self.moduleFL = SwerveModule(5, 4, 1, 296.543)
        # self.moduleBR = SwerveModule(8, 9, 3, 32.959)
        # self.moduleBL = SwerveModule(6, 7, 2, 206.455)


        self.wiggleTimer = wpilib.Timer()
    
        self.xbox = wpilib.XboxController(1)


        #vision subsystem
        #wpilib.CameraServer.launch("vision.py:main")
        #https://roborio-3881-frc.local:1181
        #https://10.38.81.101:1181/
        
        #video_capture_device_index=0
        # Java Type is "edu.wpi.cscore.CvSink"
        #self.webcam = wpilib.CameraServer.getVideo()
        #print(f"Webcam is : {self.webcam}")  


    def disabledInit(self): #fixed type 1/24/2022
        
        self.wiggleTimer.reset()
        self.wiggleTimer.start()
        #self.wiggleTimer.restart()

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

        wpilib.SmartDashboard.putString('DB/String 0',"FR: {:4.1f}  {:4.1f}".format(moduleFRState.angle.degrees() % 360, modFRCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 1',"FL: {:4.1f}  {:4.1f}".format(moduleFLState.angle.degrees() % 360, modFLCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 2',"BR: {:4.1f}  {:4.1f}".format(moduleBRState.angle.degrees() % 360, modBRCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 3',"BL: {:4.1f}  {:4.1f}".format(moduleBLState.angle.degrees() % 360, modBLCAN % 360))

        wpilib.SmartDashboard.putString('DB/String 5',"FR: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[1].angle.degrees() % 360, moduleFRState.speed))
        wpilib.SmartDashboard.putString('DB/String 6',"FL: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[0].angle.degrees() % 360, moduleFLState.speed))
        wpilib.SmartDashboard.putString('DB/String 7',"BR: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[3].angle.degrees() % 360, moduleBRState.speed))
        wpilib.SmartDashboard.putString('DB/String 8',"BL: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[2].angle.degrees() % 360, moduleBLState.speed))

        wpilib.SmartDashboard.putString('DB/String 9',"robot angle: {:4.2f}".format(self.swerve.get_heading().degrees() % 360))




        

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


    def autonomousPeriodic(self):
        
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
        #self.wiggleTimer.restart()
        self.swerve.gyro.reset()
        

    def teleopPeriodic(self):

        #these two lines are lifted from SwerveRobot.py, as is the driveWithJoystick function
        self.driveWithJoystick(True)
        self.swerve.periodic()


        moduleFRState = self.swerve.frontRight.getState()
        moduleFLState = self.swerve.frontLeft.getState()
        moduleBRState = self.swerve.backRight.getState()
        moduleBLState = self.swerve.backLeft.getState()
        modFRCAN = self.swerve.frontRight.absEnc.getAbsolutePosition()
        modFLCAN = self.swerve.frontLeft.absEnc.getAbsolutePosition()
        modBRCAN = self.swerve.backRight.absEnc.getAbsolutePosition()
        modBLCAN = self.swerve.backLeft.absEnc.getAbsolutePosition()

        # wpilib.SmartDashboard.putString('DB/String 0',"speed MPS FR: {:4.2f}".format(moduleFRState.speed))
        # wpilib.SmartDashboard.putString('DB/String 1',"speed MPS FL: {:4.2f}".format(moduleFLState.speed))
        # wpilib.SmartDashboard.putString('DB/String 2',"speed MPS BR: {:4.2f}".format(moduleBRState.speed))
        # wpilib.SmartDashboard.putString('DB/String 3',"speed MPS BL: {:4.2f}".format(moduleBLState.speed))

        wpilib.SmartDashboard.putString('DB/String 0',"FR: {:4.1f}  {:4.1f}".format(moduleFRState.angle.degrees() % 360, modFRCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 1',"FL: {:4.1f}  {:4.1f}".format(moduleFLState.angle.degrees() % 360, modFLCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 2',"BR: {:4.1f}  {:4.1f}".format(moduleBRState.angle.degrees() % 360, modBRCAN % 360))
        wpilib.SmartDashboard.putString('DB/String 3',"BL: {:4.1f}  {:4.1f}".format(moduleBLState.angle.degrees() % 360, modBLCAN % 360))

        wpilib.SmartDashboard.putString('DB/String 5',"FR: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[1].angle.degrees() % 360, moduleFRState.speed))
        wpilib.SmartDashboard.putString('DB/String 6',"FL: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[0].angle.degrees() % 360, moduleFLState.speed))
        wpilib.SmartDashboard.putString('DB/String 7',"BR: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[3].angle.degrees() % 360, moduleBRState.speed))
        wpilib.SmartDashboard.putString('DB/String 8',"BL: {:4.1f}  {:4.1f}".format(self.swerve.swerveModuleStates[2].angle.degrees() % 360, moduleBLState.speed))

        wpilib.SmartDashboard.putString('DB/String 9',"robot angle: {:4.2f}".format(self.swerve.get_heading().degrees() % 360))
        # wpilib.SmartDashboard.putString('DB/String 4',"Mod4TurnEnc: {:4.2f}".format(self.mod4turnEnc % 360))


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
        

        if self.xbox.getRawButton(10):
            self.halfSpeed = True
        elif self.xbox.getRawButton(9):
            self.halfSpeed = False
        
        if self.halfSpeed == True:
            joystick_y = -self.xbox.getLeftY() / 2
            joystick_y = applyDeadband(joystick_y, 0.02)
            xSpeed = self.xSpeedLimiter.calculate(joystick_y) * SwerveDrivetrain.getMaxSpeed()

            joystick_x = -self.xbox.getLeftX() / 2
            joystick_x = applyDeadband(joystick_x, 0.02)
            ySpeed = self.ySpeedLimiter.calculate(joystick_x) * SwerveDrivetrain.MAX_SPEED
        
        else:

            # Get the x speed. We are inverting this because Xbox controllers return
            # negative values when we push forward.
            joystick_y = -self.xbox.getLeftY()
            joystick_y = applyDeadband(joystick_y, 0.02)
            xSpeed = self.xSpeedLimiter.calculate(joystick_y) * SwerveDrivetrain.getMaxSpeed()

            # Get the y speed. We are inverting this because Xbox controllers return
            # negative values when we push to the left.
            joystick_x = -self.xbox.getLeftX()
            joystick_x = applyDeadband(joystick_x, 0.02)
            ySpeed = self.ySpeedLimiter.calculate(joystick_x) * SwerveDrivetrain.MAX_SPEED



            
        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        
        rot = -self.xbox.getRightX()
        rot = applyDeadband(rot, 0.02)
        rot = self.rotLimiter.calculate(rot) * SwerveDrivetrain.MAX_ANGULAR_SPEED
        

        # below is code writtenby Rod, 
        # rot = 0.0 # Never rotate.
        # if self.xbox.getYButton():
        #     xSpeed = 1.0 # m/sec
        #     ySpeed = 1.0
        # elif self.xbox.getAButton():
        #     xSpeed = -1.0 # m/sec
        #     ySpeed = 0.0
        # elif self.xbox.getXButton():
        #     xSpeed = 0.0 # m/sec
        #     ySpeed = 1.0
        # elif self.xbox.getBButton():
        #     xSpeed = -1.0 # m/sec
        #     ySpeed = 1.0
        # else:
        #     xSpeed = 0.0 # No button pressed, stop.
        #     ySpeed = 0.0

        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative)

    def degrees_to_counts(self, degrees):
        counts = degrees * (2048/360) * self.TURNING_GEAR_RATIO
        return counts
    
    def counts_to_degrees(self, counts):
        degrees = counts * (360/2048) / self.TURNING_GEAR_RATIO
        return degrees

if __name__ == '__main__':
    wpilib.run(Myrobot)
