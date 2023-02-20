# !/usr/bin/env python3

import wpilib
import ctre
import wpilib.drive
#import cv2
from SwerveModule import SwerveModule
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.geometry import Rotation2d, Translation2d




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

    #turn angle invert: yes
    #drive invert: no

    kSlotIdx = 0        #Falcons are awesome- we can have multiple PID loops stored for various 
                        #...applications (this year we're good though).  So since we're only using one PID loop this year,
                        #...we just need one slot (kind of like xbox(0)) when we only use one xbox controller 

    kPIDLoopIdx = 0     #Nope - don't need this.

    #kTimeoutMs = 10    #deleted this as a parameter since the default is 10ms.

    def robotInit(self):

        self.moduleFR = SwerveModule(1, 10, 4, 106.424)
        self.moduleFL = SwerveModule(5, 4, 1, 296.543)
        self.moduleBR = SwerveModule(8, 9, 3, 32.959)
        self.moduleBL = SwerveModule(6, 7, 2, 206.455)







        self.wiggleTimer = wpilib.Timer()

        # self.joystickR = wpilib.Joystick(0) #//Check the steps commented below for what to start putting back in for testing but this will be taken out
        # self.joystickL = wpilib.Joystick(1)
    
        self.xbox = wpilib.XboxController(1) #you can choose between using the xbox code or joystick but xbox is what well be sticking with


        # self.testmotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        #self.wristjoint.setNeutralMode(ctre._ctre.NeutralMode.Brake)

        # #Gear ratio is 10.71 for Falcon motor and 2048 is the motors counts in one revolution
        # self.armMPos = 2048 * 10.71 * 0.1
        # #Lower Position for arm motor
        # self.armLowerPos = 0

        # self.armUpperPos = 2048 * 10.71 * 0.2
        
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


    def disabledInit(self): #fixed type 1/24/2022
        
        #self.wiggleTimer.start()
        pass


    def disabledPeriodic(self):

        moduleFRState = self.moduleFR.getState()
        moduleFLState = self.moduleFL.getState()
        moduleBRState = self.moduleBR.getState()
        moduleBLState = self.moduleBL.getState()

        wpilib.SmartDashboard.putString('DB/String 0',"speed MPS FR: {:4.2f}".format(moduleFRState.speed))
        wpilib.SmartDashboard.putString('DB/String 1',"speed MPS FL: {:4.2f}".format(moduleFLState.speed))
        wpilib.SmartDashboard.putString('DB/String 2',"speed MPS BR: {:4.2f}".format(moduleBRState.speed))
        wpilib.SmartDashboard.putString('DB/String 3',"speed MPS BL: {:4.2f}".format(moduleBLState.speed))
        wpilib.SmartDashboard.putString('DB/String 5',"Degrees FR: {:4.2f}".format(moduleFRState.angle.degrees() % 360))
        wpilib.SmartDashboard.putString('DB/String 6',"Degrees FL: {:4.2f}".format(moduleFLState.angle.degrees() % 360))
        wpilib.SmartDashboard.putString('DB/String 7',"Degrees BR: {:4.2f}".format(moduleBRState.angle.degrees() % 360))
        wpilib.SmartDashboard.putString('DB/String 8',"Degrees BL: {:4.2f}".format(moduleBLState.angle.degrees() % 360))

    def disabledExit(self):
        
        pass

    def autonomousInit(self):

        pass

    def autonomousPeriodic(self):
        
        pass

    def autonomousExit(self):
        
        pass

    def teleopInit(self):
        
        #self.wiggleTimer.reset()
        #self.wiggleTimer.start()

        #self.testmotor.setSelectedSensorPosition(0.0)
        #self.wristjoint.setSelectedSensorPosition(0.0)

        #self.state = 0
        #self.wstate = 0
        pass
        

    def teleopPeriodic(self):

        #xbox code
        #move_y = -self.xbox.getRightY()
        #move_x = self.xbox.getRightX()
        #move_z = self.xbox.getLeftX()

        # #Get xbox values
        # right_command = -self.xbox.getRightY()
        # left_command = -self.xbox.getLeftY()

        # #arm motor
        # #value of the arm = getthebutton
        # #arm_command1 x=                        self.xbox.getBButton()
        # #value of the arm = getanothterbutton
        # #arm_command2 = -self.xbox.getAButton()

        #get the output of the motors for button values
        # if self.xbox.getBButton():
        #     self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, -0.1)
        # elif self.xbox.getAButton():
        #     self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.1)
        # else:
        #     self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.0)

        moduleFRState = self.moduleFR.getState()
        moduleFLState = self.moduleFL.getState()
        moduleBRState = self.moduleBR.getState()
        moduleBLState = self.moduleBL.getState()

        wpilib.SmartDashboard.putString('DB/String 0',"speed MPS FR: {:4.2f}".format(moduleFRState.speed))
        wpilib.SmartDashboard.putString('DB/String 1',"speed MPS FL: {:4.2f}".format(moduleFLState.speed))
        wpilib.SmartDashboard.putString('DB/String 2',"speed MPS BR: {:4.2f}".format(moduleBRState.speed))
        wpilib.SmartDashboard.putString('DB/String 3',"speed MPS BL: {:4.2f}".format(moduleBLState.speed))
        wpilib.SmartDashboard.putString('DB/String 5',"Degrees FR: {:4.2f}".format(moduleFRState.angle.degrees() % 360))
        wpilib.SmartDashboard.putString('DB/String 6',"Degrees FL: {:4.2f}".format(moduleFLState.angle.degrees() % 360))
        wpilib.SmartDashboard.putString('DB/String 7',"Degrees BR: {:4.2f}".format(moduleBRState.angle.degrees() % 360))
        wpilib.SmartDashboard.putString('DB/String 8',"Degrees BL: {:4.2f}".format(moduleBLState.angle.degrees() % 360))

        if self.xbox.getAButton():
            targetPos = 45                             #absolute encoder measures positive degrees as counterclockwise (when looking down at the module), so +90 degrees would be to the left.
            targetVel = 3
        elif self.xbox.getYButton():
            targetPos = 225
            targetVel = 2
        else:
            targetPos = 0
            targetVel = 0

        X = SwerveModuleState(targetVel, Rotation2d.fromDegrees(targetPos))                 #this section of code references swerve module in the SWERVE folder
        self.moduleFR.setDesiredState(X, True)
        self.moduleFL.setDesiredState(X, True)
        self.moduleBR.setDesiredState(X, True)
        self.moduleBL.setDesiredState(X, True)


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

        
#         if self.state == 0:
#             if self.xbox.getRightBumper():
#                 if position < self.armMPos: 
#                     self.state = 1
#                 elif position >= self.armMPos and position < self.armUpperPos: 
#                     self.state = 3
#             elif self.xbox.getLeftBumper():
#                 if position > self.armMPos: 
#                        self.state = 2
#                 elif position <= self.armMPos and position > self.armLowerPos: 
#                     self.state = 4
# #            elif self.xbox.getAButton() and position <= self.armMPos: #for now we are using buttons A and B but plan to change it later
# #                self.state = 1
#                 #if 
# #            elif self.xbox.getBButton() and position >= self.armMPos:
# #                self.state = 2

#         #jay, zack, james

#         elif self.state == 1:
#             if self.testmotor.getSelectedSensorPosition() >= self.armMPos: 
#                 self.state = 0
#         elif self.state == 3: 
#             if self.testmotor.getSelectedSensorPosition() >= self.armUpperPos:
#                self.state = 0
#         elif self.state == 2: 
#             if self.testmotor.getSelectedSensorPosition() <= self.armMPos:
#                 self.state = 0
#         elif self.state == 4:
#             if self.testmotor.getSelectedSensorPosition() <= self.armLowerPos:
#                 self.state = 0
 


#         if self.state == 0:
#             self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
#         elif self.state == 1:
#             self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.25 )
#         elif self.state == 2: 
#             self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, -0.25 )
#         elif self.state == 3:
#             self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.25 )
#         elif self.state == 4: 
#             self.testmotor.set(ctre._ctre.ControlMode.PercentOutput, -0.25 )
        
        
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


    def teleopExit(self):
        #self.webcam.release()
        pass

    def degrees_to_counts(self, degrees):
        counts = degrees * (2048/360) * self.TURNING_GEAR_RATIO
        return counts
    
    def counts_to_degrees(self, counts):
        degrees = counts * (360/2048) / self.TURNING_GEAR_RATIO
        return degrees

if __name__ == '__main__':
    wpilib.run(Myrobot)
