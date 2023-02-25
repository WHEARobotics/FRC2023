# !/usr/bin/env python3

#ERRORS? 1 CHECK THE FIRMWARE AND RUN THE MOTOR THROUGH THERE 2 CHECK IF YOU CAN SPIN THE MOTOR MANUALY (WITH HAND)
 
import wpilib
import ctre
import wpilib.drive
import math

#import cv2

class Myrobot(wpilib.TimedRobot):

    kPIDLoopIdx = 0

    kTimeoutMs = 10

    kSlotIdx  = 0

    
    def robotInit(self):

        #ground level degrees: -63
        #mid cube deg: 7
        #mid cone: 13
        #high cube: 17
        #feederstation: 17

        self.ARM_GEAR_RATIO = 35.7 * 4      #to get the gear ratio, we had 48 teeth from the sprocket and divided it from the output sahft which is 12, then we multiplied that from the gear ratio which is 12.75
        
        self.ARM_MIN = -63.0

        self.ARM_HORIZONTAL = 0.0

        self.ARM_MAX = 26.0 #might change to 13.0

        self.PEAK_ARM_FF = 0.07 #this is the amount of output we need in the motor to hold itself when it is in its desired position

        self.WRIST_GEAR_RATIO = 80 * 26 /11

        self.WRIST_MAX = 115.0

        self.WRIST_MIN = 0.0

        self.state = 0 #initializing the state for the arm

        self.wriststate = 0

        self.wiggleTimer = wpilib.Timer()
    
        self.xbox = wpilib.XboxController(1) #you can choose between using the xbox code or joystick but xbox is what well be sticking with


       
        self.wristmotor = ctre.TalonFX(11)
        self.armmotor = ctre.TalonFX(3)
        self.armmotor2 = ctre.TalonFX(2)

        # self.claw = ctre.TalonFX(4)

        
        self.wristmotor.setInverted(False)

    
        self.wristmotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)

        self.armmotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.armmotor2.setNeutralMode(ctre._ctre.NeutralMode.Brake)

        #self.wristjoint.setNeutralMode(ctre._ctre.NeutralMode.Brake)

    
        self.targetPos = (2048 * 80) * (115/360)#target position for the wrist


        #these are all the positions that will be used for the arm

        self.groundLevel = self.armDegrees_to_counts(-63) 

        self.feederStation = self.armDegrees_to_counts(17) 

        self.midCube = self.armDegrees_to_counts(7) 

        self.midCone = self.armDegrees_to_counts(13)

        self.highCube = self.armDegrees_to_counts(17)
        
         
        #35.7 then multiply by 4 for sprockets

        #positons for wrist
        
        self.wristDown = self.wristDegrees_to_counts(0)

        self.wristUp = self.wristDegrees_to_counts(115)


        
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

        self.wristmotor.configMotionCruiseVelocity(wrist_Range_Counts / 15, self.kTimeoutMs)
        self.wristmotor.configMotionAcceleration(wrist_Range_Counts / 30, self.kTimeoutMs)
        self.wristmotor.setSelectedSensorPosition(0, self.kPIDLoopIdx, self.kTimeoutMs)
        

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



    
        
        #this makes the armmotor2 follow the other one but the oppose master makes it go the opposite direction from the motor it follows
        self.armmotor2.follow(self.armmotor)
        self.armmotor2.setInverted(ctre._ctre.InvertType.OpposeMaster)


    def disabledInit(self): #fixed type 1/24/2022
        self.wiggleTimer.start()

    def disabledPeriodic(self):
        pass

    def autonomousInit(self):
        
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        self.wiggleTimer.reset()
        self.wiggleTimer.start()

        self.armmotor.setSelectedSensorPosition(self.armDegrees_to_counts(self.ARM_MIN))
        
        #self.wristmotor.setSelectedSensorPosition(self.wristDown)

        self.state = 0
       
        

    def teleopPeriodic(self):

       

        #rTrigger = self.xbox.getRightTriggerAxis()
        #AButton = self.xbox.getAButton()

        motorPos = self.armmotor.getSelectedSensorPosition()

        #wristPos = self.wristmotor.getSelectedSensorPosition()

        Arm_Angle_Deg = self.armCounts_to_degrees(motorPos)

        #Wrist_Angle_Deg = self.wristCounts_to_degrees(wristPos)



        
        wpilib.SmartDashboard.putString('DB/String 7',"Pos_Degrees: {:4.2f}".format(Arm_Angle_Deg))
        wpilib.SmartDashboard.putString('DB/String 4',"Time: {:3.2f}".format(self.wiggleTimer.get()))
        #wpilib.SmartDashboard.putString('DB/String 8',"Position: {:4.2f}".format(Wrist_Angle_Deg))
       


        rTrigger = self.xbox.getRightTriggerAxis()
        AButton = self.xbox.getAButton()
        #BButton = self.xbox.getBButton()
        lBumper = self.xbox.getLeftBumper()
        rBumper = self.xbox.getRightBumper()
        lTrigger = self.xbox.getLeftTriggerAxis()
        start = self.xbox.getStartButton() 
        lStickButton = self.xbox.getLeftStickButton()
        rStickButton = self.xbox.getRightStickButton()


        #wpilib.SmartDashboard.putString('DB/String 8',"Position: {:4.2f}".format(rTrigger))
        
        '''
        if AButton:
            self.wristmotor.set(ctre._ctre.ControlMode.MotionMagic, self.targetPos)
        elif BButton:
            self.wristmotor.set(ctre._ctre.ControlMode.MotionMagic, 0)
        else:
            self.wristmotor.set(ctre._ctre.ControlMode.PercentOutput, rTrigger)
        '''
        '''
        arm_feedforward = self.PEAK_ARM_FF * math.cos(Arm_Angle_Deg)

        if lBumper:
            self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.armDegrees_to_counts(0) , ctre.DemandType.ArbitraryFeedForward, arm_feedforward)
        elif rBumper:
            self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.armDegrees_to_counts(self.ARM_MIN) , ctre.DemandType.ArbitraryFeedForward, arm_feedforward)
        else:
            self.armmotor.set(ctre._ctre.ControlMode.PercentOutput, lTrigger) 

        wpilib.SmartDashboard.putString('DB/String 6',"Trigger: {:4.2f}".format(lTrigger))
        '''

        '''
        if lBumper:
            self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.feederStation)
        elif rBumper:
            self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.midCube)
        elif lTrigger:
            self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.midCone)
        elif rTrigger:
            self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.highCube)
        elif self.xbox.getStartButton():
            self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.groundLevel)
        else:
            self.armmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
        '''
        
        #these are the buttons dealing with the states
        if lBumper:
            self.state = 1
        elif rBumper:
            self.state = 2
        elif lTrigger > 0.1:
            self.state = 3
        elif rTrigger > 0.1:
            self.state = 4
        elif start:
            self.state = 5
            

        # these are the actions dealing with the states
        if self.state == 0:
            self.armmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
        elif self.state == 1:
            self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.midCube) 
        elif self.state == 2:
            self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.midCone)
        elif self.state == 3:
            self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.highCube)
        elif self.state == 4:
            self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.feederStation)
        elif self.state == 5:
            self.armmotor.set(ctre._ctre.ControlMode.MotionMagic, self.groundLevel)
        else:
            self.state = 0






    
            




        
        #Wrist Code
        if lStickButton:
            self.wriststate = 2
        elif rStickButton:
            self.wriststate = 1
        
        if self.wriststate == 0:
            self.wristmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
        elif self.wriststate == 1:
            self.wristmotor.set(ctre._ctre.ControlMode.MotionMagic, self.wristUp) 
        elif self.wriststate == 2:
            self.wristmotor.set(ctre._ctre.ControlMode.MotionMagic, self.wristDown)
        else:
            self.wriststate = 0

        


        '''
        if AButton:
                self.wristmotor.set(ctre._ctre.ControlMode.PercentOutput, self.targetPos)
            elif self.xbox.getBButton():
                self.wristmotor.set(ctre._ctre.ControlMode.PercentOutput, self.targetPos / 2)
            else:
                self.wristmotor.set(ctre._ctre.ControlMode.PercentOutput, rTrigger) 
        '''
        
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

            #jay, zack, james
        #these are the templates of our state
        '''
        if self.state == 0:
            if self.xbox.getRightBumper:
                if motorPos < self.armMPos: 
                    self.state = 1
                elif motorPos >= self.armMPos and motorPos < self.armUpperPos: 
                    self.state = 3
            elif self.xbox.getLeftBumper():
                if motorPos > self.armMPos: 
                       self.state = 2
                elif motorPos <= self.armMPos and motorPos > self.armLowerPos: 
                    self.state = 4
#            elif self.xbox.getAButton() and position <= self.armMPos: #for now we are using buttons A and B but plan to change it later
#                self.state = 1
#            elif self.xbox.getBButton() and position >= self.armMPos:
#                self.state = 2

        elif self.state == 1:
            if self.wristmotor.getSelectedSensorPosition() >= self.armMPos: 
                self.state = 0
        elif self.state == 3: 
            if self.wristmotor.getSelectedSensorPosition() >= self.armUpperPos:
               self.state = 0
        elif self.state == 2: 
            if self.wristmotor.getSelectedSensorPosition() <= self.armMPos:
                self.state = 0
        elif self.state == 4:
            if self.wristmotor.getSelectedSensorPosition() <= self.armLowerPos:
                self.state = 0
            ''' # ^ this defines the transitions between the states

                # this is putting the states into action
        '''
        if self.state == 0:
            self.wristmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.0)
        elif self.state == 1:
            self.wristmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.25 )
        elif self.state == 2: 
            self.wristmotor.set(ctre._ctre.ControlMode.PercentOutput, -0.25 )
        elif self.state == 3:
            self.wristmotor.set(ctre._ctre.ControlMode.PercentOutput, 0.25 )
        elif self.state == 4: 
            self.wristmotor.set(ctre._ctre.ControlMode.PercentOutput, -0.25 )
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
    

if __name__ == '__main__':
    wpilib.run(Myrobot)