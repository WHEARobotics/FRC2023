from __future__ import annotations
import ctre
import utilities


'''
ARM_TUCKED_IN = 0 # or any arbitary nuber. This is a class constant
ARM_HIGH_GOAL = 1 # another one
def set_arm_position(self,desired_pos):
    #where the desired_pos is one of ARM_TUCKED_IN or ARM_HIGH_GOAL...
    #Code to do that is here
'''
class Manipulator:

    def __init__(self):

        self.ARM_MIN = -63.0
        self.ARM_HORIZONTAL = 0.0
        self.ARM_MAX = 26.0 #might change to 13.0
        self.PEAK_ARM_FF = 0.07 #this is the amount of output we need in the motor to hold itself when it is in its desired position

        self.armmotor = ctre.TalonFX(2)
        self.armmotor2 = ctre.TalonFX(3)

        self.armmotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.armmotor2.setNeutralMode(ctre._ctre.NeutralMode.Brake)

        #these are all the positions that will be used for the arm
        self.groundLevel = utilities.armDegrees_to_counts(-63) 
        self.feederStation = utilities.armDegrees_to_counts(17) 
        self.midCube = utilities.armDegrees_to_counts(2) 
        self.midCone = utilities.armDegrees_to_counts(7)
        self.highCube = utilities.armDegrees_to_counts(10)
        self.highestpoint = utilities.armDegrees_to_counts(24)

        self.arm_Range_Counts = utilities.armDegrees_to_counts(self.ARM_MAX - self.ARM_MIN)

        #this makes the armmotor2 follow the other one but the oppose master makes it go the opposite direction from the motor it follows
        self.armmotor2.follow(self.armmotor)
        self.armmotor2.setInverted(ctre._ctre.InvertType.OpposeMaster)

        self.WRIST_START = 30  # 56 degrees from the "tucked in position"
        self.WRIST_MAX = 30 # We're calling the "tucked in position" 0 degrees
        self.WRIST_MIN = -121 # Degrees, wrist dropped in collecting position at the ground level.
        self.WRIST_MID = -90
        self.wristDesiredPos = 15

        self.wristmotor = ctre.TalonFX(11)

        self.wristmotor.setNeutralMode(ctre._ctre.NeutralMode.Brake)

            #positons for wrist
        self.wristGroudLevel = utilities.wristDegrees_to_counts(self.WRIST_MIN)
        self.wristInnerPos = utilities.wristDegrees_to_counts(self.WRIST_START)

        self.wrist_Range_Counts = utilities.wristDegrees_to_counts(self.WRIST_MAX - self.WRIST_MIN)




    