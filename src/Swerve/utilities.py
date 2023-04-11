import wpilib
import ctre


     
WRIST_GEAR_RATIO = 80
ARM_GEAR_RATIO = 35.7 * 4  #to get the gear ratio, we had 48 teeth from the sprocket and divided it from the output sahft which is 12, then we multiplied that from the gear ratio which is 12.75


def armCounts_to_degrees( counts):

    degrees = (counts * (360/2048)) / ARM_GEAR_RATIO
    return degrees


def armDegrees_to_counts( degrees):

    counts = (degrees * (2048/360)) * ARM_GEAR_RATIO
    return counts

def wristCounts_to_degrees(counts):

    degrees = (counts * (360/2048)) / WRIST_GEAR_RATIO
    return degrees


def wristDegrees_to_counts(degrees):

    counts = (degrees * (2048/360)) * WRIST_GEAR_RATIO
    return counts

def joystickscaling(input): #this function helps bring an exponential curve in the joystick value and near the zero value it uses less value and is more flat
    a = 1
    output = a * input * input * input + (1 - a) * input
    return output