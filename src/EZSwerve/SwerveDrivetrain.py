from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModulePosition, SwerveModuleState, ChassisSpeeds
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.controller import PIDController
import wpilib
from wpilib import AnalogGyro

import math

from SwerveModule import SwerveModule

class SwerveDrivetrain :
    MAX_SPEED = 3.0
    MAX_ANGULAR_SPEED = math.pi # 1/2 rotation per second
    
    def __init__(self):

        # Magic number copied from Java example
        self.frontLeftLocation = Translation2d(0.381, 0.381)
        self.frontRightLocation = Translation2d(0.381, -0.381)
        self.backLeftLocation = Translation2d(-0.381, 0.381)
        self.backRightLocation = Translation2d(-0.381, -0.381)

        self.frontLeft = SwerveModule(1, 2, 0, 1, 2, 3)
        self.frontRight = SwerveModule(3, 4, 4, 5, 6, 7)
        self.backLeft = SwerveModule(5, 6, 8, 9, 10, 11)
        self.backRight = SwerveModule(7, 8, 12, 13, 14, 15)

        self.gyro = AnalogGyro(0) 

        # The proper Kinematics and Odometry class to used is determined by the number of modules on the robot.
        # For example, this 4 module robot uses SwerveDrive4Kinematics and SwerveDrive4Odometry.
        self.kinematics = SwerveDrive4Kinematics(\
            self.frontLeftLocation, self.frontRightLocation, 
            self.backLeftLocation, self.backRightLocation)

        self.odometry = SwerveDrive4Odometry(
            self.kinematics, Rotation2d(),
            [
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition()
            ]
        )

        self.gyro.reset()

    def drive(self, xSpeed, ySpeed, rot, fieldRelative):
        chassis_speeds = ChassisSpeeds(xSpeed, ySpeed, rot) if not fieldRelative \
            else ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, self.gyro.getRotation2d())
        swerveModuleStates = self.kinematics.toSwerveModuleStates(chassis_speeds)

        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, self.MAX_SPEED)

        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])


    def updateOdometry(self):
        self.odometry.update(
            self.gyro.getRotation2d(),
            self.frontLeft.getPosition(),
            self.frontRight.getPosition(),
            self.backLeft.getPosition(),
            self.backRight.getPosition()
        )

    @classmethod
    def getMaxSpeed(cls):
        return cls.MAX_SPEED


