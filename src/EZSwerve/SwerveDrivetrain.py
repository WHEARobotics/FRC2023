from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModulePosition, SwerveModuleState, ChassisSpeeds
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.controller import PIDController
from wpilib import AnalogGyro, Field2d, SmartDashboard

import math

from SwerveModule import SwerveModule

class SwerveDrivetrain:
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
        self.swerve_modules = [ self.frontLeft, self.frontRight, self.backLeft, self.backRight ]

        self.gyro = AnalogGyro(0) 

        # The proper Kinematics and Odometry class to used is determined by the number of modules on the robot.
        # For example, this 4 module robot uses SwerveDrive4Kinematics and SwerveDrive4Odometry.
        self.kinematics = SwerveDrive4Kinematics(
            self.frontLeftLocation, self.frontRightLocation, 
            self.backLeftLocation, self.backRightLocation)

        self.odometry = SwerveDrive4Odometry(
            self.kinematics, Rotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition()
            )
        )

        # Where are the swerve modules located on the robot?
        # ? These values of 0.5 are taken from  https://github.com/4201VitruvianBots/2021SwerveSim/blob/main/WPILib_SwerveControllerCommand/src/main/java/frc/robot/Constants.java
        # But they seem odd. What units are they?
        wheel_base = 0.5
        track_width = 0.5
        half_wheel_base = wheel_base / 2
        half_track_width = track_width / 2
        self.module_positions = [
            # Front left
            Translation2d(half_wheel_base, half_track_width),
            # Front right
            Translation2d(half_wheel_base, -half_track_width),
            # Back left
            Translation2d(-half_wheel_base, half_track_width),
            # Back right
            Translation2d(-half_wheel_base, -half_track_width)
        ]

        # The current pose for each swerve module
        # These values are updated in `periodic()`
        self.module_poses = [
            Pose2d(),
            Pose2d(),
            Pose2d(),
            Pose2d()
        ]

        # Simulation support
        self.fieldSim = Field2d()
        SmartDashboard.putData('Field', self.fieldSim)

        self.gyro.reset()

    def periodic(self):
        self._updateOdometry()

        ## Update for simulation

        # Update module poses
        for i in range(len(self.module_positions)):
            rotate_by = self.module_positions[i].rotateBy(self.get_heading())
            robot_translation = self.get_pose().translation()
            module_position = rotate_by + robot_translation
            # Module's heading is its angle relative to the chassis heading
            module_angle = self.swerve_modules[i].getState().angle + self.get_pose().rotation() 
            self.module_poses[i] = Pose2d(module_position, module_angle)

        # Update field sim with information
        self.fieldSim.setRobotPose(self.get_pose())
        self.fieldSim.getObject("Swerve Modules").setPoses(self.module_poses)


    def drive(self, xSpeed, ySpeed, rot, fieldRelative):
        chassis_speeds = ChassisSpeeds(xSpeed, ySpeed, rot) if not fieldRelative \
            else ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, self.gyro.getRotation2d())
        swerveModuleStates = self.kinematics.toSwerveModuleStates(chassis_speeds)

        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, self.MAX_SPEED)

        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])


    def _updateOdometry(self):
        self.odometry.update(
            self.gyro.getRotation2d(),
            self.frontLeft.getPosition(),
            self.frontRight.getPosition(),
            self.backLeft.getPosition(),
            self.backRight.getPosition()
        )

    def get_heading(self):
        return self.gyro.getRotation2d()

    def get_pose(self):
        return self.odometry.getPose()

    @classmethod
    def getMaxSpeed(cls):
        return cls.MAX_SPEED


