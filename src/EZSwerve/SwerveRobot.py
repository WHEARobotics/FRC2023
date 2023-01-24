import wpilib
from wpilib import XboxController
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from SwerveDrivetrain import SwerveDrivetrain

class SwerveRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.__name__ = "SharkBot"
        self.controller = XboxController(0)
        self.swerve = SwerveDrivetrain()

        # Slew rate limiters to make joystick input
        # more gentle; 1/3 sec from 0 to 1
        self.xSpeedLimiter = SlewRateLimiter(3)
        self.ySpeedLimiter = SlewRateLimiter(3)
        self.rotLimiter = SlewRateLimiter(3)


    #### "Disabled" functions in this section

    # One-time initialization for "disabled" period
    def disabledInit(self) -> None:
        pass

    # One-time cleanup for "disabled" period
    def disabledExit(self) -> None:
        pass

    ##### "Autonomous" functions in this section #####

    # One-time initialization for autonomous functions here
    def autonomousInit(self) -> None:

    # Called repeatedly during auto
    def autonomousPeriodic(self):
        # During dev, allow joystick-based control
        # This could be helpful to see what is reasonable
        # in given time (how far it can move, etc.)
        self.driveWithJoystick(False)

        self.swerve.updateOdometry()

    # One-time cleanup at end of autonomous
    def autonomousExit(self) -> None:
        pass

    #### "Teleop" functions in this section ####

    # One-time initialization for teleop
    # Use joystick for manual control
    def teleopInit(self) -> None:


    def teleopPeriodic(self):
        self.driveWithJoystick(True)
        self.swerve.updateOdometry()

    def teleopExit(self) -> None:
        pass

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        joystick_y = -self.controller.getLeftY()
        joystick_y = applyDeadband(joystick_y, 0.02)
        xSpeed = self.xSpeedLimiter.calculate(joystick_y) * SwerveDrivetrain.getMaxSpeed()

        # Get the y speed. We are inverting this because Xbox controllers return
        # negative values when we push to the left.
        joystick_x = -self.controller.getLeftX()
        joystick_x = applyDeadband(joystick_x, 0.02)
        ySpeed = self.ySpeedLimiter.calculate(joystick_x) * SwerveDrivetrain.MAX_SPEED


        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = -self.controller.getRightX()
        rot = applyDeadband(rot, 0.02)
        rot = self.rotLimiter.calculate(rot) * SwerveDrivetrain.MAX_ANGULAR_SPEED


        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative)

if __name__ == "__main__":
    wpilib.run(SwerveRobot)