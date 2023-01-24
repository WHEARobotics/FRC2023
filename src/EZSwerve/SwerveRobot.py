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

    def autonomousPeriodic(self):
        self.driveWithJoystick(False)
        self.swerve.updateOdometry()

    def teleopPeriodic(self):
        self.driveWithJoystick(True)
        
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