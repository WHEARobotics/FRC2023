import wpilib
import wpilib.drive

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.l_motor = wpilib.Jaguar(0)
        self.r_motor = wpilib.Jaguar(1)
        self.motor = wpilib.Jaguar(2)

        self.gyro = wpilib.AnalogGyro(0)

        self.drive = wpilib.drive.DifferentialDrive(self.l_motor, self.r_motor)

    def teleopPeriodic(self):
        self.l_motor.set(0.5)
        self.r_motor.set(0.55)

if __name__ == '__main__':
    wpilib.run(MyRobot)


