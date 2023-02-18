import wpilib
import wpilib.drive
import ntcore
from networktables import NetworkTables
import logging

# There are 2 pipelines currently defined, one for driving and one for retroreflective targeting
# Press tk on controller in teleop to switch between them
PIPELINE_ID_DRIVER = 0
PIPELINE_ID_RETROREFLECTIVE = 1

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Switch this to `level=logging.DEBUG` for more tracing
        logging.basicConfig(level=logging.INFO)
        # NetworkTables MUST be initialized in order to provide data.
        NetworkTables.initialize(server='10.38.81.2')
        self.limelight = NetworkTables.getTable("limelight")
        self.smart_dashboard = NetworkTables.getTable("SmartDashboard")

        self.limelight.putNumber("pipeline", PIPELINE_ID_DRIVER)
        self.smart_dashboard.putBoolean("limelight driver pipeline", True)

        self.logger.info("Initialization complete")

    def testInit(self) -> None:
        self.logger.info("testInit() started")
        self.logger.info(f"FMSInfo keys: {NetworkTables.getDefault().getTable('FMSInfo').getKeys()}")
        self.logger.info(f"limelight keys: {NetworkTables.getDefault().getTable('limelight').getKeys()}")

    def testPeriodic(self) -> None:
        pass

    def teleopPeriodic(self):
        # Check the smart dashboard value and set the limelight pipeline to driver or retro
        if self.smart_dashboard.getBoolean("limelight driver pipeline", True):
            self.limelight.putNumber("pipeline", PIPELINE_ID_DRIVER)
        else:
            self.limelight.putNumber("pipeline", PIPELINE_ID_RETROREFLECTIVE)

        limelight_data = self.limelightPeriodic()
        if limelight_data is None:
            # No limelight data being returned, so leave this function
            return
        if limelight_data is not None:
            # This could, for instance, tell the robot to move
            self.actOnLimelightData(limelight_data)

    # Put code to act on `limelight_data` here
    def actOnLimelightData(self, limelight_data):
        pass

    def limelightPeriodic(self):
        # Check if _any_ data is available from the limelight
        data_available = len(self.limelight.getKeys()) > 0
        if not data_available:
            self.logger.info("Limelight: No data")
            self.smart_dashboard.putString("limelight_status", "No data")
        else:
            self.smart_dashboard.putString("limelight_status", "Data available")
            # Check if the limelight has a target
            has_target = self.limelight.getNumber("tv", 0) == 1
            if has_target:
                self.logger.info("Limelight: Target(s) found")
                self.smart_dashboard.putString("limelight_status", "Target(s) found")

if __name__ == '__main__':
    wpilib.run(MyRobot)


