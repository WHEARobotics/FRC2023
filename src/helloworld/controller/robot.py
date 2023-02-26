import wpilib
from networktables import NetworkTables
import logging

class Myrobot(wpilib.TimedRobot):
    # There are 2 pipelines currently defined, one for driving and one for retroreflective targeting
    # Press A on controller in teleop to switch between them
    PIPELINE_ID_DRIVER = 0
    PIPELINE_ID_RETROREFLECTIVE = 1

    ROBO_RIO_IP_ADDRESS = "10.38.81.2"
    def robotInit(self):
        # Initialize controller (TODO: Probably switch this to XboxController)
        self.joystick = wpilib.Joystick(0)

        # NetworkTables MUST be initialized in order to provide data.
        # Note: Hard-coded so that Robo
        NetworkTables.initialize(server=self.ROBO_RIO_IP_ADDRESS)
        self.limelight = NetworkTables.getTable("limelight")
        self.smart_dashboard = NetworkTables.getTable("SmartDashboard")

        # Start with limelight running driver pipeline
        self.limelight.putNumber("pipeline", self.PIPELINE_ID_DRIVER)
        self.smart_dashboard.putBoolean("limelight driver pipeline", True)

        # Switch this to `logging.INFO` for less clutter in console
        self.logger.setLevel(logging.DEBUG)
        self.logger.debug("Set level to DEBUG")
        self.logger.info("Initialization complete")


    def disabledInit(self):  # fixed type 1/24/2022
        pass
    def disabledPeriodic(self):
        pass

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass


    # You MAY run robot in Test mode just to confirm communication to FMS and Limelight. Not necessary.
    def testInit(self) -> None:
        self.logger.info("testInit() started")
        self.logger.info(f"FMSInfo keys: {NetworkTables.getDefault().getTable('FMSInfo').getKeys()}")
        self.logger.info(f"limelight keys: {NetworkTables.getDefault().getTable('limelight').getKeys()}")

    # Teleop loop
    def teleopPeriodic(self):
        # WPILib buttons start counting from 1
        for i in range(1,self.joystick.getButtonCount()+1):
            bp = self.joystick.getRawButtonPressed(i)
            # If button 1 is pressed, toggle pipeline
            if i == 1 and bp:
                self.toggle_limelight_pipeline()

        # Call limelight input function
        limelight_data = self.limelightPeriodic()
        if limelight_data is not None:
            # limelightPeriodic() returned data, so act on it
            # This could, for instance, set how the robot should turn or move
            self.actOnLimelightData(limelight_data)

    # Switches between pipelines and set's NetworksTables variables to properly show limelight pipeline state
    def toggle_limelight_pipeline(self):
        self.logger.debug("toggle_limelight_pipeline called")
        # Active pipeline is in self.limelight["pipeline"]
        active_pipeline = self.limelight.getNumber("pipeline", 0)
        # Toggle between pipelines
        if active_pipeline == self.PIPELINE_ID_DRIVER:
            self.logger.info("Switching to retroreflective pipeline")
            self.limelight.putNumber("pipeline", self.PIPELINE_ID_RETROREFLECTIVE)
            self.smart_dashboard.putBoolean("limelight driver pipeline", False)
        else:
            self.logger.info("Switching to driver pipeline")
            self.limelight.putNumber("pipeline", self.PIPELINE_ID_DRIVER)
            self.smart_dashboard.putBoolean("limelight driver pipeline", True)

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
    wpilib.run(Myrobot)