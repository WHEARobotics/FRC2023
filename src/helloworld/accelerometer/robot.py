import wpilib
from networktables import NetworkTables
import logging

MAX_SPEED = 10

# TODO: Fine-tune this. If robot oscillates on engaged charging station,
# increase DEAD_ZONE_Z value. If robot stays stationary after charging
# station overshoots and disengages, lower DEAD_ZONE_Z value
DEAD_ZONE_Z = 0.1


AUTO_STATE_DRIVE_TO_RAMP = 0
AUTO_STATE_CLIMB_RAMP = 1
AUTO_STATE_WAIT_FOR_SETTLE = 2
AUTO_STATE_ENGAGED = 3 

class Myrobot(wpilib.TimedRobot):
    def robotInit(self):
        '''
        Initialization code. Runs once when robot starts.

        This is just a skeleton robot but will only work with a live 
        RoboRio since it's `BuiltInAccelerometer` needs to return values.

        If this is on a RoboRio, you can just tilt the RoboRio to simulate
        the charging-station teeter-totter. Orient the tilting of the 
        RoboRio in the same orientation as the RoboRio is mounted on the robot!
        '''

        # Initialize the RoboRio accelerometer, but use the filters
        # to smooth out noise in each axis
        self.accel = BuiltInAccelerometer()
        self.xAccelFilter = LinearFilter.movingAverage(10)
        self.yAccelFilter = LinearFilter.movingAverage(10)
        self.zAccelFilter = LinearFilter.movingAverage(10)

        # The charging station is + when lowest side is near alliance side
        self.tilt_is_positive = True

    def log_euler_angles(self):
        '''
        Grabs the Euler angles from the accelerometer, filters them,
        and writes their values to Network Tables. These values are
        expected to be fractions of a 'g'. So a RoboRio sitting flat should
        be (I think) `z_accel == -1.0, y_accel == 0.0, x_accel == 0.0`. 
        TODO: Validate this with a RoboRio!!!

        This is a priority! No logic will work if `z` is not vertical!
        '''
        z_accel = self.zAccelFilter.calculate(self.accel.getZ())
        y_accel = self.yAccelFilter.calculate(self.accel.getY())
        x_accel = self.xAccelFilter.calculate(self.accel.getX())
        wpilib.SmartDashboard.putString('DB/String 0', f"z_accel: {z_accel:4.2f}")
        wpilib.SmartDashboard.putString('DB/String 1', f"x_accel: {x_accel:4.2f}")
        wpilib.SmartDashboard.putString('DB/String 2', f"y_accel: {y_accel:4.2f}")
        

    def autonomousInit(self):
        # Set the initial state of the autonomous state-machine
        self.current_autonomous_state = AUTO_STATE_DRIVE_TO_RAMP

        # At this point, the robot should be level. So this value would 
        # be _expected_ to be very close to -1.0 ("1g straight down through Z")
        self.last_tilt = self.zAccelFilter.calculate(self.accel.getZ())
        self.log_euler_angles()
        

    def set_drive_speed(self, meters_per_second):
        # TODO: Write code to set speed here!
        pass

    def auto_drive_to_ramp_periodic(self):
        '''
        Called periodically from `autonomous_periodic` when the robot 
        is in autonomous mode and
        `self.current_autonomous_state == AUTO_STATE_DRIVE_TO_RAMP`

        This needs to have whatever code is necessary to "drive the 
        robot until it tilts the charge station"
        '''
        # TODO: Put code to drive towards ramp here
        
        # Check to see if accelerometer is beginning to show "tilt"
        tilt = self.zAccelFilter.calculate(self.accel.getZ())
        if tilt <> self.last_tilt:
            # Which way are we tilting?
            if tilt < self.last_tilt:
                self.tilt_is_positive = False
            else:
                self.tilt_is_positive = True
            # The robot has started tilting
            self.current_autonomous_state = AUTO_STATE_CLIMB_RAMP
        # if not tilting, just update current tilt
        self.last_tilt = tilt
        self.ramp_speed = MAX_SPEED

    def auto_climb_ramp_periodic(self):
         '''
        Called periodically from `autonomous_periodic` when the robot 
        is in autonomous mode and
        `self.current_autonomous_state == AUTO_STATE_CLIMB_RAMP`.

        This function checks the current tilt of the ramp and compares it 
        to `last_tilt`. 

        * If the values are the same: Swell. The ramp is still tilted and 
          the robot should continue climbing.
        * If the value is in the same *direction* but *greater*: The ramp is
          tilting more "up" (you'd expect this when the robot is climbing onto
          the charging station). The robot should continue climbing.
        * If the value is in the same *direction* but *less*: That means the 
          ramp is beginning to tilt less. It will only do this when the Center
          of Gravity is past the midpoint. The robot should *stop* climbing. 
          The robot stops climbing by entering the state `AUTO_STATE_WAIT_FOR_SETTLE`

        There are two near-identical code-blocks for the `if self.tilt_is_positive` test:
        that's because this needs to work in both directions.
        '''
        set_drive_speed(self.ramp_speed) 
        # Set ramp speed to + if tilt is +, - if tilt is -
        self.ramp_speed = self.tilt_is_positive ? self.ramp_speed : -self.ramp_speed
        tilt = self.zAccelFilter.calculate(self.accel.getZ())
        # Is the ramp teetering?
        if tilt <> self.last_tilt:
            if self.tilt_is_positive:
                if tilt < self.last_tilt: 
                    # Direction has reversed!
                    self.tilt_is_positive = False
                    self.current_autonomous_state = AUTO_STATE_WAIT_FOR_SETTLE
                else:
                    # Still tilting in positive direction, continue
                    pass 
            else:
                if tilt > self.last_tilt:
                        # Direction has reversed!
                    self.tilt_is_positive = True
                    self.current_autonomous_state = AUTO_STATE_WAIT_FOR_SETTLE
                else:
                    # Still tilting in negative direction, continue
                    pass

    def auto_wait_for_settle(self):
          '''
        Called periodically from `autonomous_periodic` when the robot 
        is in autonomous mode and
        `self.current_autonomous_state == AUTO_STATE_WAIT_FOR_SETTLE`.

        Stop the robot! 

        Then, when the robot is near to flat, enter `AUTO_STATE_ENGAGED`.

        TODO (Low priority?): If the robot _stays_ in this state a long time,
        it may be that the charging station settled a little, but did not go
        "all the way" to engaged. In that case, I'm not really sure what to do.
        Maybe: set robot state to `AUTO_STATE_ENGAGED`, knowing it will diagnose
        the problem and quickly state-change to climbing again?
        '''

        # Wait for ramp to settle
 
        # Stop the robot!
        set_drive_speed(0.0)
        tilt = self.zAccelFilter.calculate(self.accel.getZ())
        # Check if the tilt is near "1 g straight down" which means level
        if abs(1.0 - tilt) < DEAD_ZONE_Z:
            self.current_autonomous_state = AUTO_STATE_ENGAGED
        else:
            # If it isn't level, it's still settling. Stay in current state.
            self.current_autonomous_state = AUTO_STATE_WAIT_FOR_SETTLE

    def auto_engaged(self):
        '''
        Called periodically from `autonomous_periodic` when the robot 
        is in autonomous mode and
        `self.current_autonomous_state == AUTO_STATE_ENGAGED`.

        This state is entered when the robot has climbed the charging station,
        waited for it to settle, and is (at least for a moment) level.

        This function should check the current tilt. If it's near-level, 
        stay in this 'engaged' state. If it's tilted, figure out which
        way to drive and enter `AUTO_STATE_CLIMB_RAMP`
        '''

        tilt = self.zAccelFilter.calculate(self.accel.getZ())
        # Confirm that the robot is still in the deadzone
        # Check if the tilt is near "1 g straight down" which means level
        if abs(1.0 - tilt) < DEAD_ZONE_Z:
            # Stay in this state
            self.current_autonomous_state = AUTO_STATE_ENGAGED
        else:
            # Uh-oh, it's oscillated out of tilt!

            # Figure out which way to drive
            if tilt < 1.0:
                self.tilt_is_positive = False
            else:
                self.tilt_is_positive = True
            # Cut movement speed in half
            self.ramp_speed /= 2.0 

            # Set last_tilt
            self.last_tilt = tilt
            # Go back to climbing state
            self.current_autonomous_state = AUTO_STATE_CLIMB_RAMP
                    
    def autonomousPeriodic(self):
        '''
        Calls functions for each state rather than everything being in one
        huge function
        '''
        if self.current_autonomous_state == AUTO_STATE_DRIVE_TO_RAMP:
            self.auto_drive_to_ramp_periodic()
        if self.current_autonomous_state == AUTO_STATE_CLIMB_RAMP:
            self.auto_climb_ramp_periodic()
        if self.current_autonomous_state == AUTO_STATE_WAIT_FOR_SETTLE: 
            self.auto_wait_for_settle()
        if self.current_autonomous_state == AUTO_STATE_ENGAGED:
            self.auto_engaged()

    def teleopInit(self):
        pass

    def testInit(self) -> None:
        pass
    # Teleop loop
    def teleopPeriodic(self):
        pass

__name__ == '__main__':
    wpilib.run(Myrobot)