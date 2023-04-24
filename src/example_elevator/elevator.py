import ctre

class Elevator():
    """Hypothetical elevator and game piece grabber to show code modularity.
       Uses 3 Falcons to:
          1. Move the elevator up and down.
          2. Slide the grabber in and out.
          3. Intake or eject game pieces.
    """
    # Create some class-level constants for CAN interaction.
    kTimeoutMs =  10 # CAN communication timeout 10 msec.
    kPIDLoopIdx =  0 # We are only using a single set of loop parameters, the one called zero.
    kSlotIdx =     0 # Similarly using only one slot.

    # Create class-level constants for intake commands.
    INTAKE_CMD_STOP = 0
    INTAKE_CMD_IN = 1
    INTAKE_CMD_OUT = 2

    # Create class-level constants for the intake state machine.
    # The leading underscore is a Python convention, meaning "don't use outside the class."
    _INTAKE_STATE_IDLE = 0
    _INTAKE_STATE_IN =   1 # Drawing in a game piece until it stops or is cancelled.
    _INTAKE_STATE_OUT =  2 # Spitting out a game piece.
    _INTAKE_STATE_LOADED = 4

    # Create class-level constants for elevator/slider position commands.
    POS_CMD_START = 0      # Lowest level, slider retracted.
    POS_CMD_LOW_GATHER = 1 # Lowest level, slider slightly out to get a piece.
    POS_CMD_HIGH_REACH = 2 # Highest level, slider fully slid out.

    # And more class-level internal constants for elevator and slider position in encoder counts.
    # Use these with the target positions.
    _ELEV_BOTTOM = 0
    _ELEV_HIGH = 10000
    _SLIDE_RETRACTED = 0
    _SLIDE_GATHER = 1000
    _SLIDE_REACH = 2000


    def __init__(self, elevator_id, slider_id, intake_id) -> None:
        """Constructor takes 3 arguments: the CAN IDs of the three Falcons.
        """
        # *******************************
        # Set up elevator motor.
        self.elevator = ctre.TalonFX(elevator_id)
        self.elevator.setNeutralMode(ctre._ctre.NeutralMode.Coast) # Start in coast to allow people to move it.
        self.elevator.configNominalOutputForward(0, self.kTimeoutMs)
        self.elevator.configNominalOutputReverse(0, self.kTimeoutMs)
        self.elevator.configPeakOutputForward(1, self.kTimeoutMs)
        self.elevator.configPeakOutputReverse(-1, self.kTimeoutMs)#MAKE SURE THE PEAK AND NOMINAL OUTPUTS ARE NOT BOTH FORWARD OR BOTH REVERSE
        
        # Choose feedback loop Index/slot and then fill with our loop parameters.
        self.elevator.selectProfileSlot(self.kSlotIdx, self.kPIDLoopIdx)
        self.elevator.config_kF(self.kSlotIdx, 0, self.kTimeoutMs)
        self.elevator.config_kP(self.kSlotIdx, 0.3, self.kTimeoutMs)
        self.elevator.config_kI(self.kSlotIdx, 0, self.kTimeoutMs)
        self.elevator.config_kD(self.kSlotIdx, 0, self.kTimeoutMs)

        # Configure motion magic parameters.
        self.elevator.configMotionCruiseVelocity(1000, self.kTimeoutMs) # TODO: this is a placeholder value.
        self.elevator.configMotionAcceleration(1000, self.kTimeoutMs)   # TODO: this is a placeholder value.

        # We need to have the elevator the lowest position when turned on.
        self.elevator.setSelectedSensorPosition(self._ELEV_BOTTOM)   


        # *******************************
        # Set up the slider motor.
        self.slider = ctre.TalonFX(slider_id)
        self.slider.setNeutralMode(ctre._ctre.NeutralMode.Coast) # Start in coast to allow people to move it.
        self.slider.configNominalOutputForward(0, self.kTimeoutMs)
        self.slider.configNominalOutputReverse(0, self.kTimeoutMs)
        self.slider.configPeakOutputForward(1, self.kTimeoutMs)
        self.slider.configPeakOutputReverse(-1, self.kTimeoutMs)#MAKE SURE THE PEAK AND NOMINAL OUTPUTS ARE NOT BOTH FORWARD OR BOTH REVERSE
        
        # Choose feedback loop Index/slot and then fill with our loop parameters.
        self.slider.selectProfileSlot(self.kSlotIdx, self.kPIDLoopIdx)
        self.slider.config_kF(self.kSlotIdx, 0, self.kTimeoutMs)
        self.slider.config_kP(self.kSlotIdx, 0.3, self.kTimeoutMs)
        self.slider.config_kI(self.kSlotIdx, 0, self.kTimeoutMs)
        self.slider.config_kD(self.kSlotIdx, 0, self.kTimeoutMs)

        # Configure motion magic parameters.
        self.slider.configMotionCruiseVelocity(1000, self.kTimeoutMs) # TODO: this is a placeholder value.
        self.slider.configMotionAcceleration(1000, self.kTimeoutMs)   # TODO: this is a placeholder value.

        # We need to have the slide fully retracted when turned on.
        self.slider.setSelectedSensorPosition(self._SLIDE_RETRACTED)   


        # *******************************
        # Set up the intake/ejector motor.
        self.intake = ctre.TalonFX(intake_id)
        self.intake.setNeutralMode(ctre._ctre.NeutralMode.Brake)

        # *******************************
        # Create some variables to hold state.
        self.target_height = self._ELEV_BOTTOM # Target elevator height above the bottom in encoder counts.
        self.target_slide = self._SLIDE_RETRACTED  # Target slide outward position in encoder counts.
        self.intake_state = self._INTAKE_STATE_IDLE


    def set_intake(self, intake_command):
        """Control the game piece intake/ejector. Parameter is one of the INTAKE_CMD_* constants. """
        if intake_command == self.INTAKE_CMD_OUT:
            self.intake_state = self._INTAKE_STATE_OUT
        elif intake_command == self.INTAKE_CMD_IN and self.intake_state != self._INTAKE_STATE_LOADED:
            # We do not want to try to take in a piece if there is already one loaded.
            self.intake_state = self._INTAKE_STATE_IN
        else:
            # Whether the command is "STOP" or something invalid, we want to stop.
            if self.intake_state != self._INTAKE_STATE_LOADED:
                # But only if not already loaded (which will be stopped anyway, and we don't want to forget it is loaded).
                self.intake_state = self._INTAKE_STATE_IDLE

    def set_position(self, position):
        """Control the height and horizontal distance of the mechanism.
           The parameter is one of the POS_CMD_* constants.
        """
        # Set target position based on command.
        if position == self.POS_CMD_LOW_GATHER:
            self.target_height = self._ELEV_BOTTOM
            self.target_slide = self._SLIDE_GATHER
        elif position == self.POS_CMD_HIGH_REACH:
            self.target_height = self._ELEV_HIGH
            self.target_slide = self._SLIDE_REACH
        else:
            # It is either POS_CMD_START or erroneous.
            self.target_height = self._ELEV_BOTTOM
            self.target_slide = self._SLIDE_RETRACTED

    def set_brake(self):
        """Call this to put the elevator and slider into brake mode"""
        self.elevator.setNeutralMode(ctre._ctre.NeutralMode.Brake)
        self.slider.setNeutralMode(ctre._ctre.NeutralMode.Brake)

    def set_coast(self):
        """Call this to put the elevator and slider into coast mode"""
        self.elevator.setNeutralMode(ctre._ctre.NeutralMode.Coast)
        self.slider.setNeutralMode(ctre._ctre.NeutralMode.Coast)


    def periodic(self):
        """A Timed robot should call periodic() on its Elevator object exactly once per autonomousPeriodic() or teleopPeriodic()."""
        # Command elevator and slide motors.
        self.elevator.set(ctre._ctre.ControlMode.MotionMagic, self.target_height)
        self.slide.set(ctre._ctre.ControlMode.MotionMagic, self.target_slide)

        # Check if intake motor current is high enough to indicate it is loaded.
        if self.intake_state == self._INTAKE_STATE_IN and self.intake.getStatorCurrent() >= 50:
            self.intake_state = self._INTAKE_STATE_LOADED

        # Now based on the intake state, set the motor correctly.
        if self.intake_state == self._INTAKE_STATE_IN:
            self.intake.set(ctre._ctre.ControlMode.PercentOutput, 0.4)
        elif self.intake_state == self._INTAKE_STATE_OUT:
            self.intake.set(ctre._ctre.ControlMode.PercentOutput, -0.75)
        else:
            # Stopped or Loaded are both no motion.
            self.intake.set(ctre._ctre.ControlMode.PercentOutput, 0.0)


    def get_actual_height(self) -> float:
        """Returns the instantaneous height of the elevator in centimeters above the start position."""
        height = self.elevator.getSelectedSensorPosition()
        # TODO: convert sensor counts to centimeters and replace the "height" return value.
        return height
    

    def is_in_position(self) -> bool:
        """Returns True if the elevator and slider are both near the target position."""
        # TODO: calculate this and replace the placeholder return value.
        return True
