from vex import *
from math import radians, degrees, cos, asin, sin, sqrt, pi

# ------------------------------------------------------------ #
### DRIVER CONTROL MOTION MODEL
# ------------------------------------------------------------ #

# FIXME: sync with main
DRIVETRAIN_EXTERNAL_GEAR_RATIO = 24/48
DRIVETRAIN_WHEEL_SIZE = 220 # mm
FORWARD_EFFICIENCY = 1 / 1.045
ROTATION_FWD_WHEEL_SIZE = 2 * 25.4 * pi # mm, example value
ROTATION_SIDE_WHEEL_SIZE = 2 * 25.4 * pi # mm, example value
ROTATION_FWD_WHEEL_OFFSET = 0 # mm, example value
ROTATION_SIDE_WHEEL_OFFSET = 0 # mm, example value

MOTOR_FREE_SPEED_RPM = 600.0 # 6:1 cartridge
# Distance the wheel surface travels per motor revolution
MM_PER_MOTOR_REV_SURFACE = DRIVETRAIN_EXTERNAL_GEAR_RATIO * DRIVETRAIN_WHEEL_SIZE
# 45 degree rollers mean the chassis travels sqrt(2) times that (matches motor_distance_step)
MM_PER_MOTOR_REV_CHASSIS = MM_PER_MOTOR_REV_SURFACE * sqrt(2) * FORWARD_EFFICIENCY
DRIVE_RADIUS = 13.6 * 25.4 / 2.0 # mm, centre of robot to wheel contact patch
MAX_ROBOT_SPEED = MOTOR_FREE_SPEED_RPM / 60.0 * MM_PER_MOTOR_REV_CHASSIS # mm/s
MAX_ROBOT_TURN_RATE = degrees(MOTOR_FREE_SPEED_RPM / 60.0 * MM_PER_MOTOR_REV_SURFACE / DRIVE_RADIUS) # deg/s

class DriveMotionModel:
    '''
    ### Closed loop chassis model for driver control

    Turns the driver's stick command into a desired chassis velocity (mm/s forward, mm/s strafe,
    deg/s rotation), measures what the robot is actually doing, and trims the command so the two
    agree.

    ### Measurement sources
        rotation_fwd, rotation_side: true ground speed - the wheels are unpowered so they cannot slip \\
        inertial: true rotation rate \\
        drive encoders: what the wheels think they are doing, and how far each one is falling \\
        behind its command

    The difference between the encoder view and the tracking wheel view is wheel slip.

    ### Corrections
        1. Per axis PI trim, so a command that is being resisted (carpet drag, a raised lift, \\
           contact with another robot) still produces the motion the driver asked for.
        2. A single authority scale applied to all three axes when a wheel cannot keep up with \\
           its command or the wheels are slipping. Scaling the whole command keeps the ratio \\
           between the four wheels intact, so the robot carries on along the commanded vector \\
           instead of veering towards whichever corner still has grip.
    '''

    # Trim gains, in output percent per percent of full scale velocity error
    KP_TRANSLATE = 0.50
    KI_TRANSLATE = 5.0 # per second
    KP_ROTATE = 0.40
    KI_ROTATE = 4.0 # per second

    # Authority limits on the trim so the model can never take the robot away from the driver
    MAX_TRANSLATE_TRIM = 25.0
    MAX_ROTATE_TRIM = 20.0
    MAX_INTEGRAL_TRIM = 12.0

    # An axis below this stick command is left alone, otherwise the loop fights the coast down
    COMMAND_THRESHOLD = 5.0

    # A wheel this far behind its commanded speed is out of torque rather than out of grip
    WHEEL_FOLLOW_ERROR = 12.0 # percent
    # Encoders reporting this much more motion than the ground truth sensors means slip
    SLIP_THRESHOLD = 12.0 # percent of full scale
    BACKOFF_GAIN = 0.4 # authority lost per loop per percent of overrun
    RECOVERY_RATE = 1.5 # authority regained per loop
    MIN_AUTHORITY = 50.0

    RATE_FILTER = 0.3 # low pass on the differentiated sensor rates

    def __init__(self, brain, rotation_fwd, rotation_side, inertial, left_front_motor, left_back_motor, right_front_motor, right_back_motor):
        self.brain = brain
        self.rotation_fwd = rotation_fwd
        self.rotation_side = rotation_side
        self.inertial = inertial
        self.left_front_motor = left_front_motor
        self.left_back_motor = left_back_motor
        self.right_front_motor = right_front_motor
        self.right_back_motor = right_back_motor

        self.enable_motion_model = False

        self.follow_error = 0.0
        self.saturated = False
        self.slipping = False
        self.authority = 100.0
        self.measured_forward = 0.0
        self.measured_strafe = 0.0
        self.measured_turn = 0.0
        self.wheel_forward = 0.0
        self.wheel_strafe = 0.0
        self.wheel_turn = 0.0
        self.i_forward = 0.0
        self.i_strafe = 0.0
        self.i_turn = 0.0
        self.reset()

    def enable(self):
        self.enable_motion_model = True

    def disable(self):
        self.enable_motion_model = False

    def reset(self):
        '''
        ### Clears the loop state and re-baselines the sensors

        Call whenever the drivetrain is released so stale error does not reappear on the next command.
        '''
        self.i_forward = 0.0
        self.i_strafe = 0.0
        self.i_turn = 0.0
        self.authority = 100.0
        self.follow_error = 0.0
        self.saturated = False
        self.slipping = False
        self.measured_forward = 0.0
        self.measured_strafe = 0.0
        self.measured_turn = 0.0
        self.wheel_forward = 0.0
        self.wheel_strafe = 0.0
        self.wheel_turn = 0.0
        self._last_time = self.brain.timer.time(MSEC)
        self._last_fwd = self.rotation_fwd.position(TURNS)
        self._last_side = self.rotation_side.position(TURNS)
        self._last_theta = self.inertial.rotation(DEGREES)

    def _measure(self, dt):
        lf = self.left_front_motor.velocity(RPM)
        lb = self.left_back_motor.velocity(RPM)
        rf = self.right_front_motor.velocity(RPM)
        rb = self.right_back_motor.velocity(RPM)

        # Same mixing as the motor commands, expressed as a rate
        self.wheel_forward = (lf + lb + rf + rb) / 4.0 / 60.0 * MM_PER_MOTOR_REV_CHASSIS
        self.wheel_strafe = (lf - lb - rf + rb) / 4.0 / 60.0 * MM_PER_MOTOR_REV_CHASSIS
        self.wheel_turn = degrees((lf + lb - rf - rb) / 4.0 / 60.0 * MM_PER_MOTOR_REV_SURFACE / DRIVE_RADIUS)

        theta = self.inertial.rotation(DEGREES)
        turn_rate = (theta - self._last_theta) / dt
        self._last_theta = theta
        self.measured_turn += (turn_rate - self.measured_turn) * self.RATE_FILTER

        fwd = self.rotation_fwd.position(TURNS)
        fwd_rate = (fwd - self._last_fwd) * ROTATION_FWD_WHEEL_SIZE / dt
        self._last_fwd = fwd
        # the tracking wheel sits right of centre, so rotation shows up there as forward travel
        fwd_rate += ROTATION_FWD_WHEEL_OFFSET * radians(self.measured_turn)
        self.measured_forward += (fwd_rate - self.measured_forward) * self.RATE_FILTER

        side = self.rotation_side.position(TURNS)
        side_rate = (side - self._last_side) * ROTATION_SIDE_WHEEL_SIZE / dt
        self._last_side = side
        # the tracking wheel sits ahead of centre, so rotation shows up there as lateral travel
        side_rate -= ROTATION_SIDE_WHEEL_OFFSET * radians(self.measured_turn)
        self.measured_strafe += (side_rate - self.measured_strafe) * self.RATE_FILTER

    def _axis_trim(self, command, error, integral, kp, ki, max_trim, dt, allow_integral):
        if abs(command) < self.COMMAND_THRESHOLD:
            return 0.0, 0.0
        if allow_integral:
            integral = dt.limit(integral + error * ki * dt, self.MAX_INTEGRAL_TRIM)
        return dt.limit(error * kp + integral, max_trim), integral

    def _update_authority(self):
        slip = max(abs(self.wheel_forward - self.measured_forward) / MAX_ROBOT_SPEED,
                   abs(self.wheel_strafe - self.measured_strafe) / MAX_ROBOT_SPEED,
                   abs(self.wheel_turn - self.measured_turn) / MAX_ROBOT_TURN_RATE) * 100.0
        self.slipping = slip > self.SLIP_THRESHOLD

        overrun = max(self.follow_error - self.WHEEL_FOLLOW_ERROR, slip - self.SLIP_THRESHOLD)
        if overrun > 0.0:
            self.authority -= overrun * self.BACKOFF_GAIN
        else:
            self.authority += self.RECOVERY_RATE

        if self.authority > 100.0: self.authority = 100.0
        elif self.authority < self.MIN_AUTHORITY: self.authority = self.MIN_AUTHORITY

    def update(self, forward, strafe, turn):
        '''
        ### Adjusts a stick command so the robot moves the way it was asked to

        ### Arguments
            forward: commanded forward speed in percent \\
            strafe: commanded strafe speed in percent \\
            turn: commanded turn rate in percent

        ### Returns
            Corrected forward, strafe and turn in percent
        '''
        if not self.enable_motion_model:
            return forward, strafe, turn

        now = self.brain.timer.time(MSEC)
        dt = (now - self._last_time) / 1000.0
        self._last_time = now
        # first pass after a reset, or the loop stalled, so there is no usable rate this time round
        if dt <= 0.0 or dt > 0.25:
            return forward, strafe, turn

        self._measure(dt)
        self._update_authority()

        # the command is already being held back, so stop the integrators winding up against it
        allow_integral = self.authority >= 100.0

        forward_error = forward - self.measured_forward / MAX_ROBOT_SPEED * 100.0
        strafe_error = strafe - self.measured_strafe / MAX_ROBOT_SPEED * 100.0
        turn_error = turn - self.measured_turn / MAX_ROBOT_TURN_RATE * 100.0

        forward_trim, self.i_forward = self._axis_trim(forward, forward_error, self.i_forward,
                                                      self.KP_TRANSLATE, self.KI_TRANSLATE,
                                                      self.MAX_TRANSLATE_TRIM, dt, allow_integral)
        strafe_trim, self.i_strafe = self._axis_trim(strafe, strafe_error, self.i_strafe,
                                                    self.KP_TRANSLATE, self.KI_TRANSLATE,
                                                    self.MAX_TRANSLATE_TRIM, dt, allow_integral)
        turn_trim, self.i_turn = self._axis_trim(turn, turn_error, self.i_turn,
                                                 self.KP_ROTATE, self.KI_ROTATE,
                                                 self.MAX_ROTATE_TRIM, dt, allow_integral)

        scale = self.authority / 100.0
        return ((forward + forward_trim) * scale,
                (strafe + strafe_trim) * scale,
                (turn + turn_trim) * scale)

    def observe_wheels(self, left_front, left_back, right_front, right_back):
        '''
        ### Records how far the worst wheel is falling behind the speed it was last commanded

        Call with the final mixed wheel speeds each loop. A wheel that is being carried along by
        the other three is not overloaded, so only a shortfall in the commanded direction counts.
        '''
        if not self.enable_motion_model:
            return

        worst = 0.0
        for command, motor in ((left_front, self.left_front_motor), (left_back, self.left_back_motor),
                               (right_front, self.right_front_motor), (right_back, self.right_back_motor)):
            if abs(command) < self.COMMAND_THRESHOLD:
                continue
            actual = motor.velocity(PERCENT)
            shortfall = command - actual if command > 0 else actual - command
            if shortfall > worst: worst = shortfall

        self.follow_error = worst
        self.saturated = worst > self.WHEEL_FOLLOW_ERROR

