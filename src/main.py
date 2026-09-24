# ----------------------------------------------------------------------------- #
#                                                                               #                                    
#    Project:        Aurora                                                     #                             
#    Module:         main.py                                                    #
#    Author:         VEX                                                        #
#    Created:        Aug 05 2026                                                #
#    Description:    Override competition robot                                 #
#                                                                               #
#    Configuration:
#       44W XDrive, 45degree wheels, 350rpm at wheels, 220mm wheel circumference
#       - Trackwidth (e.g. between left front and right back wheels) 13.6"
#       Inertial sensor with rough heading compensation at 183/180 degrees
#       Foward and side tracking wheels, 2" diameter omni wheels
#       Two back facing distance sensors
#       Left and right facing distance sensors
#       Front facing distance sensor mounted on claw arm
#       11W 3-stage cascade lift. Max theoretical extension speed ~15"/sec, 11"/sec achievable
#       Chain bar "claw arm" with two 5.5W motors geared 3:1. Arm length 9"
#       Pneumatic claw
#       Pneumatic color toggle retract
#       Total weight: 16lbs
#                                                                               #                                                                          
# ----------------------------------------------------------------------------- #

# Library imports
from vex import *
from v5pythonlibrary import * # Loaded from SDCard
from robotconfiguration import RobotConfiguration
from math import radians, degrees, cos, asin, sin, sqrt, pi

# ------------------------------------------------------------ #
### SETUP DEFAULT ALLIANCE AND AUTONOMOUS SEQUENCE HERE
# ------------------------------------------------------------ #

CALIBRATION = True

ALLIANCE_COLOR = AllianceColor.RED
# ALLIANCE_COLOR = AllianceColor.BLUE

AUTON_SEQUENCE = AutonSequence.SKILLS
# AUTON_SEQUENCE = AutonSequence.MATCH_LEFT
# AUTON_SEQUENCE = AutonSequence.MATCH_RIGHT
# AUTON_SEQUENCE = AutonSequence.MATCH_NONE

# ------------------------------------------------------------ #
### DECLARE DEVICES
# ------------------------------------------------------------ #
brain=Brain()

# Robot configuration code
claw_arm_motor1 = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)
claw_arm_motor2 = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
lift_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
controller_1 = Controller(PRIMARY)

left_front_motor = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
left_back_motor = Motor(Ports.PORT13, GearSetting.RATIO_6_1, True)
right_front_motor = Motor(Ports.PORT3, GearSetting.RATIO_6_1, False)
right_back_motor = Motor(Ports.PORT12, GearSetting.RATIO_6_1, False)
DRIVETRAIN_EXTERNAL_GEAR_RATIO = 24/48
DRIVETRAIN_WHEEL_ANGLES = 45 # deg from straight
DRIVETRAIN_WHEEL_SIZE = 220 # mm circumference
DRIVETRAIN_MAX_TORQUE = 0.35 # Nm

inertial = InertialWrapper(Ports.PORT5, 181.5/180.0)
claw_distance = Distance(Ports.PORT2)

HIDDEN_PERIMITER = 15 #mm

# at 592 - back1 reads 595.3, back2 reads 590.9
BACK_DISTANCE_COMPENSATION = 1500 / 1525
BACK_DISTANCE_FROM_BACK = 66 # mm
back_distance1 = Distance(Ports.PORT4)
back_distance2 = Distance(Ports.PORT9)

ROBOT_WIDTH = 15 * 25.4 # (mm)
ROBOT_LENGTH = 185 * 2 # (mm) 185 measured from back wall to center line

LEFT_DISTANCE_DISTABLE = True
LEFT_DISTANCE_COMPENSATION = 1.0
LEFT_DISTANCE_FROM_LEFT = 10 # mm
left_distance = Distance(Ports.PORT6)

RIGHT_DISTANCE_COMPENSATION = 1.0
RIGHT_DISTANCE_FROM_RIGHT = 10 # mm
right_distance = Distance(Ports.PORT8)

ROTATION_SIDE_WHEEL_SIZE = 2 * 25.4 * pi # mm circumference
ROTATION_SIDE_WHEEL_OFFSET = 61.0 # mm (forward)
rotation_side = Rotation(Ports.PORT18, True)

ROTATION_FWD_WHEEL_SIZE = 2 * 25.4 * pi # mm circumference
ROTATION_FWD_WHEEL_OFFSET = 23.0 # mm (right)
rotation_fwd = Rotation(Ports.PORT20, False)

claw_solenoid = DigitalOut(brain.three_wire_port.a)
toggle_solenoid = DigitalOut(brain.three_wire_port.h)

all_motors = [left_front_motor, left_back_motor,
              right_front_motor, right_back_motor,
              claw_arm_motor1, claw_arm_motor2, lift_motor]

all_motor_names = ["LEFT_FRONT", "LEFT_BACK",
                   "RIGHT_FRONT", "RIGHT_BACK",
                   "ARM_LEFT", "ARM_RIGHT", "LIFT"]

motor_monitor = None

all_sensors = [inertial, claw_distance, back_distance1, back_distance2, left_distance, right_distance, rotation_side, rotation_fwd]
all_sensors_names = ["INERTIAL", "CLAW_DISTANCE", "BACK_DISTANCE1", "BACK_DISTANCE2", "LEFT_DISTANCE", "RIGHT_DISTANCE", "ROTATION_SIDE", "ROTATION_FWD"]

# ------------------------------------------------------------ #
### ROBOT STATE
# ------------------------------------------------------------ #

ROBOT_INITIALIZED = False
ROBOT_ENABLED = False
QUIET_MODE = False

# ------------------------------------------------------------ #
### ROBOT CONFIGURATION
# ------------------------------------------------------------ #

robot_config = RobotConfiguration(brain, controller_1)

### DRIVETRAIN UTILITIES

def calculate_effective_wheel_size():
    wheel_efficiency = DRIVETRAIN_WHEEL_ANGLES / 90
    effective_wheel_size = DRIVETRAIN_WHEEL_SIZE * wheel_efficiency / DRIVETRAIN_EXTERNAL_GEAR_RATIO
    return effective_wheel_size

def current_robot_speed(): # mm/s
    # Returns the robot's speed based on the effective wheel size and motor RPM
    effective_wheel_size = calculate_effective_wheel_size()

    left_motor_rpms = (left_front_motor.velocity(RPM) + left_back_motor.velocity(RPM)) / 2
    right_motor_rpms = (right_front_motor.velocity(RPM) + right_back_motor.velocity(RPM)) / 2
    average_motor_rpm = (left_motor_rpms + right_motor_rpms) / 2
    average_motor_rps = average_motor_rpm / 60

    speed = average_motor_rps * effective_wheel_size
    return speed

### TOGGLE CONTROL

def raise_toggle():
    toggle_solenoid.set(0)

def lower_toggle():
    toggle_solenoid.set(1)

def toggle_raised():
    return toggle_solenoid.value() == 0

### LIFT CONTROL

LIFT_RUNNING = False
LIFT_HOLDING = False
LIFT_LINKS = 31
LIFT_TEETH = 6
LIFT_DEGREES_PER_LINK = 360 / LIFT_TEETH
lift_thread = None
lift_hold_time_start = 0

def initialize_lift():
    lift_motor.set_stopping(HOLD)
    lift_motor.set_velocity(100, PERCENT)
    lift_motor.set_timeout(3, SECONDS)
    lift_motor.spin(REVERSE)
    wait(1, SECONDS)
    lift_motor.stop(HOLD)
    wait(100, MSEC)
    lift_motor.set_position(0, DEGREES)
    lift_motor.stop(COAST)

def lift_height(percent=False):
    if percent:
        return (lift_motor.position(DEGREES) / LIFT_DEGREES_PER_LINK) * (100 / LIFT_LINKS)
    return lift_motor.position(DEGREES) / LIFT_DEGREES_PER_LINK

def command_lift(links):
    global lift_hold_time_start, LIFT_RUNNING, LIFT_HOLDING
    if LIFT_RUNNING: return
    LIFT_RUNNING = True
    LIFT_HOLDING = False
    starting_position = lift_motor.position(DEGREES)
    lift_motor.set_velocity(100, PERCENT)
    lift_motor.set_stopping(HOLD)
    lift_motor.set_timeout(5, SECONDS)
    lift_motor.spin_to_position(links * LIFT_DEGREES_PER_LINK, DEGREES)
    lift_motor.stop()
    LIFT_RUNNING = False
    LIFT_HOLDING = True
    lift_hold_time_start = brain.timer.time(SECONDS)
    ending_position = lift_motor.position(DEGREES)
    total_links_moved = (ending_position - starting_position) / LIFT_DEGREES_PER_LINK
    #print("Lift up from {} to {}, total {} links".format(starting_position, ending_position, total_links_moved))

def raise_lift():
    global lift_hold_time_start, LIFT_RUNNING, LIFT_HOLDING
    if LIFT_RUNNING: return
    LIFT_RUNNING = True
    LIFT_HOLDING = False
    starting_position = lift_motor.position(DEGREES)
    lift_motor.set_velocity(100, PERCENT)
    lift_motor.set_stopping(HOLD)
    lift_motor.set_timeout(5, SECONDS)
    lift_motor.spin_to_position(LIFT_LINKS * LIFT_DEGREES_PER_LINK, DEGREES)
    lift_motor.stop()
    LIFT_RUNNING = False
    LIFT_HOLDING = True
    lift_hold_time_start = brain.timer.time(SECONDS)
    ending_position = lift_motor.position(DEGREES)
    total_links_moved = (ending_position - starting_position) / LIFT_DEGREES_PER_LINK
    #print("Lift up from {} to {}, total {} links".format(starting_position, ending_position, total_links_moved))

def lower_lift():
    global lift_hold_time_start, LIFT_RUNNING, LIFT_HOLDING
    if LIFT_RUNNING: return
    LIFT_RUNNING = True
    LIFT_HOLDING = False
    starting_position = lift_motor.position(DEGREES)
    lift_motor.set_velocity(100, PERCENT)
    lift_motor.set_stopping(HOLD)
    lift_motor.set_timeout(5, SECONDS)
    lift_motor.spin_to_position(0, DEGREES)
    if lift_height(percent=True) <= 1:
        print("Lift is near the bottom, coasting")
        lift_motor.stop(COAST)
    else:
        lift_motor.stop(HOLD)
    LIFT_RUNNING = False
    LIFT_HOLDING = True
    lift_hold_time_start = brain.timer.time(SECONDS)
    ending_position = lift_motor.position(DEGREES)
    total_links_moved = (starting_position - ending_position) / LIFT_DEGREES_PER_LINK
    #print("Lift down from {} to {}, total {} links".format(starting_position, ending_position, total_links_moved))

def check_lift_hold():
    global lift_hold_time_start, LIFT_RUNNING, LIFT_HOLDING
    print("Checking lift hold")
    brain.timer.event(check_lift_hold, 10000)
    if not LIFT_HOLDING: return
    if LIFT_RUNNING: return
    if brain.timer.time(SECONDS) - lift_hold_time_start > 10.0:
        lift_motor.stop(COAST)
        LIFT_HOLDING = False

### CLAW CONTROL

CLAW_INITIALIZED = False
CLAW_ARM_RUNNING = False
CLAW_ARM_CANCEL_OPERATION = False
CLAW_ARM_WAS_CANCELLED = False
CLAW_ARM_UP_DEGREES = 160 * 3
CLAW_ARM_MID3_DEGREES = 30 * 3 # was 24.5 * 3
CLAW_ARM_MID2_DEGREES = 24.5 * 3 # was 24.5 * 3
CLAW_ARM_MID1_DEGREES = 20 * 3 # was 18 * 3
CLAW_ARM_DOWN_DEGREES = 0 * 3
CLAW_ARM_DOWN = 0
CLAW_ARM_MID1 = 1
CLAW_ARM_MID2 = 2
CLAW_ARM_MID3 = 3
CLAW_ARM_UP = 4
CLAW_ARM_POSITION = CLAW_ARM_DOWN  # 0 = down, 1 = mid1, 2 = mid2, 3 = mid3, 4 = up
CLAW_ARM_TIMEOUT = 2.0
CLAW_ARM_SPEED = 50

def initialize_claw():
    global CLAW_INITIALIZED
    if CLAW_INITIALIZED: return
    claw_arm_motor1.set_velocity(30, PERCENT)
    claw_arm_motor1.set_stopping(HOLD)
    claw_arm_motor1.set_timeout(2, SECONDS)
    claw_arm_motor2.set_velocity(30, PERCENT)
    claw_arm_motor2.set_stopping(HOLD)
    claw_arm_motor2.set_timeout(2, SECONDS)
    claw_arm_motor1.spin_to_position(-30, DEGREES, wait=False)
    claw_arm_motor2.spin_to_position(-30, DEGREES)
    wait(0.25, SECONDS)
    claw_arm_motor1.set_position(0, DEGREES)
    claw_arm_motor2.set_position(0, DEGREES)
    claw_arm_motor1.stop(HOLD)
    claw_arm_motor2.stop(HOLD)
    CLAW_INITIALIZED = True

CLAW_ARM_COMMAND_NONE = 0
CLAW_ARM_COMMAND_RAISE = 1
CLAW_ARM_COMMAND_LOWER = 2
CLAW_ARM_COMMAND_TO_POSITION = 3
CLAW_ARM_COMMAND_CANCEL = 5

def run_claw_arm(command, target_position=-1):
    global CLAW_ARM_RUNNING, CLAW_ARM_POSITION, CLAW_ARM_CANCEL_OPERATION, CLAW_ARM_WAS_CANCELLED

    # WARNING: RE-ENTRANT CODE
    if CLAW_ARM_RUNNING and command != CLAW_ARM_COMMAND_CANCEL: return

    if command == CLAW_ARM_COMMAND_CANCEL:
        CLAW_ARM_CANCEL_OPERATION = True
        return
    CLAW_ARM_CANCEL_OPERATION = False
    # END RE-ENTRANT CODE

    positions_list = [CLAW_ARM_DOWN, CLAW_ARM_MID1, CLAW_ARM_MID2, CLAW_ARM_MID3, CLAW_ARM_UP]
    target_list = [CLAW_ARM_DOWN_DEGREES, CLAW_ARM_MID1_DEGREES, CLAW_ARM_MID2_DEGREES, CLAW_ARM_MID3_DEGREES, CLAW_ARM_UP_DEGREES]

    if command == CLAW_ARM_COMMAND_NONE: return

    if command == CLAW_ARM_COMMAND_RAISE:
        if CLAW_ARM_WAS_CANCELLED:
            claw_target_position = CLAW_ARM_UP
        else:
            if CLAW_ARM_POSITION >= CLAW_ARM_UP : return
            claw_target_position = CLAW_ARM_POSITION + 1
            # MID3 only used for autonomous
            if (claw_target_position == CLAW_ARM_MID3): claw_target_position += 1
        arm_speed = CLAW_ARM_SPEED
    elif command == CLAW_ARM_COMMAND_LOWER:
        if CLAW_ARM_WAS_CANCELLED:
            claw_target_position = CLAW_ARM_DOWN
        else:
            if CLAW_ARM_POSITION <= CLAW_ARM_DOWN: return
            claw_target_position = CLAW_ARM_POSITION - 1
            # MID3 only used for autonomous
            if (claw_target_position == CLAW_ARM_MID3): claw_target_position -= 1
        arm_speed = CLAW_ARM_SPEED * 0.75
    elif command == CLAW_ARM_COMMAND_TO_POSITION:
        if target_position == CLAW_ARM_POSITION: return
        if target_position < CLAW_ARM_DOWN or target_position > CLAW_ARM_UP: return
        claw_target_position = target_position
        if target_position > CLAW_ARM_POSITION: arm_speed = CLAW_ARM_SPEED
        else: arm_speed = CLAW_ARM_SPEED * 0.75

    if claw_target_position >= CLAW_ARM_MID3:
        raise_toggle()

    claw_target_degrees = target_list[claw_target_position]

    CLAW_ARM_RUNNING = True
    CLAW_ARM_WAS_CANCELLED = False
    starting_position = (claw_arm_motor1.position(DEGREES), claw_arm_motor2.position(DEGREES))
    claw_arm_motor1.set_velocity(arm_speed, PERCENT)
    claw_arm_motor1.set_stopping(HOLD)
    claw_arm_motor1.set_timeout(CLAW_ARM_TIMEOUT, SECONDS)
    claw_arm_motor2.set_velocity(arm_speed, PERCENT)
    claw_arm_motor2.set_stopping(HOLD)
    claw_arm_motor2.set_timeout(CLAW_ARM_TIMEOUT, SECONDS)
    claw_arm_motor1.spin_to_position(claw_target_degrees, DEGREES, wait=False)
    claw_arm_motor2.spin_to_position(claw_target_degrees, DEGREES, wait=False)
    count = 0
    while not (claw_arm_motor1.is_done() and claw_arm_motor2.is_done()) and not CLAW_ARM_CANCEL_OPERATION: 
        wait(10, MSEC)
        count += 1
    # wait(333, MSEC)
    claw_arm_motor1.stop()
    claw_arm_motor2.stop()
    CLAW_ARM_RUNNING = False
    if CLAW_ARM_CANCEL_OPERATION:
        CLAW_ARM_CANCEL_OPERATION = False
        CLAW_ARM_WAS_CANCELLED = True
    CLAW_ARM_POSITION = claw_target_position
    ending_position = (claw_arm_motor1.position(DEGREES), claw_arm_motor2.position(DEGREES))
    print("Claw from {} to {}, {}ms".format(starting_position, ending_position, count * 10))

def raise_claw_arm():
    run_claw_arm(CLAW_ARM_COMMAND_RAISE)

def lower_claw_arm():
    run_claw_arm(CLAW_ARM_COMMAND_LOWER)

def move_claw_arm_to_position(target_position, unused = 0):
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, target_position)

def claw_arm_current_position():
    return CLAW_ARM_POSITION

CLAW_OPEN = 1
CLAW_CLOSED = 0

def claw_is_open():
    return claw_solenoid.value() == CLAW_OPEN

def claw_is_closed():
    return claw_solenoid.value() == CLAW_CLOSED

claw_open_time = 0

def open_claw():
    global claw_open_time
    claw_open_time = brain.timer.time(SECONDS)
    claw_solenoid.set(CLAW_OPEN)

def close_claw():
    claw_solenoid.set(CLAW_CLOSED)

def auto_claw_thread():
    while True:
        current_time = brain.timer.time(SECONDS)
        claw_ready = (current_time - claw_open_time) > 1
        if ROBOT_ENABLED and claw_ready and claw_is_open() and lift_height(True) < 1:
            if (robot_config.enable_auto_claw_down and CLAW_ARM_POSITION == CLAW_ARM_DOWN):
                if (claw_distance.object_distance() < 70):
                    close_claw()
                    wait(1, SECONDS)
            elif (robot_config.enable_auto_claw_mid1 and CLAW_ARM_POSITION == CLAW_ARM_MID1):
                if (claw_distance.object_distance() < 70):
                    close_claw()
                    wait(1, SECONDS)
        wait(10, MSEC)

### AUTONOMOUS

# ------------------------------------------------------------ #
### DRIVETRAIN FUNCTIONS
# ------------------------------------------------------------ #

### ROBOT LOCATION

X = 0
Pxx = 1.0
Y = 0
Pyy = 1.0
THETA = 0

### MOTOR COMMAND VELOCITIES

class XDriveTrain():

    FOWARD_EFFICIENCY = 1 / 1.045
    LEFT_POWER_SCALING = 1.0
    RIGHT_POWER_SCALING = 0.85
    FRONT_POWER_SCALING = 1.0
    BACK_POWER_SCALING = 1.0

    def __init__(self, lfm: Motor, lbm: Motor, rfm: Motor, rbm: Motor):
        self.lfm = lfm
        self.lbm = lbm
        self.rfm = rfm
        self.rbm = rbm

        self.last_lfm_command_vel = 0
        self.last_lbm_command_vel = 0
        self.last_rfm_command_vel = 0
        self.last_rbm_command_vel = 0

        self.last_fwd_command = 0
        self.last_strafe_command = 0
        self.last_turn_command = 0

    def log_motor_commands(self, lfm_vel, lbm_vel, rfm_vel, rbm_vel):
        self.last_lfm_command_vel = lfm_vel
        self.last_lbm_command_vel = lbm_vel
        self.last_rfm_command_vel = rfm_vel
        self.last_rbm_command_vel = rbm_vel

    def log_drive_commands(self, fwd, strafe, turn):
        self.last_fwd_command = fwd
        self.last_strafe_command = strafe
        self.last_turn_command = turn

    @staticmethod
    def limit(input, limit_value):
        if (input > limit_value): return limit_value
        elif (input < -limit_value): return -limit_value
        return input

    @staticmethod
    def ramp_limit(current, previous, limit):
        if (current - previous) > limit:
            return previous + limit
        elif (current - previous) < -limit:
            return previous - limit
        return current

    def stop_all(self, mode = COAST):
        self.lfm.stop(mode)
        self.lbm.stop(mode)
        self.rfm.stop(mode)
        self.rbm.stop(mode)

    def turn_for(self, turn_degrees, speed=66, timeout=10000):
        current_heading = inertial.rotation()
        target_heading = current_heading + turn_degrees
        heading_error = target_heading - current_heading
        target_tolerance = 1  # degrees
        settle_count = 0
        timeout_count = 0
        is_timeout = False
        is_settle = False
        loop_count = 0
        done = False
        turn_kp = 6.0

        while not done:

            current_heading = inertial.rotation()
            heading_error = target_heading - current_heading

            if abs(heading_error) < target_tolerance:
                settle_count += 1
            else:
                settle_count = 0

            if timeout_count > int(timeout / 10): is_timeout = True
            if settle_count > 10: is_settle = True

            if is_timeout or is_settle:
                self.lfm.stop(BRAKE)
                self.lbm.stop(BRAKE)
                self.rfm.stop(BRAKE)
                self.rbm.stop(BRAKE)
                done = True
            else:
                turn_control = turn_kp * heading_error / 360.0
                turn_control = self.limit(turn_control, 1.0) * speed

                self.lfm.spin(FORWARD, turn_control, PERCENT)
                self.lbm.spin(FORWARD, turn_control, PERCENT)
                self.rfm.spin(REVERSE, turn_control, PERCENT)
                self.rbm.spin(REVERSE, turn_control, PERCENT)
                self.log_motor_commands(turn_control, turn_control, -turn_control, -turn_control)
                self.log_drive_commands(0, 0, turn_control)

            timeout_count += 1
            loop_count += 1
            wait(10, MSEC)

        if not QUIET_MODE:
            print("turn_for: x={}, y={}, heading={}, time={}, timeout={}, settle={}".format(X, Y, THETA, loop_count * 10, is_timeout, is_settle))

        return is_timeout, is_settle

    def drive_for(self, distance, strafe=False, speed=100, heading=None, timeout=10000): # distance is in mm, speed is in percent
        # setup
        turn_speed = 100 # max turn speed in percent
        wheel_efficiency = 1 / cos(radians(DRIVETRAIN_WHEEL_ANGLES))
        effective_wheel_size = 220 * wheel_efficiency * DRIVETRAIN_EXTERNAL_GEAR_RATIO
        forward_target_revs = (distance / effective_wheel_size) / self.FOWARD_EFFICIENCY if not strafe else 0
        strafe_target_revs = distance / effective_wheel_size if strafe else 0
        if not QUIET_MODE:
            print("Target revolutions: {:.2f} {:.2f}".format(forward_target_revs, strafe_target_revs))
        target_tolerance = 10 / effective_wheel_size
        ramp_rate = 1
        drive_kp = 50.0  # Proportional gain for drive control
        turn_kp = 500.0  # Proportional gain for turn control

        # save initial motor positions
        starting_left_front_position = self.lfm.position(TURNS)
        starting_left_back_position = self.lbm.position(TURNS)
        starting_right_front_position = self.rfm.position(TURNS)
        starting_right_back_position = self.rbm.position(TURNS)

        # save starting rotation
        target_rotation = heading if heading is not None else inertial.rotation()

        if not QUIET_MODE:
            print("LF: {}, LB: {}, RF: {}, RB: {}".format(starting_left_front_position, starting_left_back_position, starting_right_front_position, starting_right_back_position))

        done = False
        timeout_count = 0
        settle_count = 0
        loop_count = 0
        is_timeout = False
        is_settle = False
        last_fwd = 0
        last_strafe = 0
        fwd_ramp_enabled = True
        strafe_ramp_enabled = True
        
        while not done:
            current_left_front_position = self.lfm.position(TURNS)
            current_left_back_position = self.lbm.position(TURNS)
            current_right_front_position = self.rfm.position(TURNS)
            current_right_back_position = self.rbm.position(TURNS)

            current_rotation = inertial.rotation()
            rotation_error = (target_rotation - current_rotation) / 360.0 # saturate at 360 degrees

            left_front_delta = current_left_front_position - starting_left_front_position
            left_back_delta = current_left_back_position - starting_left_back_position
            right_front_delta = current_right_front_position - starting_right_front_position
            right_back_delta = current_right_back_position - starting_right_back_position

            left_error = forward_target_revs - (left_front_delta + left_back_delta) / 2.0
            right_error = forward_target_revs - (right_front_delta + right_back_delta) / 2.0
            average_fwd_error = (left_error + right_error) / 2.0

            front_error = strafe_target_revs - (-right_front_delta + left_front_delta) / 2.0
            back_error = strafe_target_revs - (right_back_delta - left_back_delta) / 2.0
            average_strafe_error = (front_error + back_error) / 2.0

            # print("{:0.2f} {:0.2f} {:0.2f} {:0.2f}".format(left_error, right_error, front_error, back_error))

            average_error = average_fwd_error if not strafe else average_strafe_error
            if abs(average_error) < target_tolerance:
                settle_count += 1
            else:
                settle_count = 0

            if (timeout_count > timeout): is_timeout = True
            if (settle_count > 10): is_settle = True

            if is_timeout or is_settle:
                done = True
                self.lfm.stop(BRAKE)
                self.lbm.stop(BRAKE)
                self.rfm.stop(BRAKE)
                self.rbm.stop(BRAKE)
            else:
                fwd_control = drive_kp * average_fwd_error
                fwd_control = self.limit(fwd_control, speed)
                if abs(fwd_control) < abs(last_fwd): fwd_ramp_enabled = False
                if fwd_ramp_enabled: fwd_control = self.ramp_limit(fwd_control, last_fwd, ramp_rate)
                last_fwd = fwd_control
                fwd_control_percent = fwd_control

                strafe_control = drive_kp * average_strafe_error
                strafe_control = self.limit(strafe_control, speed)
                if abs(strafe_control) < abs(last_strafe): strafe_ramp_enabled = False
                if strafe_ramp_enabled: strafe_control = self.ramp_limit(strafe_control, last_strafe, ramp_rate)
                last_strafe = strafe_control
                strafe_control_percent = strafe_control

                turn_control = turn_kp * rotation_error
                turn_control_percent = self.limit(turn_control, turn_speed)

                left_power = self.LEFT_POWER_SCALING
                right_power = self.RIGHT_POWER_SCALING
                front_power = self.FRONT_POWER_SCALING
                back_power = self.BACK_POWER_SCALING

                left_front_speed = fwd_control_percent * right_power + strafe_control_percent * front_power + turn_control_percent
                left_back_speed = fwd_control_percent * left_power - strafe_control_percent * back_power + turn_control_percent
                right_front_speed = fwd_control_percent * left_power - strafe_control_percent * front_power - turn_control_percent
                right_back_speed = fwd_control_percent * right_power + strafe_control_percent * back_power - turn_control_percent

                max_speed = max(abs(left_front_speed), abs(left_back_speed), abs(right_front_speed), abs(right_back_speed))
                if max_speed > 100:
                    left_front_speed = left_front_speed * (100 / max_speed)
                    left_back_speed = left_back_speed * (100 / max_speed)
                    right_front_speed = right_front_speed * (100 / max_speed)
                    right_back_speed = right_back_speed * (100 / max_speed)

                self.lfm.spin(FORWARD, left_front_speed, PERCENT)
                self.lbm.spin(FORWARD, left_back_speed, PERCENT)
                self.rfm.spin(FORWARD, right_front_speed, PERCENT)
                self.rbm.spin(FORWARD, right_back_speed, PERCENT)
                self.log_motor_commands(left_front_speed, left_back_speed, right_front_speed, right_back_speed)
                self.log_drive_commands(fwd_control_percent, strafe_control_percent, turn_control_percent)

            timeout_count += 1
            loop_count += 1
            wait(10, MSEC)

        # save initial motor positions
        left_front_position = self.lfm.position(TURNS)
        left_back_position = self.lbm.position(TURNS)
        right_front_position = self.rfm.position(TURNS)
        right_back_position = self.rbm.position(TURNS)

        if not QUIET_MODE:
            print("drive_for: LF: {}, LB: {}, RF: {}, RB: {}".format(left_front_position, left_back_position, right_front_position, right_back_position))
            print("drive_for: x={}, y={}, heading={}, time={}, timeout={}, settle={}".format(X, Y, THETA, loop_count * 10, is_timeout, is_settle))

        return is_timeout, is_settle

    def drive_to_xy(self, target_x, target_y, strafe=False, speed=100, heading=None, timeout=10000): # distance is in mm, speed is in percent

        if not QUIET_MODE:
            print("Driving to X: {}, Y: {} from X: {}, Y: {}".format(target_x, target_y, X, Y))

        # setup
        wheel_efficiency = 1 / cos(radians(DRIVETRAIN_WHEEL_ANGLES))
        effective_wheel_size = 220 * wheel_efficiency * DRIVETRAIN_EXTERNAL_GEAR_RATIO
        turn_speed = 100 # max turn speed in percent
        target_tolerance = 10 # mm
        ramp_rate = 1
        drive_kp = 50.0  # Proportional gain for drive control # 50 / 256
        drive_kd = 256.0 # Derivative gain for drive control
        turn_kp = 550.0  # Proportional gain for turn control
        derivative_alpha = 0.2
        previous_fwd_error_revs = None
        previous_strafe_error_revs = None
        filtered_fwd_derivative = 0.0
        filtered_strafe_derivative = 0.0

        # save starting rotation
        target_rotation = heading if heading is not None else inertial.rotation()

        done = False
        timeout_count = 0
        settle_count = 0
        loop_count = 0
        is_timeout = False
        is_settle = False
        last_fwd = 0
        last_strafe = 0
        fwd_ramp_enabled = True
        strafe_ramp_enabled = True

        while not done:
            current_rotation = inertial.rotation()
            rotation_error = (target_rotation - current_rotation) / 360.0 # saturate at 360 degrees

            # errors need to be rotated based on heading

            # rotate errors based on current heading
            rotated_x_error = cos(radians(current_rotation)) * (target_x - X) + sin(radians(current_rotation)) * (target_y - Y)
            rotated_y_error = -sin(radians(current_rotation)) * (target_x - X) + cos(radians(current_rotation)) * (target_y - Y)

            average_fwd_error = rotated_x_error
            average_strafe_error = rotated_y_error

            # print("{:0.2f} {:0.2f} {:0.2f} {:0.2f}".format(left_error, right_error, front_error, back_error))

            average_error = average_fwd_error if not strafe else average_strafe_error
            if abs(average_error) < target_tolerance:
                settle_count += 1
            else:
                settle_count = 0

            # convert to approximate motor revolutions based on errors
            forward_error_revs = (average_fwd_error / effective_wheel_size) # if not strafe else 0
            strafe_error_revs = (average_strafe_error / effective_wheel_size) # if strafe else 0
            if previous_fwd_error_revs is None: previous_fwd_error_revs = forward_error_revs
            if previous_strafe_error_revs is None: previous_strafe_error_revs = strafe_error_revs

            raw_fwd_derivative = forward_error_revs - previous_fwd_error_revs
            raw_strafe_derivative = strafe_error_revs - previous_strafe_error_revs
            filtered_fwd_derivative += derivative_alpha * (raw_fwd_derivative - filtered_fwd_derivative)
            filtered_strafe_derivative += derivative_alpha * (raw_strafe_derivative - filtered_strafe_derivative)

            if (timeout_count > timeout): is_timeout = True
            if (settle_count > 10): is_settle = True

            if is_timeout or is_settle:
                done = True
                self.lfm.stop(BRAKE)
                self.lbm.stop(BRAKE)
                self.rfm.stop(BRAKE)
                self.rbm.stop(BRAKE)
            else:
                fwd_control = drive_kp * forward_error_revs + drive_kd * filtered_fwd_derivative
                fwd_control = self.limit(fwd_control, speed)
                if abs(fwd_control) < abs(last_fwd): fwd_ramp_enabled = False
                if fwd_ramp_enabled: fwd_control = self.ramp_limit(fwd_control, last_fwd, ramp_rate)
                last_fwd = fwd_control
                fwd_control_percent = fwd_control

                strafe_control = drive_kp * strafe_error_revs + drive_kd * filtered_strafe_derivative
                strafe_control = self.limit(strafe_control, speed)
                if abs(strafe_control) < abs(last_strafe): strafe_ramp_enabled = False
                if strafe_ramp_enabled: strafe_control = self.ramp_limit(strafe_control, last_strafe, ramp_rate)
                last_strafe = strafe_control
                strafe_control_percent = strafe_control

                previous_fwd_error_revs = forward_error_revs
                previous_strafe_error_revs = strafe_error_revs

                turn_control = turn_kp * rotation_error
                turn_control_percent = self.limit(turn_control, turn_speed)

                left_power = 1.0 # LEFT_POWER_SCALING
                right_power = 1.0 # RIGHT_POWER_SCALING
                front_power = 1.0 # FRONT_POWER_SCALING
                back_power = 1.0 # BACK_POWER_SCALING

                left_front_speed = fwd_control_percent * right_power + strafe_control_percent * front_power + turn_control_percent
                left_back_speed = fwd_control_percent * left_power - strafe_control_percent * back_power + turn_control_percent
                right_front_speed = fwd_control_percent * left_power - strafe_control_percent * front_power - turn_control_percent
                right_back_speed = fwd_control_percent * right_power + strafe_control_percent * back_power - turn_control_percent

                max_speed = max(abs(left_front_speed), abs(left_back_speed), abs(right_front_speed), abs(right_back_speed))
                if max_speed > 100:
                    left_front_speed = left_front_speed * (100 / max_speed)
                    left_back_speed = left_back_speed * (100 / max_speed)
                    right_front_speed = right_front_speed * (100 / max_speed)
                    right_back_speed = right_back_speed * (100 / max_speed)

                self.lfm.spin(FORWARD, left_front_speed, PERCENT)
                self.lbm.spin(FORWARD, left_back_speed, PERCENT)
                self.rfm.spin(FORWARD, right_front_speed, PERCENT)
                self.rbm.spin(FORWARD, right_back_speed, PERCENT)
                self.log_motor_commands(left_front_speed, left_back_speed, right_front_speed, right_back_speed)
                self.log_drive_commands(fwd_control_percent, strafe_control_percent, turn_control_percent)

            timeout_count += 1
            loop_count += 1
            wait(10, MSEC)

        if not QUIET_MODE:
            print("drive_to_xy: x={}, y={}, heading={}, time={}, timeout={}, settle={}".format(X, Y, THETA, loop_count * 10, is_timeout, is_settle))

        return is_timeout, is_settle

dt = XDriveTrain(left_front_motor, left_back_motor, right_front_motor, right_back_motor)

# + 30mm at 1440mm
# + 22mm at 950mm
# + 19.4mm at 460mm, encoder average reading 501mm
# Distance from back to center = 180mm
# Distance sensor to back = 66mm
# Forward travel
def average_back_distance(samples=10):
    BACK_DISTANCE_SEPARATION = 14 * 25.4 # mm
    back1_distance = 0
    back1_count = 0

    back2_distance = 0
    back2_count = 0

    total_distance = 0
    total_count = 0

    total_angle = 0
    angle_valid = True
    
    for _ in range(samples):
        if back_distance1.is_object_detected():
            back1_value = back_distance1.object_distance(MM)
            back1_count += 1
            back1_distance += back1_value
        else:
            back1_value = None

        if back_distance2.is_object_detected():
            back2_value = back_distance2.object_distance(MM)
            back2_count += 1
            back2_distance += back2_value
        else:
            back2_value = None

        if back1_value is None or back2_value is None: angle_valid = False

        if back1_value is not None and back2_value is not None:
            total_count += 1
            total_distance += (back1_value + back2_value) / 2
        elif back1_value is not None:
            total_count += 1
            total_distance += back1_value
        elif back2_value is not None:
            total_count += 1
            total_distance += back2_value

        if angle_valid and back1_value is not None and back2_value is not None:
            try:
                total_angle += degrees(asin(((back1_value - back2_value)) / BACK_DISTANCE_SEPARATION))
            except:
                #brain.screen.clear_screen(Color.RED)
                #brain.screen.set_cursor(1, 1)
                #brain.screen.print("Error back1={} back2={}\n".format(back1_value, back2_value))
                #print("Error back1={} back2={}\n".format(back1_value, back2_value))
                angle_valid = False

        wait(33, MSEC)
    
    back1_distance = back1_distance / back1_count if back1_count > 0 else 0
    back2_distance = back2_distance / back2_count if back2_count > 0 else 0
    
    total_distance = total_distance / total_count if total_count > 0 else 0

    if not angle_valid: total_angle = 0.0
    total_angle = total_angle / samples

    print("back1 {} back2 {} back distance {} angle {}".format(back1_distance, back2_distance, total_distance, total_angle))
    return total_distance, total_angle

def motor_distance_step(current, previous):
    lf = current[0] - previous[0]
    lb = current[1] - previous[1]
    rf = current[2] - previous[2]
    rb = current[3] - previous[3]
    forward = (lf + lb + rf + rb) / 4
    side = (lf - rf - lb + rb) / 4
    forward = forward * DRIVETRAIN_EXTERNAL_GEAR_RATIO * DRIVETRAIN_WHEEL_SIZE * sqrt(2) * dt.FOWARD_EFFICIENCY
    side = side * DRIVETRAIN_EXTERNAL_GEAR_RATIO * DRIVETRAIN_WHEEL_SIZE * sqrt(2)
    return forward, side

def tracking_distance_step(current, previous):
    if current[0] is None: forward = None
    else:
        forward = current[0] - previous[0]
    if current[1] is None: side = None
    else:
        side = current[1] - previous[1]

    if forward is not None:
        forward = forward * ROTATION_FWD_WHEEL_SIZE
    if side is not None:
        side = side * ROTATION_SIDE_WHEEL_SIZE
    return forward, side

def motor_total_distance():
    lf = left_front_motor.position(TURNS)
    lb = left_back_motor.position(TURNS)
    rf = right_front_motor.position(TURNS)
    rb = right_back_motor.position(TURNS)
    forward, side = motor_distance_step([lf, lb, rf, rb], [0, 0, 0, 0])
    return forward, side

class KalmanXY:
    def __init__(self, X0=0.0, Y0=0.0):
        # State
        self.X = X0
        self.Y = Y0

        # Covariance matrix P
        self.Pxx = 1.0
        self.Pxy = 0.0
        self.Pyy = 1.0

        # Process noise (tune these)
        self.Qx = 0.01
        self.Qy = 0.01

        # Measurement noise (tune per sensor)
        self.Rx = 2.0
        self.Ry = 2.0

    def predict(self, dx, dy):
        # State prediction
        self.X += dx
        self.Y += dy

        # Covariance prediction
        self.Pxx += self.Qx
        self.Pyy += self.Qy
        # Pxy stays the same (no cross‑coupling in motion model)

    def update_x(self, meas_x):
        # Innovation covariance
        S = self.Pxx + self.Rx

        # Kalman gain
        Kx = self.Pxx / S
        Ky = self.Pxy / S

        # Update state
        self.X += Kx * (meas_x - self.X)
        self.Y += Ky * (meas_x - self.Y)

        # Update covariance
        self.Pxx = (1 - Kx) * self.Pxx
        self.Pxy = (1 - Kx) * self.Pxy
        self.Pyy = self.Pyy - Ky * self.Pxy

        return self.X, self.Y

    def update_y(self, meas_y):
        S = self.Pyy + self.Ry

        Kx = self.Pxy / S
        Ky = self.Pyy / S

        self.X += Kx * (meas_y - self.X)
        self.Y += Ky * (meas_y - self.Y)

        self.Pyy = (1 - Ky) * self.Pyy
        self.Pxy = (1 - Ky) * self.Pxy
        self.Pxx = self.Pxx - Kx * self.Pxy

        return self.X, self.Y

previous_motor_positions = [0.0, 0.0, 0.0, 0.0]
previous_rotation_positions = [0.0, 0.0]
previous_theta = THETA
previous_back_distance = [0.0, 0]
previous_left_distance = [0.0, 0]
previous_right_distance = [0.0, 0]

def initialize_wheels():
    global previous_motor_positions, previous_theta

    previous_motor_positions = [left_front_motor.position(TURNS), left_back_motor.position(TURNS), right_front_motor.position(TURNS), right_back_motor.position(TURNS)]
    previous_theta = THETA

def initialize_rotation():
    global previous_rotation_positions
    rotation_fwd.set_position(0, TURNS)
    rotation_side.set_position(0, TURNS)
    previous_rotation_positions = [rotation_fwd.position(TURNS), rotation_side.position(TURNS)]

def initialize_distances():
    global previous_back_distance, previous_left_distance, previous_right_distance

    previous_back_distance[0] = (back_distance1.object_distance(MM) + back_distance2.object_distance(MM)) / 2.0
    previous_back_distance[1] = max(back_distance1.timestamp(), back_distance2.timestamp())
 
    previous_left_distance[0] = left_distance.object_distance(MM)
    previous_left_distance[1] = left_distance.timestamp()

    previous_right_distance[0] = right_distance.object_distance(MM)
    previous_right_distance[1] = right_distance.timestamp()

def filter_distance(heading, tolerance):
    if heading > 360 - tolerance or heading < tolerance:
        return True
    elif heading > 90 - tolerance and heading < 90 + tolerance:
        return True
    elif heading > 180 - tolerance and heading < 180 + tolerance:
        return True
    elif heading > 270 - tolerance and heading < 270 + tolerance:
        return True
    return False
    
def get_back_distance():
    global previous_back_distance

    if not filter_distance(THETA, 5):
        return None

    new_back1_timestamp = back_distance1.timestamp()
    new_back2_timestamp = back_distance2.timestamp()

    max_timestamp = max(new_back1_timestamp, new_back2_timestamp)

    if max_timestamp > previous_back_distance[1]:
        if not back_distance1.is_object_detected() or not back_distance2.is_object_detected(): return None
        new_back1 = back_distance1.object_distance(MM)
        new_back2 = back_distance2.object_distance(MM)
        new_back_distance_value = (new_back1 + new_back2) / 2.0
        new_back_distance_timestamp = max_timestamp
        previous_back_distance[0] = new_back_distance_value
        previous_back_distance[1] = new_back_distance_timestamp

        if new_back_distance_value > 1200.0:
            return None
        
        return new_back_distance_value

    return None

def get_left_distance():
    global previous_left_distance

    if LEFT_DISTANCE_DISTABLE:
        return None

    if not filter_distance(THETA, 5):
        return None

    loader_offset = 0
    if AUTON_SEQUENCE == AutonSequence.MATCH_RIGHT:
        if X >= 260 and X <=360:
            loader_offset = 90

    new_left_timestamp = left_distance.timestamp()

    if new_left_timestamp > previous_left_distance[1]:
        if not left_distance.is_object_detected(): return None
        new_left_distance_value = left_distance.object_distance(MM)
        new_left_distance_timestamp = new_left_timestamp
        previous_left_distance[0] = new_left_distance_value
        previous_left_distance[1] = new_left_distance_timestamp

        if new_left_distance_value > 1700.0:
            return None

        # print(new_left_distance_value)
        return new_left_distance_value + loader_offset

    return None

def get_right_distance():
    global previous_right_distance

    if not filter_distance(THETA, 5):
        return None

    loader_offset = 0
    if AUTON_SEQUENCE == AutonSequence.MATCH_RIGHT:
        if X >= 260 and X <=360:
            loader_offset = 90

    new_right_timestamp = right_distance.timestamp()

    if new_right_timestamp > previous_right_distance[1]:
        if not right_distance.is_object_detected(): return None
        new_right_distance_value = right_distance.object_distance(MM)
        new_right_distance_timestamp = new_right_timestamp
        previous_right_distance[0] = new_right_distance_value
        previous_right_distance[1] = new_right_distance_timestamp

        if new_right_distance_value > 1700.0:
            return None

        # print(new_right_distance_value)
        return new_right_distance_value + loader_offset

    return None

def predict_wheels():
    global previous_motor_positions, previous_theta
    global previous_rotation_positions

    current_motor_positions = [left_front_motor.position(TURNS), left_back_motor.position(TURNS), right_front_motor.position(TURNS), right_back_motor.position(TURNS)]
    current_rotation_positions = [rotation_fwd.position(TURNS), rotation_side.position(TURNS)]
    current_theta = inertial.rotation()

    delta_motor_forward, delta_motor_side = motor_distance_step(current_motor_positions, previous_motor_positions)
    delta_rotation_forward, delta_rotation_side = tracking_distance_step(current_rotation_positions, previous_rotation_positions)
    if delta_rotation_forward is not None:
        delta_forward = delta_rotation_forward
        # positive right offset means recorded radius is smaller than at center of robot for right turns, so add offset
        forward_offset = ROTATION_FWD_WHEEL_OFFSET
    else:
        delta_forward = delta_motor_forward
        forward_offset = 0.0

    if delta_rotation_side is not None:
        delta_side = delta_rotation_side
        # positive forward offset means recorded radius is smaller than at center of robot for forward turns, so subtract offset
        side_offset = -ROTATION_SIDE_WHEEL_OFFSET
    else:
        delta_side = delta_motor_side
        side_offset = 0.0
    delta_theta = current_theta - previous_theta

    if delta_theta == 0.0:
        to_global_rotation_angle = current_theta
        delta_local_x = delta_forward
        delta_local_y = delta_side
    else:
        r_forward = forward_offset + delta_forward / radians(delta_theta) # mm
        r_side = -side_offset + delta_side / radians(delta_theta) # mm

        to_global_rotation_angle = current_theta + delta_theta / 2.0
        delta_local_x = r_forward * 2.0 * sin(radians(delta_theta) / 2.0)
        delta_local_y = r_side * 2.0 * sin(radians(delta_theta) / 2.0)

    delta_global_x = delta_local_x * cos(radians(to_global_rotation_angle)) - delta_local_y * sin(radians(to_global_rotation_angle))
    delta_global_y = delta_local_x * sin(radians(to_global_rotation_angle)) + delta_local_y * cos(radians(to_global_rotation_angle))

    previous_motor_positions = current_motor_positions
    previous_rotation_positions = current_rotation_positions
    previous_theta = current_theta

    new_X = X + delta_global_x
    new_Y = Y + delta_global_y
    new_theta = current_theta

    return new_X, new_Y, new_theta

NORTH = 0
EAST = 1
SOUTH = 2
WEST = 3

def compass_heading(heading):
    heading = heading % 360

    if heading >= 315 or heading < 45:
        return NORTH
    elif heading >= 45 and heading < 135:
        return EAST
    elif heading >= 135 and heading < 225:
        return SOUTH
    else:
        return WEST

ENABLE_BACK_DISTANCE = True
ENABLE_LEFT_DISTANCE = not CALIBRATION
ENABLE_RIGHT_DISTANCE = True

def odom_distance_enable(back, left, right):
    global ENABLE_BACK_DISTANCE, ENABLE_LEFT_DISTANCE, ENABLE_RIGHT_DISTANCE

    ENABLE_BACK_DISTANCE = back
    ENABLE_LEFT_DISTANCE = left
    ENABLE_RIGHT_DISTANCE = right

def odom_print():
    if not QUIET_MODE:
        print("X: {:4.0f}/{:0.1f}, Y: {:4.0f}/{:0.1f}, H: {:3.2f}".format(X, Pxx, Y, Pyy, THETA % 360.0))


def odom_thread():
    global X, Y, THETA, Pxx, Pyy
    THETA = inertial.rotation()
    initialize_wheels()
    initialize_rotation()
    initialize_distances()
    filter = KalmanXY(X, Y)

    count = 0
    while True:
        X, Y, THETA = predict_wheels()
        filter.predict(X - filter.X, Y - filter.Y)

        # the walls can only be guaranteed unobstructed during autonomous
        heading = compass_heading(THETA)

        meas_back_distance = get_back_distance()
        if ENABLE_BACK_DISTANCE and meas_back_distance is not None:
            meas_back_distance += HIDDEN_PERIMITER + ROBOT_LENGTH / 2 - BACK_DISTANCE_FROM_BACK
            if heading == NORTH:
                X, Y = filter.update_x(meas_back_distance)
            elif heading == SOUTH:
                X, Y = filter.update_x(3600.0 - meas_back_distance)
            elif heading == WEST:
                X, Y = filter.update_y(3600.0 - meas_back_distance)
            elif heading == EAST:
                X, Y = filter.update_y(meas_back_distance)

        meas_right_distance = get_right_distance()
        if ENABLE_RIGHT_DISTANCE and meas_right_distance is not None:
            meas_right_distance += HIDDEN_PERIMITER + ROBOT_WIDTH / 2 - RIGHT_DISTANCE_FROM_RIGHT
            if heading == NORTH:
                X, Y = filter.update_y(3600.0 - meas_right_distance)
            elif heading == SOUTH:
                X, Y = filter.update_y(meas_right_distance)
            elif heading == WEST:
                X, Y = filter.update_x(3600.0 - meas_right_distance)
            elif heading == EAST:
                X, Y = filter.update_x(meas_right_distance)

        meas_left_distance = get_left_distance()
        if ENABLE_LEFT_DISTANCE and meas_left_distance is not None:
            meas_left_distance += HIDDEN_PERIMITER + ROBOT_WIDTH / 2 - LEFT_DISTANCE_FROM_LEFT
            if heading == NORTH:
                X, Y = filter.update_y(meas_left_distance)
            elif heading == SOUTH:
                X, Y = filter.update_y(3600.0 - meas_left_distance)
            elif heading == WEST:
                X, Y = filter.update_x(meas_left_distance)
            elif heading == EAST:
                X, Y = filter.update_x(3600.0 - meas_left_distance)

        Pxx, Pyy = filter.Pxx, filter.Pyy

        if count % 200 == 0:
            odom_print()

        count += 1

        wait(10, MSEC)

def log_drivetrain():
    global QUIET_MODE

    motors = [left_front_motor, left_back_motor, right_front_motor, right_back_motor]
    headers = ["lfm", "lbm", "rfm", "rbm"]
    units = ["CMD", "POS", "VEL", "TRQ"]

    log = []

    # Run ramp test

    TOTAL_SAMPLES = 400
    PRINT_DELAY = 250 # ms between samples. Set to around 250 for wireless or 50 for USB

    for i in range(TOTAL_SAMPLES):

        entry = [inertial.rotation(DEGREES), (back_distance1.object_distance(MM)+back_distance2.object_distance(MM))/2]
        for motor, cmd_vel in zip(motors, [dt.last_lfm_command_vel, dt.last_lbm_command_vel, dt.last_rfm_command_vel, dt.last_rbm_command_vel]):
            entry.append(cmd_vel)
            if motor is not None:
                entry.append(motor.position(RotationUnits.REV))
                entry.append(motor.velocity(VelocityUnits.PERCENT))
                torque_percent = motor.torque(TorqueUnits.NM) / DRIVETRAIN_MAX_TORQUE * 100.0
                entry.append(torque_percent)
            else:
                entry.append(0.0)
                entry.append(0.0)
                entry.append(0.0)

        log.append(entry)
        wait (10, MSEC)

    QUIET_MODE = True
    wait(100, MSEC)

    output = "idx, heading, dist, "
    for j in range(0, len(headers)):
        header = headers[j]
        for k in range(0, len(units)):
            unit = units[k]
            if j == len(headers) - 1 and k == len(units) - 1:
                output += "{} {}".format(header, unit)
            else:
                output += "{} {}, ".format(header, unit)
    print(output)

    for i in range(TOTAL_SAMPLES):
        log_entry = log[i]
        log_length = len(log_entry)
        output = "{}, ".format(i)

        for j in range(0, log_length):
            if j < log_length - 1:
                output += "{:0.1f}, ".format(log_entry[j])
            else:
                output += "{:0.1f}".format(log_entry[j])

        print(output)
        wait(PRINT_DELAY, MSEC)


def log_odom():
    global QUIET_MODE

    log = []

    # Run ramp test

    TOTAL_SAMPLES = 450
    PRINT_DELAY = 333 # ms between samples. Set to around 250 for wireless or 50 for USB

    for i in range(TOTAL_SAMPLES):

        entry = [
            dt.last_fwd_command,
            dt.last_strafe_command,
            dt.last_turn_command,
            previous_rotation_positions[0] * ROTATION_FWD_WHEEL_SIZE,
            previous_rotation_positions[1] * ROTATION_FWD_WHEEL_SIZE,
            previous_left_distance[0],
            previous_right_distance[0],
            previous_back_distance[0],
            X,
            Y,
            THETA,
            Pxx,
            Pyy]

        log.append(entry)
        wait (10, MSEC)

    QUIET_MODE = True
    wait(100, MSEC)

    output = "idx, drive, strafe, turn, fwd, side, left, right, back, X, Y, THETA, Pxx, Pyy"
    print(output)

    for i in range(TOTAL_SAMPLES):
        log_entry = log[i]
        log_length = len(log_entry)
        output = "{}, ".format(i)

        for j in range(0, log_length):
            if j < log_length - 1:
                output += "{}, ".format(log_entry[j])
            else:
                output += "{}".format(log_entry[j])

        print(output)
        wait(PRINT_DELAY, MSEC)

# ------------------------------------------------------------ #
### AUTONOMOUS ROUTINES
# ------------------------------------------------------------ #

def autonomous_calibration():
    global DISTANCE_ENABLED

    # Thread(odom_thread)
    # Thread(log_drivetrain)
    # Thread(log_odom)
    # place automonous code here
    starting_distance, starting_angle = average_back_distance()
    #print("Back distance: {}".format(starting_distance))
    odom_distance_enable(True, False, True)

    wait(100, MSEC)

    # drive_for(1200, False, 50, heading = 0)
    # wait(100, MSEC)
    # return

    speed = 33

    if True:
        dt.drive_to_xy(300.0, 2750.0, False, speed, heading = 0)

        while True:

            use_distance = True
            odom_distance_enable(True, False, True)
            wait(500, MSEC)
            if not use_distance: odom_distance_enable(False, False, False)

            overtemp = False
            for motors in [left_front_motor, left_back_motor, right_front_motor, right_back_motor]:
                if motors.temperature(PERCENT) > 50:
                    overtemp = True

            if overtemp:
                dt.stop_all()
                wait(1, SECONDS)
                continue

            dt.drive_to_xy(300.0, 2750 + 400.0, True, speed, heading = 0)
            #wait(500, MSEC)
            dt.drive_to_xy(300.0 + 200.0, 2750 + 400.0, False, speed, heading = 0)
            #wait(500, MSEC)
            dt.drive_to_xy(300.0 + 200.0, 2750 - 400.0, True, speed, heading = 0)
            #wait(500, MSEC)
            dt.drive_to_xy(300.0 + 100, 2750 - 400.0, False, speed, heading = 0)
            #wait(500, MSEC)
            if use_distance: odom_distance_enable(False, False, False)
            dt.turn_for(-90, speed)
            if use_distance: odom_distance_enable(True, True, False)
            #wait(500, MSEC)
            Thread(log_odom)
            wait(100, MSEC)
            dt.drive_to_xy(300.0 + 100, 2750 + 400.0, False, speed, heading = -90)
            #wait(500, MSEC)
            dt.drive_to_xy(300.0 + 100, 2750 - 400.0, False, speed, heading = -90)
            #wait(500, MSEC)
            if use_distance: odom_distance_enable(False, False, False)
            dt.turn_for(90, speed)
            if use_distance: odom_distance_enable(True, False, True)
            #wait(500, MSEC)
            dt.drive_to_xy(300.0, 2750 - 400.0, False, speed, heading = 0)


    while True:
        dt.drive_to_xy(900.0, 1800.0, False, 66, heading = 0)
        #wait(500, MSEC)
        dt.drive_to_xy(900.0, 700.0, True, 66, heading = 0)
        #wait(500, MSEC)
        dt.drive_to_xy(300.0, 700.0, False, 66, heading = 0)
        #wait(500, MSEC)
        dt.drive_to_xy(300.0, 1800.0, True, 66, heading = 0)
        #wait(500, MSEC)
        # break
    # ending_distance = average_back_distance()
    # print("Back distance: {}".format(ending_distance))
    # print("Back distance delta: {}".format(ending_distance - starting_distance))
    # print("Odometer distance: {}".format(motor_total_distance()))
    # print("Rotation: {}".format(inertial.rotation()))

def autonomous_skills():
    # Thread(odom_thread)
    # place automonous code here
    while not CLAW_INITIALIZED:
        wait(10, MSEC)
    wait(1, SECONDS)
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_MID1)
    wait(500, MSEC)
    # Thread(log_drivetrain)
    dt.drive_for(51 * 25.4, False, 50, heading = 0)
    dt.drive_for(-450, True, 50, heading = 0)
    command_lift(13)
    dt.drive_for(11 * 25.4, False, 50, timeout = 5000, heading = 0)
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_MID2)
    command_lift(10)
    wait(500, MSEC)
    open_claw()
    dt.drive_for(-150, False, 50, heading = 0)
    # TODO: Move back to safe distance

def autonomous_none():
    # place automonous code here
    lower_toggle()
    dt.drive_for(50, False, 50, heading = 0)
    dt.drive_for(-50, False, 50, heading = 0)
    dt.drive_for(50, False, 50, heading = 0)
    dt.drive_for(-50, False, 50, heading = 0)
    dt.drive_for(100, False, 50)

def claw_move1():
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_MID1)
    command_lift(5)

def claw_move2():
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_DOWN)
    command_lift(0)

# Score 7 pins, 3 goals with 2 pins
def autonomous_left():
    # place automonous code here

    lower_toggle()
    dt.drive_for(50, False, 100, heading = 0)
    dt.drive_for(-50, False, 100, heading = 0)
    dt.drive_for(50, False, 100, heading = 0)
    dt.drive_for(-50, False, 100, heading = 0)
    raise_toggle()

    Thread(claw_move1)
    wait(250, MSEC)

    dt.drive_to_xy(300, 1800, False, 66, heading = 0)
    odom_print()
    dt.drive_to_xy(300, 2400, True, 66, heading = 0)
    odom_print()

    dt.drive_for(100, False, 50, heading = 0)
    command_lift(3)
    open_claw()
    wall_distance = average_back_distance()[0] - BACK_DISTANCE_FROM_BACK
    print("Wall distance: {}".format(wall_distance))
    target_distance = 120
    reverse_by = target_distance - wall_distance
    dt.drive_for(reverse_by, False, 50, heading = 0)
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_DOWN)
    command_lift(0)
    current_heading = inertial.rotation()
    print("Current heading: {}".format(current_heading))
    target_heading = 180
    dt.turn_for(target_heading - current_heading, 100)
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_MID3)
    wait(250, MSEC)
    close_claw()
    command_lift(5)
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_DOWN)
    current_heading = inertial.rotation()
    print("Current heading: {}".format(current_heading))
    target_heading = 0
    dt.turn_for(target_heading - current_heading, 100)
    command_lift(11)
    dt.drive_for(-reverse_by+20, False, 50, heading = 0)
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_MID1)
    wait(250, MSEC)
    command_lift(9)
    open_claw()
    wait(250, MSEC)
    dt.drive_for(reverse_by, False, 50, heading = 0)
    Thread(claw_move2)
    dt.drive_to_xy(300, 1200, False, 100, heading = 0)

def autonomous_right():
    # place automonous code here

    lower_toggle()
    dt.drive_for(50, False, 50, heading = 0)
    dt.drive_for(-50, False, 50, heading = 0)
    dt.drive_for(50, False, 50, heading = 0)
    dt.drive_for(-50, False, 50, heading = 0)
    raise_toggle()

    Thread(claw_move1)
    wait(250,MSEC)
    dt.drive_for(100, False, 50, heading = 0)
    dt.drive_for(-700, True, 50, heading = 0)
    dt.drive_for(100, False, 50, heading = 0)
    command_lift(3)
    open_claw()
    wall_distance = average_back_distance()[0] - BACK_DISTANCE_FROM_BACK
    target_distance = 120 
    reverse_by = target_distance - wall_distance
    dt.drive_for(reverse_by, False, 50, heading = 0)
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_DOWN)
    command_lift(0)
    current_heading = inertial.rotation()
    target_heading = 180
    dt.turn_for(target_heading - current_heading, 66)
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_MID3)
    wait(250, MSEC)
    close_claw()
    command_lift(5)
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_DOWN)
    current_heading = inertial.rotation()
    target_heading = 0
    dt.turn_for(target_heading - current_heading, 66)
    command_lift(11)
    dt.drive_for(-reverse_by+20, False, 50, heading = 0)
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_MID1)
    wait(250, MSEC)
    command_lift(9)
    open_claw()
    wait(250, MSEC)
    dt.drive_for(reverse_by, False, 50, heading = 0)
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_DOWN)
    command_lift(0)

def autonomous():
    global ROBOT_ENABLED
    while not ROBOT_INITIALIZED:
        wait(100, MSEC)
    ROBOT_ENABLED = True

    Thread(initialize_claw)

    if CALIBRATION:
        autonomous_calibration()
        return
    if AUTON_SEQUENCE == AutonSequence.SKILLS:
        autonomous_skills()
    elif AUTON_SEQUENCE == AutonSequence.MATCH_LEFT:
        autonomous_left()
    elif AUTON_SEQUENCE == AutonSequence.MATCH_RIGHT:
        autonomous_right()
    else:
        autonomous_none()

pitch_offset = 0.0

def connection_checker():
    while True:
        cleared = False
        for sensor, name in zip(all_sensors, all_sensors_names):
            if not sensor.installed():
                if not cleared:
                    brain.screen.clear_screen(Color.RED)
                    cleared = True
                brain.screen.print("Sensor {} is not connected!".format(name))
                brain.screen.new_line()
        for motor, name in zip(all_motors, all_motor_names):
            if not motor.installed():
                if not cleared:
                    brain.screen.clear_screen(Color.RED)
                    cleared = True
                brain.screen.print("Motor {} is not connected!".format(name))
                brain.screen.new_line()
        wait(1000, MSEC)

def pre_autonomous():
    global ROBOT_INITIALIZED
    global ALLIANCE_COLOR, AUTON_SEQUENCE
    global pitch_offset
    global motor_monitor
    # actions to do when the program starts
    brain.screen.clear_screen()
    brain.screen.print("pre auton code")
    inertial.calibrate()
    robot_config.load_settings()
    while inertial.is_calibrating():
        wait(100, MSEC)

    Thread(connection_checker)

    for i in range(10):
        pitch_offset += inertial.orientation(OrientationType.ROLL, DEGREES)
        wait(10, MSEC)
    pitch_offset /= 10.0
    print("Pitch offset: {:.1f}".format(pitch_offset))

    ROBOT_INITIALIZED = True

    Thread(odom_thread)

    ui = PreAutonUI(brain, ALLIANCE_COLOR, AUTON_SEQUENCE)
    ui.start()
    while (not ROBOT_ENABLED):
        ALLIANCE_COLOR, AUTON_SEQUENCE = ui.get_current_selection()
        wait(10, MSEC)
    ui.stop()

    motor_monitor = MotorMonitor(brain, all_motors, all_motor_names)
    motor_monitor.start()

# ------------------------------------------------------------ #
### USER LIFT AND CLAW CONTROL FUNCTIONS
# ------------------------------------------------------------ #

def StopLift():
    global lift_thread, lift_hold_time_start, LIFT_RUNNING, LIFT_HOLDING
    if LIFT_RUNNING:
        print("Was Running")
        if lift_thread is not None: lift_thread.stop()
        lift_motor.stop(HOLD)
        LIFT_HOLDING = True
        LIFT_RUNNING = False
        lift_hold_time_start = brain.timer.time(SECONDS)
        return True
    return False

def OnLowerLiftPressed(): # R2
    global lift_thread
    if not ROBOT_ENABLED: return
    if StopLift(): return
    lift_thread = Thread(lower_lift)

def OnRaiseLiftPressed(): # R1
    global lift_thread
    if not ROBOT_ENABLED: return
    if StopLift(): return
    lift_thread = Thread(raise_lift)

def OnLowerClawPressed(): # L2
    if not ROBOT_ENABLED: return
    if CLAW_ARM_RUNNING:
        print("Was Running")
        # claw_arm_motor1.stop(HOLD)
        # claw_arm_motor2.stop(HOLD)
        run_claw_arm(CLAW_ARM_COMMAND_CANCEL)
        return

    pressed_counter = 0
    while pressed_counter < 5: # about 1/4 second
        wait(50, MSEC)
        if not controller_1.buttonL2.pressing():
            thread = Thread(lower_claw_arm)
            return
        pressed_counter += 1

    thread = Thread(move_claw_arm_to_position, (CLAW_ARM_DOWN, 0))

def OnRaiseClawPressed(): # L1
    if not ROBOT_ENABLED: return
    if CLAW_ARM_RUNNING:
        print("Was Running")
        #claw_arm_motor1.stop(HOLD)
        #claw_arm_motor2.stop(HOLD)
        run_claw_arm(CLAW_ARM_COMMAND_CANCEL)
        return

    pressed_counter = 0
    while pressed_counter < 5: # about 1/4 second
        wait(50, MSEC)
        if not controller_1.buttonL1.pressing():
            thread = Thread(raise_claw_arm)
            return
        pressed_counter += 1

    thread = Thread(move_claw_arm_to_position, (CLAW_ARM_UP, 0))

def OnControlButtonAPressed():
    if not ROBOT_ENABLED: return
    if claw_is_open():
        close_claw()
    else:
        open_claw()
    StopLift()

def OnControlButtonBPressed():
    if not ROBOT_ENABLED: return
    if toggle_raised() and claw_arm_current_position() < CLAW_ARM_MID3:
        lower_toggle()
    else:
        raise_toggle()

def OnControlButtonUpPressed():
    global ROBOT_ENABLED

    pressed_counter = 0
    while pressed_counter < 30:
        wait(100, MSEC)
        if not controller_1.buttonUp.pressing():
            return
        pressed_counter += 1
    # Button has been held for 30 cycles (3 seconds)
    ROBOT_ENABLED = False
    if motor_monitor is not None: motor_monitor.mute(True)
    robot_config.configuration_UI()
    ROBOT_ENABLED = True
    if motor_monitor is not None:
        motor_monitor.mute(False)
        motor_monitor.refresh()

samples = []

def add_sample():
    if len(samples) >= 200: return True
    samples.append([
        inertial.orientation(OrientationType.ROLL, DEGREES),
        inertial.orientation(OrientationType.PITCH, DEGREES),
        inertial.acceleration(AxisType.XAXIS),
        inertial.acceleration(AxisType.YAXIS),
        inertial.acceleration(AxisType.ZAXIS),
        inertial.gyro_rate(AxisType.XAXIS, DPS),
        inertial.gyro_rate(AxisType.YAXIS, DPS),
        inertial.gyro_rate(AxisType.ZAXIS, DPS)
    ])
    return False

def dump_samples_thread():
    print("Roll,Pitch,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ")
    for sample in samples:
        print("{:0.2f},{:0.2f},{:0.2f},{:0.2f},{:0.2f},{:0.2f},{:0.2f},{:0.2f}".format(*sample))
        wait(333, MSEC)

dumped = False
def dumpsamples():
    global dumped
    if dumped: return
    dumped = True
    thread = Thread(dump_samples_thread)

# Default maximum drive and turn rates
DEFAULT_TURN_MAX = 100.0 # maximum turn rate
DEFAULT_DRIVE_MAX = 100.0 # maximum drive rate
# Default ramp control limit
DEFAULT_MAX_CONTROL_RAMP = 5.0 # percent per timestep (assumed to be 10ms)

# Default detwitch control
DEFAULT_PIVOT_MAX_TURN_SPEED = 33.0
DEFAULT_PIVOT_MIN_DRIVE_SPEED = 33.0
DEFAULT_FULL_TURN_DRIVE_SPEED = 66.0

turn_max = DEFAULT_TURN_MAX
drive_max = DEFAULT_DRIVE_MAX
ramp_max = DEFAULT_MAX_CONTROL_RAMP
pivot_max_turn = DEFAULT_PIVOT_MAX_TURN_SPEED  # maximum turn speed during pivot
pivot_min_drive_speed = DEFAULT_PIVOT_MIN_DRIVE_SPEED # drive speed at which to start increasing turn rate
full_turn_drive_speed = DEFAULT_FULL_TURN_DRIVE_SPEED # drive speed at which to use full turn rate

def drivetrain_detwitch(speed, turn, detwitch, enabled):
    '''
    ### (INTERNAL) )ETWITCH - reduce turn sensitiviy when robot is moving slowly (turning in place)

    NOTE: speed is not altered only turn

    :param speed: speed in percent - from -100 to +100 (full reverse to full forward)
    :param turn: turn in percent - from -100 to +100 (full left turn to full right turn)
    :param detwitch: indicates whether to apply detwitching based on speed
    :param enabled: indicates whether to enable the detwitch code or not

    :return: speed (unmodified) and turn based on simple straight line segments
    '''

    if not enabled:
        return speed * drive_max / 100.0, turn * turn_max / 100.0

    if abs(speed) < pivot_min_drive_speed:
        if abs(detwitch) < 10:
            turn_scale = pivot_max_turn / 100.0
            turn = turn * turn_scale
            turn_expo = ((turn / pivot_max_turn) ** 2) * pivot_max_turn
            if turn < 0: turn_expo = -turn_expo
            return speed * drive_max / 100.0, turn_expo

        detwitch_scale = 2 * abs(detwitch) / 100.0
        if detwitch_scale > 1.0: detwitch_scale = 1.0

        a = pivot_max_turn / 100.0
        b = ((turn_max - pivot_max_turn) / 100.0)
        turn_scale = a + b * detwitch_scale

        return speed * drive_max / 100.0, turn * turn_scale

    # Region 1: below minimum drive speed - use minimum turn rate
    turn_scale = pivot_max_turn / 100.0 # start off with minimum turn rate
    # Region 2: between minimum drive speed and full turn drive speed - linearly increase turn rate
    if (abs(speed) >= pivot_min_drive_speed and abs(speed) < full_turn_drive_speed):
        # linearly increase the turn rate between the drive speed setpoints using a straight line equation
        #  y = a + b * x
        a = pivot_max_turn / 100.0
        b = ((turn_max - pivot_max_turn) / 100.0) / (full_turn_drive_speed - pivot_min_drive_speed)
        turn_scale = a + b * (abs(speed) - pivot_min_drive_speed)
    # Region 3: above full turn drive speed - use full turn rate
    elif (abs(speed) >= full_turn_drive_speed):
        turn_scale = turn_max / 100.0

    turn = turn * turn_scale
    speed = speed * drive_max / 100.0

    return speed, turn

CONTROLLER_DEADBAND = 5

def apply_deadband(value, deadband=CONTROLLER_DEADBAND):
    if abs(value) < deadband:
        value = 0
    elif value > 0:
        value = (value - deadband) * 100/ (100 - deadband)
    else:
        value = (value + deadband) * 100 / (100 - deadband)
    return value

MAX_ROTATION_PER_SECOND = 360
AUTO_TURN_KP = 1.5 # 2.0 # starting 0.25
AUTO_TURN_KD = 2.5 # 10.0
NO_INPUT_TIMEOUT = 250

# ------------------------------------------------------------ #
### DRIVER CONTROL MOTION MODEL
# ------------------------------------------------------------ #

MOTOR_FREE_SPEED_RPM = 600.0 # 6:1 cartridge
# Distance the wheel surface travels per motor revolution
MM_PER_MOTOR_REV_SURFACE = DRIVETRAIN_EXTERNAL_GEAR_RATIO * DRIVETRAIN_WHEEL_SIZE
# 45 degree rollers mean the chassis travels sqrt(2) times that (matches motor_distance_step)
MM_PER_MOTOR_REV_CHASSIS = MM_PER_MOTOR_REV_SURFACE * sqrt(2) * dt.FOWARD_EFFICIENCY
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

    def __init__(self):
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
        self._last_time = brain.timer.time(MSEC)
        self._last_fwd = rotation_fwd.position(TURNS)
        self._last_side = rotation_side.position(TURNS)
        self._last_theta = inertial.rotation(DEGREES)

    def _measure(self, dt):
        lf = left_front_motor.velocity(RPM)
        lb = left_back_motor.velocity(RPM)
        rf = right_front_motor.velocity(RPM)
        rb = right_back_motor.velocity(RPM)

        # Same mixing as the motor commands, expressed as a rate
        self.wheel_forward = (lf + lb + rf + rb) / 4.0 / 60.0 * MM_PER_MOTOR_REV_CHASSIS
        self.wheel_strafe = (lf - lb - rf + rb) / 4.0 / 60.0 * MM_PER_MOTOR_REV_CHASSIS
        self.wheel_turn = degrees((lf + lb - rf - rb) / 4.0 / 60.0 * MM_PER_MOTOR_REV_SURFACE / DRIVE_RADIUS)

        theta = inertial.rotation(DEGREES)
        turn_rate = (theta - self._last_theta) / dt
        self._last_theta = theta
        self.measured_turn += (turn_rate - self.measured_turn) * self.RATE_FILTER

        fwd = rotation_fwd.position(TURNS)
        fwd_rate = (fwd - self._last_fwd) * ROTATION_FWD_WHEEL_SIZE / dt
        self._last_fwd = fwd
        # the tracking wheel sits right of centre, so rotation shows up there as forward travel
        fwd_rate += ROTATION_FWD_WHEEL_OFFSET * radians(self.measured_turn)
        self.measured_forward += (fwd_rate - self.measured_forward) * self.RATE_FILTER

        side = rotation_side.position(TURNS)
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
        if not robot_config.enable_motion_model:
            return forward, strafe, turn

        now = brain.timer.time(MSEC)
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
        if not robot_config.enable_motion_model:
            return

        worst = 0.0
        for command, motor in ((left_front, left_front_motor), (left_back, left_back_motor),
                               (right_front, right_front_motor), (right_back, right_back_motor)):
            if abs(command) < self.COMMAND_THRESHOLD:
                continue
            actual = motor.velocity(PERCENT)
            shortfall = command - actual if command > 0 else actual - command
            if shortfall > worst: worst = shortfall

        self.follow_error = worst
        self.saturated = worst > self.WHEEL_FOLLOW_ERROR

def user_control():
    global ROBOT_ENABLED

    # place driver control in this while loop
    last_fwd = 0 
    while not ROBOT_INITIALIZED:
        wait(100, MSEC)

    Thread(initialize_claw)

    starting_distance, starting_angle = average_back_distance()
    print("Back distance: {}, Back angle: {}".format(starting_distance, starting_angle))

    controller_1.buttonA.pressed(OnControlButtonAPressed)
    controller_1.buttonB.pressed(OnControlButtonBPressed)

    controller_1.buttonR2.pressed(OnLowerLiftPressed)
    controller_1.buttonR1.pressed(OnRaiseLiftPressed)

    controller_1.buttonL2.pressed(OnLowerClawPressed)
    controller_1.buttonL1.pressed(OnRaiseClawPressed)

    controller_1.buttonUp.pressed(OnControlButtonUpPressed)

    # brain.timer.event(check_lift_hold, 10000)

    rotation_set = inertial.rotation(DEGREES)
    no_drive_timeout = NO_INPUT_TIMEOUT / 10 # timeout in ms, want in 10ms loops
    no_turn_timeout = NO_INPUT_TIMEOUT / 10 # timeout in ms, want in 10ms loops
    drive_active = False
    turn_active = False
    anti_tilt_active = False
    anti_tilt_timer = 10

    # ramp control
    last_forward = 0
    last_strafe = 0
    last_turn_error = 0

    motion_model = DriveMotionModel()

    left_front_motor.set_stopping(COAST)
    left_back_motor.set_stopping(COAST)
    right_front_motor.set_stopping(COAST)
    right_back_motor.set_stopping(COAST)

    # initialize_lift()
    Thread(auto_claw_thread)
    # Thread(odom_thread)

    ROBOT_ENABLED = True

    loop_count = 0

    # Thread(log_drivetrain)

    # place driver control in this while loop
    while True:
        # if add_sample():
        #     dumpsamples()

        if not ROBOT_ENABLED:
            wait(100, MSEC)
            continue
            
        raw_forward = apply_deadband(controller_1.axis3.position())
        raw_strafe = apply_deadband(controller_1.axis4.position())
        raw_turn = apply_deadband(controller_1.axis1.position())
        raw_detwitch = apply_deadband(controller_1.axis2.position())

        # Remap from field to robot
        FIELD_ORIENTED = robot_config.enable_field_orient
        if FIELD_ORIENTED:
            robot_forward = raw_forward * cos(radians(inertial.rotation(DEGREES))) + raw_strafe * sin(radians(inertial.rotation(DEGREES)))
            robot_strafe = -raw_forward * sin(radians(inertial.rotation(DEGREES))) + raw_strafe * cos(radians(inertial.rotation(DEGREES)))
        else:
            robot_forward = raw_forward
            robot_strafe = raw_strafe

        raw_forward = robot_forward
        raw_strafe = robot_strafe

        MAX_RANP = 2 if robot_config.enable_slow_ramp else 3
        MIN_RAMP = 1
        RAMP_RANGE = MAX_RANP - MIN_RAMP

        # Ramp control - forward
        ramp_max = MAX_RANP - RAMP_RANGE * lift_height(percent=True) / 100
        safe_forward = dt.ramp_limit(raw_forward, last_forward, ramp_max)
        forward = safe_forward
        last_forward = forward

        # Ramp control - strafe
        ramp_max = MAX_RANP - RAMP_RANGE * lift_height(percent=True) / 100
        safe_strafe = dt.ramp_limit(raw_strafe, last_strafe, ramp_max)
        strafe = safe_strafe
        last_strafe = strafe

        turn = drivetrain_detwitch(forward, raw_turn, raw_detwitch, True)[1]

        TILT_ENABLE = False

        # Tilt detection
        forward_tilt = -inertial.orientation(OrientationType.ROLL, DEGREES)
        sideways_tilt = inertial.orientation(OrientationType.PITCH, DEGREES)
        if TILT_ENABLE and (abs(forward_tilt) > 10 or abs(sideways_tilt) > 10):
            if not LIFT_RUNNING:
                # print("Tilting! Forward: {}, Sideways: {}".format(forward_tilt, sideways_tilt))
                thread = Thread(lower_lift)

        #  print("{:.1f}".format(auto_forward))

        # Driving vs. coasting logic - cancels any auto corrections after timeout
        if turn:
            no_turn_timeout = NO_INPUT_TIMEOUT / 10
        else:
            no_turn_timeout -= 1
            if no_turn_timeout < 0:
                no_turn_timeout = 0

        if forward or strafe:
            no_drive_timeout = NO_INPUT_TIMEOUT / 10
        else:
            no_drive_timeout -= 1
            if no_drive_timeout < 0:
                no_drive_timeout = 0

        # Hold onto turn or drive commands for a short period after input stops (including ramped inputs)
        # as inertia will keep robot moving briefly
        turn_active = turn or no_turn_timeout > 0
        drive_active = forward or strafe or no_drive_timeout > 0

        no_input = not turn_active and not drive_active
        if no_input:
            left_front_motor.stop(COAST)
            left_back_motor.stop(COAST)
            right_front_motor.stop(COAST)
            right_back_motor.stop(COAST)
            rotation_set = inertial.rotation(DEGREES)
            motion_model.reset()
            all_stop = True
        else:
            all_stop = False

        # Heading hold
        # rotation_set += (raw_turn / 100) * (MAX_ROTATION_PER_SECOND / 100)
        # Case 1: Active turn input - overrides drive active
        if turn_active:
            rotation_set = inertial.rotation(DEGREES)
            auto_turn = 0
            last_turn_error = 0
        # Case 2: Forward or strafe still active
        elif robot_config.enable_heading_hold and drive_active: # (forward != 0 or strafe != 0):
            turn_error = rotation_set - inertial.rotation(DEGREES)
            auto_turn = turn_error * AUTO_TURN_KP + (turn_error - last_turn_error) * AUTO_TURN_KD
            last_turn_error = turn_error
        # Case 3: Coasting with no input
        else:
            rotation_set = inertial.rotation(DEGREES)
            auto_turn = 0
            last_turn_error = 0

        combined_forward, combined_strafe, combined_turn = motion_model.update(forward, strafe, turn + auto_turn)
        
        if not all_stop:
            left_power = 1.0 # LEFT_POWER_SCALING
            right_power = 1.0 # RIGHT_POWER_SCALING
            front_power = 1.0 # FRONT_POWER_SCALING
            back_power = 1.0 # BACK_POWER_SCALING

            # mixing the combined forward, strafe, and turn inputs to calculate individual motor speeds
            left_front_speed = combined_forward * right_power + combined_turn + combined_strafe * front_power
            left_back_speed = combined_forward * left_power + combined_turn - combined_strafe * back_power
            right_front_speed = combined_forward * left_power - combined_turn - combined_strafe * front_power
            right_back_speed = combined_forward * right_power - combined_turn + combined_strafe * back_power

            # check for saturation and scale motor speeds if necessary
            max_raw_speed = max(abs(left_front_speed), abs(left_back_speed), abs(right_front_speed), abs(right_back_speed))
            if max_raw_speed > 100:
                left_front_speed = left_front_speed * (100 / max_raw_speed)
                left_back_speed = left_back_speed * (100 / max_raw_speed)
                right_front_speed = right_front_speed * (100 / max_raw_speed)
                right_back_speed = right_back_speed * (100 / max_raw_speed)

            # set resulting velocities for each motor and spin
            left_front_motor.set_velocity(left_front_speed, PERCENT)
            left_back_motor.set_velocity(left_back_speed, PERCENT)
            right_front_motor.set_velocity(right_front_speed, PERCENT)
            right_back_motor.set_velocity(right_back_speed, PERCENT)

            left_front_motor.spin(FORWARD)
            left_back_motor.spin(FORWARD)
            right_front_motor.spin(FORWARD)
            right_back_motor.spin(FORWARD)

            motion_model.observe_wheels(left_front_speed, left_back_speed, right_front_speed, right_back_speed)

        wait(10, MSEC)

        # if loop_count % 100 == 0:
            #print("Lift {}".format(lift_motor.position(DEGREES)))
            # print("Rotation: {:.1f}".format(inertial.rotation(DEGREES)))

        loop_count += 1

# create competition instance
comp = Competition(user_control, autonomous)
pre_autonomous()