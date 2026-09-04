# ----------------------------------------------------------------------------- #
#                                                                               #                                    
#    Project:        Aurora                                       #                             
#    Module:         main.py                                                    #
#    Author:         VEX                                                        #
#    Created:        Fri Aug 05 2022                                            #
#    Description:    This example will use the left Y and right X               #
#                    Controller axis to control the Clawbot.                    #
#                                                                               #                                    
#    Configuration:                                 #
#                                                                               #                                                                          
# ----------------------------------------------------------------------------- #

# Library imports
from vex import *
from math import radians, cos, sin, sqrt
import json

from v5pythonlibrary import * # Loaded from SDCard

# ------------------------------------------------------------ #
### SETUP DEFAULT ALLIANCE AND AUTONOMOUS SEQUENCE HERE
# ------------------------------------------------------------ #

CALIBRATION = False

# ALLIANCE_COLOR = AllianceColor.RED
ALLIANCE_COLOR = AllianceColor.BLUE

# AUTON_SEQUENCE = AutonSequence.SKILLS
# AUTON_SEQUENCE = AutonSequence.MATCH_LEFT
AUTON_SEQUENCE = AutonSequence.MATCH_RIGHT
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

inertial = InertialWrapper(Ports.PORT5, 183/180)
claw_distance = Distance(Ports.PORT2)

HIDDEN_PERIMITER = 15 #mm

BACK_DISTANCE_COMPENSATION = 1500 / 1525
BACK_DISTANCE_FROM_BACK = 66 # mm
back_distance1 = Distance(Ports.PORT4)
back_distance2 = Distance(Ports.PORT9)

ROBOT_WIDTH = 15 * 25.4 # (mm)
ROBOT_LENGTH = 185 * 2 # (mm) 185 measured from back wall to center line

LEFT_DISTANCE_COMPENSATION = 1.0
LEFT_DISTANCE_FROM_LEFT = 10 # mm
left_distance = Distance(Ports.PORT6)

RIGHT_DISTANCE_COMPENSATION = 1.0
RIGHT_DISTANCE_FROM_RIGHT = 10 # mm
right_distance = Distance(Ports.PORT8)

claw_solenoid = DigitalOut(brain.three_wire_port.a)
toggle_solenoid = DigitalOut(brain.three_wire_port.h)

all_motors = [left_front_motor, left_back_motor,
              right_front_motor, right_back_motor,
              claw_arm_motor1, claw_arm_motor2, lift_motor]

all_motor_names = ["LEFT_FRONT", "LEFT_BACK",
                   "RIGHT_FRONT", "RIGHT_BACK",
                   "ARM_LEFT", "ARM_RIGHT", "LIFT"]

motor_monitor = None

# ------------------------------------------------------------ #
### ROBOT STATE
# ------------------------------------------------------------ #

ROBOT_INITIALIZED = False
ROBOT_ENABLED = False
QUIET_MODE = False

# ------------------------------------------------------------ #
### ROBOT CONFIGURATION
# ------------------------------------------------------------ #

ENABLE_HEADING_HOLD = False
ENABLE_FIELD_ORIENT = False
ENABLE_AUTO_CLAW_DOWN = False
ENABLE_AUTO_CLAW_MID1 = False
ENABLE_SLOW_RAMP = False

def save_settings():
    print("saving settings")
    settings = {
        "heading_hold": ENABLE_HEADING_HOLD,
        "field_orient": ENABLE_FIELD_ORIENT,
        "slow_ramp": ENABLE_SLOW_RAMP,
        "auto_claw_down": ENABLE_AUTO_CLAW_DOWN,
        "auto_claw_mid1": ENABLE_AUTO_CLAW_MID1
    }
    print("settings to save:", settings)
    with open("settings.json", "w") as f:
        json.dump(settings, f)

def load_settings():
    global ENABLE_HEADING_HOLD, ENABLE_FIELD_ORIENT, ENABLE_SLOW_RAMP, ENABLE_AUTO_CLAW_DOWN, ENABLE_AUTO_CLAW_MID1
    print("loading settings")
    try:
        with open("settings.json", "r") as f:
            settings = json.load(f)
            print("settings loaded:", settings)
            ENABLE_HEADING_HOLD = settings.get("heading_hold", ENABLE_HEADING_HOLD)
            ENABLE_FIELD_ORIENT = settings.get("field_orient", ENABLE_FIELD_ORIENT)
            ENABLE_SLOW_RAMP = settings.get("slow_ramp", ENABLE_SLOW_RAMP)
            ENABLE_AUTO_CLAW_DOWN = settings.get("auto_claw_down", ENABLE_AUTO_CLAW_DOWN)
            ENABLE_AUTO_CLAW_MID1 = settings.get("auto_claw_mid1", ENABLE_AUTO_CLAW_MID1)
    except:
        print("settings file not found, saving default settings")
        save_settings()
    print("heading_hold:", ENABLE_HEADING_HOLD)
    print("field_orient:", ENABLE_FIELD_ORIENT)
    print("slow_ramp:", ENABLE_SLOW_RAMP)
    print("auto_claw_down:", ENABLE_AUTO_CLAW_DOWN)
    print("auto_claw_mid1:", ENABLE_AUTO_CLAW_MID1)

def configuration_UI():
    global ROBOT_ENABLED
    global ENABLE_HEADING_HOLD, ENABLE_FIELD_ORIENT, ENABLE_SLOW_RAMP, ENABLE_AUTO_CLAW_DOWN, ENABLE_AUTO_CLAW_MID1
    ROBOT_ENABLED = False
    if motor_monitor is not None: motor_monitor.mute(True)
    # Use up and down arrows to select different menu items on screen
    # Left and right arrows to change the value of the selected menu item
    # Pressing A confirms the selected option
    # Changes saved to SDCard and read upon next initialization
    # Current options are:
    # - Enable / Disable Heading Hold
    # - Enable / Disable Field Orient
    # - Enable / Disable Auto Claw Down
    # - Enable / Disable Auto Claw Mid1

    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)
    brain.screen.print("Robot Configuration")
    brain.screen.new_line()
    brain.screen.print("Use arrows to navigate")
    brain.screen.new_line()
    brain.screen.print("Hold A to save and exit")
    brain.screen.new_line()
    brain.screen.print("Press B to discard changesand exit")
    brain.screen.new_line()
    brain.screen.new_line()

    menu_data = [
        {"name": "Heading Hold", "enabled": ENABLE_HEADING_HOLD},
        {"name": "Field Orient", "enabled": ENABLE_FIELD_ORIENT},
        {"name": "Slow Ramp", "enabled": ENABLE_SLOW_RAMP},
        {"name": "Auto Claw Down", "enabled": ENABLE_AUTO_CLAW_DOWN},
        {"name": "Auto Claw Mid1", "enabled": ENABLE_AUTO_CLAW_MID1}
    ]

    menu_selection = 0
    brain.screen.print("[{}] {}".format("X" if menu_data[menu_selection]["enabled"] else " ", menu_data[menu_selection]["name"]))

    # Wait for user input to navigate the menu
    pressing_timer = 0
    options_changed = False
    while True:
        if controller_1.buttonA.pressing():
            if pressing_timer > 2000:
                brain.screen.clear_screen()
                brain.screen.set_cursor(1, 1)
                brain.screen.print("Saving settings...")
                break
            else:
                pressing_timer += 20
        else:
            pressing_timer = 0

        if controller_1.buttonB.pressing():
            # Discard changes and exit
            brain.screen.clear_screen()
            brain.screen.set_cursor(1, 1)
            brain.screen.print("Discarding changes...")
            if motor_monitor is not None:
                motor_monitor.mute(False)
                motor_monitor.refresh()
            ROBOT_ENABLED = True
            return
        if controller_1.buttonDown.pressing():
            menu_selection = (menu_selection + 1) % len(menu_data)
            wait(200, MSEC)  # Debounce delay
            options_changed = True
        if controller_1.buttonRight.pressing():
            menu_data[menu_selection]["enabled"] = not menu_data[menu_selection]["enabled"]
            options_changed = True
            wait(200, MSEC)  # Debounce delay
        if options_changed:
            options_changed = False
            brain.screen.set_cursor(6, 1)
            brain.screen.print("[{}] {}          ".format("X" if menu_data[menu_selection]["enabled"] else " ", menu_data[menu_selection]["name"]))
        wait(20, MSEC)

    ENABLE_HEADING_HOLD = menu_data[0]["enabled"]
    ENABLE_FIELD_ORIENT = menu_data[1]["enabled"]
    ENABLE_SLOW_RAMP = menu_data[2]["enabled"]
    ENABLE_AUTO_CLAW_DOWN = menu_data[3]["enabled"]
    ENABLE_AUTO_CLAW_MID1 = menu_data[4]["enabled"]

    brain.screen.new_line()
    save_settings()
    brain.screen.print("Settings saved!")
    wait(1000, MSEC)
    ROBOT_ENABLED = True
    if motor_monitor is not None:
        motor_monitor.mute(False)
        motor_monitor.refresh()

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
CLAW_ARM_UP_DEGREES = 160 * 3
CLAW_ARM_MID3_DEGREES = 30 * 3 # was 24.5 * 3
CLAW_ARM_MID2_DEGREES = 18 * 3 # was 24.5 * 3
CLAW_ARM_MID1_DEGREES = 13 * 3 # was 18 * 3
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
    claw_arm_motor1.set_velocity(20, PERCENT)
    claw_arm_motor1.set_stopping(HOLD)
    claw_arm_motor1.set_timeout(CLAW_ARM_TIMEOUT, SECONDS)
    claw_arm_motor2.set_velocity(20, PERCENT)
    claw_arm_motor2.set_stopping(HOLD)
    claw_arm_motor2.set_timeout(1, SECONDS)
    claw_arm_motor1.spin_to_position(-20, DEGREES, wait=False)
    claw_arm_motor2.spin_to_position(-20, DEGREES)
    wait(0.25, SECONDS)
    claw_arm_motor1.set_position(0, DEGREES)
    claw_arm_motor2.set_position(0, DEGREES)
    claw_arm_motor1.stop(HOLD)
    claw_arm_motor2.stop(HOLD)
    CLAW_INITALIZED = True

CLAW_ARM_COMMAND_NONE = 0
CLAW_ARM_COMMAND_RAISE = 1
CLAW_ARM_COMMAND_LOWER = 2
CLAW_ARM_COMMAND_TO_POSITION = 3

def run_claw_arm(command, target_position=-1):
    global CLAW_ARM_RUNNING, CLAW_ARM_POSITION
    if CLAW_ARM_RUNNING: return

    positions_list = [CLAW_ARM_DOWN, CLAW_ARM_MID1, CLAW_ARM_MID2, CLAW_ARM_MID3, CLAW_ARM_UP]
    target_list = [CLAW_ARM_DOWN_DEGREES, CLAW_ARM_MID1_DEGREES, CLAW_ARM_MID2_DEGREES, CLAW_ARM_MID3_DEGREES, CLAW_ARM_UP_DEGREES]

    if command == CLAW_ARM_COMMAND_NONE: return
    if command == CLAW_ARM_COMMAND_RAISE:
        if CLAW_ARM_POSITION >= CLAW_ARM_UP: return
        claw_target_position = CLAW_ARM_POSITION + 1
        # MID3 only used for autonomous
        if (claw_target_position == CLAW_ARM_MID3): claw_target_position += 1
        arm_speed = CLAW_ARM_SPEED
    elif command == CLAW_ARM_COMMAND_LOWER:
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

    claw_target_degrees = target_list[claw_target_position]

    CLAW_ARM_RUNNING = True
    starting_position = claw_arm_motor1.position(DEGREES)
    claw_arm_motor1.set_velocity(arm_speed, PERCENT)
    claw_arm_motor1.set_stopping(HOLD)
    claw_arm_motor1.set_timeout(CLAW_ARM_TIMEOUT, SECONDS)
    claw_arm_motor2.set_velocity(arm_speed, PERCENT)
    claw_arm_motor2.set_stopping(HOLD)
    claw_arm_motor2.set_timeout(CLAW_ARM_TIMEOUT, SECONDS)
    claw_arm_motor1.spin_to_position(claw_target_degrees, DEGREES, wait=False)
    claw_arm_motor2.spin_to_position(claw_target_degrees, DEGREES)
    claw_arm_motor1.stop()
    claw_arm_motor2.stop()
    CLAW_ARM_RUNNING = False
    CLAW_ARM_POSITION = claw_target_position
    ending_position = claw_arm_motor1.position(DEGREES)
    total_links_moved = (ending_position - starting_position) / LIFT_DEGREES_PER_LINK
    print("Claw from {} to {}, total {} links".format(starting_position, ending_position, total_links_moved))

def raise_claw_arm():
    run_claw_arm(CLAW_ARM_COMMAND_RAISE)

def lower_claw_arm():
    run_claw_arm(CLAW_ARM_COMMAND_LOWER)

def move_claw_arm_to_position(target_position):
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, target_position)

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
            if (ENABLE_AUTO_CLAW_DOWN and CLAW_ARM_POSITION == CLAW_ARM_DOWN):
                if (claw_distance.object_distance() < 70):
                    close_claw()
                    wait(1, SECONDS)
            elif (ENABLE_AUTO_CLAW_MID1 and CLAW_ARM_POSITION == CLAW_ARM_MID1):
                if (claw_distance.object_distance() < 70):
                    close_claw()
                    wait(1, SECONDS)
        wait(10, MSEC)


def raise_toggle():
    toggle_solenoid.set(0)

def lower_toggle():
    toggle_solenoid.set(1)

### AUTONOMOUS

# ------------------------------------------------------------ #
### DRIVETRAIN FUNCTIONS
# ------------------------------------------------------------ #

def limit(input, limit_value):
    if (input > limit_value): return limit_value
    elif (input < -limit_value): return -limit_value
    return input

def ramp_limit(current, previous, limit):
    if (current - previous) > limit:
        return previous + limit
    elif (current - previous) < -limit:
        return previous - limit
    return current

def turn_for(turn_degrees, speed=66):
    current_heading = inertial.rotation()
    target_heading = current_heading + turn_degrees
    heading_error = target_heading - current_heading
    target_tolerance = 1  # degrees
    settle_count = 0
    timeout_count = 0
    done = False
    turn_kp = 8.0
    while not done:

        if abs(heading_error) < target_tolerance:
            settle_count += 1
        else:
            settle_count = 0

        if timeout_count > 1000 or settle_count > 10:
            done = True
            left_front_motor.stop(BRAKE)
            left_back_motor.stop(BRAKE)
            right_front_motor.stop(BRAKE)
            right_back_motor.stop(BRAKE)
            break

        turn_control = turn_kp * heading_error / 360.0
        turn_control = limit(turn_control, 1.0) * speed

        left_front_motor.spin(FORWARD, turn_control, PERCENT)
        left_back_motor.spin(FORWARD, turn_control, PERCENT)
        right_front_motor.spin(REVERSE, turn_control, PERCENT)
        right_back_motor.spin(REVERSE, turn_control, PERCENT)
        wait(10, MSEC)
        current_heading = inertial.rotation()
        heading_error = target_heading - current_heading

    print("Turn completed. Final heading: {:.1f}".format(inertial.heading()))

    left_front_motor.stop()
    left_back_motor.stop()
    right_front_motor.stop()
    right_back_motor.stop()

FOWARD_EFFICIENCY = 1 / 1.045
LEFT_POWER_SCALING = 1.0
RIGHT_POWER_SCALING = 0.85
FRONT_POWER_SCALING = 1.0
BACK_POWER_SCALING = 1.0

def drive_for(distance, strafe=False, speed=100, heading=None, timeout=10000): # distance is in mm, speed is in percent
    # setup
    turn_speed = 100 # max turn speed in percent
    wheel_efficiency = 1 / cos(radians(DRIVETRAIN_WHEEL_ANGLES))
    effective_wheel_size = 220 * wheel_efficiency * DRIVETRAIN_EXTERNAL_GEAR_RATIO
    forward_target_revs = (distance / effective_wheel_size) / FOWARD_EFFICIENCY if not strafe else 0
    strafe_target_revs = distance / effective_wheel_size if strafe else 0
    if not QUIET_MODE:
        print("Target revolutions: {:.2f} {:.2f}".format(forward_target_revs, strafe_target_revs))
    target_tolerance = 10 / effective_wheel_size
    ramp_rate = 1
    drive_kp = 50.0  # Proportional gain for drive control
    turn_kp = 500.0  # Proportional gain for turn control

    # save initial motor positions
    starting_left_front_position = left_front_motor.position(TURNS)
    starting_left_back_position = left_back_motor.position(TURNS)
    starting_right_front_position = right_front_motor.position(TURNS)
    starting_right_back_position = right_back_motor.position(TURNS)

    # save starting rotation
    target_rotation = heading if heading is not None else inertial.rotation()

    if not QUIET_MODE:
        print("LF: {}, LB: {}, RF: {}, RB: {}".format(starting_left_front_position, starting_left_back_position, starting_right_front_position, starting_right_back_position))

    done = False
    timeout_count = 0
    settle_count = 0
    last_fwd = 0
    last_strafe = 0
    fwd_ramp_enabled = True
    strafe_ramp_enabled = True
    while not done:
        current_left_front_position = left_front_motor.position(TURNS)
        current_left_back_position = left_back_motor.position(TURNS)
        current_right_front_position = right_front_motor.position(TURNS)
        current_right_back_position = right_back_motor.position(TURNS)

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

        if timeout_count > timeout or settle_count > 10:
            done = True
            left_front_motor.stop(BRAKE)
            left_back_motor.stop(BRAKE)
            right_front_motor.stop(BRAKE)
            right_back_motor.stop(BRAKE)
        else:
            fwd_control = drive_kp * average_fwd_error
            fwd_control = limit(fwd_control, speed)
            if abs(fwd_control) < abs(last_fwd): fwd_ramp_enabled = False
            if fwd_ramp_enabled: fwd_control = ramp_limit(fwd_control, last_fwd, ramp_rate)
            last_fwd = fwd_control
            fwd_control_percent = fwd_control

            strafe_control = drive_kp * average_strafe_error
            strafe_control = limit(strafe_control, speed)
            if abs(strafe_control) < abs(last_strafe): strafe_ramp_enabled = False
            if strafe_ramp_enabled: strafe_control = ramp_limit(strafe_control, last_strafe, ramp_rate)
            last_strafe = strafe_control
            strafe_control_percent = strafe_control

            turn_control = turn_kp * rotation_error
            turn_control_percent = limit(turn_control, turn_speed)

            left_power = LEFT_POWER_SCALING
            right_power = RIGHT_POWER_SCALING
            front_power = FRONT_POWER_SCALING
            back_power = BACK_POWER_SCALING

            left_front_speed = fwd_control_percent * right_power + strafe_control_percent * front_power + turn_control_percent
            left_back_speed = fwd_control_percent * left_power - strafe_control_percent * back_power + turn_control_percent
            right_front_speed = fwd_control_percent * left_power - strafe_control_percent * front_power - turn_control_percent
            right_back_speed = fwd_control_percent * right_power + strafe_control_percent * back_power - turn_control_percent

            max_speed = max(abs(left_front_speed), abs(left_back_speed), abs(right_front_speed), abs(right_back_speed))
            if max_speed > 100:
                print("s")
                left_front_speed = left_front_speed * (100 / max_speed)
                left_back_speed = left_back_speed * (100 / max_speed)
                right_front_speed = right_front_speed * (100 / max_speed)
                right_back_speed = right_back_speed * (100 / max_speed)

            left_front_motor.spin(FORWARD, left_front_speed, PERCENT)
            left_back_motor.spin(FORWARD, left_back_speed, PERCENT)
            right_front_motor.spin(FORWARD, right_front_speed, PERCENT)
            right_back_motor.spin(FORWARD, right_back_speed, PERCENT)

        timeout_count += 1
        wait(10, MSEC)

    # save initial motor positions
    left_front_position = left_front_motor.position(TURNS)
    left_back_position = left_back_motor.position(TURNS)
    right_front_position = right_front_motor.position(TURNS)
    right_back_position = right_back_motor.position(TURNS)

    if not QUIET_MODE:
        print("LF: {}, LB: {}, RF: {}, RB: {}".format(left_front_position, left_back_position, right_front_position, right_back_position))

### ROBOT LOCATION

X = 0
Pxx = 1.0
Y = 0
Pyy = 1.0
THETA = 0

def drive_to_xy(target_x, target_y, strafe=False, speed=100, heading=None, timeout=10000): # distance is in mm, speed is in percent
    print("Driving to X: {}, Y: {} from X: {}, Y: {}".format(target_x, target_y, X, Y))

    # setup
    wheel_efficiency = 1 / cos(radians(DRIVETRAIN_WHEEL_ANGLES))
    effective_wheel_size = 220 * wheel_efficiency * DRIVETRAIN_EXTERNAL_GEAR_RATIO
    turn_speed = 100 # max turn speed in percent
    target_tolerance = 10 # mm
    ramp_rate = 1
    drive_kp = 50.0  # Proportional gain for drive control
    turn_kp = 500.0  # Proportional gain for turn control

    # save starting rotation
    target_rotation = heading if heading is not None else inertial.rotation()

    done = False
    timeout_count = 0
    settle_count = 0
    last_fwd = 0
    last_strafe = 0
    fwd_ramp_enabled = True
    strafe_ramp_enabled = True
    while not done:
        current_rotation = inertial.rotation()
        rotation_error = (target_rotation - current_rotation) / 360.0 # saturate at 360 degrees

        average_fwd_error = target_x - X
        average_strafe_error = target_y - Y

        # print("{:0.2f} {:0.2f} {:0.2f} {:0.2f}".format(left_error, right_error, front_error, back_error))

        average_error = average_fwd_error if not strafe else average_strafe_error
        if abs(average_error) < target_tolerance:
            settle_count += 1
        else:
            settle_count = 0

        # convert to approximate motor revolutions based on errors
        forward_target_revs = (average_fwd_error / effective_wheel_size) # if not strafe else 0
        strafe_target_revs = (average_strafe_error / effective_wheel_size) # if strafe else 0

        if timeout_count > timeout or settle_count > 10:
            done = True
            left_front_motor.stop(BRAKE)
            left_back_motor.stop(BRAKE)
            right_front_motor.stop(BRAKE)
            right_back_motor.stop(BRAKE)
        else:
            fwd_control = drive_kp * forward_target_revs
            fwd_control = limit(fwd_control, speed)
            if abs(fwd_control) < abs(last_fwd): fwd_ramp_enabled = False
            if fwd_ramp_enabled: fwd_control = ramp_limit(fwd_control, last_fwd, ramp_rate)
            last_fwd = fwd_control
            fwd_control_percent = fwd_control

            strafe_control = drive_kp * strafe_target_revs
            strafe_control = limit(strafe_control, speed)
            if abs(strafe_control) < abs(last_strafe): strafe_ramp_enabled = False
            if strafe_ramp_enabled: strafe_control = ramp_limit(strafe_control, last_strafe, ramp_rate)
            last_strafe = strafe_control
            strafe_control_percent = strafe_control

            turn_control = turn_kp * rotation_error
            turn_control_percent = limit(turn_control, turn_speed)

            left_power = LEFT_POWER_SCALING
            right_power = RIGHT_POWER_SCALING
            front_power = FRONT_POWER_SCALING
            back_power = BACK_POWER_SCALING

            left_front_speed = fwd_control_percent * right_power + strafe_control_percent * front_power + turn_control_percent
            left_back_speed = fwd_control_percent * left_power - strafe_control_percent * back_power + turn_control_percent
            right_front_speed = fwd_control_percent * left_power - strafe_control_percent * front_power - turn_control_percent
            right_back_speed = fwd_control_percent * right_power + strafe_control_percent * back_power - turn_control_percent

            max_speed = max(abs(left_front_speed), abs(left_back_speed), abs(right_front_speed), abs(right_back_speed))
            if max_speed > 100:
                print("s")
                left_front_speed = left_front_speed * (100 / max_speed)
                left_back_speed = left_back_speed * (100 / max_speed)
                right_front_speed = right_front_speed * (100 / max_speed)
                right_back_speed = right_back_speed * (100 / max_speed)

            left_front_motor.spin(FORWARD, left_front_speed, PERCENT)
            left_back_motor.spin(FORWARD, left_back_speed, PERCENT)
            right_front_motor.spin(FORWARD, right_front_speed, PERCENT)
            right_back_motor.spin(FORWARD, right_back_speed, PERCENT)

        timeout_count += 1
        wait(10, MSEC)

# + 30mm at 1440mm
# + 22mm at 950mm
# + 19.4mm at 460mm, encoder average reading 501mm
# Distance from back to center = 180mm
# Distance sensor to back = 66mm
# Forward travel
def average_back_distance(samples=10):
    total_distance = 0
    for _ in range(samples):
        total_distance += (back_distance1.object_distance(MM) + back_distance2.object_distance(MM)) / 2
        wait(33, MSEC)
    return total_distance / samples

def motor_distance_step(current, previous):
    lf = current[0] - previous[0]
    lb = current[1] - previous[1]
    rf = current[2] - previous[2]
    rb = current[3] - previous[3]
    forward = (lf + lb + rf + rb) / 4
    side = (lf - rf - lb + rb) / 4
    forward = forward * DRIVETRAIN_EXTERNAL_GEAR_RATIO * DRIVETRAIN_WHEEL_SIZE * sqrt(2) * FOWARD_EFFICIENCY
    side = side * DRIVETRAIN_EXTERNAL_GEAR_RATIO * DRIVETRAIN_WHEEL_SIZE * sqrt(2)
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
previous_theta = THETA
previous_back_distance = [0.0, 0]
previous_left_distance = [0.0, 0]
previous_right_distance = [0.0, 0]

def initialize_wheels():
    global previous_motor_positions, previous_theta

    previous_motor_positions = [left_front_motor.position(TURNS), left_back_motor.position(TURNS), right_front_motor.position(TURNS), right_back_motor.position(TURNS)]
    previous_theta = THETA

def initialize_distances():
    global previous_back_distance, previous_left_distance, previous_right_distance

    previous_back_distance[0] = (back_distance1.object_distance(MM) + back_distance2.object_distance(MM)) / 2.0
    previous_back_distance[1] = max(back_distance1.timestamp(), back_distance2.timestamp())
 
    previous_left_distance[0] = left_distance.object_distance(MM)
    previous_left_distance[1] = left_distance.timestamp()

    previous_right_distance[0] = right_distance.object_distance(MM)
    previous_right_distance[1] = right_distance.timestamp()

def get_back_distance():
    global previous_back_distance

    if abs(THETA) > 5:
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

    if abs(THETA) > 5:
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

    if abs(THETA) > 5:
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

    current_motor_positions = [left_front_motor.position(TURNS), left_back_motor.position(TURNS), right_front_motor.position(TURNS), right_back_motor.position(TURNS)]
    current_theta = inertial.rotation()

    delta_forward, delta_side = motor_distance_step(current_motor_positions, previous_motor_positions)
    delta_theta = current_theta - previous_theta

    if delta_theta == 0.0:
        to_global_rotation_angle = current_theta
        delta_local_x = delta_forward
        delta_local_y = delta_side
    else:
        r_forward = delta_forward / radians(delta_theta) # mm
        r_side = delta_side / radians(delta_theta) # mm

        to_global_rotation_angle = current_theta + delta_theta / 2.0
        delta_local_x = r_forward * 2.0 * sin(radians(delta_theta) / 2.0)
        delta_local_y = r_side * 2.0 * sin(radians(delta_theta) / 2.0)

    delta_global_x = delta_local_x * cos(radians(to_global_rotation_angle)) - delta_local_y * sin(radians(to_global_rotation_angle))
    delta_global_y = delta_local_x * sin(radians(to_global_rotation_angle)) + delta_local_y * cos(radians(to_global_rotation_angle))

    previous_motor_positions = current_motor_positions
    previous_theta = current_theta

    new_X = X + delta_global_x
    new_Y = Y + delta_global_y
    new_theta = current_theta

    return new_X, new_Y, new_theta

def odom_thread():
    global X, Y, THETA, Pxx, Pyy
    THETA = inertial.rotation()
    initialize_wheels()
    initialize_distances()
    filter = KalmanXY(X, Y)

    count = 0
    while True:
        X, Y, THETA = predict_wheels()
        filter.predict(X - filter.X, Y - filter.Y)

        meas_back_distance = get_back_distance()
        if meas_back_distance is not None:
            meas_back_distance += HIDDEN_PERIMITER + ROBOT_LENGTH / 2 - BACK_DISTANCE_FROM_BACK
            X, Y = filter.update_x(meas_back_distance)

        meas_right_distance = get_right_distance()
        if meas_right_distance is not None:
            meas_right_distance += HIDDEN_PERIMITER + ROBOT_WIDTH / 2 - RIGHT_DISTANCE_FROM_RIGHT
            X, Y = filter.update_y(3600.0 - meas_right_distance)

        meas_left_distance = get_left_distance()
        if meas_left_distance is not None:
            meas_left_distance += HIDDEN_PERIMITER + ROBOT_WIDTH / 2 - LEFT_DISTANCE_FROM_LEFT
            X, Y = filter.update_y(meas_left_distance)

        Pxx, Pyy = filter.Pxx, filter.Pyy

        if not QUIET_MODE and count % 200 == 0:
            print("X: {:4.0f}, Y: {:4.0f}, H: {:3.2f}".format(X, Y, THETA))

        count += 1

        wait(10, MSEC)

def log_drivetrain():
    global QUIET_MODE

    motors = [left_front_motor, left_back_motor, right_front_motor, right_back_motor]
    headers = ["lfm", "lbm", "rfm", "rbm"]
    units = ["POS", "VEL", "TRQ"]

    log = []

    # Run ramp test

    TOTAL_SAMPLES = 400
    PRINT_DELAY = 250 # ms between samples. Set to around 250 for wireless or 50 for USB

    for i in range(TOTAL_SAMPLES):

        entry = [inertial.rotation(DEGREES), (back_distance1.object_distance(MM)+back_distance2.object_distance(MM))/2]
        for motor in motors:
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
                output += "{}, ".format(log_entry[j])
            else:
                output += "{}".format(log_entry[j])

        print(output)
        wait(PRINT_DELAY, MSEC)


def log_odom():
    global QUIET_MODE

    motors = [left_front_motor, left_back_motor, right_front_motor, right_back_motor]

    log = []

    # Run ramp test

    TOTAL_SAMPLES = 400
    PRINT_DELAY = 250 # ms between samples. Set to around 250 for wireless or 50 for USB

    for i in range(TOTAL_SAMPLES):

        avg_motor_position = 0.0
        for motor in motors:
            avg_motor_position += motor.position(RotationUnits.REV)
        avg_motor_position /= len(motors)

        entry = [
            avg_motor_position,
            X,
            Y,
            THETA,
            Pxx,
            Pyy]

        log.append(entry)
        wait (10, MSEC)

    QUIET_MODE = True
    wait(100, MSEC)

    output = "idx, pos, X, Y, THETA, Pxx, Pyy"
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
    # Thread(odom_thread)
    # Thread(log_drivetrain)
    # Thread(log_odom)
    # place automonous code here
    # starting_distance = average_back_distance()
    #print("Back distance: {}".format(starting_distance))
    wait(100, MSEC)
    while True:
        drive_to_xy(900.0, 1800.0, False, 33, heading = 0)
        wait(500, MSEC)
        drive_to_xy(900.0, 700.0, True, 33, heading = 0)
        wait(500, MSEC)
        drive_to_xy(300.0, 700.0, False, 33, heading = 0)
        wait(500, MSEC)
        drive_to_xy(300.0, 1800.0, True, 33, heading = 0)
        wait(500, MSEC)
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

    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_UP)
    wait(500, MSEC)
    # Thread(log_drivetrain)
    drive_for(51 * 25.4, False, 50)
    drive_for(-400, True, 50)
    drive_for(11 * 25.4, False, 50)
    command_lift(11)
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_MID2)
    command_lift(10)
    wait(500, MSEC)
    open_claw()
    drive_for(-150, False, 50)
    # TODO: Move back to safe distance

def autonomous_none():
    # place automonous code here
    lower_toggle()
    drive_for(50, False, 50, heading = 0)
    drive_for(-50, False, 50, heading = 0)
    drive_for(50, False, 50, heading = 0)
    drive_for(-50, False, 50, heading = 0)
    drive_for(100, False, 50)

def claw_move1():
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_MID1)
    command_lift(5)

# Score 7 pins, 3 goals with 2 pins
def autonomous_left():
    # place automonous code here

    lower_toggle()
    drive_for(50, False, 50, heading = 0)
    drive_for(-50, False, 50, heading = 0)
    drive_for(50, False, 50, heading = 0)
    drive_for(-50, False, 50, heading = 0)
    raise_toggle()

    Thread(claw_move1)
    wait(250, MSEC)
    drive_for(100, False, 50, heading = 0)
    drive_for(675, True, 50, heading = 0)
    drive_for(100, False, 50, heading = 0)
    command_lift(3)
    open_claw()
    wall_distance = average_back_distance() - BACK_DISTANCE_FROM_BACK
    print("Wall distance: {}".format(wall_distance))
    target_distance = 120
    reverse_by = target_distance - wall_distance
    drive_for(reverse_by, False, 50, heading = 0)
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_DOWN)
    command_lift(0)
    current_heading = inertial.rotation()
    print("Current heading: {}".format(current_heading))
    target_heading = 180
    turn_for(target_heading - current_heading, 66)
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_MID3)
    wait(250, MSEC)
    close_claw()
    command_lift(5)
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_DOWN)
    current_heading = inertial.rotation()
    print("Current heading: {}".format(current_heading))
    target_heading = 0
    turn_for(target_heading - current_heading, 66)
    command_lift(11)
    drive_for(-reverse_by+20, False, 50, heading = 0)
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_MID1)
    wait(250, MSEC)
    command_lift(9)
    open_claw()
    wait(250, MSEC)
    drive_for(reverse_by, False, 50, heading = 0)
    run_claw_arm(CLAW_ARM_COMMAND_TO_POSITION, CLAW_ARM_DOWN)
    command_lift(0)

def autonomous_right():
    # place automonous code here

    lower_toggle()
    drive_for(50, False, 50, heading = 0)
    drive_for(-50, False, 50, heading = 0)
    drive_for(50, False, 50, heading = 0)
    drive_for(-50, False, 50, heading = 0)
    raise_toggle()

def autonomous():
    global ROBOT_ENABLED
    while not ROBOT_INITIALIZED:
        wait(100, MSEC)
    ROBOT_ENABLED = True
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
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

def pre_autonomous():
    global ROBOT_INITIALIZED
    global ALLIANCE_COLOR, AUTON_SEQUENCE
    global pitch_offset
    global motor_monitor
    # actions to do when the program starts
    brain.screen.clear_screen()
    brain.screen.print("pre auton code")
    inertial.calibrate()
    load_settings()
    while inertial.is_calibrating():
        wait(100, MSEC)
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
        claw_arm_motor1.stop(HOLD)
        claw_arm_motor2.stop(HOLD)
        return
    thread = Thread(lower_claw_arm)

def OnRaiseClawPressed():
    if not ROBOT_ENABLED: return
    if CLAW_ARM_RUNNING:
        print("Was Running")
        claw_arm_motor1.stop(HOLD)
        claw_arm_motor2.stop(HOLD)
        return
    thread = Thread(raise_claw_arm)

def OnControlButtonAPressed():
    if not ROBOT_ENABLED: return
    if claw_is_open():
        close_claw()
    else:
        open_claw()
    StopLift()

def OnControlButtonBPressed():
    if not ROBOT_ENABLED: return
    if toggle_solenoid.value() == 1:
        toggle_solenoid.set(0)
    else:
        toggle_solenoid.set(1)

def OnControlButtonUpPressed():
    pressed_counter = 0
    while pressed_counter < 30:
        wait(100, MSEC)
        if not controller_1.buttonUp.pressing():
            return
        pressed_counter += 1
    # Button has been held for 30 cycles (3 seconds)
    configuration_UI()

# Default maximum drive and turn rates
DEFAULT_TURN_MAX = 75.0 # maximum turn rate
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
AUTO_TURN_KP = 0.25
AUTO_TURN_KD = 0.0
NO_INPUT_TIMEOUT = 200

def user_control():
    global ROBOT_ENABLED

    brain.screen.clear_screen()
    brain.screen.print("driver control")
    # place driver control in this while loop
    last_fwd = 0 
    while not ROBOT_INITIALIZED:
        wait(100, MSEC)

    Thread(initialize_claw)

    brain.screen.clear_screen()
    brain.screen.print("user control")

    controller_1.buttonA.pressed(OnControlButtonAPressed)
    controller_1.buttonB.pressed(OnControlButtonBPressed)

    controller_1.buttonR2.pressed(OnLowerLiftPressed)
    controller_1.buttonR1.pressed(OnRaiseLiftPressed)

    controller_1.buttonL2.pressed(OnLowerClawPressed)
    controller_1.buttonL1.pressed(OnRaiseClawPressed)

    controller_1.buttonUp.pressed(OnControlButtonUpPressed)

    # brain.timer.event(check_lift_hold, 10000)

    rotation_set = inertial.rotation(DEGREES)
    no_input_timeout = NO_INPUT_TIMEOUT
    all_stop = True
    anti_tilt_active = False
    anti_tilt_timer = 10

    last_forward = 0
    last_strafe = 0
    last_turn_error = 0

    left_front_motor.set_stopping(COAST)
    left_back_motor.set_stopping(COAST)
    right_front_motor.set_stopping(COAST)
    right_back_motor.set_stopping(COAST)

    # initialize_lift()
    Thread(auto_claw_thread)
    # Thread(odom_thread)

    ROBOT_ENABLED = True

    loop_count = 0

    # place driver control in this while loop
    while True:
        #if add_sample():
        #    dumpsamples()

        if not ROBOT_ENABLED:
            wait(100, MSEC)
            continue
            
        raw_forward = apply_deadband(controller_1.axis3.position())
        raw_strafe = apply_deadband(controller_1.axis4.position())
        raw_turn = apply_deadband(controller_1.axis1.position())
        raw_detwitch = apply_deadband(controller_1.axis2.position())

        # Remap from field to robot
        FIELD_ORIENTED = ENABLE_FIELD_ORIENT
        if FIELD_ORIENTED:
            robot_forward = raw_forward * cos(radians(inertial.rotation(DEGREES))) + raw_strafe * sin(radians(inertial.rotation(DEGREES)))
            robot_strafe = -raw_forward * sin(radians(inertial.rotation(DEGREES))) + raw_strafe * cos(radians(inertial.rotation(DEGREES)))
        else:
            robot_forward = raw_forward
            robot_strafe = raw_strafe

        raw_forward = robot_forward
        raw_strafe = robot_strafe

        MAX_RANP = 2 if ENABLE_SLOW_RAMP else 3
        MIN_RAMP = 1
        RAMP_RANGE = MAX_RANP - MIN_RAMP

        # Ramp control - forward
        ramp_max = MAX_RANP - RAMP_RANGE * lift_height(percent=True) / 100
        safe_forward = ramp_limit(raw_forward, last_forward, ramp_max)
        forward = safe_forward
        last_forward = forward

        # Ramp control - strafe
        ramp_max = MAX_RANP - RAMP_RANGE * lift_height(percent=True) / 100
        safe_strafe = ramp_limit(raw_strafe, last_strafe, ramp_max)
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
        no_input = forward == 0 and turn == 0 and strafe == 0
        if no_input:
            no_input_timeout -= 1
            if no_input_timeout <= 0:
                left_front_motor.stop(COAST)
                left_back_motor.stop(COAST)
                right_front_motor.stop(COAST)
                right_back_motor.stop(COAST)
                rotation_set = inertial.rotation(DEGREES)
                all_stop = True
        else:
            no_input_timeout = NO_INPUT_TIMEOUT
            all_stop = False

        # Heading hold
        # rotation_set += (raw_turn / 100) * (MAX_ROTATION_PER_SECOND / 100)
        # Case 1: Active turn input
        if turn != 0:
            rotation_set = inertial.rotation(DEGREES)
            auto_turn = 0
            last_turn_error = 0
        # Case 2: Forward or strafe still active
        elif ENABLE_HEADING_HOLD and (forward != 0 or strafe != 0):
            turn_error = rotation_set - inertial.rotation(DEGREES)
            auto_turn = turn_error * AUTO_TURN_KP + (turn_error - last_turn_error) * AUTO_TURN_KD
            last_turn_error = turn_error
        # Case 3: Coasting with no input
        else:
            rotation_set = inertial.rotation(DEGREES)
            auto_turn = 0
            last_turn_error = 0

        combined_forward = forward
        combined_strafe = strafe
        combined_turn = turn + auto_turn
        
        if not all_stop:
            left_power = LEFT_POWER_SCALING
            right_power = RIGHT_POWER_SCALING
            front_power = FRONT_POWER_SCALING
            back_power = BACK_POWER_SCALING

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

        wait(10, MSEC)

        # if loop_count % 100 == 0:
            #print("Lift {}".format(lift_motor.position(DEGREES)))
            # print("Rotation: {:.1f}".format(inertial.rotation(DEGREES)))

        loop_count += 1

# create competition instance
comp = Competition(user_control, autonomous)
pre_autonomous()