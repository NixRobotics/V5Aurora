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
from math import radians, cos, sin
import json

from v5pythonlibrary import *

# Brain should be defined by default
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

inertial = InertialWrapper(Ports.PORT5, 1.0)
claw_distance = Distance(Ports.PORT2)

claw_solenoid = DigitalOut(brain.three_wire_port.a)
toggle_solenoid = DigitalOut(brain.three_wire_port.h)

all_motors = [left_front_motor, left_back_motor,
              right_front_motor, right_back_motor,
              claw_arm_motor1, claw_arm_motor2, lift_motor]

all_motor_names = ["LEFT_FRONT", "LEFT_BACK",
                   "RIGHT_FRONT", "RIGHT_BACK",
                   "ARM_LEFT", "ARM_RIGHT", "LIFT"]

motor_monitor = None

# Begin project code
ROBOT_INITIALIZED = False
ROBOT_ENABLED = False

### ROBOT CONFIGURATION

ENABLE_HEADING_HOLD = False
ENABLE_FIELD_ORIENT = False
ENABLE_AUTO_CLAW_DOWN = False
ENABLE_AUTO_CLAW_MID1 = False

def save_settings():
    print("saving settings")
    settings = {
        "heading_hold": ENABLE_HEADING_HOLD,
        "field_orient": ENABLE_FIELD_ORIENT,
        "auto_claw_down": ENABLE_AUTO_CLAW_DOWN,
        "auto_claw_mid1": ENABLE_AUTO_CLAW_MID1
    }
    print("settings to save:", settings)
    with open("settings.json", "w") as f:
        json.dump(settings, f)

def load_settings():
    global ENABLE_HEADING_HOLD, ENABLE_FIELD_ORIENT, ENABLE_AUTO_CLAW_DOWN, ENABLE_AUTO_CLAW_MID1
    print("loading settings")
    try:
        with open("settings.json", "r") as f:
            settings = json.load(f)
            print("settings loaded:", settings)
            ENABLE_HEADING_HOLD = settings.get("heading_hold", ENABLE_HEADING_HOLD)
            ENABLE_FIELD_ORIENT = settings.get("field_orient", ENABLE_FIELD_ORIENT)
            ENABLE_AUTO_CLAW_DOWN = settings.get("auto_claw_down", ENABLE_AUTO_CLAW_DOWN)
            ENABLE_AUTO_CLAW_MID1 = settings.get("auto_claw_mid1", ENABLE_AUTO_CLAW_MID1)
    except:
        print("settings file not found, saving default settings")
        save_settings()
    print("heading_hold:", ENABLE_HEADING_HOLD)
    print("field_orient:", ENABLE_FIELD_ORIENT)
    print("auto_claw_down:", ENABLE_AUTO_CLAW_DOWN)
    print("auto_claw_mid1:", ENABLE_AUTO_CLAW_MID1)

def configuration_UI():
    global ROBOT_ENABLED
    global ENABLE_HEADING_HOLD, ENABLE_FIELD_ORIENT, ENABLE_AUTO_CLAW_DOWN, ENABLE_AUTO_CLAW_MID1
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
    ENABLE_AUTO_CLAW_DOWN = menu_data[2]["enabled"]
    ENABLE_AUTO_CLAW_MID1 = menu_data[3]["enabled"]

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

CLAW_ARM_RUNNING = False
CLAW_ARM_UP_DEGREES = 165 * 3
CLAW_ARM_MID2_DEGREES = 30 * 3
CLAW_ARM_MID1_DEGREES = 24 * 3
CLAW_ARM_DOWN_DEGREES = 0 * 3
CLAW_ARM_DOWN = 0
CLAW_ARM_MID1 = 1
CLAW_ARM_MID2 = 2
CLAW_ARM_UP = 3
CLAW_ARM_POSITION = CLAW_ARM_DOWN  # 0 = down, 1 = mid1, 2 = mid2, 3 = up
CLAW_ARM_TIMEOUT = 2.0
CLAW_ARM_SPEED = 50

def initialize_claw():
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

def raise_claw_arm():
    global CLAW_ARM_RUNNING, CLAW_ARM_POSITION
    if CLAW_ARM_RUNNING: return

    if CLAW_ARM_POSITION == CLAW_ARM_DOWN:
        claw_target_position = CLAW_ARM_MID1
        claw_target_degrees = CLAW_ARM_MID1_DEGREES
    elif CLAW_ARM_POSITION == CLAW_ARM_MID1:
        claw_target_position = CLAW_ARM_MID2
        claw_target_degrees = CLAW_ARM_MID2_DEGREES
    elif CLAW_ARM_POSITION == CLAW_ARM_MID2:
        claw_target_position = CLAW_ARM_UP
        claw_target_degrees = CLAW_ARM_UP_DEGREES
    else: return

    CLAW_ARM_RUNNING = True
    starting_position = claw_arm_motor1.position(DEGREES)
    claw_arm_motor1.set_velocity(CLAW_ARM_SPEED, PERCENT)
    claw_arm_motor1.set_stopping(HOLD)
    claw_arm_motor1.set_timeout(CLAW_ARM_TIMEOUT, SECONDS)
    claw_arm_motor2.set_velocity(CLAW_ARM_SPEED, PERCENT)
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
    print("Claw up from {} to {}, total {} links".format(starting_position, ending_position, total_links_moved))

def lower_claw_arm():
    global CLAW_ARM_RUNNING, CLAW_ARM_POSITION
    if CLAW_ARM_RUNNING: return

    if CLAW_ARM_POSITION == CLAW_ARM_UP:
        claw_target_position = CLAW_ARM_MID2
        claw_target_degrees = CLAW_ARM_MID2_DEGREES
    elif CLAW_ARM_POSITION == CLAW_ARM_MID2:
        claw_target_position = CLAW_ARM_MID1
        claw_target_degrees = CLAW_ARM_MID1_DEGREES
    elif CLAW_ARM_POSITION == CLAW_ARM_MID1:
        claw_target_position = CLAW_ARM_DOWN
        claw_target_degrees = CLAW_ARM_DOWN_DEGREES
    else: return

    CLAW_ARM_RUNNING = True
    starting_position = claw_arm_motor1.position(DEGREES)
    claw_arm_motor1.set_velocity(CLAW_ARM_SPEED * 0.75, PERCENT)
    claw_arm_motor1.set_stopping(HOLD)
    claw_arm_motor1.set_timeout(CLAW_ARM_TIMEOUT, SECONDS)
    claw_arm_motor2.set_velocity(CLAW_ARM_SPEED * 0.75, PERCENT)
    claw_arm_motor2.set_stopping(HOLD)
    claw_arm_motor2.set_timeout(CLAW_ARM_TIMEOUT, SECONDS)
    claw_arm_motor1.spin_to_position(claw_target_degrees, DEGREES, wait=False)
    claw_arm_motor2.spin_to_position(claw_target_degrees, DEGREES)
    claw_arm_motor1.stop()
    claw_arm_motor2.stop()
    CLAW_ARM_RUNNING = False
    CLAW_ARM_POSITION = claw_target_position
    ending_position = claw_arm_motor1.position(DEGREES)
    total_links_moved = (starting_position - ending_position) / LIFT_DEGREES_PER_LINK
    print("Claw down from {} to {}, total {} links".format(starting_position, ending_position, total_links_moved))

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

### AUTONOMOUS

pitch_offset = 0.0

def pre_autonomous():
    global ROBOT_INITIALIZED
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

    motor_monitor = MotorMonitor(brain, all_motors, all_motor_names)
    motor_monitor.start()
    
    ROBOT_INITIALIZED = True

def limit(input, limit_value):
    if (input > limit_value): return limit_value
    elif (input < -limit_value): return -limit_value
    return input

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

def drive_for(distance, strafe=False, speed=100, heading=None): # distance is in mm, speed is in percent
    # setup
    turn_speed = 100 # max turn speed in percent
    wheel_efficiency = 1 / cos(radians(DRIVETRAIN_WHEEL_ANGLES))
    effective_wheel_size = 220 * wheel_efficiency * DRIVETRAIN_EXTERNAL_GEAR_RATIO
    forward_target_revs = distance / effective_wheel_size if not strafe else 0
    strafe_target_revs = distance / effective_wheel_size if strafe else 0
    print("Target revolutions: {:.2f} {:.2f}".format(forward_target_revs, strafe_target_revs))
    target_tolerance = 10 / effective_wheel_size
    drive_kp = 1.0  # Proportional gain for drive control
    turn_kp = 5.0  # Proportional gain for turn control

    # save initial motor positions
    starting_left_front_position = left_front_motor.position(TURNS)
    starting_left_back_position = left_back_motor.position(TURNS)
    starting_right_front_position = right_front_motor.position(TURNS)
    starting_right_back_position = right_back_motor.position(TURNS)

    # save starting rotation
    target_rotation = heading if heading is not None else inertial.rotation()

    print("LF: {}, LB: {}, RF: {}, RB: {}".format(starting_left_front_position, starting_left_back_position, starting_right_front_position, starting_right_back_position))

    done = False
    timeout_count = 0
    settle_count = 0
    last_fwd = 0
    last_strafe = 0
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

        if timeout_count > 1000 or settle_count > 10:
            done = True
            left_front_motor.stop(BRAKE)
            left_back_motor.stop(BRAKE)
            right_front_motor.stop(BRAKE)
            right_back_motor.stop(BRAKE)
        else:
            fwd_control = drive_kp * average_fwd_error
            fwd_control = limit(fwd_control, 1.0)
            ramp_rate = 0.05
            if abs(fwd_control - last_fwd) > ramp_rate:
                if (fwd_control > last_fwd):
                    fwd_control = last_fwd + ramp_rate
                else:
                    fwd_control = last_fwd - ramp_rate
            last_fwd = fwd_control
            fwd_control_percent = fwd_control * speed

            strafe_control = drive_kp * average_strafe_error
            strafe_control = limit(strafe_control, 1.0)
            ramp_rate = 0.05
            if abs(strafe_control - last_strafe) > ramp_rate:
                if (strafe_control > last_strafe):
                    strafe_control = last_strafe + ramp_rate
                else:
                    strafe_control = last_strafe - ramp_rate
            last_strafe = strafe_control
            strafe_control_percent = strafe_control * speed

            turn_control = turn_kp * rotation_error
            turn_control_percent = limit(turn_control, 1.0) * turn_speed

            left_power = 1.0
            right_power = 0.85
            front_power = 1.0
            back_power = 1.0

            left_front_speed = fwd_control_percent * right_power + strafe_control_percent * front_power + turn_control_percent
            left_back_speed = fwd_control_percent * left_power - strafe_control_percent * back_power + turn_control_percent
            right_front_speed = fwd_control_percent * left_power - strafe_control_percent * front_power - turn_control_percent
            right_back_speed = fwd_control_percent * right_power + strafe_control_percent * back_power - turn_control_percent

            max_speed = max(abs(left_front_speed), abs(left_back_speed), abs(right_front_speed), abs(right_back_speed))
            if max_speed > 100:
                # print("s")
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

    print("LF: {}, LB: {}, RF: {}, RB: {}".format(left_front_position, left_back_position, right_front_position, right_back_position))
    
def autonomous():
    while not ROBOT_INITIALIZED:
        wait(100, MSEC)
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    Thread(initialize_claw)
    # place automonous code here
    drive_for(100, False, 50)
    # turn_for(360)
    # drive_for(600, False, 50)
    # drive_for(600, True, 50)

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

def OnControlButtonL2Pressed():
    if not ROBOT_ENABLED: return
    if CLAW_ARM_RUNNING:
        print("Was Running")
        claw_arm_motor1.stop(HOLD)
        claw_arm_motor2.stop(HOLD)
        return
    thread = Thread(lower_claw_arm)

def OnControlButtonL1Pressed():
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

custom_pitch = 0.0

def calibrated_pitch():
    global custom_pitch
    dt = 0.01
    alpha = 0.90
    raw_pitch = -inertial.orientation(OrientationType.ROLL, DEGREES) + pitch_offset
    raw_pitch_rate = -inertial.gyro_rate(AxisType.XAXIS, DPS)
    custom_pitch = alpha * (custom_pitch + (raw_pitch_rate * dt)) + (1.0 - alpha) * raw_pitch

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
        wait(25, MSEC)

dumped = False
def dumpsamples():
    global dumped
    if dumped: return
    dumped = True
    thread = Thread(dump_samples_thread)

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

MAX_ROTATION_PER_SECOND = 360
AUTO_TURN_KP = 0.25
AUTO_TURN_KD = 0.0
AUTO_FORWARD_KP = 5.0
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

    controller_1.buttonL2.pressed(OnControlButtonL2Pressed)
    controller_1.buttonL1.pressed(OnControlButtonL1Pressed)

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

    ROBOT_ENABLED = True

    loop_count = 0

    # place driver control in this while loop
    while True:
        #if add_sample():
        #    dumpsamples()

        if not ROBOT_ENABLED:
            wait(100, MSEC)
            continue
            
        raw_forward = controller_1.axis3.position()
        raw_strafe = controller_1.axis4.position()
        raw_turn = controller_1.axis1.position()
        raw_detwitch = controller_1.axis2.position()

        # Deadband
        if abs(raw_forward) < CONTROLLER_DEADBAND:
            raw_forward = 0
        elif raw_forward > 0:
            raw_forward = (raw_forward - CONTROLLER_DEADBAND) * 100/ (100 - CONTROLLER_DEADBAND)
        else:
            raw_forward = (raw_forward + CONTROLLER_DEADBAND) * 100 / (100 - CONTROLLER_DEADBAND)
            
        if abs(raw_strafe) < CONTROLLER_DEADBAND:
            raw_strafe = 0
        elif raw_strafe > 0:
            raw_strafe = (raw_strafe - CONTROLLER_DEADBAND) * 100 / (100 - CONTROLLER_DEADBAND)
        else:
            raw_strafe = (raw_strafe + CONTROLLER_DEADBAND) * 100 / (100 - CONTROLLER_DEADBAND)
        
        if abs(raw_turn) < CONTROLLER_DEADBAND:
            raw_turn = 0
        elif raw_turn > 0:
            raw_turn = (raw_turn - CONTROLLER_DEADBAND) * 100 / (100 - CONTROLLER_DEADBAND)
        else:
            raw_turn = (raw_turn + CONTROLLER_DEADBAND) * 100 / (100 - CONTROLLER_DEADBAND)

        if abs(raw_detwitch) < CONTROLLER_DEADBAND:
            raw_detwitch = 0
        elif raw_detwitch > 0:
            raw_detwitch = (raw_detwitch - CONTROLLER_DEADBAND) * 100 / (100 - CONTROLLER_DEADBAND)
        else:
            raw_detwitch = (raw_detwitch + CONTROLLER_DEADBAND) * 100 / (100 - CONTROLLER_DEADBAND)

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

        # Ramp control - forward
        ramp_max = 20 - 17 * lift_height(percent=True) / 100
        if (abs(raw_forward - last_forward) > ramp_max):
            if (raw_forward > last_forward): safe_forward = last_forward + ramp_max
            else: safe_forward = last_forward - ramp_max
        else:
            safe_forward = raw_forward

        forward = safe_forward
        last_forward = forward

        # Ramp control - strafe
        ramp_max = 20 - 17 * lift_height(percent=True) / 100
        if (abs(raw_strafe - last_strafe) > ramp_max):
            if (raw_strafe > last_strafe): safe_strafe = last_strafe + ramp_max
            else: safe_strafe = last_strafe - ramp_max
        else:
            safe_strafe = raw_strafe

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

        # print("{:.1f}".format(forward_tilt))
        calibrated_pitch()
        # if loop_count % 10 == 0:
        #     print("{:.1f}".format(custom_pitch))
        if TILT_ENABLE and not anti_tilt_active and abs(custom_pitch) >= 6.0:
            anti_tilt_active = True
            anti_tilt_timer = 50
            print("anti-tilt on")
        elif anti_tilt_active and abs(custom_pitch) < 1:
            anti_tilt_timer -= 1
            if anti_tilt_timer <= 0:
                anti_tilt_active = False
                print("anti-tilt off")

        if anti_tilt_active:
            # print("|")
            auto_forward = custom_pitch * AUTO_FORWARD_KP
        else:
            auto_forward = 0

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

        combined_forward = forward + auto_forward
        combined_strafe = strafe
        combined_turn = turn + auto_turn
        
        if not all_stop:
            # mixing the combined forward, strafe, and turn inputs to calculate individual motor speeds
            left_front_speed = combined_forward + combined_turn + combined_strafe
            left_back_speed = combined_forward + combined_turn - combined_strafe
            right_front_speed = combined_forward - combined_turn - combined_strafe
            right_back_speed = combined_forward - combined_turn + combined_strafe

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