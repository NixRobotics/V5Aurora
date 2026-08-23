# ----------------------------------------------------------------------------- #
#                                                                               #                                    
#    Project:        Split Arcade Control                                       #                             
#    Module:         main.py                                                    #
#    Author:         VEX                                                        #
#    Created:        Fri Aug 05 2022                                            #
#    Description:    This example will use the left Y and right X               #
#                    Controller axis to control the Clawbot.                    #
#                                                                               #                                    
#    Configuration:  V5 Clawbot (Individual Motors)                             #
#                    Controller                                                 #
#                    Claw Motor in Port 3                                       #
#                    Arm Motor in Port 8                                        #
#                    Left Motor in Port 1                                       #
#                    Right Motor in Port 10                                     #
#                                                                               #                                                                          
# ----------------------------------------------------------------------------- #

# Library imports
from math import radians, cos, sin

from vex import *

# Brain should be defined by default
brain=Brain()

# Robot configuration code
claw_motor1 = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)
claw_motor2 = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
if False:
    lift_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)
else:
    lift_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
controller_1 = Controller(PRIMARY)

if False:
    left_front_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
    left_back_motor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
    right_front_motor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
    right_back_motor = Motor(Ports.PORT4, GearSetting.RATIO_18_1, True)
else:
    left_front_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
    left_back_motor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)
    right_front_motor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
    right_back_motor = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)

inertial = Inertial(Ports.PORT5)

claw_solenoid = DigitalOut(brain.three_wire_port.a)

# Begin project code
# Main Controller loop to set motors to controller axis postiions
ROBOT_INITIALIZED = False
pitch_offset = 0.0

def pre_autonomous():
    global ROBOT_INITIALIZED
    global pitch_offset
    # actions to do when the program starts
    brain.screen.clear_screen()
    brain.screen.print("pre auton code")
    inertial.calibrate()
    while inertial.is_calibrating():
        wait(100, MSEC)
    for i in range(10):
        pitch_offset += inertial.orientation(OrientationType.ROLL, DEGREES)
        wait(10, MSEC)
    pitch_offset /= 10.0
    print("Pitch offset: {:.1f}".format(pitch_offset))
    
    ROBOT_INITIALIZED = True

def autonomous():
    while not ROBOT_INITIALIZED:
        wait(100, MSEC)
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place automonous code here

LIFT_RUNNING = False
LIFT_HOLDING = False
LIFT_LINKS = 30
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

def raise_lift():
    global lift_hold_time_start, LIFT_RUNNING, LIFT_HOLDING
    if LIFT_RUNNING: return
    LIFT_RUNNING = True
    LIFT_HOLDING = False
    starting_position = lift_motor.position(DEGREES)
    lift_motor.set_velocity(100, PERCENT)
    lift_motor.set_stopping(HOLD)
    lift_motor.set_timeout(3, SECONDS)
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
    lift_motor.set_timeout(3, SECONDS)
    lift_motor.spin_to_position(0, DEGREES)
    lift_motor.stop(HOLD)
    LIFT_RUNNING = False
    LIFT_HOLDING = True
    lift_hold_time_start = brain.timer.time(SECONDS)
    ending_position = lift_motor.position(DEGREES)
    total_links_moved = (starting_position - ending_position) / LIFT_DEGREES_PER_LINK
    #print("Lift down from {} to {}, total {} links".format(starting_position, ending_position, total_links_moved))

def lift_height(percent=False):
    if percent:
        return (lift_motor.position(DEGREES) / LIFT_DEGREES_PER_LINK) * (100 / LIFT_LINKS)
    return lift_motor.position(DEGREES) / LIFT_DEGREES_PER_LINK

def check_lift_hold():
    global lift_hold_time_start, LIFT_RUNNING, LIFT_HOLDING
    print("Checking lift hold")
    brain.timer.event(check_lift_hold, 10000)
    if not LIFT_HOLDING: return
    if LIFT_RUNNING: return
    if brain.timer.time(SECONDS) - lift_hold_time_start > 10.0:
        lift_motor.stop(COAST)
        LIFT_HOLDING = False

def OnControlButtonR2Pressed():
    global lift_thread, lift_hold_time_start, LIFT_RUNNING, LIFT_HOLDING
    if LIFT_RUNNING:
        print("Was Running")
        if lift_thread is not None: lift_thread.stop()
        lift_motor.stop(HOLD)
        LIFT_HOLDING = True
        LIFT_RUNNING = False
        lift_hold_time_start = brain.timer.time(SECONDS)
        return
    lift_thread = Thread(lower_lift)

def OnControlButtonR1Pressed():
    global lift_thread, lift_hold_time_start, LIFT_RUNNING, LIFT_HOLDING
    if LIFT_RUNNING:
        print("Was Running")
        if lift_thread is not None: lift_thread.stop()
        lift_motor.stop(HOLD)
        LIFT_RUNNING = False
        LIFT_HOLDING = True
        lift_hold_time_start = brain.timer.time(SECONDS)
        return
    lift_thread = Thread(raise_lift)

CLAW_RUNNING = False
CLAW_UP_REVS = 1.25
CLAW_MID2_REVS = 0.35 
CLAW_MID1_REVS = 0.20
CLAW_DOWN_REVS = 0.0
lift_links = 30
CLAW_DOWN = 0
CLAW_MID1 = 1
CLAW_MID2 = 2
CLAW_UP = 3
CLAW_POSITION = CLAW_DOWN  # 0 = down, 1 = mid1, 2 = mid2, 3 = up
CLAW_TIMEOUT = 1.0
CLAW_SPEED = 50

def raise_claw():
    global CLAW_RUNNING, CLAW_POSITION
    if CLAW_RUNNING: return

    if CLAW_POSITION == CLAW_DOWN:
        claw_target_position = CLAW_MID1
        claw_target_revs = CLAW_MID1_REVS
    elif CLAW_POSITION == CLAW_MID1:
        claw_target_position = CLAW_MID2
        claw_target_revs = CLAW_MID2_REVS
    elif CLAW_POSITION == CLAW_MID2:
        claw_target_position = CLAW_UP
        claw_target_revs = CLAW_UP_REVS
    else: return

    CLAW_RUNNING = True
    starting_position = claw_motor1.position(DEGREES)
    claw_motor1.set_velocity(CLAW_SPEED, PERCENT)
    claw_motor1.set_stopping(HOLD)
    claw_motor1.set_timeout(CLAW_TIMEOUT, SECONDS)
    claw_motor2.set_velocity(CLAW_SPEED, PERCENT)
    claw_motor2.set_stopping(HOLD)
    claw_motor2.set_timeout(CLAW_TIMEOUT, SECONDS)
    claw_motor1.spin_to_position(claw_target_revs * 360.0, DEGREES, wait=False)
    claw_motor2.spin_to_position(claw_target_revs * 360.0, DEGREES)
    claw_motor1.stop()
    claw_motor2.stop()
    CLAW_RUNNING = False
    CLAW_POSITION = claw_target_position
    ending_position = claw_motor1.position(DEGREES)
    total_links_moved = (ending_position - starting_position) / LIFT_DEGREES_PER_LINK
    print("Claw up from {} to {}, total {} links".format(starting_position, ending_position, total_links_moved))

def lower_claw():
    global CLAW_RUNNING, CLAW_POSITION
    if CLAW_RUNNING: return

    if CLAW_POSITION == CLAW_UP:
        claw_target_position = CLAW_MID2
        claw_target_revs = CLAW_MID2_REVS
    elif CLAW_POSITION == CLAW_MID2:
        claw_target_position = CLAW_MID1
        claw_target_revs = CLAW_MID1_REVS
    elif CLAW_POSITION == CLAW_MID1:
        claw_target_position = CLAW_DOWN
        claw_target_revs = CLAW_DOWN_REVS
    else: return

    CLAW_RUNNING = True
    starting_position = claw_motor1.position(DEGREES)
    claw_motor1.set_velocity(CLAW_SPEED / 2, PERCENT)
    claw_motor1.set_stopping(HOLD)
    claw_motor1.set_timeout(CLAW_TIMEOUT, SECONDS)
    claw_motor2.set_velocity(CLAW_SPEED / 2, PERCENT)
    claw_motor2.set_stopping(HOLD)
    claw_motor2.set_timeout(CLAW_TIMEOUT, SECONDS)
    claw_motor1.spin_to_position(claw_target_revs * 360.0, DEGREES, wait=False)
    claw_motor2.spin_to_position(claw_target_revs * 360.0, DEGREES)
    claw_motor1.stop()
    claw_motor2.stop()
    CLAW_RUNNING = False
    CLAW_POSITION = claw_target_position
    ending_position = claw_motor1.position(DEGREES)
    total_links_moved = (starting_position - ending_position) / LIFT_DEGREES_PER_LINK
    print("Claw down from {} to {}, total {} links".format(starting_position, ending_position, total_links_moved))

def OnControlButtonL2Pressed():
    if CLAW_RUNNING:
        print("Was Running")
        claw_motor1.stop(HOLD)
        claw_motor2.stop(HOLD)
        return
    thread = Thread(lower_claw)

def OnControlButtonL1Pressed():
    if CLAW_RUNNING:
        print("Was Running")
        claw_motor1.stop(HOLD)
        claw_motor2.stop(HOLD)
        return
    thread = Thread(raise_claw)

def OnControlButtonAPressed():
    if claw_solenoid.value() == 1:
        claw_solenoid.set(0)
    else:
        claw_solenoid.set(1)

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
DEFAULT_TURN_MAX = 66.0 # maximum turn rate
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

def drivetrain_detwitch(speed, turn, enabled):
    '''
    ### (INTERNAL) )ETWITCH - reduce turn sensitiviy when robot is moving slowly (turning in place)

    NOTE: speed is not altered only turn

    :param speed: speed in percent - from -100 to +100 (full reverse to full forward)
    :param turn: turn in percent - from -100 to +100 (full left turn to full right turn)
    :param enabled: indicates whether to enable the detwitch code or not

    :return: speed (unmodified) and turn based on simple straight line segments
    '''

    if not enabled:
        return speed * drive_max / 100.0, turn * turn_max / 100.0

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
AUTO_TURN_KP = 1.4
AUTO_TURN_KD = 0.1
AUTO_FORWARD_KP = 5.0
NO_INPUT_TIMEOUT = 200

def user_control():

    brain.screen.clear_screen()
    brain.screen.print("driver control")
    # place driver control in this while loop
    last_fwd = 0 
    while not ROBOT_INITIALIZED:
        wait(100, MSEC)

    brain.screen.clear_screen()
    brain.screen.print("user control")

    controller_1.buttonA.pressed(OnControlButtonAPressed)

    controller_1.buttonR2.pressed(OnControlButtonR2Pressed)
    controller_1.buttonR1.pressed(OnControlButtonR1Pressed)

    controller_1.buttonL2.pressed(OnControlButtonL2Pressed)
    controller_1.buttonL1.pressed(OnControlButtonL1Pressed)

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

    loop_count = 0

    # place driver control in this while loop
    while True:
        #if add_sample():
        #    dumpsamples()
            
        raw_forward = controller_1.axis3.position()
        raw_strafe = controller_1.axis4.position()
        raw_turn = controller_1.axis1.position()

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

        # Remap from field to robot
        FIELD_ORIENTED = False
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

        turn = drivetrain_detwitch(forward, raw_turn, True)[1]

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
        elif forward != 0 or strafe != 0:
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
            left_front_motor.set_velocity((combined_forward + combined_turn + combined_strafe), PERCENT)
            left_back_motor.set_velocity((combined_forward + combined_turn - combined_strafe), PERCENT)
            right_front_motor.set_velocity((combined_forward - combined_turn - combined_strafe), PERCENT)
            right_back_motor.set_velocity((combined_forward - combined_turn + combined_strafe), PERCENT)

            left_front_motor.spin(FORWARD)
            left_back_motor.spin(FORWARD)
            right_front_motor.spin(FORWARD)
            right_back_motor.spin(FORWARD)

        wait(10, MSEC)

        if loop_count % 100 == 0:
            #print("Lift {}".format(lift_motor.position(DEGREES)))
            print("Rotation: {:.1f}".format(inertial.rotation(DEGREES)))

        loop_count += 1

# create competition instance
comp = Competition(user_control, autonomous)
pre_autonomous()