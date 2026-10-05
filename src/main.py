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
from xdrivetrain import XDriveTrain
from math import radians, degrees, cos, asin, sin, sqrt, pi

DRIVE_MOTION_MODEL = False
try:
    from motionmodel import DriveMotionModel
    DRIVE_MOTION_MODEL = True
except ImportError:
    print("Failed to import DriveMotionModel")

# ------------------------------------------------------------ #
### SETUP DEFAULT ALLIANCE AND AUTONOMOUS SEQUENCE HERE
# ------------------------------------------------------------ #

CALIBRATION = 99

ALLIANCE_COLOR = AllianceColor.RED
# ALLIANCE_COLOR = AllianceColor.BLUE

# AUTON_SEQUENCE = AutonSequence.SKILLS
# AUTON_SEQUENCE = AutonSequence.MATCH_LEFT
AUTON_SEQUENCE = AutonSequence.MATCH_RIGHT
# AUTON_SEQUENCE = AutonSequence.MATCH_NONE
# AUTON_SEQUENCE = CALIBRATION

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
BACK_DISTANCE1_FROM_BACK = 75 # mm
BACK_DISTANCE2_FROM_BACK = 69 # mm
BACK_DISTANCE1_CLOSE_ERROR = 0 # mm (at 145mm reads 145mm)
BACK_DISTANCE2_CLOSE_ERROR = 6 #mm (at 145mm reads 151mm)
back_distance1 = Distance(Ports.PORT4) # left side
back_distance2 = Distance(Ports.PORT9) # right side

ROBOT_WIDTH = 192 * 2 # (mm)
ROBOT_LENGTH = 183 * 2 # (mm) 185 measured from back wall to center line

LEFT_DISTANCE_COMPENSATION = 1.0
LEFT_DISTANCE_FROM_LEFT = 5 # mm
LEFT_DISTANCE_CLOSE_ERROR = -15 # mm (at 145mm reads 130mm, at 200mm reads 185mm, at 250mm reads 250mm, at 295mm reads 295mm)
LEFT_DISTANCE_FAR_ERROR = 0 # mm roughly at 250mm
LEFT_DISTANCE_FAR_TRANSITION = 250 # mm
left_distance = Distance(Ports.PORT6)

RIGHT_DISTANCE_COMPENSATION = 1.0
RIGHT_DISTANCE_FROM_RIGHT = 5 # mm
RIGHT_DISTANCE_CLOSE_ERROR = 0 # mm (at 145mm reads 145mm)
RIGHT_DISTANCE_CLOSE_TRANSITION = 400 # mm
RIGHT_DISTANCE_FAR_ERROR = 18 # mm (at 1605mm reads 1623mm, at 1205mm reads 1015mm, at 407mm reads 407mm)
RIGHT_DISTANCE_FAR_TRANSITION = 1600 # mm
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

class Toggle:

    @staticmethod
    def raise_toggle():
        toggle_solenoid.set(0)

    @staticmethod
    def lower_toggle():
        toggle_solenoid.set(1)

    @staticmethod
    def toggle_raised():
        return toggle_solenoid.value() == 0

### LIFT CONTROL

class Lift:

    LIFT_LINKS = 31
    LIFT_TEETH = 6
    LIFT_DEGREES_PER_LINK = 360 / LIFT_TEETH

    def __init__(self, motor1: Motor):
        self.motor1 = motor1
        self.running = False
        self.holding = False
        self.hold_time_start = 0

    def initialize_lift(self):
        self.motor1.set_stopping(BrakeType.HOLD)
        self.motor1.set_velocity(100, PERCENT)
        self.motor1.set_timeout(3, SECONDS)
        self.motor1.spin(REVERSE)
        wait(1, SECONDS)
        self.motor1.stop(BrakeType.HOLD)
        wait(100, MSEC)
        self.motor1.set_position(0, DEGREES)
        self.motor1.stop(COAST)

    def get_height(self, percent=False):
        if percent:
            return (self.motor1.position(DEGREES) / self.LIFT_DEGREES_PER_LINK) * (100 / self.LIFT_LINKS)
        return self.motor1.position(DEGREES) / self.LIFT_DEGREES_PER_LINK

    def is_running(self):
        return self.running

    def command(self, links):
        if self.running: return

        self.running = True
        self.holding = False

        starting_position = self.motor1.position(DEGREES)

        self.motor1.set_velocity(100, PERCENT)
        self.motor1.set_stopping(BrakeType.HOLD)
        self.motor1.set_timeout(5, SECONDS)
        self.motor1.spin_to_position(links * self.LIFT_DEGREES_PER_LINK, DEGREES)
        self.motor1.stop()

        self.running = False
        self.holding = True
        self.hold_time_start = brain.timer.time(SECONDS)
        ending_position = self.motor1.position(DEGREES)
        total_links_moved = (ending_position - starting_position) / self.LIFT_DEGREES_PER_LINK

        print("Lift up from {} to {}, total {} links".format(starting_position, ending_position, total_links_moved))

        return total_links_moved

    def raise_lift(self):
        if self.running: return

        self.running = True
        self.holding = False

        starting_position = self.motor1.position(DEGREES)

        self.motor1.set_velocity(100, PERCENT)
        self.motor1.set_stopping(BrakeType.HOLD)
        self.motor1.set_timeout(5, SECONDS)
        self.motor1.spin_to_position(self.LIFT_LINKS * self.LIFT_DEGREES_PER_LINK, DEGREES)
        self.motor1.stop()

        self.running = False
        self.holding = True

        self.hold_time_start = brain.timer.time(SECONDS)
        ending_position = self.motor1.position(DEGREES)
        total_links_moved = (ending_position - starting_position) / self.LIFT_DEGREES_PER_LINK

        print("Lift up from {} to {}, total {} links".format(starting_position, ending_position, total_links_moved))
        return total_links_moved

    def lower_lift(self):
        if self.running: return

        self.running = True
        self.holding = False
        starting_position = self.motor1.position(DEGREES)

        self.motor1.set_velocity(100, PERCENT)
        self.motor1.set_stopping(BrakeType.HOLD)
        self.motor1.set_timeout(5, SECONDS)
        self.motor1.spin_to_position(0, DEGREES)

        if self.get_height(percent=True) <= 1:
            print("Lift is near the bottom, coasting")
            self.motor1.stop(BrakeType.COAST)
        else:
            self.motor1.stop(BrakeType.HOLD)
        self.running = False
        self.holding = True

        self.hold_time_start = brain.timer.time(SECONDS)
        ending_position = self.motor1.position(DEGREES)
        total_links_moved = (starting_position - ending_position) / self.LIFT_DEGREES_PER_LINK

        print("Lift down from {} to {}, total {} links".format(starting_position, ending_position, total_links_moved))

        return total_links_moved

    def check_lift_hold(self):
        print("Checking lift hold")
        brain.timer.event(self.check_lift_hold, 10000)
        if not self.holding: return
        if self.running: return
        if brain.timer.time(SECONDS) - self.hold_time_start > 10.0:
            self.motor1.stop(BrakeType.COAST)
            self.holding = False

    def stop(self):
        if lift.running:
            print("Was Running")
    
        lift.motor1.stop(BrakeType.HOLD)
        lift.holding = True
        lift.running = False
    
        lift.hold_time_start = brain.timer.time(SECONDS)

lift = Lift(lift_motor)

### CLAW CONTROL

class Arm:

    CLAW_ARM_UP_DEGREES = 170
    CLAW_ARM_MID3_DEGREES = 34 # was 24.5 * 3
    CLAW_ARM_MID2_DEGREES = 24.5 # was 24.5 * 3
    CLAW_ARM_MID1_DEGREES = 12 # was 18 * 3
    CLAW_ARM_DOWN_DEGREES = 0

    CLAW_ARM_GEAR_RATIO = 3

    CLAW_ARM_DOWN = 0
    CLAW_ARM_MID1 = 1
    CLAW_ARM_MID2 = 2
    CLAW_ARM_MID3 = 3
    CLAW_ARM_UP = 4
    CLAW_ARM_UNKNOWN = 5

    CLAW_ARM_TIMEOUT = 2.0
    CLAW_ARM_SPEED = 50

    CLAW_ARM_COMMAND_NONE = 0
    CLAW_ARM_COMMAND_RAISE = 1
    CLAW_ARM_COMMAND_LOWER = 2
    CLAW_ARM_COMMAND_TO_POSITION = 3
    CLAW_ARM_COMMAND_TO_ANGLE = 5
    CLAW_ARM_COMMAND_CANCEL = 6

    def __init__(self, motor1, motor2):
        self.motor1 = motor1
        self.motor2 = motor2
        self.position = self.CLAW_ARM_DOWN  # 0 = down, 1 = mid1, 2 = mid2, 3 = mid3, 4 = up
        self.initialized = False
        self.running = False
        self.cancel_operation = False
        self.was_cancelled = False

        self.positions_list = [self.CLAW_ARM_DOWN, self.CLAW_ARM_MID1, self.CLAW_ARM_MID2, self.CLAW_ARM_MID3, self.CLAW_ARM_UP]
        self.target_list = [self.CLAW_ARM_DOWN_DEGREES, self.CLAW_ARM_MID1_DEGREES, self.CLAW_ARM_MID2_DEGREES, self.CLAW_ARM_MID3_DEGREES, self.CLAW_ARM_UP_DEGREES]

    def initialize(self):
        if self.initialized: return
        self.motor1.set_velocity(30, PERCENT)
        self.motor1.set_stopping(BrakeType.HOLD)
        self.motor1.set_timeout(2, SECONDS)
        self.motor2.set_velocity(30, PERCENT)
        self.motor2.set_stopping(BrakeType.HOLD)
        self.motor2.set_timeout(2, SECONDS)
        self.motor1.spin_to_position(-30, DEGREES, wait=False)
        self.motor2.spin_to_position(-30, DEGREES)
        wait(0.25, SECONDS)
        self.motor1.set_position(0, DEGREES)
        self.motor2.set_position(0, DEGREES)
        self.motor1.stop(BrakeType.HOLD)
        self.motor2.stop(BrakeType.HOLD)
        self.initialized = True

    def get_position(self):
        return self.position

    def is_initialized(self):
        return self.initialized

    def is_running(self):
        return self.running

    def run_claw_arm(self, command, target_position=-1, speed=-1):

        # WARNING: RE-ENTRANT CODE
        if self.running and command != self.CLAW_ARM_COMMAND_CANCEL: return

        if command == self.CLAW_ARM_COMMAND_CANCEL:
            self.cancel_operation = True
            return
        self.cancel_operation = False
        # END RE-ENTRANT CODE

        if command == self.CLAW_ARM_COMMAND_NONE: return

        if command == self.CLAW_ARM_COMMAND_RAISE:
            if self.was_cancelled:
                claw_target_position = self.CLAW_ARM_UP
            elif self.position == self.CLAW_ARM_UNKNOWN:
                claw_target_position = self.CLAW_ARM_UP
            else:
                if self.position >= self.CLAW_ARM_UP : return
                claw_target_position = self.position + 1
                # MID3 only used for autonomous
                if (claw_target_position == self.CLAW_ARM_MID3): claw_target_position += 1
            arm_speed = self.CLAW_ARM_SPEED
        elif command == self.CLAW_ARM_COMMAND_LOWER:
            if self.was_cancelled:
                claw_target_position = self.CLAW_ARM_DOWN
            elif self.position == self.CLAW_ARM_UNKNOWN:
                claw_target_position = self.CLAW_ARM_DOWN
            else:
                if self.position <= self.CLAW_ARM_DOWN: return
                claw_target_position = self.position - 1
                # MID3 only used for autonomous
                if (claw_target_position == self.CLAW_ARM_MID3): claw_target_position -= 1
            arm_speed = self.CLAW_ARM_SPEED * 0.75
        elif command == self.CLAW_ARM_COMMAND_TO_POSITION:
            if target_position == self.position: return
            if target_position < self.CLAW_ARM_DOWN or target_position > self.CLAW_ARM_UP: return
            claw_target_position = target_position
            if target_position > self.position: arm_speed = self.CLAW_ARM_SPEED
            else: arm_speed = self.CLAW_ARM_SPEED * 0.75
        elif command == self.CLAW_ARM_COMMAND_TO_ANGLE:
            claw_target_degrees = target_position
            claw_target_position = self.CLAW_ARM_UNKNOWN
            arm_speed = self.CLAW_ARM_SPEED

        if claw_target_position >= self.CLAW_ARM_MID3:
            Toggle.raise_toggle()

        if claw_target_position != self.CLAW_ARM_UNKNOWN:
            claw_target_degrees = self.target_list[claw_target_position]

        if speed > 0:
            arm_speed = speed

        self.running = True
        self.was_cancelled = False
        starting_position = (self.motor1.position(DEGREES), self.motor2.position(DEGREES))
        self.motor1.set_velocity(arm_speed, PERCENT)
        self.motor1.set_stopping(HOLD)
        self.motor1.set_timeout(self.CLAW_ARM_TIMEOUT, SECONDS)
        self.motor2.set_velocity(arm_speed, PERCENT)
        self.motor2.set_stopping(HOLD)
        self.motor2.set_timeout(self.CLAW_ARM_TIMEOUT, SECONDS)
        self.motor1.spin_to_position(claw_target_degrees * self.CLAW_ARM_GEAR_RATIO, DEGREES, wait=False)
        self.motor2.spin_to_position(claw_target_degrees * self.CLAW_ARM_GEAR_RATIO, DEGREES, wait=False)
        count = 0
        while not (self.motor1.is_done() and self.motor2.is_done()) and not self.cancel_operation: 
            wait(10, MSEC)
            count += 1
        # wait(333, MSEC)
        self.motor1.stop()
        self.motor2.stop()
        self.running = False
        if self.cancel_operation:
            self.cancel_operation = False
            self.was_cancelled = True
        self.position = claw_target_position
        ending_position = (self.motor1.position(DEGREES), self.motor2.position(DEGREES))
        print("Claw from {} to {}, {}ms".format(starting_position, ending_position, count * 10))

    def raise_claw_arm(self):
        self.run_claw_arm(self.CLAW_ARM_COMMAND_RAISE)

    def lower_claw_arm(self):
        self.run_claw_arm(self.CLAW_ARM_COMMAND_LOWER)

    def move_claw_arm_to_position(self, target_position, speed=-1):
        self.run_claw_arm(self.CLAW_ARM_COMMAND_TO_POSITION, target_position, speed)

    def claw_arm_current_position(self):
        return self.position

arm = Arm(claw_arm_motor1, claw_arm_motor2)

class Claw:

    OPEN = 1
    CLOSED = 0

    def __init__(self, solenoid):
        self.solenoid = solenoid
        self.open_time = 0

    def is_open(self):
        return self.solenoid.value() == Claw.OPEN

    def is_closed(self):
        return self.solenoid.value() == Claw.CLOSED

    def open(self):
        self.open_time = brain.timer.time(SECONDS)
        self.solenoid.set(Claw.OPEN)

    def close(self):
        self.solenoid.set(Claw.CLOSED)

claw = Claw(claw_solenoid)

def auto_claw_thread():
    while True:
        current_time = brain.timer.time(SECONDS)
        claw_ready = (current_time - claw.open_time) > 1
        if ROBOT_ENABLED and claw_ready and claw.is_open() and lift.get_height(True) < 1:
            if (robot_config.enable_auto_claw_down and arm.get_position() == Arm.CLAW_ARM_DOWN):
                if (claw_distance.object_distance() < 70):
                    claw.close()
                    wait(1, SECONDS)
            elif (robot_config.enable_auto_claw_mid1 and arm.get_position() == Arm.CLAW_ARM_MID1):
                if (claw_distance.object_distance() < 70):
                    claw.close()
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

def location_callback():
    return X, Y, THETA

### MOTOR COMMAND VELOCITIES

dt = XDriveTrain(left_front_motor, left_back_motor, right_front_motor, right_back_motor, inertial, location_callback)

def set_quiet_mode(quiet):
    global QUIET_MODE
    QUIET_MODE = quiet
    dt.set_quiet_mode(quiet)

### DISTANCE SENSORS

ENABLE_BACK_DISTANCE = True
ENABLE_LEFT_DISTANCE = True
ENABLE_RIGHT_DISTANCE = True
ENABLE_LOCATION_FILTER = False

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
            back1_value = back_distance1.object_distance(MM) - BACK_DISTANCE1_CLOSE_ERROR
            back1_count += 1
            back1_distance += back1_value
        else:
            back1_value = None

        if back_distance2.is_object_detected():
            back2_value = back_distance2.object_distance(MM) - BACK_DISTANCE2_CLOSE_ERROR
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
    forward = forward * DRIVETRAIN_EXTERNAL_GEAR_RATIO * DRIVETRAIN_WHEEL_SIZE * sqrt(2) * dt.FORWARD_EFFICIENCY
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

def filter_distance_by_angle(rotation, tolerance):
    # Returns True if angle is valid
    heading = rotation

    heading = rotation % 360.0
    if heading >= 360.0:
        heading = 0.0
        
    if heading < 0.0 or heading >= 360.0:
        raise ValueError("filter_distance({}, {}): Heading must be in the range [0, 360).".format(heading, rotation))
    
    if heading > 360.0 - tolerance or heading < tolerance:
        return True
    elif heading > 90.0 - tolerance and heading < 90.0 + tolerance:
        return True
    elif heading > 180.0 - tolerance and heading < 180.0 + tolerance:
        return True
    elif heading > 270.0 - tolerance and heading < 270.0 + tolerance:
        return True
    return False

EFORWARD = 0
ERIGHT = 1
EBACK = 2
ELEFT = 3

NORTH = 0
EAST = 1
SOUTH = 2
WEST = 3

def compass_heading(rotation: float) -> int:
    heading = rotation

    heading = rotation % 360.0
    # needed as floating vs. double precision module may produce slightly different results for values very close to 360.0
    if heading >= 360.0:
        heading -= 360.0

    if heading >= 315 or heading < 45:
        return NORTH
    elif heading >= 45 and heading < 135:
        return EAST
    elif heading >= 135 and heading < 225:
        return SOUTH
    else:
        return WEST

def filter_distance_by_location(direction, x, y, rotation):
    compass = compass_heading(rotation)
    if direction == EFORWARD:
        if compass == NORTH: return x > 1800
        elif compass == EAST: return y > 1800
        elif compass == SOUTH: return x < 1800
        elif compass == WEST: return y < 1800
    elif direction == ERIGHT:
        if compass == NORTH: return y > 1600
        elif compass == EAST: return x < 1800
        elif compass == SOUTH: return y < 2000
        elif compass == WEST: return x > 1800
    elif direction == EBACK:
        if compass == NORTH: return x < 1800
        elif compass == EAST: return y < 1800
        elif compass == SOUTH: return x > 1800
        elif compass == WEST: return y > 1800
    elif direction == ELEFT:
        if compass == NORTH: return y < 1600
        elif compass == EAST: return x > 1800
        elif compass == SOUTH: return y > 2000
        elif compass == WEST: return x < 1800

    return False
    
def get_back_distance(rotation):
    global previous_back_distance

    if not filter_distance_by_angle(rotation, 5):
        return None

    if ENABLE_LOCATION_FILTER and not filter_distance_by_location(EBACK, X, Y, rotation):
        return None

    new_back1_timestamp = back_distance1.timestamp()
    new_back2_timestamp = back_distance2.timestamp()

    max_timestamp = max(new_back1_timestamp, new_back2_timestamp)

    if max_timestamp > previous_back_distance[1]:
        if not back_distance1.is_object_detected() or not back_distance2.is_object_detected(): return None
        new_back1 = back_distance1.object_distance(MM) - BACK_DISTANCE1_CLOSE_ERROR
        new_back2 = back_distance2.object_distance(MM) - BACK_DISTANCE2_CLOSE_ERROR
        new_back_distance_value = (new_back1 + new_back2) / 2.0
        new_back_distance_timestamp = max_timestamp
        previous_back_distance[0] = new_back_distance_value
        previous_back_distance[1] = new_back_distance_timestamp

        if new_back_distance_value > 1700.0:
            return None
        
        return new_back_distance_value

    return None

def logistic_error(d, error, midpoint = 175, steepness = 0.05):
    # Midpoint at 175mm, steepness 0.05
    # Uses math.exp instead of np.exp
    return error / (1 + math.exp(steepness * (d - midpoint)))

def get_left_distance(rotation):
    global previous_left_distance

    if not filter_distance_by_angle(rotation, 5):
        return None

    if ENABLE_LOCATION_FILTER and not filter_distance_by_location(ELEFT, X, Y, rotation):
        return None

    loader_offset = 0
    if AUTON_SEQUENCE == AutonSequence.MATCH_RIGHT:
        if X >= 260 and X <=360:
            loader_offset = 90

    new_left_timestamp = left_distance.timestamp()

    if new_left_timestamp > previous_left_distance[1]:
        if not left_distance.is_object_detected(): return None
        new_left_distance_value = left_distance.object_distance(MM)
        try:
            error = logistic_error(new_left_distance_value, LEFT_DISTANCE_CLOSE_ERROR)
        except Exception as e:
            # print("Error calculating logistic error: {}, {}".format(e, new_left_distance_value))
            error = 0
        new_left_distance_value -= error
        new_left_distance_timestamp = new_left_timestamp
        previous_left_distance[0] = new_left_distance_value
        previous_left_distance[1] = new_left_distance_timestamp

        if new_left_distance_value > 1700.0:
            return None

        # print(new_left_distance_value)
        return new_left_distance_value + loader_offset

    return None

def get_right_distance(rotation):
    global previous_right_distance

    if not filter_distance_by_angle(rotation, 5):
        return None

    if ENABLE_LOCATION_FILTER and not filter_distance_by_location(ERIGHT, X, Y, rotation):
        return None

    loader_offset = 0
    if AUTON_SEQUENCE == AutonSequence.MATCH_RIGHT:
        if X >= 260 and X <=360:
            loader_offset = 90

    new_right_timestamp = right_distance.timestamp()

    if new_right_timestamp > previous_right_distance[1]:
        if not right_distance.is_object_detected(): return None
        new_right_distance_value = right_distance.object_distance(MM)
        if new_right_distance_value < RIGHT_DISTANCE_CLOSE_TRANSITION:
            # Close region just subtract the close value
            new_right_distance_value -= RIGHT_DISTANCE_CLOSE_ERROR
        else:
            # Far region apply linear interpolation using delta between far and close transition and far and close errors
            slope = (RIGHT_DISTANCE_FAR_ERROR - RIGHT_DISTANCE_CLOSE_ERROR) / (RIGHT_DISTANCE_FAR_TRANSITION - RIGHT_DISTANCE_CLOSE_TRANSITION)
            error = (new_right_distance_value - RIGHT_DISTANCE_CLOSE_TRANSITION) * slope
            new_right_distance_value -= error
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
        side_offset = ROTATION_SIDE_WHEEL_OFFSET
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

def odom_distance_enable(back, left, right):
    global ENABLE_BACK_DISTANCE, ENABLE_LEFT_DISTANCE, ENABLE_RIGHT_DISTANCE

    ENABLE_BACK_DISTANCE = back
    ENABLE_LEFT_DISTANCE = left
    ENABLE_RIGHT_DISTANCE = right

def odom_location_filter_enable(enable):
    global ENABLE_LOCATION_FILTER
    ENABLE_LOCATION_FILTER = enable

def odom_print():
    if not QUIET_MODE:
        print("X: {:4.0f}/{:0.1f}, Y: {:4.0f}/{:0.1f}, H: {:3.2f}, B: {:.0f}/{:.0f}, L: {:.0f}, R: {:.0f}".format(X, Pxx, Y, Pyy, THETA % 360.0, back_distance1.object_distance(MM), back_distance2.object_distance(MM), left_distance.object_distance(MM), right_distance.object_distance(MM)))

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
        compass = compass_heading(THETA)

        if ENABLE_BACK_DISTANCE:
            meas_back_distance = get_back_distance(THETA)
            if meas_back_distance is not None:
                meas_back_distance -= (BACK_DISTANCE1_FROM_BACK + BACK_DISTANCE2_FROM_BACK) / 2 # subtract distance to back of robot
                meas_back_distance += HIDDEN_PERIMITER + ROBOT_LENGTH / 2 # add the hidden perimeter and half the robot length
                if compass == NORTH:
                    X, Y = filter.update_x(meas_back_distance)
                elif compass == SOUTH:
                    X, Y = filter.update_x(3600.0 - meas_back_distance)
                elif compass == WEST:
                    X, Y = filter.update_y(3600.0 - meas_back_distance)
                elif compass == EAST:
                    X, Y = filter.update_y(meas_back_distance)

        if ENABLE_RIGHT_DISTANCE:
            meas_right_distance = get_right_distance(THETA)
            if meas_right_distance is not None:
                meas_right_distance += HIDDEN_PERIMITER + ROBOT_WIDTH / 2 - RIGHT_DISTANCE_FROM_RIGHT
                if compass == NORTH:
                    X, Y = filter.update_y(3600.0 - meas_right_distance)
                elif compass == SOUTH:
                    X, Y = filter.update_y(meas_right_distance)
                elif compass == WEST:
                    X, Y = filter.update_x(3600.0 - meas_right_distance)
                elif compass == EAST:
                    X, Y = filter.update_x(meas_right_distance)

        if ENABLE_LEFT_DISTANCE:
            meas_left_distance = get_left_distance(THETA)
            if meas_left_distance is not None:
                meas_left_distance += HIDDEN_PERIMITER + ROBOT_WIDTH / 2 - LEFT_DISTANCE_FROM_LEFT
                if compass == NORTH:
                    X, Y = filter.update_y(meas_left_distance)
                elif compass == SOUTH:
                    X, Y = filter.update_y(3600.0 - meas_left_distance)
                elif compass == WEST:
                    X, Y = filter.update_x(meas_left_distance)
                elif compass == EAST:
                    X, Y = filter.update_x(3600.0 - meas_left_distance)

        Pxx, Pyy = filter.Pxx, filter.Pyy

        if count % 200 == 0:
            odom_print()

        count += 1

        wait(10, MSEC)

def turns():
    wait(100, MSEC)
    dt.turn_for(-10, speed = 50)
    wait(100, MSEC)
    dt.turn_for(20, speed = 50)
    wait(100, MSEC)
    dt.turn_for(-20, speed = 50)
    wait(100, MSEC)
    dt.turn_for(20, speed = 50)
    wait(100, MSEC)
    dt.turn_for(-20, speed = 50)
    wait(100, MSEC)
    dt.turn_for(20, speed = 50)
    wait(100, MSEC)
    dt.turn_for(-20, speed = 50)
    wait(100, MSEC)
    left_front_motor.stop(COAST)
    left_back_motor.stop(COAST)
    right_front_motor.stop(COAST)
    right_back_motor.stop(COAST)
    lift_motor.stop(COAST)
    claw_arm_motor1.stop(COAST)
    claw_arm_motor2.stop(COAST)

# NOTE: Current setup seems to have an angle offset of about 1 deg, ie 1.0 to the the cup angle
# sensor updates are around 100ms
# sensor lag is about 180-200ms

def center_on_cup():
    set_quiet_mode(True)
    claw.open()

    CUP_ID = 4

    # intrinsics
    fov_h_deg = 74
    fov_v_deg = 63
    img_w = 320
    img_h = 240

    # principal point
    c_x = img_w / 2.0
    c_y = img_h / 2.0

    # focal lengths in pixels
    f_y = (img_h / 2.0) / math.tan(math.radians(fov_v_deg) / 2.0)
    f_x = (img_w / 2.0) / math.tan(math.radians(fov_h_deg) / 2.0)

    # camera down pitch
    camera_down_pitch_deg = 24
    
    ai = AiVision(Ports.PORT7, AiVision.ALL_AIOBJS)

    loop_count = 0
    CAMERA_LAG_MS = 199
    THETA_HISTORY_CAPACITY = 64
    theta_history_times = [0] * THETA_HISTORY_CAPACITY
    theta_history_values = [0.0] * THETA_HISTORY_CAPACITY
    theta_history_next = 0
    theta_history_count = 0
    target_heading = THETA
    error_accum = 0.0
    have_target = False
    ALPHA = 0.3
    save_buffer = []
    Thread(turns)
    while loop_count < 500:
        if True or loop_count % 10 == 0:
            no_objects = True
            aiobjects_by_id = sorted(ai.take_snapshot(AiVision.ALL_AIOBJS), key=id)

            snapshot_time = brain.timer.time(MSEC)
            theta_history_times[theta_history_next] = snapshot_time
            theta_history_values[theta_history_next] = THETA
            theta_history_next = (theta_history_next + 1) % THETA_HISTORY_CAPACITY
            if theta_history_count < THETA_HISTORY_CAPACITY:
                theta_history_count += 1

            target_sample_time = snapshot_time - CAMERA_LAG_MS
            delayed_THETA = THETA
            for history_age in range(theta_history_count):
                history_index = (theta_history_next - 1 - history_age) % THETA_HISTORY_CAPACITY
                if theta_history_times[history_index] <= target_sample_time:
                    delayed_THETA = theta_history_values[history_index]
                    break

            for aiobject in aiobjects_by_id:

                if aiobject.id == CUP_ID and aiobject.score > 95:
                    no_objects = False

                    pitch = math.radians(camera_down_pitch_deg)

                    x = (aiobject.centerX - c_x) / f_x
                    y = (aiobject.centerY - c_y) / f_y

                    cup_angle = math.degrees(math.atan2(
                        x,
                        math.cos(pitch) - y * math.sin(pitch)
                    ))

                    if loop_count < 500:
                        save_buffer.append([-cup_angle, delayed_THETA, aiobject.centerX, aiobject.centerY, snapshot_time])
                    cup_heading = (delayed_THETA + cup_angle) % 360
                    if cup_heading >= 360: cup_heading = 0 # this is needed as python library can prodoce 360 with small negative angles
                    if not have_target:
                        target_heading = cup_heading
                        have_target = True
                    else:
                        delta = (cup_heading - target_heading + 180) % 360 - 180
                        target_heading = (target_heading + ALPHA * delta) % 360
                    # print(target_heading, aiobject.score)
                    break

            if no_objects:
                    if loop_count < 500:
                        save_buffer.append([0, delayed_THETA, -1, -1, snapshot_time])

        # heading_error = (target_heading - THETA + 180) % 360 - 180
        # error_accum += heading_error

        # if loop_count % 100 == 0:
        #    print("Heading error:", heading_error)

        # turn_control = 5.0 * heading_error / 360.0 + 0.01 * error_accum / 360.0
        # turn_control = XDriveTrain.limit(turn_control, 1.0) * 100.0

        # left_front_motor.spin(FORWARD, turn_control, PERCENT)
        # left_back_motor.spin(FORWARD, turn_control, PERCENT)
        # right_front_motor.spin(REVERSE, turn_control, PERCENT)
        # right_back_motor.spin(REVERSE, turn_control, PERCENT)

        # dt.turn_to(cup_heading, speed = 25, settle_error = 0.5)
        loop_count += 1
        wait(10, MSEC)

    print("neg_cpu_angle, delayed_THETA, centerX, centerY, snapshot_time")
    buffer_count = len(save_buffer)
    for i in range(buffer_count):
        entry = save_buffer[i]
        print("{}, {}, {}, {}, {}".format(entry[0], entry[1], entry[2], entry[3], entry[4]))
        wait(250, MSEC)

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

    set_quiet_mode(True)
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

    TOTAL_SAMPLES = 500
    PRINT_DELAY = 250 # ms between samples. Set to around 250 for wireless or 50 for USB

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

    set_quiet_mode(True)
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

    #odom_distance_enable(False, False, False)
    #wait(100, MSEC)
    #dt.turn_for(90, 25)
    #wait(100, MSEC)
    #return

    center_on_cup()
    return

    odom_distance_enable(True, False, True)

    wait(100, MSEC)

    dt.drive_to_xy(400, 1800, False, 25, heading = 0)
    dt.drive_to_xy(400, 3000, True, 25, heading = 0)
    wait(100, MSEC)
    return

    speed = 33
    turn_speed = 75
    first_run = True

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
            if first_run:
                Thread(log_odom)
                wait(100, MSEC)
                first_run = False
            if use_distance: odom_distance_enable(False, False, False)
            dt.turn_for(-90, turn_speed)
            if use_distance: odom_distance_enable(True, True, False)
            #wait(500, MSEC)
            dt.drive_to_xy(300.0 + 100, 2750 + 400.0, False, speed, heading = -90)
            #wait(500, MSEC)
            dt.drive_to_xy(300.0 + 100, 2750 - 400.0, False, speed, heading = -90)
            #wait(500, MSEC)
            if use_distance: odom_distance_enable(False, False, False)
            dt.turn_for(90, turn_speed)
            if use_distance: odom_distance_enable(True, False, True)
            #wait(500, MSEC)
            dt.drive_to_xy(300.0, 2750 - 400.0, False, speed, heading = 0)

            speed += 33
            if speed > 100:
                speed = 33


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

def autonomous_skills_old():
    # Thread(odom_thread)
    # place automonous code here
    while not arm.is_initialized():
        wait(10, MSEC)
    wait(1, SECONDS)
    arm.run_claw_arm(Arm.CLAW_ARM_COMMAND_TO_POSITION, Arm.CLAW_ARM_MID1)
    wait(500, MSEC)
    # Thread(log_drivetrain)
    dt.drive_for(51 * 25.4, False, 50, heading = 0)
    dt.drive_for(-450, True, 50, heading = 0)
    lift.command(13)
    dt.drive_for(11 * 25.4, False, 50, timeout = 5000, heading = 0)
    arm.run_claw_arm(Arm.CLAW_ARM_COMMAND_TO_POSITION, Arm.CLAW_ARM_MID2)
    lift.command(10)
    wait(500, MSEC)
    claw.open()
    dt.drive_for(-150, False, 50, heading = 0)
    # TODO: Move back to safe distance

def autonomous_skills_cup_from_floor():
    claw.open()
    dt.drive_to_xy(900.0, 1800.0, False, 100, heading = 0)
    dt.drive_to_xy(900.0, 2400.0-25, True, 66, heading = 0)
    wait(500, MSEC)
    dt.drive_to_xy(900.0, 2400.0-25.0, True, 66, heading = 0)
    distance = claw_distance.object_distance(MM)
    dt.drive_for(distance - 40, False, 33, heading = 0)
    claw.close()

def autonomous_skills():
    claw.open()
    dt.drive_to_xy(300.0, 1800.0, False, 100, heading = 0)
    dt.turn_for(180, 75)
    dt.drive_to_xy(300.0, 3600-300, True, 66, heading = 180)
    distance = claw_distance.object_distance(MM)
    print("Claw_distance: {}".format(distance))
    dt.drive_for(distance - 70, False, 33, heading = 180)
    claw.close()
    dt.drive_for(50, False, 25, heading = 180)

def autonomous_none():
    # place automonous code here
    Toggle.lower_toggle()
    dt.drive_for(50, False, 50, heading = 0)
    dt.drive_for(-50, False, 50, heading = 0)
    dt.drive_for(50, False, 50, heading = 0)
    dt.drive_for(-50, False, 50, heading = 0)
    dt.drive_for(100, False, 50)

def claw_move1():
    lift.command(5.5)

def claw_move10():
    pin_distance = claw_distance.object_distance(MM)
    print("Pin distance: {}".format(pin_distance))
    pin_compensation = (pin_distance - 40) / 1.5
    angle = int(Arm.CLAW_ARM_MID1_DEGREES + pin_compensation)
    print("Pin compensation: {}".format(pin_compensation))
    arm.run_claw_arm(Arm.CLAW_ARM_COMMAND_TO_ANGLE, angle)
    lift.command(3)    

def claw_move2():
    arm.run_claw_arm(Arm.CLAW_ARM_COMMAND_TO_POSITION, Arm.CLAW_ARM_DOWN)
    lift.command(0)

def spin_motors(turns):
    left_front_motor.set_velocity(85, PERCENT)
    left_back_motor.set_velocity(85, PERCENT)
    right_front_motor.set_velocity(100, PERCENT)
    right_back_motor.set_velocity(100, PERCENT)

    right_front_motor.set_stopping(BrakeType.HOLD)
    right_back_motor.set_stopping(BrakeType.HOLD)
    left_front_motor.set_stopping(BrakeType.HOLD)
    left_back_motor.set_stopping(BrakeType.HOLD)

    left_front_motor.spin_for(FORWARD, turns, TURNS, wait = False)
    left_back_motor.spin_for(FORWARD, turns, TURNS, wait = False)
    right_front_motor.spin_for(FORWARD, turns, TURNS, wait = False)
    right_back_motor.spin_for(FORWARD, turns, TURNS)

def fast_toggle():
    turns = 2 * (200/math.sqrt(2))/220
    spin_motors(turns)
    spin_motors(-turns)

    spin_motors(turns)
    spin_motors(-turns)

# Score 7 pins, 3 goals with 2 pins
def autonomous_left():
    # place automonous code here

    print("--- Toggle")
    Toggle.lower_toggle()
    # fast_toggle()
    dt.drive_for(50, False, 100, heading = 0)
    dt.drive_for(-50, False, 100, heading = 0)
    dt.drive_for(50, False, 100, heading = 0)
    dt.drive_for(-50, False, 100, heading = 0)
    Toggle.raise_toggle()

    Thread(claw_move1)
    # wait(250, MSEC)

    print("--- Move 1")
    dt.drive_to_xy(300, 1800, False, 66, heading = 0)
    odom_print()
    dt.drive_to_xy(300, 2400, True, 66, heading = 0)
    odom_print()

    print("--- Place Pin")
    dt.drive_for(175, False, 50, heading = 0)
    claw_move10()
    claw.open()

    print("--- Reverse and recenter")
    dt.drive_to_xy(300, 2400, False, 66, heading = 0)
    dt.drive_to_xy(300, 2400, True, 66, heading = 0)
    arm.run_claw_arm(Arm.CLAW_ARM_COMMAND_TO_POSITION, Arm.CLAW_ARM_DOWN)
    lift.command(0)

    print("--- Rotate to wall")
    current_heading = inertial.rotation()
    print("Current heading: {}".format(current_heading))
    target_heading = 180
    dt.turn_for(target_heading - current_heading, 85)

    print("--- Capture Cup")
    arm.run_claw_arm(Arm.CLAW_ARM_COMMAND_TO_POSITION, Arm.CLAW_ARM_MID3, 33)
    wait(250, MSEC)
    claw.close()
    lift.command(5)
    arm.run_claw_arm(Arm.CLAW_ARM_COMMAND_TO_POSITION, Arm.CLAW_ARM_DOWN)

    print("--- Rotate to goal")
    current_heading = inertial.rotation()
    print("Current heading: {}".format(current_heading))
    target_heading = 0
    dt.turn_for(target_heading - current_heading, 100)
    lift.command(11.5)

    print("--- Approach scoring position")
    dt.drive_for(175, False, 50, heading = 0)
    arm.run_claw_arm(Arm.CLAW_ARM_COMMAND_TO_POSITION, Arm.CLAW_ARM_DOWN)
    wait(250, MSEC)

    print("--- Score Cup")
    lift.command(9)
    claw.open()
    wait(250, MSEC)

    print("--- Reverse and prepare for next move")
    dt.drive_to_xy(300, 2400, False, 66, heading = 0)
    Thread(claw_move2)
    dt.drive_to_xy(300, 1800, True, 100, heading = 0)

def autonomous_right():
    # place automonous code here

    odom_print()
    
    print("--- Toggle")
    odom_distance_enable(True, False, False)
    Toggle.lower_toggle()
    dt.drive_for(60, False, 100, heading = 0)
    dt.drive_for(-70, False, 100, heading = 0)
    dt.drive_for(60, False, 100, heading = 0)
    dt.drive_for(-70, False, 100, heading = 0)
    Toggle.raise_toggle()

    Thread(claw_move1)
    # wait(250,MSEC)

    print("--- Move 1")
    dt.drive_to_xy(305, 1800, False, 66, heading = 0)
    odom_print()
    dt.drive_to_xy(305, 1190, True, 66, heading = 0)
    odom_print()

    print("--- Place Pin")
    dt.drive_for(170, False, 50, heading = 0)
    claw_move10()
    claw.open()

    print("--- Reverse and recenter")
    dt.drive_to_xy(310, 1190, False, 66, heading = 0)
    dt.drive_to_xy(310, 1190, True, 66, heading = 0)
    arm.run_claw_arm(Arm.CLAW_ARM_COMMAND_TO_POSITION, Arm.CLAW_ARM_DOWN)
    lift.command(0)
    odom_print()

    print("--- Rotate to wall")
    odom_print()
    current_heading = inertial.rotation()
    print("Current heading: {}".format(current_heading))
    target_heading = 180
    odom_distance_enable(False, False, False)
    dt.turn_for(target_heading - current_heading, 100)
    odom_print()

    print("--- Capture Cup")
    arm.run_claw_arm(Arm.CLAW_ARM_COMMAND_TO_POSITION, Arm.CLAW_ARM_MID3, 33)
    wait(250, MSEC)
    claw.close()
    lift.command(5)
    arm.run_claw_arm(Arm.CLAW_ARM_COMMAND_TO_POSITION, Arm.CLAW_ARM_DOWN)

    print("--- Rotate to goal")
    odom_print()
    current_heading = inertial.rotation()
    print("Current heading: {}".format(current_heading))
    target_heading = 0
    dt.turn_for(target_heading - current_heading, 100)
    odom_distance_enable(True, False, False)
    lift.command(11.5)
    odom_print()

    print("--- Approach scoring position")
    odom_print()
    dt.drive_for(165, False, 50, heading = 0)
    arm.run_claw_arm(Arm.CLAW_ARM_COMMAND_TO_POSITION, Arm.CLAW_ARM_DOWN)
    wait(250, MSEC)
    odom_print()

    print("--- Score Cup")
    lift.command(9)
    claw.open()
    wait(250, MSEC)

    print("--- Reverse and prepare for next move")
    odom_print()
    dt.drive_to_xy(305, 1200, False, 66, heading = 0)
    Thread(claw_move2)
    dt.drive_to_xy(305, 600, True, 100, heading = 0)
    dt.turn_for(-90, 100)

def autonomous():
    global ROBOT_ENABLED
    while not ROBOT_INITIALIZED:
        wait(100, MSEC)
    ROBOT_ENABLED = True

    Thread(arm.initialize)

    if AUTON_SEQUENCE == CALIBRATION:
        autonomous_calibration()
    elif AUTON_SEQUENCE == AutonSequence.SKILLS:
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

lift_thread = None

def StopLift():
    global lift_thread

    if lift.is_running():
        print("Was Running")
        if lift_thread is not None: lift_thread.stop()
        lift.stop()
        return True
    
    return False

def OnLowerLiftPressed(): # R2
    global lift_thread
    
    if not ROBOT_ENABLED: return
    if StopLift(): return
    def _lower_lift() -> None:
        lift.lower_lift()
    lift_thread = Thread(_lower_lift)

def OnRaiseLiftPressed(): # R1
    global lift_thread
    if not ROBOT_ENABLED: return
    if StopLift(): return
    def _raise_lift() -> None:
        lift.raise_lift()
    lift_thread = Thread(_raise_lift)

def OnLowerClawPressed(): # L2
    if not ROBOT_ENABLED: return
    if arm.is_running():
        print("Was Running")
        # claw_arm_motor1.stop(HOLD)
        # claw_arm_motor2.stop(HOLD)
        arm.run_claw_arm(Arm.CLAW_ARM_COMMAND_CANCEL)
        return

    pressed_counter = 0
    while pressed_counter < 5: # about 1/4 second
        wait(50, MSEC)
        if not controller_1.buttonL2.pressing():
            thread = Thread(arm.lower_claw_arm)
            return
        pressed_counter += 1

    thread = Thread(arm.move_claw_arm_to_position, (Arm.CLAW_ARM_DOWN, 0))

def OnRaiseClawPressed(): # L1
    if not ROBOT_ENABLED: return
    if arm.is_running():
        print("Was Running")
        #claw_arm_motor1.stop(HOLD)
        #claw_arm_motor2.stop(HOLD)
        arm.run_claw_arm(Arm.CLAW_ARM_COMMAND_CANCEL)
        return

    pressed_counter = 0
    while pressed_counter < 5: # about 1/4 second
        wait(50, MSEC)
        if not controller_1.buttonL1.pressing():
            thread = Thread(arm.raise_claw_arm)
            return
        pressed_counter += 1

    thread = Thread(arm.move_claw_arm_to_position, (Arm.CLAW_ARM_UP, 0))

def OnControlButtonAPressed():
    if not ROBOT_ENABLED: return
    if claw.is_open():
        claw.close()
    else:
        claw.open()
    StopLift()

def OnControlButtonBPressed():
    if not ROBOT_ENABLED: return
    if Toggle.toggle_raised() and arm.claw_arm_current_position() < Arm.CLAW_ARM_MID3:
        Toggle.lower_toggle()
    else:
        Toggle.raise_toggle()

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

def OnButtonXPressed():
    if not ROBOT_ENABLED: return
    # Add the desired functionality for button X here
    Thread(claw_move1)

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

def user_control():
    global ROBOT_ENABLED

    # place driver control in this while loop
    last_fwd = 0 
    while not ROBOT_INITIALIZED:
        wait(100, MSEC)

    Thread(arm.initialize)

    starting_distance, starting_angle = average_back_distance()
    print("Back distance: {}, Back angle: {}".format(starting_distance, starting_angle))

    controller_1.buttonA.pressed(OnControlButtonAPressed)
    controller_1.buttonB.pressed(OnControlButtonBPressed)

    controller_1.buttonR2.pressed(OnLowerLiftPressed)
    controller_1.buttonR1.pressed(OnRaiseLiftPressed)

    controller_1.buttonL2.pressed(OnLowerClawPressed)
    controller_1.buttonL1.pressed(OnRaiseClawPressed)

    controller_1.buttonUp.pressed(OnControlButtonUpPressed)

    # controller_1.buttonX.pressed(OnButtonXPressed)

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

    if DRIVE_MOTION_MODEL:
        motion_model = DriveMotionModel(brain, rotation_fwd, rotation_side, inertial, left_front_motor, left_back_motor, right_front_motor, right_back_motor)
    else:
        motion_model = None

    left_front_motor.set_stopping(COAST)
    left_back_motor.set_stopping(COAST)
    right_front_motor.set_stopping(COAST)
    right_back_motor.set_stopping(COAST)

    # initialize_lift()
    Thread(auto_claw_thread)
    # Thread(odom_thread)
    # odom_distance_enable(False, False, False)

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
        ramp_max = MAX_RANP - RAMP_RANGE * lift.get_height(percent=True) / 100
        safe_forward = dt.ramp_limit(raw_forward, last_forward, ramp_max)
        forward = safe_forward
        last_forward = forward

        # Ramp control - strafe
        ramp_max = MAX_RANP - RAMP_RANGE * lift.get_height(percent=True) / 100
        safe_strafe = dt.ramp_limit(raw_strafe, last_strafe, ramp_max)
        strafe = safe_strafe
        last_strafe = strafe

        turn = drivetrain_detwitch(forward, raw_turn, raw_detwitch, True)[1]

        TILT_ENABLE = False

        # Tilt detection
        forward_tilt = -inertial.orientation(OrientationType.ROLL, DEGREES)
        sideways_tilt = inertial.orientation(OrientationType.PITCH, DEGREES)
        if TILT_ENABLE and (abs(forward_tilt) > 10 or abs(sideways_tilt) > 10):
            if not lift.running:
                # print("Tilting! Forward: {}, Sideways: {}".format(forward_tilt, sideways_tilt))
                def lower_lift_for_tilt() -> None:
                    lift.lower_lift()

                thread = Thread(lower_lift_for_tilt)

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
            if motion_model is not None:
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

        if motion_model is not None:
            combined_forward, combined_strafe, combined_turn = motion_model.update(forward, strafe, turn + auto_turn)
        else:
            combined_forward, combined_strafe, combined_turn = forward, strafe, turn + auto_turn
        
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

            if motion_model is not None:
                motion_model.observe_wheels(left_front_speed, left_back_speed, right_front_speed, right_back_speed)

        wait(10, MSEC)

        '''
        if loop_count % 500 == 0:
            print("Back1: {}".format(back_distance1.object_distance(MM) - BACK_DISTANCE1_FROM_BACK - BACK_DISTANCE1_CLOSE_ERROR + ROBOT_LENGTH / 2))
            print("Back2: {}".format(back_distance2.object_distance(MM) - BACK_DISTANCE2_FROM_BACK - BACK_DISTANCE2_CLOSE_ERROR + ROBOT_LENGTH / 2))
            print("Left: {}".format(left_distance.object_distance(MM) - LEFT_DISTANCE_FROM_LEFT - LEFT_DISTANCE_CLOSE_ERROR + ROBOT_WIDTH / 2))
            print("Right: {}".format(right_distance.object_distance(MM) - RIGHT_DISTANCE_FROM_RIGHT - RIGHT_DISTANCE_CLOSE_ERROR + ROBOT_WIDTH / 2))
        '''

        loop_count += 1

# create competition instance
comp = Competition(user_control, autonomous)
pre_autonomous()