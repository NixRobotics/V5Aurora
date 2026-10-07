from vex import *
from v5pythonlibrary import InertialWrapper
from math import radians, degrees, cos, asin, sin, sqrt, pi

BEETLE = False

class XDriveTrain():

    if BEETLE:
        FORWARD_EFFICIENCY = 1.0
        LEFT_POWER_SCALING = 1.0
        RIGHT_POWER_SCALING = 1.0
        FRONT_POWER_SCALING = 1.0
        BACK_POWER_SCALING = 1.0
        DRIVETRAIN_WHEEL_ANGLES = 45.0
        DRIVETRAIN_WHEEL_SIZE = 260.0
        DRIVETRAIN_EXTERNAL_GEAR_RATIO = 1.0
    else:
        FORWARD_EFFICIENCY = 1 / 1.045
        LEFT_POWER_SCALING = 1.0
        RIGHT_POWER_SCALING = 0.85
        FRONT_POWER_SCALING = 1.0
        BACK_POWER_SCALING = 1.0
        DRIVETRAIN_WHEEL_ANGLES = 45.0
        DRIVETRAIN_WHEEL_SIZE = 220.0
        DRIVETRAIN_EXTERNAL_GEAR_RATIO = 24/48

    def __init__(self, lfm: Motor, lbm: Motor, rfm: Motor, rbm: Motor, g: InertialWrapper, location_callback: Callable):
        self.lfm = lfm
        self.lbm = lbm
        self.rfm = rfm
        self.rbm = rbm
        self.gyro = g
        self.location_callback = location_callback

        self.last_lfm_command_vel = 0
        self.last_lbm_command_vel = 0
        self.last_rfm_command_vel = 0
        self.last_rbm_command_vel = 0

        self.last_fwd_command = 0
        self.last_strafe_command = 0
        self.last_turn_command = 0

        self.X = 0
        self.Y = 0
        self.THETA = 0

        self.quiet_mode = False

    def set_quiet_mode(self, quiet: bool):
        self.quiet_mode = quiet

    def set_location(self, x: float, y: float, theta: float):
        self.X = x
        self.Y = y
        self.THETA = theta

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

    def turn_for(self, turn_degrees, speed=66, timeout=10000, turn_kp=6.0, settle_error=1.0):
        current_heading = self.gyro.rotation()
        target_heading = current_heading + turn_degrees
        heading_error = target_heading - current_heading
        settle_count = 0
        timeout_count = 0
        is_timeout = False
        is_settle = False
        loop_count = 0
        done = False

        while not done:

            current_heading = self.gyro.rotation()
            heading_error = target_heading - current_heading

            if abs(heading_error) < settle_error:
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

        if not self.quiet_mode:
            print("turn_for: x={}, y={}, heading={}, time={}, timeout={}, settle={}".format(self.X, self.Y, self.THETA, loop_count * 10, is_timeout, is_settle))

        return is_timeout, is_settle

    def turn_to(self, target_heading, speed=66, timeout=10000, turn_kp=6.0, settle_error=1.0):
        if target_heading >= 360 or target_heading < 0:
            raise ValueError("Target heading must be between 0 and 359 degrees")
        target_angle = target_heading - 360 if target_heading >= 180 else target_heading
        current_angle = self.gyro.angle()
        turn_degrees = target_angle - current_angle
        return self.turn_for(turn_degrees, speed=speed, timeout=timeout, turn_kp=turn_kp, settle_error=settle_error)

    def drive_for(self, distance, strafe=False, speed=100, heading=None, timeout=10000, drive_kp=50.0, turn_kp=500.0, settle_error=10.0): # distance is in mm, speed is in percent
        # setup
        turn_speed = 100 # max turn speed in percent
        wheel_efficiency = 1 / cos(radians(self.DRIVETRAIN_WHEEL_ANGLES))
        effective_wheel_size = self.DRIVETRAIN_WHEEL_SIZE * wheel_efficiency * self.DRIVETRAIN_EXTERNAL_GEAR_RATIO
        forward_target_revs = (distance / effective_wheel_size) / self.FORWARD_EFFICIENCY if not strafe else 0
        strafe_target_revs = distance / effective_wheel_size if strafe else 0
        if not self.quiet_mode:
            print("Target revolutions: {:.2f} {:.2f}".format(forward_target_revs, strafe_target_revs))
        target_tolerance = settle_error / effective_wheel_size
        ramp_rate = 1

        # save initial motor positions
        starting_left_front_position = self.lfm.position(TURNS)
        starting_left_back_position = self.lbm.position(TURNS)
        starting_right_front_position = self.rfm.position(TURNS)
        starting_right_back_position = self.rbm.position(TURNS)

        # save starting rotation
        target_rotation = heading if heading is not None else self.gyro.rotation()

        if not self.quiet_mode:
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

            current_rotation = self.gyro.rotation()
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

            if (timeout_count > int(timeout / 10)): is_timeout = True
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

        if not self.quiet_mode:
            print("drive_for: LF: {}, LB: {}, RF: {}, RB: {}".format(left_front_position, left_back_position, right_front_position, right_back_position))
            print("drive_for: x={}, y={}, heading={}, time={}, timeout={}, settle={}".format(self.X, self.Y, self.THETA, loop_count * 10, is_timeout, is_settle))

        return is_timeout, is_settle

    def drive_to_xy(self, target_x, target_y, strafe=False, speed=100, heading=None, timeout=10000,
                    major_kp=50.0, major_kd=256.0, minor_kp=25.0, minor_kd=0.0, turn_kp=550.0,
                    settle_error=10.0): # distance is in mm, speed is in percent
        '''
        Drive the robot to the specified (target_x, target_y) coordinates.

        Parameters:
        target_x (float): Target X coordinate in mm.
        target_y (float): Target Y coordinate in mm.
        strafe (bool): Whether to strafe while driving.
        speed (float): Maximum speed in percent.
        heading (float): Desired heading in degrees.
        timeout (int): Timeout in milliseconds.
        major_kp (float): Proportional gain for the major direction control.
        major_kd (float): Derivative gain for the major direction control.
        minor_kp (float): Proportional gain for the minor direction control.
        minor_kd (float): Derivative gain for the minor direction control.
        turn_kp (float): Proportional gain for turn control.
        settle_error (float): Acceptable error in mm for considering the target as settled.

        Returns:
        (bool, bool): Tuple indicating if timeout occurred and if the target was settled.
        '''

        self.set_location(*self.location_callback())
        if not self.quiet_mode:
            print("Driving to X: {}, Y: {} from X: {}, Y: {}, is_strafe={}".format(target_x, target_y, self.X, self.Y, strafe))

        # setup
        wheel_efficiency = 1 / cos(radians(self.DRIVETRAIN_WHEEL_ANGLES))
        effective_wheel_size = self.DRIVETRAIN_WHEEL_SIZE * wheel_efficiency * self.DRIVETRAIN_EXTERNAL_GEAR_RATIO
        turn_speed = 100 # max turn speed in percent
        lateral_speed = 100 # max lateral speed in percent (this is the perpenicular speed to the commanded direction)
        target_tolerance = settle_error # mm
        ramp_rate = 1 # percent of increase per loop iteration (10ms loop) for acceleration

        # derivative (D) filtering
        drive_derivative_alpha = 0.2        
        previous_major_error_revs = None
        previous_minor_error_revs = None
        filtered_major_derivative = 0.0
        filtered_minor_derivative = 0.0

        # Define a fixed path frame from the starting position to the target.
        path_x = target_x - self.X
        path_y = target_y - self.Y
        path_length = sqrt(path_x * path_x + path_y * path_y)
        if path_length <= target_tolerance:
            self.stop_all(BRAKE)
            if not self.quiet_mode:
                    print("drive_to_xy: x={}, y={}, heading={}, time={}, timeout={}, settle={}".format(self.X, self.Y, self.THETA, 0, False, False))
            return False, True
        
        # normalize the path vector to unit length
        path_x /= path_length
        path_y /= path_length
        normal_x = -path_y
        normal_y = path_x

        # save starting rotation
        target_rotation = heading if heading is not None else self.gyro.rotation()

        # loop control variables
        done = False
        timeout_count = 0
        settle_count = 0
        loop_count = 0
        is_timeout = False
        is_settle = False
        last_major = 0
        last_minor = 0
        major_ramp_enabled = True
        minor_ramp_enabled = True

        while not done:
            current_rotation = self.gyro.rotation()
            rotation_error = (target_rotation - current_rotation) / 360.0 # saturate at 360 degrees

            # compute target errors in the global frame
            self.set_location(*self.location_callback())
            target_error_x = target_x - self.X
            target_error_y = target_y - self.Y
            major_error = target_error_x * path_x + target_error_y * path_y
            minor_error = target_error_x * normal_x + target_error_y * normal_y

            # Only check for settle in the major direction
            if abs(major_error) < target_tolerance:
                settle_count += 1
            else:
                settle_count = 0

            # convert major and minor errors to wheel revolutions
            major_error_revs = major_error / effective_wheel_size
            minor_error_revs = minor_error / effective_wheel_size
            if previous_major_error_revs is None: previous_major_error_revs = major_error_revs
            if previous_minor_error_revs is None: previous_minor_error_revs = minor_error_revs

            # compute raw and filtered derivatives for major and minor errors
            raw_major_derivative = major_error_revs - previous_major_error_revs
            raw_minor_derivative = minor_error_revs - previous_minor_error_revs
            filtered_major_derivative += drive_derivative_alpha * (raw_major_derivative - filtered_major_derivative)
            filtered_minor_derivative += drive_derivative_alpha * (raw_minor_derivative - filtered_minor_derivative)

            # check for timeout and settle conditions
            if (timeout_count > int(timeout / 10)): is_timeout = True
            if (settle_count > 10): is_settle = True

            if is_timeout or is_settle:
                done = True
                self.lfm.stop(BRAKE)
                self.lbm.stop(BRAKE)
                self.rfm.stop(BRAKE)
                self.rbm.stop(BRAKE)
            else:
                # PID loops for major and minor errors
                major_control = major_kp * major_error_revs + major_kd * filtered_major_derivative
                major_control = self.limit(major_control, speed)
                if abs(major_control) < abs(last_major): major_ramp_enabled = False
                if major_ramp_enabled: major_control = self.ramp_limit(major_control, last_major, ramp_rate)
                last_major = major_control

                minor_control = minor_kp * minor_error_revs + minor_kd * filtered_minor_derivative
                minor_control = self.limit(minor_control, lateral_speed)
                if abs(minor_control) < abs(last_minor): minor_ramp_enabled = False
                if minor_ramp_enabled: minor_control = self.ramp_limit(minor_control, last_minor, ramp_rate)
                last_minor = minor_control

                previous_major_error_revs = major_error_revs
                previous_minor_error_revs = minor_error_revs

                # rotate control from global to robot-centric frame
                global_x_control = major_control * path_x + minor_control * normal_x
                global_y_control = major_control * path_y + minor_control * normal_y
                current_rotation_radians = radians(current_rotation)
                fwd_control_percent = cos(current_rotation_radians) * global_x_control + sin(current_rotation_radians) * global_y_control
                strafe_control_percent = -sin(current_rotation_radians) * global_x_control + cos(current_rotation_radians) * global_y_control

                # PID loop for rotation error
                turn_control = turn_kp * rotation_error
                turn_control_percent = self.limit(turn_control, turn_speed)

                # control mixing and drive motors
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

        if not self.quiet_mode:
            self.set_location(*self.location_callback())
            print("drive_to_xy: x={}, y={}, heading={}, time={}, timeout={}, settle={}".format(self.X, self.Y, self.THETA, loop_count * 10, is_timeout, is_settle))

        return is_timeout, is_settle
