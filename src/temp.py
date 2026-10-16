
def autonomous_skills_to_park2():
    print("--- Autonomous Skills to Park ---")
    dt.turn_to(0, 85)
    odom_print()
    odom_distance_enable(False, False, True)
    current_heading = inertial.rotation()
    print("Current heading: {}".format(current_heading))
    if current_heading >0:
        current_heading = int((current_heading / 360.0) + 0.5) * 360.0
    else:
        current_heading = int((current_heading / 360.0) - 0.5) * 360.0
    print("Rounding heading: {}".format(current_heading))
    dt.drive_to_xy(900.0, 2100, True, 75, heading = current_heading)
    odom_print()
    # right back distance sensor has a slight chance of seeing the stack
    odom_distance_enable(True, False, False, back2 = False)
    dt.drive_to_xy(1500, 2100, False, 75, heading = current_heading)
    odom_print()
    odom_distance_enable(False, False, False)
    dt.turn_for(-45, 75)

    lift.command(13)
    dt.drive_for(350, False, 33, timeout = 2000)
    arm.run_claw_arm(Arm.CLAW_ARM_COMMAND_TO_POSITION, Arm.CLAW_ARM_MID1)
    lift.command(10)
    wait(500, MSEC)
    claw.open()
    dt.drive_for(-150, False, 50)
    arm.run_claw_arm(Arm.CLAW_ARM_COMMAND_TO_POSITION, Arm.CLAW_ARM_DOWN)