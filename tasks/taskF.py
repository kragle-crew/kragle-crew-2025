
# Task F: carries a part on the raised lift and drops it in the far right
# corner.
#
# It starts like Task Y (start check, gyro, the GREEN/WHITE pattern) with one
# difference: it does NOT reset the lift. The lift should start at 2000
# (where Task Y leaves it) with the part on it:
#   - If the lift was reset earlier in this program (Task Y ran first), it
#     knows its height, so it lifts to 2000 first - in case it's off.
#   - If not (the program was just started), its height means nothing yet,
#     so it doesn't move and we assume it really is at 2000.
#
# TO CHANGE THE PATH: edit the steps in path() just below the settings.
# Each step prints a line in the log (AIM for turns, DRIVE + POS for drives),
# so a logged run shows exactly where it went.
# POS = mm from the pattern stop (x = right, y = forward), then the heading.

from init import robot, lift
from utilities.lift import (lift_to_height, lift_height_is_known,
                            reset_lift_safely)
from utilities.run_helpers import (wait_for_gyro, check_start_position,
                                   HeadingChecker, Position, print_csv,
                                   one_decimal)
from tasks.taskY import find_pattern, PATTERN_MAX_MM
from pybricks.tools import multitask, wait
from pybricks.parameters import Color


# ---------------- Settings you can change ----------------

CHECK_START = True    # check the colors under the sensors before moving
USE_GYRO = True

F_SPEED = 300         # mm per second for drives (same as Task Y's fast drives)
F_TURN_RATE = 120     # degrees per second for turns (same as Task Y)
F_LIFT_HEIGHT = 2000  # the lift height the part is carried at
F_LIFT_SPEED = 600    # lift speed (600 = normal)
# mm per second for backing away after the first lower, so the part can
# settle. 30 = 10 cm in about 3.5 seconds.
F_SETTLE_SPEED = 30

# On the way home: True = reset the lift, False = lift to F_LIFT_HEIGHT
# (2000) instead, ready to load the part and test again.
HOME_RESET_LIFT = False


# ---------------- The path - edit the steps here ----------------
#
#   await turn(45)          turn 45 degrees right (negative = left)
#   await drive(300)        drive 300 mm forward (negative = backward)
#   await drive(300, 150)   ... at 150 mm/s instead of F_SPEED
#   await lift_to(1500)     move the lift to height 1500 (bigger = higher)
#   await pause(500)        wait half a second
#   await go_home()         drive straight back to the start spot, resetting
#                           the lift on the way (or lifting to 2000 - see
#                           HOME_RESET_LIFT)
#
# Turns are measured from the way the robot SHOULD be facing, not the way it
# happens to be facing, so small misses don't add up. Drives steer to stay
# on that heading the whole way.

async def path():
    await turn(-10.5)    # face along Task Y's long drive
    await drive(480)     # out - 11 cm short of where Task Y pulls its lever
    await turn(50)       # right, toward the corner (was 45 - 3 more right)
    await drive(230)     # into the corner

    # The drop:
    await lift_to(1200)  # drop part way
    await drive(-100, F_SETTLE_SPEED)  # back up 10 cm, very slowly so it settles
    await lift_to(2000)  # lift
    await drive(100)     # forward 10 cm
    await lift_to(1300)  # drop part way again
    await lift_to(1500)  # reset to 2000 so we can try again

    # Home:
    await drive(-150)    # back straight out of the corner, the way we came in
    await go_home()      # straight back to the start (see HOME_RESET_LIFT)


# ---------------- Step helpers ----------------

headings = HeadingChecker()
position = Position(headings)
# The heading the robot should be facing (0 = how it faced at the start),
# a step counter for the log, and how far the start spot is behind the
# pattern stop.
plan = {"heading": 0, "step": 0, "start_mm": 0}


def step_name(kind):
    plan["step"] += 1
    return str(plan["step"]) + "_" + kind


def log_position(name):
    position.update()
    print_csv("POS", name, one_decimal(position.x), one_decimal(position.y),
              one_decimal(headings.heading()))


async def turn(degrees):
    """Turn `degrees` right (negative = left)."""
    plan["heading"] += degrees
    step_name("turn")
    await headings.turn_to(plan["heading"])


async def drive(mm, speed=F_SPEED):
    """Drive `mm` forward (negative = backward), steering to stay on the
    heading the robot should be facing."""
    name = step_name("drive")
    # Short drives (and the last 120 mm of long ones) use robot.straight(),
    # which goes at the robot.settings() speed - so set that too.
    robot.settings(straight_speed=speed)
    await headings.drive_on_heading(name, plan["heading"], mm, speed)
    robot.settings(straight_speed=F_SPEED)
    log_position(name)


async def lift_to(height, speed=F_LIFT_SPEED):
    """Move the lift to `height` (bigger = higher)."""
    print_csv("LIFT", step_name("lift"), height)
    await lift_to_height(height, speed)


async def pause(ms):
    """Wait `ms` milliseconds (1000 = 1 second)."""
    step_name("pause")
    await wait(ms)


async def go_home():
    """Turn toward the start spot and drive straight there (backward if that
    needs less turning). At the same time it resets the lift, or lifts it
    to F_LIFT_HEIGHT if HOME_RESET_LIFT is False.
    The start spot is where the robot was in the jig, before it drove
    forward to find the pattern."""
    name = step_name("home")
    if HOME_RESET_LIFT:
        lift_move = reset_lift_safely()
    else:
        lift_move = lift_to_height(F_LIFT_HEIGHT, F_LIFT_SPEED)
    await multitask(
        position.go_to(name, 0, -plan["start_mm"], next_heading=0,
                       speed=F_SPEED),
        lift_move,
    )
    plan["heading"] = headings.heading()


# ---------------- The run ----------------

async def run_f():
    robot.settings(straight_speed=F_SPEED, turn_rate=F_TURN_RATE)
    home = robot.distance()

    # Same start as Task Y - but no lift reset, the part is on the lift.
    if lift_height_is_known():
        # The lift was reset earlier in this program (e.g. Task Y ran), so
        # lift_to_height() goes to the right place. Make sure it's at 2000.
        await lift_to_height(F_LIFT_HEIGHT, F_LIFT_SPEED)
    else:
        # The lift doesn't know its height yet - moving it now could jam
        # it. Leave it where it is, and tell it that's 2000, so later
        # lift_to() steps work. (Height 2000 = motor angle -2000.)
        print("lift not reset yet - assuming it is at", F_LIFT_HEIGHT)
        lift.reset_angle(-F_LIFT_HEIGHT)
    lift.hold()

    found = await find_pattern(Color.GREEN, Color.WHITE,
                               max_distance=PATTERN_MAX_MM)
    if not found:
        print("going home")
        await robot.straight(home - robot.distance())
        return
    position.start()   # the pattern stop is (0, 0)
    plan["heading"] = 0
    plan["step"] = 0
    plan["start_mm"] = robot.distance() - home

    # Keep track of where the robot is while it follows the path.
    await multitask(path(), position.keep_track(), race=True)


async def task():
    if CHECK_START:
        await check_start_position()
    headings.start()
    gyro_on = False
    if USE_GYRO:
        gyro_on = await wait_for_gyro()
    robot.use_gyro(gyro_on)
    normal_settings = robot.settings()
    try:
        await run_f()
    finally:
        # Put things back so the other tasks drive exactly like before.
        robot.settings(*normal_settings)
        robot.use_gyro(False)
