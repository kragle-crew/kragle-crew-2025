# Task Y: Task A, plus changes suggested by the diagnostic runs.
#
# It drives the same path as Task A. Search for "CHANGE" to find what's new:
#   1. Uses the gyro to drive straight and turn more accurately.
#   2. After finding the GREEN/WHITE pattern, it moves to the same spot every
#      time, instead of wherever the brakes happened to stop it.
#   3. After lifting the fork and backing up, it checks that it actually
#      pulled the model. (In the diagnostic runs, the motors worked much
#      harder - load over 120 - in almost every successful run.) If it
#      missed, it tries once more.
#   4. If it never finds the pattern, it drives home instead of carrying on.
#   5. Removed the stray "await()" line from the end of Task A.
#   6. Smaller turns toward challenges 2 and 3. With the gyro, turns are
#      exact - but Task A's numbers were tuned on wheel turns that came up
#      short. So Task A's turn(135) really only reached heading ~47, and
#      that's where challenge 2 works. Task Y uses the headings Task A
#      ACTUALLY reached in its successful runs.
#   7. Checks it is in the right starting spot before it moves. In every
#      logged run, at home the right sensor saw WHITE and the left saw YELLOW.
#   8. On the long drives, prints every line each color sensor crosses
#      ("LINES"). This doesn't change the driving - it's to find lines we can
#      use later to check position and squareness.
#   9. Raises the fork slightly for the challenge 2 push, so the arm doesn't
#      slide under the lever.
#  10. Drives the challenge 3 part without the gyro, like Task A.
#  11. Each hook retry aims a tiny bit to the side, not at the same spot.
#
# taskA.py is not changed - you can still run A to compare.

from init import hub, robot, left_motor, right_motor, colorRight, colorLeft
from utilities.lift import reset_lift, lift_to_height
from utilities.readings import get, color_name
from tasks.taskA import CircularBuffer
from pybricks.tools import multitask, wait, StopWatch
from pybricks.parameters import Color, Button


# ---------------- Settings you can change ----------------

USE_GYRO = True       # CHANGE 1: drive with the gyro. False = drive like Task A

# CHANGE 2: how far past the pattern edge to stop, in mm. Each run prints
# "PATTERN,...,overshoot". After a few runs, set this to the average overshoot
# from the SUCCESSFUL runs, so the robot always stops at that spot.
PATTERN_STOP_MM = 8

# CHANGE 3: motor load while backing up that means "we pulled the model".
# All runs so far: pulled = 95 to 148, missed = 41 to 67. 75 is in the middle.
HOOK_LOAD = 75

# CHANGE 11: where to aim each retry, in degrees from the first try.
# A miss usually means the post on the arm lined up exactly with the lever,
# so trying the SAME spot again misses again. Each retry aims a tiny bit to
# one side instead (2 degrees moves the fork tip about one Lego stud).
#   [2, -2] = 1st retry a little RIGHT, 2nd retry a little LEFT.
#   []      = never retry.
# Each retry adds about 6 seconds to the run.
RETRY_AIM = [2, -2]

# CHANGE 6: turn sizes toward challenges 2 and 3 (Task A used 135 and 90).
# Each run prints "CHECK" lines with the real heading at these points.
# Successful Task A runs: C2 push at heading 45-47, C3 drive at 138-139.
#   Robot pointing too far RIGHT at C2?  Make C2_TURN smaller.
#   Robot pointing too far RIGHT at C3?  Make C3_TURN smaller.
#   (At C3 the robot faces back toward home, so "its right" is YOUR left.)
C2_TURN = 126
C3_TURN = 92

# CHANGE 9: challenge 2 arm height and push distance.
# The robot now lines up and pushes as hard as Task A's good runs, but the
# arm sometimes slides UNDER the lever. So raise the fork a little first.
#   Arm still slides under the lever?   Make C2_ARM_HEIGHT bigger (by ~20).
#   Arm goes OVER the top of the lever?  Make it smaller.
#   0 = don't raise (same as before).
# (Bigger numbers = higher. Challenge 1 lifts to 750.)
C2_ARM_HEIGHT = 40
#   Lever still not pushed all the way in?  Make C2_PUSH_MM bigger (by ~10).
C2_PUSH_MM = 400      # Task A pushed 400

# CHANGE 10: drive the last part (challenge 3) WITHOUT the gyro, like Task A.
# When the robot touches the challenge 3 model, the gyro fights to keep
# the heading and spins a wheel (run 2 spun one wheel for over 3 seconds).
# Task A just let the robot give a little, and challenge 3 worked.
GYRO_FOR_C3 = False

# CHANGE 7: what the color sensors see when the robot is placed at home.
CHECK_START = True
START_RIGHT_COLOR = Color.WHITE
START_LEFT_COLOR = Color.YELLOW


# ---------------- Helpers ----------------

def print_csv(*values):
    print(",".join(str(v) for v in values))


start_heading = {"value": 0}


def check(name, target):
    """Print the robot's real heading (from the gyro) at an important point,
    next to the heading a successful run should have."""
    heading = hub.imu.heading() - start_heading["value"]
    print_csv("CHECK", name, round(heading), target, round(heading - target))


async def check_start_position():
    """CHANGE 7: make sure the robot is in its starting spot before moving.
    If the colors look wrong: fix the robot, then press RIGHT to check again,
    or press LEFT to go anyway."""
    while True:
        right = await get(colorRight.color())
        left = await get(colorLeft.color())
        ok = right == START_RIGHT_COLOR and left == START_LEFT_COLOR
        print_csv("START", color_name(right), color_name(left), ok)
        if ok:
            return

        print("Robot may be in the wrong spot! Fix it, then press RIGHT to"
              " check again, or LEFT to go anyway.")
        hub.display.char("!")
        hub.light.on(Color.RED)
        await hub.speaker.beep(frequency=300, duration=500)

        pressed = ()
        while not pressed:
            pressed = hub.buttons.pressed()
            await wait(10)
        while hub.buttons.pressed():  # wait for the button to be let go
            await wait(10)
        hub.light.on(Color.ORANGE)
        if Button.LEFT in pressed:
            print("going anyway")
            return


async def straight_and_watch(name, distance):
    """CHANGE 8: drives exactly like robot.straight(distance), but also prints
    every color change each sensor sees, with how far into the drive (mm)."""
    start = robot.distance()
    lines = {"R": [], "L": []}

    async def watch_sensors():
        while True:
            for side, sensor in (("R", colorRight), ("L", colorLeft)):
                color = color_name(await get(sensor.color()))
                seen = lines[side]
                if (len(seen) == 0 or seen[-1][1] != color) and len(seen) < 30:
                    seen.append((round(abs(robot.distance() - start)), color))
            await wait(5)

    # race=True: stop watching as soon as the drive is done.
    await multitask(robot.straight(distance), watch_sensors(), race=True)
    for side in ("R", "L"):
        print_csv("LINES", name, side,
                  " ".join(str(mm) + ":" + color for mm, color in lines[side]))


async def wait_for_gyro(timeout_ms=2000):
    """The gyro needs the robot to sit still for a moment before it's ready."""
    timer = StopWatch()
    while not hub.imu.ready():
        if timer.time() > timeout_ms:
            print("gyro not ready - driving without it this run")
            return False
        await wait(50)
    return True


async def find_pattern(*pattern: Color, max_distance=1000):
    # Same as Task A, plus CHANGE 2 at the end.
    pattern = [c for i, c in enumerate(pattern) if i == 0 or c != pattern[i - 1]]
    start_distance = robot.distance()
    robot.drive(100, 0)
    print("looking for pattern", pattern)
    recently_seen = CircularBuffer(maxlen=len(pattern))
    seen_at = start_distance
    while recently_seen != pattern:
        if abs(robot.distance() - start_distance) > max_distance:
            print("pattern not found within", max_distance, "mm")
            robot.brake()
            return False
        current_color = await colorRight.color()
        if current_color not in pattern:
            continue
        if len(recently_seen) == 0 or recently_seen[-1] != current_color:
            print("seeing color", current_color)
            recently_seen.append(current_color)
            seen_at = robot.distance()  # where we saw this color change
    robot.brake()
    print("pattern found!")

    # CHANGE 2: the brakes don't stop in exactly the same place every time,
    # so move to exactly PATTERN_STOP_MM past where we saw the pattern.
    await wait(100)  # let the robot settle
    overshoot = robot.distance() - seen_at
    print_csv("PATTERN", round(seen_at - start_distance), round(overshoot))
    await robot.straight(PATTERN_STOP_MM - overshoot)
    return True


async def back_up_and_check_hook(attempt):
    """CHANGE 3: back up 100 mm and measure how hard the motors work.
    Returns True if we pulled the model (we hooked it)."""
    biggest = {"load": 0}

    async def watch_motors():
        while True:
            left = abs(await get(left_motor.load()))
            right = abs(await get(right_motor.load()))
            biggest["load"] = max(biggest["load"], left, right)
            await wait(10)

    start = robot.distance()
    # race=True: stop watching the motors as soon as the back-up is done.
    await multitask(robot.straight(-100), watch_motors(), race=True)

    hooked = biggest["load"] >= HOOK_LOAD
    print_csv("HOOK", attempt, round(biggest["load"]),
              round(start - robot.distance()), hooked)
    return hooked


# ---------------- The run ----------------

async def doChallenge1():
    robot.settings(straight_speed=200, turn_rate=90)
    home = robot.distance()

    _, found = await multitask(
        reset_lift(),
        find_pattern(Color.GREEN, Color.WHITE)
    )
    if not found:
        # CHANGE 4: don't carry on blind - go back to where we started.
        print("going home")
        await robot.straight(home - robot.distance())
        return

    await robot.turn(-11)
    await straight_and_watch("580", 580)
    await robot.turn(12)

    # Challenge 1: lift the fork, then back up to pull the model.
    await lift_to_height(750)
    hooked = await back_up_and_check_hook(1)

    # CHANGE 3 + 11: if we missed, lower the fork, aim a tiny bit to one
    # side, drive back in, and try again.
    aim = 0  # how many degrees we are turned away from the first try
    attempt = 1
    for new_aim in RETRY_AIM:
        if hooked:
            break
        attempt += 1
        print("missed the hook - trying again, aiming", new_aim, "degrees")
        await reset_lift()
        await robot.turn(new_aim - aim)
        aim = new_aim
        await robot.straight(100)
        await lift_to_height(750)
        hooked = await back_up_and_check_hook(attempt)

    if hooked:
        # Same as Task A.
        await robot.straight(100)
        await reset_lift()
    else:
        # CHANGE 3: still missed. Lower the fork FIRST - driving forward with
        # the fork up can ride it up onto the model and tip the robot over.
        print("never hooked - lowering the fork before moving on")
        await reset_lift()
        await robot.straight(100)

    # CHANGE 11: if a retry aimed to the side, turn back so challenges 2 and 3
    # start from exactly the same heading as always.
    if aim != 0:
        await robot.turn(-aim)

    # From here on, the same path as Task A.
    check("C1_done", 0)
    await robot.turn(-90)
    await straight_and_watch("300", 300)
    await robot.turn(C2_TURN)  # CHANGE 6: was 135
    # CHANGE 9: raise the fork a little while backing up and turning.
    await multitask(
        robot.straight(-75),
        lift_to_height(C2_ARM_HEIGHT),
    )
    await robot.turn(10)

    # Challenge 2: slow push.
    check("C2_push", 47)
    robot.settings(straight_speed=100)
    await straight_and_watch("C2_push", C2_PUSH_MM)
    robot.settings(straight_speed=200)
    # CHANGE 9: put the fork back down while backing away.
    await multitask(
        robot.straight(-30),
        lift_to_height(0),
    )
    await robot.turn(C3_TURN)  # CHANGE 6: was 90

    # Challenge 3: long drive.
    check("C3_drive", 139)
    if not GYRO_FOR_C3:
        robot.use_gyro(False)  # CHANGE 10
    await straight_and_watch("C3_drive", 1000)
    check("end", 143)


async def task():
    if CHECK_START:
        await check_start_position()
    start_heading["value"] = hub.imu.heading()
    gyro_on = False
    if USE_GYRO:
        gyro_on = await wait_for_gyro()
    robot.use_gyro(gyro_on)
    try:
        await doChallenge1()
    finally:
        # Put things back so the other tasks drive exactly like before.
        robot.use_gyro(False)
