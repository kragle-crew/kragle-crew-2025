# Task A: the original Task A's three challenges, rebuilt from logged runs
# to be more reliable and faster.
#
#   Challenge 1 - lift the fork into the model, back up to pull it (the hook)
#   Challenge 2 - slow push on the lever
#   Challenge 3 - the long drive
#
# Pick A on the hub to run it, or Ad to run it with the recorder (logs the
# sensors).
# Old names: this used to be Task Y. In the comments below, the old
# names still appear: Task A/B/C = the ORIGINAL runs, Task Y = this
# Task A, Task Q = the improved original Task B, Task F = this Task B,
# Task M = this Task C.
#
# What Task Y does differently from Task A:
#   * Checks it is in the right starting spot before it moves.
#   * Steers with the gyro, and turns to exact gyro headings (turn_to).
#   * On the long drives it keeps steering toward the exact heading the
#     whole way (drive_on_heading). Over 580 mm, being 1.5 degrees off puts
#     the fork 15 mm to the side - that is what made the hook miss.
#   * Stops at the same spot after the GREEN/WHITE pattern every time, and
#     drives home if the pattern isn't where it always is.
#   * Checks that the hook really caught (the motors work much harder when
#     they pull the model). If it missed, it tries again aimed a little to
#     one side.
#   * Turns to the headings that worked in the logged runs. Task A's
#     turn(135) only really reached about 46 - with the gyro, 135 goes too
#     far right and the arm glances off the challenge 2 lever.
#   * Raises the arm a little for the challenge 2 push.
#   * Drives the long, empty stretches faster, and lowers the fork directly
#     instead of doing a full lift reset in the middle of the run.
#   * Prints CHECK / AIM / LINES / HOOK lines so the logs show what happened.

from init import robot, left_motor, right_motor, colorRight
from utilities.lift import reset_lift, lift_to_height
from utilities.readings import get
from utilities.run_helpers import (print_csv, wait_for_gyro,
                                   check_start_position, HeadingChecker,
                                   straight_and_watch)
from utilities.circular_buffer import CircularBuffer
from pybricks.tools import multitask, wait
from pybricks.parameters import Color


# ---------------- Settings you can change ----------------
#
# Headings are gyro degrees: 0 = the way the robot faced at the start,
# negative = turned left, positive = turned right.
#   Robot pointing too far RIGHT at a challenge?  SUBTRACT 2 from its heading.
#   Too far LEFT?                                 ADD 2.

USE_GYRO = True       # False = steer by the wheels only, like Task A
CHECK_START = True    # check the colors under the sensors before moving

# --- Speeds ---
NORMAL_SPEED = 200    # mm/s for most moves (Task A's speed)
# For the two long drives where the robot touches nothing (580 and 300 mm).
# Set to 200 to drive them like before.
FAST_SPEED = 300
TURN_RATE = 120       # degrees per second (Task A: 90). turn_to() finishes
                      # every important turn slowly, so the start can be quick.
# Lift speed when the fork isn't holding anything (normal is 600).
LIFT_FREE_SPEED = 900

# --- The pattern at the start ---
PATTERN_STOP_MM = 8   # stop this far past the GREEN/WHITE edge every time
# In every logged run the pattern was 144-151 mm from the start. If it isn't
# found by this distance, something is wrong, so the robot goes home.
PATTERN_MAX_MM = 250

# --- Challenge 1: the hook ---
C1_DRIVE_HEADING = -10.5  # the long 580 mm drive. In the 4-for-4 batch the
                          # robot really averaged about -10.5 here (aiming
                          # at -11, with steering that ran a bit to one
                          # side). The hook has missed at -13.
C1_HEADING = 0           # facing the model when the fork lifts
# How high the fork lifts before the robot backs up to pull. Was 750 - the
# robot pulled the model (the motors felt it) but the lever didn't actuate,
# so lift higher first.
#   Still doesn't actuate?            bigger (by about 100).
#   Fork lifts the model too far?     smaller.
C1_LIFT_HEIGHT = 1050
# Motor load while backing up that means "we pulled the model".
# All runs so far: pulled = 95 to 165, missed = 41 to 67.
HOOK_LOAD = 75
# Where to aim each retry, in degrees from the first try. A miss usually
# means the post on the arm lined up exactly with the lever, so each retry
# aims a tiny bit to one side (2 degrees is about one Lego stud at the fork).
#   [2, -2] = 1st retry a little RIGHT, 2nd retry a little LEFT.
#   []      = never retry.          Each retry adds about 4 seconds.
RETRY_AIM = [2, -2]

# --- Challenge 2: the slow push ---
C2_DRIVE_HEADING = -88   # the 300 mm drive toward challenge 2. The robot
                         # really averaged about -88 in the 4-for-4 batch.
C2_HEADING = 46       # worked at 45 to 47. (Task A's turns add up to 56 -
                      # that is too far right.)
C2_ARM_HEIGHT = 40    # raise the fork this much so the arm doesn't slide
                      # under the lever. 0 = don't raise.
C2_PUSH_MM = 400
# The arm meets the lever about 70 mm in and pushes it the rest of the way,
# so this is slow on purpose. Trying 150 would save about 1.3 seconds - but
# test it on its own, it could change how the lever moves.
C2_PUSH_SPEED = 100

# --- Challenge 3: the long drive ---
C3_HEADING = 139      # worked at 137 to 141
C3_DRIVE_MM = 1000
# In 2 of 6 logged runs the robot got knocked 4-10 degrees about 315 mm into
# this drive, and again near the end. Faster would save time (300 saves
# about 1.5 seconds) but hits harder - test it on its own.
C3_DRIVE_SPEED = 200
# False = drive this part without the gyro, like Task A. When the robot
# touches the model here, the gyro fights to keep its heading and spins a wheel.
GYRO_FOR_C3 = False
# The lift goes up to this height DURING the long drive back.
# None = leave the lift where it is.
RETURN_LIFT_HEIGHT = 2000

headings = HeadingChecker()


# ---------------- Helpers ----------------

async def find_pattern(*pattern: Color, max_distance=1000):
    # Same as Task A, plus the "stop at the same spot" part at the end.
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

    # The brakes don't stop in exactly the same place every time, so move to
    # exactly PATTERN_STOP_MM past where we saw the pattern.
    await wait(100)  # let the robot settle
    overshoot = robot.distance() - seen_at
    print_csv("PATTERN", round(seen_at - start_distance), round(overshoot))
    await robot.straight(PATTERN_STOP_MM - overshoot)
    return True


async def fast_drive(name, heading, distance):
    """A long drive with nothing in the way. Drives at FAST_SPEED and steers
    toward the exact gyro heading the whole way, so it ends up in the same
    place even if the turn before it was a degree or two off."""
    await headings.turn_to(heading, 2)   # roughly right is enough here
    robot.settings(straight_speed=FAST_SPEED)
    await headings.drive_on_heading(name, heading, distance, FAST_SPEED)
    robot.settings(straight_speed=NORMAL_SPEED)


async def back_up_and_check_hook(attempt):
    """Back up 100 mm and measure how hard the motors work.
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

async def do_hook():
    """Challenge 1. Starts at the pattern stop. Ends back at the lifting spot
    with the fork down."""
    # The long drive. It must stay on its heading - see the top of the file.
    await fast_drive("580", C1_DRIVE_HEADING, 580)
    await headings.turn_to(C1_HEADING)
    headings.check("C1_hook", C1_HEADING)

    # Lift the fork into the model, then back up to pull it.
    await lift_to_height(C1_LIFT_HEIGHT)
    hooked = await back_up_and_check_hook(1)

    # If we missed: fork down, aim a tiny bit to one side, drive back in, and
    # try again.
    attempt = 1
    for aim in RETRY_AIM:
        if hooked:
            break
        attempt += 1
        print("missed the hook - trying again, aiming", aim, "degrees")
        await lift_to_height(0, LIFT_FREE_SPEED)
        await headings.turn_to(C1_HEADING + aim)
        await robot.straight(100)
        await lift_to_height(C1_LIFT_HEIGHT)
        hooked = await back_up_and_check_hook(attempt)

    if hooked:
        # Same as Task A: drive back in, then let go. (Normal lift speed -
        # the fork is still holding the model here.)
        await robot.straight(100)
        await lift_to_height(0)
    else:
        # Still missed. Lower the fork FIRST - driving forward with the fork
        # up can ride it up onto the model and tip the robot over.
        print("never hooked - lowering the fork before moving on")
        await lift_to_height(0, LIFT_FREE_SPEED)
        await robot.straight(100)
    headings.check("C1_done", C1_HEADING)


async def do_push():
    """Challenge 2. Starts at the lifting spot. Ends 30 mm back from the end
    of the push, with the fork down."""
    await fast_drive("300", C2_DRIVE_HEADING, 300)
    # Swing round and back up a little, raising the fork on the way. (This
    # turn doesn't have to be exact - the next one is.)
    await headings.turn_to(36, 3)
    await multitask(
        robot.straight(-75),
        lift_to_height(C2_ARM_HEIGHT),
    )
    await headings.turn_to(C2_HEADING)
    headings.check("C2_push", C2_HEADING)

    robot.settings(straight_speed=C2_PUSH_SPEED)
    await straight_and_watch("C2_push", C2_PUSH_MM)
    robot.settings(straight_speed=NORMAL_SPEED)
    # Put the fork back down while backing away.
    await multitask(
        robot.straight(-30),
        lift_to_height(0),
    )


async def do_long_drive():
    """Challenge 3. Starts 30 mm back from the end of the push."""
    await headings.turn_to(C3_HEADING)
    headings.check("C3_drive", C3_HEADING)
    if not GYRO_FOR_C3:
        robot.use_gyro(False)
    robot.settings(straight_speed=C3_DRIVE_SPEED)
    if RETURN_LIFT_HEIGHT is None:
        await straight_and_watch("C3_drive", C3_DRIVE_MM)
    else:
        # Raise the lift while driving back.
        await multitask(
            straight_and_watch("C3_drive", C3_DRIVE_MM),
            lift_to_height(RETURN_LIFT_HEIGHT),
        )
    robot.settings(straight_speed=NORMAL_SPEED)
    headings.check("end", 143)


async def doChallenge1():
    robot.settings(straight_speed=NORMAL_SPEED, turn_rate=TURN_RATE)
    home = robot.distance()

    # Find the pattern while the lift resets.
    _, found = await multitask(
        reset_lift(),
        find_pattern(Color.GREEN, Color.WHITE, max_distance=PATTERN_MAX_MM)
    )
    if not found:
        # Don't carry on blind - go back to where we started.
        print("going home")
        await robot.straight(home - robot.distance())
        return

    await do_hook()
    await do_push()
    await do_long_drive()


async def task():
    if CHECK_START:
        await check_start_position()
    headings.start()
    gyro_on = False
    if USE_GYRO:
        gyro_on = await wait_for_gyro()
    robot.use_gyro(gyro_on)
    try:
        await doChallenge1()
    finally:
        # Put things back so the other tasks drive exactly like before.
        robot.stop()
        robot.use_gyro(False)
        try:
            robot.settings(straight_speed=200, turn_rate=90)
        except Exception as e:
            print("could not reset the drive settings:", e)
