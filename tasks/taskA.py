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

from init import hub, robot, lift, left_motor, right_motor, colorRight
from utilities.lift import reset_lift, lift_to_height
from utilities.readings import get
from utilities.run_helpers import (print_csv, wait_for_gyro,
                                   check_start_position, HeadingChecker,
                                   straight_and_watch, one_decimal)
from utilities.circular_buffer import CircularBuffer
from pybricks.tools import multitask, wait, StopWatch
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
# so lift higher first. Then 1050 - in 2 of 3 runs (taskA-runs-2) it didn't
# fully pull the lever: too high. 950 didn't work either (taskA-runs-3), so
# back to 750.
#   Still doesn't actuate?            bigger (by about 50).
#   Fork lifts the model too far?     smaller.
C1_LIFT_HEIGHT = 750
# Motor load while backing up that means "we pulled the model".
# All runs so far: pulled = 95 to 165, missed = 41 to 67.
HOOK_LOAD = 75
# Where to aim each retry, in degrees from the first try. A miss usually
# means the post on the arm lined up exactly with the lever, so each retry
# aims a tiny bit to one side (2 degrees is about one Lego stud at the fork).
#   [2, -2] = 1st retry a little RIGHT, 2nd retry a little LEFT.
#   []      = never retry.          Each retry adds about 4 seconds.
RETRY_AIM = [2, -2]
# Letting go of the model after the pull: if the robot tilts more than this
# many degrees while the fork comes down, the fork is landing ON the model -
# the hook missed. Good runs stay under 1; the run that tipped hit 8.5.
C1_LOWER_MAX_TILT = 3

# --- Challenge 2: the slow push ---
# The 300 mm drive toward challenge 2. Was -88: in 3 of 6 runs (taskA-runs-1)
# the robot's right side caught the leaf cutter ant model about 180 mm into
# this drive, got twisted 30 degrees and wheel-spun - then challenge 2 and 3
# were off. -91 passes about 9 mm further from it.
#   Still catching the ant model?  -92 (more left).
C2_DRIVE_HEADING = -91
# Was 46. In the failed pushes the robot's LEFT side dragged along the lever
# arm the whole push. The drive change above already moves the push about
# 11 mm to the right; this turns it 1 degree more right too.
# (Task A's turns add up to 56 - that is too far right.)
C2_HEADING = 47
C2_ARM_HEIGHT = 40    # raise the fork this much so the arm doesn't slide
                      # under the lever. 0 = don't raise.
# Was 400. The drive change above starts the push about 11 mm further back.
C2_PUSH_MM = 410
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


async def lower_fork_carefully():
    """Lower the fork to 0 (letting go of the model), but STOP straight away
    if the fork lands on something. Returns True if it got all the way down.

    This is how we really know the hook missed. When the peg lines up with
    the lever, the pull back feels just as heavy as a real hook (runs 3 and
    6 in taskA-runs-4), but then the fork comes down ON the model:
      - it tilts the robot (run 3: 8.5 degrees, then it tipped over), or
      - it gets stuck part way down (run 6: stopped a third of the way).
    On a real hook the fork drops straight down and the robot stays flat.
    Prints:  C1_LOWER,<height it got to>,<worst tilt change>,<all the way?>"""
    start_tilt = max(abs(t) for t in hub.imu.tilt())
    found = {"tilt": 0, "stuck": False}

    async def watch():
        timer = StopWatch()
        still_ms = 0
        while True:
            await wait(10)
            tilt = max(abs(t) for t in hub.imu.tilt()) - start_tilt
            found["tilt"] = max(found["tilt"], tilt)
            if tilt > C1_LOWER_MAX_TILT:
                return
            # Not moving, but not down yet = resting on something.
            height = -lift.angle()
            if timer.time() > 300 and height > 100 and abs(lift.speed()) < 50:
                still_ms += 10
                if still_ms >= 150:
                    found["stuck"] = True
                    return
            else:
                still_ms = 0

    # race=True: stop lowering as soon as the watcher sees a problem.
    await multitask(lift_to_height(0), watch(), race=True)
    lift.hold()
    height = -lift.angle()
    all_the_way = height < 50 and found["tilt"] <= C1_LOWER_MAX_TILT
    print_csv("C1_LOWER", round(height), one_decimal(found["tilt"]),
              all_the_way)
    return all_the_way


# ---------------- The run ----------------

async def do_hook():
    """Challenge 1. Starts at the pattern stop. Ends back at the lifting spot
    with the fork down."""
    # The long drive. It must stay on its heading - see the top of the file.
    await fast_drive("580", C1_DRIVE_HEADING, 580)
    await headings.turn_to(C1_HEADING)
    headings.check("C1_hook", C1_HEADING)

    # First try straight on, then the retries in RETRY_AIM.
    aims = [0] + RETRY_AIM
    done = False
    for attempt in range(1, len(aims) + 1):
        aim = aims[attempt - 1]
        if attempt > 1:
            # Missed. We are backed up, well away from the model, so it is
            # safe to lower the fork here. Then aim a tiny bit to one side,
            # drive back in, and try again.
            print("missed the hook - trying again, aiming", aim, "degrees")
            await lift_to_height(0, LIFT_FREE_SPEED)
            await headings.turn_to(C1_HEADING + aim)
            await robot.straight(100)

        # Lift the fork into the model, then back up to pull it.
        await lift_to_height(C1_LIFT_HEIGHT)
        if not await back_up_and_check_hook(attempt):
            continue   # the motors didn't feel the model - try again

        # Same as Task A: drive back in, then let go - but watch the fork
        # come down, because a miss can still feel heavy on the pull.
        await robot.straight(100)
        if await lower_fork_carefully():
            done = True
            break
        # The fork landed on the model, so the hook really missed. Lift it
        # back off the model and REVERSE before lowering it, then retry.
        print("fork landed on the model - the hook missed")
        await lift_to_height(C1_LIFT_HEIGHT)
        await robot.straight(-100)

    if not done:
        # Still missed. We are backed up with the fork up. Lower the fork
        # FIRST - driving forward with the fork up can ride it up onto the
        # model and tip the robot over.
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
