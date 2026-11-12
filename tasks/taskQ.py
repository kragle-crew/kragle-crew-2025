# Task Q: Task B's four challenges, rebuilt from logged runs to be more
# reliable and faster.
#
#   Seed 1    - lift a seed off its branch (the robot comes in from the front)
#   Seed 2    - lift the second seed off its branch
#   Lever up  - lift a lever
#   Lever down - drop the arm onto a second lever to push it down
#
# Run it with the recorder from menu T (see taskT.py) to log the sensors.
# taskB.py is the original - you can still run B to compare.
#
# What Task Q does differently from Task B:
#   * Checks it is in the right starting spot before it moves.
#   * Steers with the gyro, and turns to EXACT gyro headings at each
#     challenge (turn_to). robot.turn() alone stops 2-4 degrees short.
#   * Stops at the same spot after the GREEN/WHITE pattern every time, and
#     drives home if the pattern isn't where it always is.
#   * Goes into seed 1 from the front, along the same line Task A drives
#     (11 degrees left of straight ahead - we know that line is clear).
#   * Drives straight lines between spots instead of forward-turn-sideways
#     steps. Fewer moves = faster, and less error building up.
#   * Moves the arm while driving where it is safe to.
#   * On the long drives it keeps steering toward the exact heading the
#     whole way (drive_on_heading), so it ends up in the same place each run.
#   * Feels whether the arm landed on the second lever (by how far down the
#     arm gets), and only pushes again if it missed.
#   * Prints CHECK / AIM / LINES lines so the logs show what happened.
#
# The older, slower version of this file is saved in git if we need it.

from init import robot, lift, pressure, colorRight
from utilities.lift import reset_lift, lift_to_height
from utilities.readings import get
from utilities.run_helpers import (print_csv, wait_for_gyro,
                                   check_start_position, HeadingChecker,
                                   straight_and_watch)
from pybricks.tools import multitask, wait, StopWatch
from pybricks.parameters import Color
from umath import atan2, sqrt, pi, sin, cos


# ---------------- Settings you can change ----------------
#
# Positions are in mm, measured from where the robot stops after the
# GREEN/WHITE pattern. "Forward" is the way the robot faces at the start.
# Headings are gyro degrees: 0 = the way it faced at the start,
# negative = turned left, positive = turned right.
#   Robot too far LEFT at a challenge?   ADD 2 to its heading (-40 -> -38).
#   Too far RIGHT?                       SUBTRACT 2 (-40 -> -42).

USE_GYRO = True       # False = steer by the wheels only, like Task B
CHECK_START = True    # check the colors under the sensors before moving

# Long drives must stay on their heading: over 500 mm, being 1 degree off
# puts the robot about 9 mm to the side - that is what made seed 2 miss
# "too far left" (one run was 1.5 degrees off).
# True  = steer toward the exact heading WHILE driving (drive_on_heading).
#         Tested on Task Y: quick, and holds the heading within a degree.
# False = the old way: turn very exactly first, then drive straight. A
#         standing robot can't really turn that exactly, so it can waste up
#         to 2 seconds per turn.
STEER_LONG_DRIVES = True
LONG_DRIVE_TOLERANCE = 0.5   # only used when STEER_LONG_DRIVES is False

# Lift speed for moves where the arm isn't touching anything (dropping
# between tries, coming down before the lever). Lifting seeds and levers
# stays at the normal 600. Set this to 600 if the arm seems too rough.
LIFT_FREE_SPEED = 900

# --- The pattern at the start ---
# Stop this far past the GREEN/WHITE edge every time (Task B's brakes
# stopped about 5 mm past it on average).
PATTERN_STOP_MM = 5
# In every logged run the pattern was 146-158 mm from the start. If it isn't
# found by this distance, something is wrong, so the robot goes home.
PATTERN_MAX_MM = 250

# --- Seed 1 ---
# The spot where the robot stops and turns right to face seed 1.
C1_TURN_FORWARD_MM = 503   # too far into the map? smaller.  not far enough? bigger.
C1_TURN_LEFT_MM = 100      # how far left of the start line
C1_FRONT_HEADING = 90      # facing the seed: turned right 90 degrees
# How far it drives in (facing the seed) before lifting.
#   Fork stops short of the seed?  bigger.   Hits it hard?  smaller.
C1_FRONT_IN_MM = 70
C1_APPROACH_SPEED = 100    # mm/s for that last drive in (rest of the run: 200)
C1_LIFT_SPEED = 600        # lift speed for seed 1
# Test mode for tuning seed 1 at the table: True = stop right after seed 1.
# Set back to False for full runs!
TEST_C1_ONLY = False

# --- The corner where the robot turns toward seed 2 ---
# Moves everything after seed 1 this many mm to the RIGHT (as the robot sees
# it at seed 2 and both levers). 0 = where Task B's path put it.
#   Seed 2 / levers too far LEFT?   bigger (by about 5).
#   Now too far RIGHT?              smaller.
# All four challenges worked with this at 10.
# How to check where the robot is, in a log: on the "to_C2" drive, look at
# where the first BLUE is (right sensor / left sensor). All four challenges
# worked at about 161 / 65. That BLUE line lies almost square across the
# drive, so a BIGGER number means the robot started that drive further BACK
# (170 / 74 = about 11 mm short: the levers come up short). This setting
# slides the robot sideways along that line, so it does NOT change the number.
AFTER_C1_SHIFT_MM = 10

# --- Seed 2 ---
# How far the robot drives toward seed 2 (Task B: 400). In 2 of 4 logged
# runs it touched the seed 2 model in the last few mm of this drive (once it
# got turned 3.5 degrees). It still worked - but if the clipping gets worse,
# try 395.
C2_DRIVE_MM = 400
C2_HEADING = -40      # worked at -40 (Task B pointed at about -46)
# How high the arm is when it starts each lift at seed 2. The arm comes down
# to this height on the drive to seed 2. (It used to stay at 900.)
#   Arm still too high to get under the seed?   smaller (by about 50).
#   Arm dragging on the mat or the model?       bigger.
C2_START_HEIGHT = 800
# True = lift seed 2 twice: lower the arm, turn a little right, lift again.
# OFF now: the first lift is hitting, and the second one broke the limb off
# the mission model. (The robot can't feel whether the first lift worked,
# so the second lift would happen every run.)
C2_RETRY = False
C2_RETRY_TURN = 3     # degrees MORE clockwise for the second lift

# --- Lever up (challenge 3) ---
C3_HEADING = -71      # worked at -71; missed at -72 to -75
# How far it drives in under the lever (Task B: 113). Was 118 - with the
# steered drives the robot starts this approach about 2 cm further back
# (the left sensor leaves the RED patch at 59 mm instead of 32-45), so it
# came up short.
#   Lever still short?  bigger.   Pushes into the lever model?  smaller.
C3_APPROACH_MM = 138
C3_WAIT_MS = 500      # how long it holds the lever up (Task B: 2000)
# True = the arm comes down from seed 2 WHILE the robot drives to the lever.
# False = the robot waits for the arm first (about 1.5 seconds slower).
C3_LOWER_WHILE_DRIVING = True

# --- Lever down (challenge 4) ---
C4_BACKUP_MM = 15     # back up this far from the first lever before pushing
C4_HEADING = -59      # at -62 it hit 1 of 2; at -59 it hit almost every time
C4_DROP_SPEED = 600   # lift speed for pushing down
# The pushes on the second lever, in order. Each one is
#   (degrees MORE clockwise than C4_HEADING, mm further forward).
# The robot stops pushing as soon as it feels the lever (see below), so the
# later pushes only happen after a miss. Each extra push takes about 4 seconds.
C4_PUSHES = [
    (0, 0),     # 1st push: straight at C4_HEADING
    (3, 0),     # 2nd push (only after a miss): 3 degrees more to the right
]
# How high the arm lifts between pushes. It must clear the lever.
# Lower = faster, e.g. 700 saves about 1.5 seconds per push - try it and watch.
C4_RETRY_LIFT = 1100

# Feeling for the lever. After each push the arm keeps going down GENTLY
# until it stops, and the robot looks at HOW FAR DOWN it got:
#   - it landed on the lever (or the model): it stopped at about -52 to -75
#   - it missed and dropped to the bottom:   it stopped at about -89 to -92
# (From 10 logged pushes. The lift LOAD was no help - it was 3 to 16 on every
# push, hit or miss - and the force sensor was pressed every time.)
# Prints: C4FEEL,<push>,<lift load>,<height it stopped at>,<sensor pressed?>,<felt lever?>
C4_FEEL = True        # False = skip the feel and always do every push
C4_FEEL_POWER = 40    # how hard it presses while feeling (percent). Gentle!
# Stopping HIGHER than this means "the arm is on the lever".
#   Robot says it felt the lever when it missed?   Move this up (e.g. -78).
#   Robot pushes again after a good hit?           Move this down (e.g. -86).
C4_FELT_ABOVE = -82
# True  = stop pushing as soon as a push felt the lever.
# False = do every push in C4_PUSHES anyway, and just print what it felt.
C4_STOP_WHEN_FELT = True

# --- The end ---
# True = finish with the long drive to the far home base (for competition).
# False = stop near the levers (for practice, when others are working there).
DRIVE_HOME = False
HOME_DRIVE_MM = 1300

headings = HeadingChecker()

# Where Task B's old path ended up just before it turned toward seed 2:
# 7 mm right of and 13 mm behind the pattern stop. Task Q comes back to the
# same corner (plus AFTER_C1_SHIFT_MM) so seed 2 and the levers don't move.
OLD_CORNER_RIGHT_MM = 7.1
OLD_CORNER_FORWARD_MM = -12.9


def line_between(from_right, from_forward, to_right, to_forward):
    """The heading (degrees) and length (mm) of the straight line from one
    spot to another. Spots are (mm to the right, mm forward)."""
    right = to_right - from_right
    forward = to_forward - from_forward
    heading = atan2(right, forward) * 180 / pi
    length = sqrt(right * right + forward * forward)
    return heading, length


def corner_spot():
    """The corner where the robot turns toward seed 2, as (right, forward).
    AFTER_C1_SHIFT_MM slides it in the direction that is "to the right" for
    the robot at seed 2 (facing -40) AND at the levers (facing -71): about
    35 degrees right of straight ahead."""
    right = OLD_CORNER_RIGHT_MM + AFTER_C1_SHIFT_MM * sin(35 * pi / 180)
    forward = OLD_CORNER_FORWARD_MM + AFTER_C1_SHIFT_MM * cos(35 * pi / 180)
    return right, forward


class CircularBuffer:
    def __init__(self, maxlen):
        self.maxlen = maxlen
        self.data = []

    def append(self, item):
        self.data.append(item)
        if len(self.data) > self.maxlen:
            self.data.pop(0)

    # 1. Allows direct comparison using ==
    def __eq__(self, other):
        if isinstance(other, list):
            return self.data == other
        if isinstance(other, CircularBuffer):
            return self.data == other.data
        return False

    # 2. Allows len(buffer)
    def __len__(self):
        return len(self.data)

    # 3. Allows item indexing (buffer[0]) and "item in buffer" checks
    def __getitem__(self, index):
        return self.data[index]
    def __repr__(self):
        return repr(self.data)


async def find_pattern(*pattern: Color, max_distance=1000):
    # Same as Task B, plus the "stop at the same spot" part at the end.
    # We only record color changes, so repeats in the pattern could never match.
    # Collapse them (e.g. WHITE, WHITE -> WHITE).
    pattern = [c for i, c in enumerate(pattern) if i == 0 or c != pattern[i - 1]]
    start_distance = robot.distance()
    robot.drive(100,0)
    print("looking for pattern", pattern)
    recently_seen = CircularBuffer(maxlen=len(pattern))
    seen_at = start_distance
    while recently_seen != pattern:
        # Give up if we've driven too far without finding the pattern
        if abs(robot.distance() - start_distance) > max_distance:
            print("pattern not found within", max_distance, "mm")
            robot.brake()
            return False
        current_color = await colorRight.color()
        # Ignore colors not in the pattern (e.g. stray readings at color edges)
        if current_color not in pattern:
            continue
        # Skip repeat readings so the buffer holds color changes, not every sample
        if len(recently_seen) == 0 or recently_seen[-1] != current_color:
            print("seeing color", current_color)
            recently_seen.append(current_color)
            seen_at = robot.distance()  # where we saw this color change
    print("pattern found!")
    robot.brake()

    # The brakes don't stop in exactly the same place every time, so move to
    # exactly PATTERN_STOP_MM past where we saw the pattern.
    await wait(100)  # let the robot settle
    overshoot = robot.distance() - seen_at
    print_csv("PATTERN", round(seen_at - start_distance), round(overshoot))
    await robot.straight(PATTERN_STOP_MM - overshoot)
    return True


async def reset_lift_safely(timeout_ms=5000):
    """reset_lift(), but it gives up after timeout_ms. reset_lift() waits for
    the force sensor - if that never gets pressed, it would wait forever."""
    finished = {"ok": False}

    async def do_reset():
        await reset_lift()
        finished["ok"] = True

    # race=True: whichever finishes first (the reset or the timer) wins.
    await multitask(do_reset(), wait(timeout_ms), race=True)
    if not finished["ok"]:
        lift.stop()
        print("lift reset took too long - stopped it")


async def feel_for_lever(push):
    """After a push: keep lowering the arm gently until it stops or presses
    the force sensor. Returns True if it felt the lever - that is, if the arm
    stopped higher than C4_FELT_ABOVE instead of dropping to the bottom.

    It uses lift.dc() (a fixed, low power) instead of a speed, so when the
    arm lands on something it just rests there - it doesn't push harder."""
    biggest_load = 0
    pressed = False
    still_ms = 0          # how long the arm has not been moving
    timer = StopWatch()
    lift.dc(C4_FEEL_POWER)            # positive = down, same as reset_lift
    while timer.time() < 2500:        # never wait longer than this
        if await get(pressure.pressed(force=0.5)):
            pressed = True
            break
        load = abs(await get(lift.load()))
        biggest_load = max(biggest_load, load)
        # Stopped moving for 0.2 seconds = it is resting on something.
        if timer.time() > 200 and abs(lift.speed()) < 20:
            still_ms += 10
            if still_ms >= 200:
                break
        else:
            still_ms = 0
        await wait(10)
    lift.stop()

    height = -lift.angle()   # lift_to_height() uses negative angles for up
    felt = height > C4_FELT_ABOVE
    print_csv("C4FEEL", push, round(biggest_load), round(height), pressed, felt)
    return felt


async def long_drive(name, heading, distance):
    """A long drive (negative distance = backward) that has to end up in
    exactly the right place. Used for the three long legs of the run."""
    if STEER_LONG_DRIVES:
        await headings.turn_to(heading, 2)   # roughly right is enough...
        # ...because this keeps steering toward the heading as it drives.
        await headings.drive_on_heading(name, heading, distance, 200)
    else:
        await headings.turn_to(heading, LONG_DRIVE_TOLERANCE)
        await straight_and_watch(name, distance)


async def do_seed_1():
    """Seed 1, from the front. Starts at the pattern stop. Ends at the corner
    where the robot turns toward seed 2, facing along the line it came back on."""
    # 1. One straight line to the spot beside seed 1. This is the same line
    #    Task A drives (about 11 degrees left), so we know it is clear.
    heading, length = line_between(0, 0, -C1_TURN_LEFT_MM, C1_TURN_FORWARD_MM)
    await long_drive("to_C1", heading, length)

    # 2. Turn right to face the seed, drive in slowly, and lift.
    await headings.turn_to(C1_FRONT_HEADING)
    headings.check("C1_seed", C1_FRONT_HEADING)
    robot.settings(straight_speed=C1_APPROACH_SPEED)
    await straight_and_watch("C1_front_in", C1_FRONT_IN_MM)
    robot.settings(straight_speed=200)
    await lift_to_height(1200, C1_LIFT_SPEED)
    await lift_to_height(900, C1_LIFT_SPEED)

    # 3. Back out the way it came in.
    await robot.straight(-C1_FRONT_IN_MM)
    if TEST_C1_ONLY:
        return

    # 4. One straight line, driving backward, to the corner where the robot
    #    turns toward seed 2. (It faces away from the corner and reverses.)
    corner_right, corner_forward = corner_spot()
    heading, length = line_between(corner_right, corner_forward,
                                   -C1_TURN_LEFT_MM, C1_TURN_FORWARD_MM)
    await long_drive("back_from_C1", heading, -length)


async def do_seed_2():
    """Seed 2. Starts at the corner. Ends at seed 2 with the arm up at 2000."""
    # Drive to seed 2, lowering the arm to its starting height on the way.
    await multitask(
        long_drive("to_C2", -64, C2_DRIVE_MM),
        lift_to_height(C2_START_HEIGHT),
    )
    await headings.turn_to(C2_HEADING)
    headings.check("C2_seed", C2_HEADING)
    await lift_to_height(2000)
    if C2_RETRY:
        # Second lift: arm back down, turn a bit right, lift again.
        await lift_to_height(C2_START_HEIGHT, LIFT_FREE_SPEED)
        await headings.turn_to(C2_HEADING + C2_RETRY_TURN)
        headings.check("C2_retry", C2_HEADING + C2_RETRY_TURN)
        await lift_to_height(2000)


async def do_levers():
    """Lever up, then lever down. Starts at seed 2 with the arm at 2000."""

    async def drive_to_lever():
        await robot.straight(100)
        await headings.turn_to(C3_HEADING)

    # Turn away from seed 2 first. Then the arm comes down to 600 - while the
    # robot drives, if C3_LOWER_WHILE_DRIVING is on.
    await headings.turn_to(-89)
    if C3_LOWER_WHILE_DRIVING:
        await multitask(lift_to_height(600, LIFT_FREE_SPEED), drive_to_lever())
    else:
        await drive_to_lever()
        await lift_to_height(600, LIFT_FREE_SPEED)

    # Lever up: drive in under it, lift, hold, let go.
    headings.check("C3_lever", C3_HEADING)
    await straight_and_watch("C3_approach", C3_APPROACH_MM)
    await lift_to_height(1500)
    await wait(C3_WAIT_MS)
    await lift_to_height(1100)

    # Lever down: back up a little, then do every push in C4_PUSHES.
    await robot.straight(-C4_BACKUP_MM)
    forward_now = 0   # how far forward of the first push spot we are
    push = 0
    for extra_turn, extra_forward in C4_PUSHES:
        push += 1
        if push > 1:
            # lift the arm off the lever
            await lift_to_height(C4_RETRY_LIFT, LIFT_FREE_SPEED)
        await headings.turn_to(C4_HEADING + extra_turn)
        if extra_forward != forward_now:
            await robot.straight(extra_forward - forward_now)
            forward_now = extra_forward
        headings.check("C4_push" + str(push), C4_HEADING + extra_turn)
        await lift_to_height(50, C4_DROP_SPEED)
        if C4_FEEL:
            felt = await feel_for_lever(push)
            if felt and C4_STOP_WHEN_FELT:
                print("felt the lever - no more pushes needed")
                break
    # Back to the first push spot, so the way out starts from the same place.
    if forward_now != 0:
        await robot.straight(-forward_now)


async def finish():
    """Back away from the levers (and drive home, if DRIVE_HOME is on) while
    the lift resets at the same time."""

    async def drive_away():
        await robot.straight(-60)
        await headings.turn_to(-102)
        headings.check("home", -102)
        if DRIVE_HOME:
            await robot.straight(HOME_DRIVE_MM)

    await headings.turn_to(-72)
    await multitask(drive_away(), reset_lift_safely())


async def doChallenge1():
    robot.settings(straight_speed=200,turn_rate=90)
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

    await do_seed_1()
    if TEST_C1_ONLY:
        print("TEST_C1_ONLY is on - stopping after seed 1")
        return
    await do_seed_2()
    await do_levers()
    await finish()


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
        robot.use_gyro(False)
