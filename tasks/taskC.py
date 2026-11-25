# Task C: the original Task C's two challenges, with the improved Task B's
# (Task Q's) two levers done in between - made reliable the same way as
# Task Q and Task Y.
#
#   Challenge 1 (Task C)  ->  lever up + lever down (Task Q)  ->  Challenge 2
#   (Task C). Task Q's seeds don't score, so they're left out. The lever
#   settings are in the "Levers" section below. DO_LEVERS = False skips them.
#
# A position tracker (Position) adds up every mm the robot drives in the
# direction the gyro says, so the hops to and from the levers start from
# where the robot really is.
#
# Pick C on the hub to run it, or Cd to run it with the recorder: when a
# recorded run finishes the hub shows "?" - press RIGHT if the run WORKED,
# LEFT if it FAILED. To stop a run early, press CENTER.
# Old names: this used to be Task M. In the comments below, the old
# names still appear: Task A/B/C = the ORIGINAL runs, Task Y = this
# Task A, Task Q = the improved original Task B, Task F = this Task B,
# Task M = this Task C.
#
# Batch 1 drove exactly like Task C (wheel turns) and measured the real gyro
# heading after every turn. Now it turns to exact gyro headings, using where
# the wheel turns really ended up (see GYRO_HEADINGS below).
#
# What it does differently from Task C:
#   * Turns to exact gyro headings, and steers on the long drives.
#   * Lowers the arm right to the bottom (and holds it) BEFORE driving in
#     under the challenge 1 lever, and stops before it bumps the challenge
#     on the right.
#   * After challenge 1: turns less before backing out (so it doesn't back
#     into the model on its right), and lowers the arm less. The drives
#     after that are adjusted so challenge 2 stays in the same place.
#   * Uses the RED/WHITE line on the mat to end the 340 mm drive in the
#     same place every run, backs up less before challenge 2 (so it doesn't
#     back into the model on its left), and keeps pressing the lever.
#   * Logs bumps: every steered drive prints its worst heading error and
#     highest motor load.
#   * On the moves that kept touching models, drives slowly and STOPS as
#     soon as the gyro feels a bump - so it doesn't push on and break them.
#   * Challenge 2: lower arm, and a "jam" push after the press.
#   * Checks it is in the right starting spot before it moves.
#   * Stops at the same spot after the GREEN/WHITE pattern every time, and
#     drives home if the pattern isn't where it always is.
#   * The lift reset can't hang (it gives up after 5 seconds).
#   * Prints CHECK / LINES lines so the logs show what happened.
#
# For quick testing: run_once.py (menu, one run, then the program ends).

from init import robot, lift, pressure, colorRight, left_motor, right_motor
from utilities.readings import get
from utilities.lift import lift_to_height, reset_lift_safely, lower_to_bottom
from utilities.run_helpers import (Position, nearest_angle,
                                   print_csv, wait_for_gyro,
                                   check_start_position, HeadingChecker,
                                   straight_and_watch, one_decimal)
from utilities.circular_buffer import CircularBuffer
from pybricks.tools import multitask, wait, StopWatch
from pybricks.parameters import Color
from umath import sin, cos, pi


# ---------------- Settings you can change ----------------

CHECK_START = True    # check the colors under the sensors before moving.
                      # (Task C starts from the same spot as Tasks A and B,
                      # where the right sensor sees WHITE and the left YELLOW.)

# Stop this far past the GREEN/WHITE edge every time. The brakes stopped
# about 5 mm past it on average, which Task C's numbers were tuned with.
PATTERN_STOP_MM = 5
# In every logged run the pattern was 144-158 mm from the start. If it isn't
# found by this distance, something is wrong, so the robot goes home.
PATTERN_MAX_MM = 250

# --- Gyro ---
# True  = turn to exact gyro headings (turn_to) and steer on the long drives
#         (drive_on_heading), like Tasks Q and Y.
# False = turn by the wheels, exactly like Task C.
# Batch 1 (wheel turns): the first turn landed anywhere from -76 to -79, so
# over the long first drive no two runs ended up in the same place.
USE_GYRO = True
# The gyro heading to turn to at each turn. A turn not listed here uses
# Task C's numbers added up (shown in its CHECK line).
# These three are where the wheel turns ACTUALLY ended up in batch 1 (the
# average of 4 runs), so the path stays where it was - just the same every
# run now. (Lesson from Task Y: use where the robot really pointed, not
# Task C's numbers added up.)
GYRO_HEADINGS = {
    "turn1": -77.5,   # Task C's numbers add up to -75
    "turn2": -137,    # adds up to -135
    "turn3": -140.5,  # adds up to -140
    # From here on: Task C's numbers added up. Batch 3 reached every one of
    # these within 1 degree. They are written out so that changing
    # C1_TURN_AFTER_LIFT doesn't move them.
    "turn6": -120,
    "turn7": -75,
    "turn8": -10,
    "turn9": 35,
    "C2": 83,         # was 80: pushed the lever in but not all the way -
                      # "could rotate a bit more"
}

# --- Challenge 1 ---
# The long first drive. Task C used 450 and hit a model; 480 overshot C1.
OUT_DRIVE_MM = 465
# The last drive in, under the challenge 1 lever (Task C: 85). In batches 1
# and 2 the robot started catching the challenge on its RIGHT about 230 mm
# into the approach (the last 45 mm), so this stops before that.
#   Arm stops short of the lever?             bigger (by about 10).
#   Still bumping the challenge on the right?  smaller.
C1_LAST_MM = 35   # was 55. After the model was moved, the robot hit the
                  # model on its right 45 mm into this drive (run 9). C1
                  # still worked when it stopped at 33-34 mm (batch 7).
# After that drive: turn this many degrees (negative = left), then the C1
# turn and lift scoop up the lever (the "drone arm").
#   Arm meets the lever in the wrong place?  change this by 2.
C1_TURN_BEFORE_DIP = -5
# How far it backs out after the C1 lift (Task C: 270). It backed into the
# mission model in the middle, so 10 cm less. The hop to the levers starts
# from wherever the robot really ends up, so nothing else needs changing.
C1_BACK_MM = 170

# --- Not breaking mission models ---
# On the moves that keep touching models, the robot drives slowly and STOPS
# as soon as the gyro says it has been twisted more than this many degrees -
# that twist is the robot pushing against a model, before pieces break off.
# (Normal driving stays within about 1 degree.) It is used on the last drive
# into challenge 1 and on the back-up before challenge 2; in batch 5 the
# contact started near the END of both, so stopping there loses very little.
BUMP_TWIST = 3
# It also stops if the robot TILTS more than this many degrees - one side
# riding up onto a model. Run 6 hit the left model on the way into
# challenge 1 without twisting, but it tipped sideways 5.8 degrees.
# (Normal driving stays under about 1.5.)
BUMP_TILT = 3.5
CAREFUL_SPEED = 100   # mm/s on those moves (most of the run: 200)
# The arm dips right down to the bottom - until the force sensor reads this
# many newtons - during the 100 mm drive, well before the robot squeezes in
# to challenge 1, and stays there. 3-4 newtons is how low it has to be to
# get under the lever.
#   Still not getting under the lever?   bigger (e.g. 5) = pressed further.
C1_BOTTOM_FORCE = 3
# The turn after the challenge 1 lift, in degrees (Task C: -15). Turning
# 15 left pointed the robot's back at the mission model on its right, and
# it backed into it (batch 3). Less turn = it backs out further from it.
# The next two drives are made a little longer to make up for it (see
# drives_after_back_up), so challenge 2 stays in the same place.
C1_TURN_AFTER_LIFT = -10
# How far the arm comes down after the challenge 1 lift (Task C: 600). It
# drops from 2200 while the robot has only backed out 50 mm, and coming all
# the way to 600 bumped the model. 900 is also challenge 2's height, so the
# arm doesn't need to move again there.
#   Still bumping the model?           bigger (e.g. 1100).
#   Arm too high for the drives after?  smaller.
C1_ARM_AFTER_HEIGHT = 900

# --- Challenge 2 ---
# Use the line on the mat to end the 340 mm drive in the same place every
# run: the right color sensor comes off a long RED patch onto WHITE 294-311
# mm into that drive. It drives C2_LINE_EXTRA_MM past that edge. If it
# doesn't see the edge (e.g. after a bump) it just drives the normal 340.
# Now False: the edge shows up anywhere from 253 to 311 mm in, so it moves
# the stop around instead of fixing it. Since the waypoint change it is
# rarely seen; when it was (taskC-runs-6, run 1) the robot stopped 10 mm
# short of every other run and both fast pushes missed the lever. With
# False it drives the exact distance (still stopping if it bumps a model).
C2_USE_LINE = False
C2_LINE_EXTRA_MM = 38      # the robot used to stop about 38 mm past it
# How far it backs up before turning in to challenge 2 (Task C: 140). In
# every run of batch 4 the back-up was clean for the first 100-117 mm, then
# backed into the mission model on the left. The 340 mm drive is made a bit
# shorter to make up for it (see keep_c2_lined_up), so challenge 2 stays in
# line - and the push ends up about 2 cm further in.
C2_BACK_MM = 110
# How high the arm is for the challenge 2 push (was 900). At 900 the arm
# went over the lever - the press moved 40 mm in 0.7 s with nothing pushing
# back. Lower = smaller number.
C2_ARM_HEIGHT = 800
C2_PUSH_MM = 200
# After the push, keep pressing the lever: drive forward slowly for up to
# C2_PRESS_MS, or C2_PRESS_MM further, whichever comes first. (The wheels
# skid on the lever, and it needs more "pressed time".)
# Was 40. In taskC-runs-2 the lever didn't go all the way in, and every
# press moved its full 40 mm in about 0.7 s - so it stopped because of this
# limit, not because the lever stopped it. Push further.
C2_PRESS_MM = 60
C2_PRESS_MS = 1500
C2_PRESS_SPEED = 60        # mm/s - slow and steady
# "Jam" retry: after the press, back up a little and push into the lever
# again to jam it the rest of the way in. Always done (the robot can't
# tell whether the lever went all the way). False = no jam.
C2_JAM = True
# Run 6: the press was pushing the lever (it felt it - load 158) but a slow
# push wasn't enough to get it all the way in, so the jam is a real hit:
# How far it backs up before pushing again - the run-up. Was 60: with only
# 60 + 25 mm to speed up, the jam never got above about 280 mm/s
# (taskC-runs-5). 100 hit hard (416-487 mm/s) but backed into the mission
# model behind it (taskC-runs-6, runs 2 and 4). 70 still gets to about
# 430 mm/s with the fast speed-up below.
C2_JAM_BACK_MM = 70
C2_JAM_SPEED = 500         # mm/s top speed going in (was 350; most moves: 200)
# How quickly it speeds up and slows down on the jam (mm/s per second), so
# it is still going fast when it reaches the lever. It still STOPS at the
# same spot every time (C2_JAM_EXTRA_MM past where it backed up from), so a
# faster hit doesn't change where the robot ends up.
C2_JAM_ACCEL = 2000
# Never keep shoving for longer than this (ms) if the lever stops the robot.
C2_JAM_TIMEOUT_MS = 1500
C2_JAM_EXTRA_MM = 25       # (was 15) drives this much FURTHER than where it backed
                           # up from, to push the lever further in
# Before the jam (the second push), turn this many degrees right of the
# challenge 2 heading. Was 0: the lever didn't go all the way in
# (taskC-runs-3, run 2), and the team saw it needed about 10 degrees more.
C2_JAM_TURN = 10
# A THIRD push, after the jam: back up the same way, raise the arm this much
# higher than C2_ARM_HEIGHT, and drive in fast again, then press.
# In taskC-runs-5 nothing ever pushed back: the arm touches the lever about
# 6-8 cm into the first push, then slides past it, and every press after
# that moves freely. So the third push raises the lift more, to hit the
# lever with the bottom plate of the lift instead of the arm.
# Was 50 (arm at 850). 0 = no third push.
# 150 (arm at 950) hit the lever in 3 of 4 runs (taskC-runs-6), but in
# run 3 it grazed the lever and went over the top. So a little lower.
#   JAM line load under about 180?  It missed the lever.
C2_THIRD_PUSH_RAISE = 125

# Retry if the lever wasn't hit. A fast push that HITS the lever shows a
# drive motor load of about 240-310 on its JAM line; one that misses shows
# about 90-110 (taskC-runs-6). If neither the jam nor the third push got
# above C2_HIT_LOAD, it backs up and jams again at C2_ARM_HEIGHT, going
# C2_RETRY_EXTRA_MM further in each time, up to C2_RETRIES times.
# (It can tell "hit" from "missed", but not "pushed all the way" from
# "grazed over the top" - both of those feel like a hit.)
C2_HIT_LOAD = 180
C2_RETRIES = 2
C2_RETRY_EXTRA_MM = 15
# The edge on the 340 mm drive must be seen at least this far in. It was
# seen at 285-311 mm; once at 253, which stopped the robot 35 mm short, so
# anything earlier than this is ignored.
C2_LINE_SEARCH_FROM = 275
# If the robot bumps a model on the 340 mm drive (run 4 of batch 7 got
# knocked 32 degrees there), it stops and does NOT try challenge 2 - from
# the wrong spot it would miss anyway, and pushing on breaks pieces off.
# False = carry on and try challenge 2 anyway.
C2_SKIP_AFTER_BUMP = True
# How much twist counts as a bump on that 340 mm drive. Was BUMP_TWIST (3):
# in both runs of taskC-runs-1 the robot brushed something about 120 mm in
# (3.4 and 3.9 degrees) and stopped, though nothing looked wrong. Clean runs
# stay under 1.5; the real crash in batch 7 was 32.
C2_DRIVE_MAX_TWIST = 6

# --- Going home to the LEFT home area at the end ---
# True = after challenge 2, drive back to the left home area.
# False = stay where it finishes.
GO_HOME = True
# The way home, from the challenge 2 lever (all backward):
#   back out HOME_BACK_OUT_MM, turn HOME_TURN_1 right, back HOME_BACK_1_MM,
#   turn HOME_TURN_2 left, back HOME_BACK_2_MM into the left home area.
# (The first try went in straight lines from spot to spot and backed into
# the seed bomb model - this is the team's route.)
HOME_BACK_OUT_MM = 50
# Degrees right of the challenge 2 heading (83). Was 15 - it backed into
# the seed bomb (taskC-runs-3, run 2). Then 25 - in runs 1 and 4 of
# taskC-runs-6 the first back-up home hit a model on the robot's right and
# got twisted 40 degrees, so a bit more again.
HOME_TURN_1 = 32
HOME_BACK_1_MM = 500      # was 600
HOME_TURN_2 = -100        # degrees (negative = left). Was -90 (taskC-runs-4)
HOME_BACK_2_MM = 600

# --- Task Q's two levers, done between challenge 1 and challenge 2 ---
# Task Q's seeds don't score, so its two levers (lever up, lever down) are
# done here instead, straight after challenge 1. Their settings - headings,
# how far in, the second push - are in the "Levers" section below.
# False = skip the levers and go straight from challenge 1 to challenge 2.
DO_LEVERS = True
# Spots on the mat, in mm from where the robot stops after the GREEN/WHITE
# pattern (x = right, y = forward). Worked out by adding up Task Q's and
# Task M's moves - both tasks start from the same spot. After challenge 1
# the robot is only about 10 cm from the lever start, and the lever-down
# spot is only 2 cm from Task C's path, so the detour is short.
LEVER_START = (-446.7, 172.4)      # where Task Q starts its lever-up drive
C2_WAYPOINT_1 = (-818.6, 80.8)     # end of Task C's 300 mm drive
# End of Task C's 150 mm drive. Was (-963.5, 119.6): on the 340 mm drive
# after it, the robot's LEFT side kept catching something 100-150 mm in
# (taskC-runs-1: twisted 3.4 and 3.9 degrees; runs-3: 7.1, and it stopped).
# 12 mm further right (east). That also starts the challenge 2 push 12 mm
# closer to the lever, so it pushes it a bit further in.
#   Still catching on the left?  move x up a bit more (e.g. -940).
C2_WAYPOINT_2 = (-951.5, 119.6)
# True = stop right after the levers (for testing the new part quickly).
STOP_AFTER_LEVERS = False
# After the lever-down push, back straight off the lever this far before
# turning away toward challenge 2 (otherwise the arm drags on the lever).
#   Still catching the lever when it turns?  bigger (by about 20).
LEVER_EXIT_MM = 50

# ---------------- Levers (from Task Q) ----------------
# Lever up and lever down, between challenge 1 and challenge 2.

# Lift speed for moves where the arm isn't touching anything (dropping
# between tries, coming down before the lever). Lifting seeds and levers
# stays at the normal 600. Set this to 600 if the arm seems too rough.
LIFT_FREE_SPEED = 900

# --- Lever up (challenge 3) ---
C3_HEADING = -71      # worked at -71; missed at -72 to -75
# How far it drives in under the lever (Task B: 113). Was 118 - with the
# steered drives the robot starts this approach about 2 cm further back
# (the left sensor leaves the RED patch at 59 mm instead of 32-45), so it
# came up short.
#   Lever still short?  bigger.   Pushes into the lever model?  smaller.
C3_APPROACH_MM = 138
C3_WAIT_MS = 500      # how long it holds the lever up (Task B: 2000)

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


headings = HeadingChecker()
position = Position(headings)   # where the robot is on the mat
# "heading": where Task C's turns, added up, say the robot points.
# "target":  the heading the last turn aimed at (used by the next drive).
path = {"heading": 0, "target": 0}


# ---------------- Helpers ----------------

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

async def find_pattern(*pattern: Color, max_distance=1000):
    # Same as Task B, plus the "stop at the same spot" part at the end.
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
    print("pattern found!")
    robot.brake()

    # The brakes don't stop in exactly the same place every time, so move to
    # exactly PATTERN_STOP_MM past where we saw the pattern.
    await wait(100)  # let the robot settle
    overshoot = robot.distance() - seen_at
    print_csv("PATTERN", round(seen_at - start_distance), round(overshoot))
    await robot.straight(PATTERN_STOP_MM - overshoot)
    return True


def drives_after_back_up(back_heading, c1_short_mm, c1_heading):
    """Works out how much LONGER the next two drives (the 300 mm one and the
    150 mm one) must be so the robot lands back on the path that works for
    challenge 2. They go in two different directions, so together they can
    cancel any small shift. Two things shift the robot:
      1. After challenge 1 it backs out 50 + C1_BACK_MM mm. Task C backed
         out 320 mm at heading -170; a different heading or distance ends up
         somewhere else.
      2. If the careful drive into challenge 1 stopped early (it bumped a
         model), the robot is c1_short_mm further back along c1_heading.
         Batch 7: that alone sent runs 2 and 3 down a different path.
    Returns (extra mm for the 300 drive, extra mm for the 150 drive)."""
    def direction(heading):
        r = heading * pi / 180
        return sin(r), cos(r)       # (mm right, mm forward) per mm driven

    new_x, new_y = direction(back_heading)
    old_x, old_y = direction(-170)
    # Backing up moves the robot the OPPOSITE way it points. Task C backed
    # out 50 + 270 mm; now it's 50 + C1_BACK_MM.
    new_back = 50 + C1_BACK_MM
    shift_x = -new_back * new_x + 320 * old_x
    shift_y = -new_back * new_y + 320 * old_y
    # Stopping short at challenge 1 leaves it back along its approach line.
    c1_x, c1_y = direction(c1_heading)
    shift_x -= c1_short_mm * c1_x
    shift_y -= c1_short_mm * c1_y

    # Find a and b so that driving a mm at turn6's heading plus b mm at
    # turn7's heading cancels the shift.
    ax, ay = direction(GYRO_HEADINGS["turn6"])   # the 300 mm drive
    bx, by = direction(GYRO_HEADINGS["turn7"])   # the 150 mm drive
    det = ax * by - ay * bx
    a = (-shift_x * by + shift_y * bx) / det
    b = (-ax * shift_y + ay * shift_x) / det
    return a, b


def keep_c2_lined_up(back_mm):
    """Backing up less before challenge 2 leaves the robot further along
    the back-up line, which would put challenge 2 off to the side. This
    works out how much to change the 340 mm drive so that the robot is back
    in line with challenge 2 (it then just ends up a little further in).
    Returns the change in mm for the 340 drive (negative = shorter)."""
    shorter_by = 140 - back_mm
    back_heading = GYRO_HEADINGS["turn9"]     # the back-up (35)
    drive_heading = GYRO_HEADINGS["turn8"]    # the 340 mm drive (-10)
    c2_heading = GYRO_HEADINGS["C2"]          # facing challenge 2 (83)
    # How far each move pushes the robot sideways, as seen at challenge 2:
    # sideways = distance * sin(move heading - challenge 2 heading)
    side_from_back = shorter_by * sin((back_heading - c2_heading) * pi / 180)
    side_per_mm = sin((drive_heading - c2_heading) * pi / 180)
    return -side_from_back / side_per_mm


async def press_forward(extra_mm, time_ms, speed):
    """Keeps pushing forward slowly - for time_ms, or until it has gone
    extra_mm further. Gives a lever more "pressed time", even when the
    wheels skid. Prints:  PRESS,<mm it moved>,<ms>,<highest drive load>"""
    start = robot.distance()
    timer = StopWatch()
    biggest_load = 0
    robot.drive(speed, 0)
    while robot.distance() - start < extra_mm and timer.time() < time_ms:
        load = max(abs(await get(left_motor.load())),
                   abs(await get(right_motor.load())))
        biggest_load = max(biggest_load, load)
        await wait(10)
    robot.stop()
    print_csv("PRESS", round(robot.distance() - start), timer.time(),
              round(biggest_load))


async def jam_in(name, extra_mm=None):
    """Drive back in FAST to hit the lever: C2_JAM_BACK_MM + C2_JAM_EXTRA_MM
    forward, at C2_JAM_SPEED, speeding up and slowing down quickly
    (C2_JAM_ACCEL). Stops at the same spot every time, or after
    C2_JAM_TIMEOUT_MS if the lever stops it.
    Prints:  JAM,<name>,<arm height>,<mm moved>,<mm meant>,<top speed>,
                 <highest drive load>
    Did it hit the lever?  A hit shows a high load (about 240-310) and/or
    stops short of "mm meant". Missing it looks like a normal drive (about
    90-110). Returns the highest load."""
    if extra_mm is None:
        extra_mm = C2_JAM_EXTRA_MM
    distance = C2_JAM_BACK_MM + extra_mm
    normal = robot.settings()
    robot.settings(straight_speed=C2_JAM_SPEED,
                   straight_acceleration=C2_JAM_ACCEL)
    start = robot.distance()
    top = {"speed": 0, "load": 0}

    async def watch():
        while True:
            top["speed"] = max(top["speed"], robot.state()[1])
            load = max(abs(await get(left_motor.load())),
                       abs(await get(right_motor.load())))
            top["load"] = max(top["load"], load)
            await wait(10)

    # race=True: done when the drive finishes (or the time runs out).
    await multitask(robot.straight(distance), watch(),
                    wait(C2_JAM_TIMEOUT_MS), race=True)
    robot.stop()
    robot.settings(*normal)
    print_csv("JAM", name, round(-lift.angle()), round(robot.distance() - start),
              distance, round(top["speed"]), round(top["load"]))
    return top["load"]


async def turn(name, degrees):
    """Turn like robot.turn(degrees) - or, with USE_GYRO on, to the exact
    gyro heading for this turn. Prints the real heading afterwards:
      CHECK,<name>,<real heading>,<where Task C's turns add up to>,<off by>"""
    path["heading"] += degrees
    target = GYRO_HEADINGS.get(name, path["heading"])
    path["target"] = target
    if USE_GYRO:
        await headings.turn_to(target)
    else:
        await robot.turn(degrees)
    headings.check(name, target)


async def drive(name, distance):
    """A drive worth logging (prints the lines the sensors cross). With
    USE_GYRO on, it steers toward the heading the whole way."""
    if USE_GYRO:
        # Steer toward the heading the last turn aimed at.
        await headings.drive_on_heading(name, path["target"], distance, 200)
    else:
        await straight_and_watch(name, distance)


# ---------------- The run (Task C's moves, same numbers) ----------------

async def do_levers():
    """Task Q's lever up and lever down, with Task Q's settings. Starts
    right after challenge 1 (backed out, arm up); ends at the lever-down
    spot with the arm down."""
    # Hop to Task Q's lever start (about 10 cm), carefully, lowering the arm
    # to 600 on the way - the height Task Q drives in under the lever with.
    await multitask(
        position.go_to("to_levers", LEVER_START[0], LEVER_START[1],
                       C3_HEADING, True),
        lift_to_height(600, LIFT_FREE_SPEED),
    )
    await headings.turn_to(nearest_angle(C3_HEADING, headings.heading()))
    headings.check("C3_lever", C3_HEADING)

    # Lever up: drive in under it, lift, hold, let go. (Same as Task Q.)
    await straight_and_watch("C3_approach", C3_APPROACH_MM)
    await lift_to_height(1500)
    await wait(C3_WAIT_MS)
    await lift_to_height(1100)

    # Lever down: back up a little, then push. If the arm feels it landed
    # on the lever, it stops; if it dropped to the bottom (a miss), it tries
    # the next push in Task Q's C4_PUSHES list. (Same as Task Q.)
    await robot.straight(-C4_BACKUP_MM)
    forward_now = 0
    push = 0
    for extra_turn, extra_forward in C4_PUSHES:
        push += 1
        if push > 1:
            await lift_to_height(C4_RETRY_LIFT, LIFT_FREE_SPEED)
        await headings.turn_to(nearest_angle(C4_HEADING + extra_turn,
                                             headings.heading()))
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
    if forward_now != 0:
        await robot.straight(-forward_now)
    # Back straight off the lever before turning away. Turning with the arm
    # still resting on the lever got the robot stuck (run 11: the wheels
    # turned 128 degrees but the robot only 20, until the arm lifted free).
    await robot.straight(-LEVER_EXIT_MM)


async def back_to_c2_route():
    """From the lever-down spot back onto Task C's path to challenge 2. The
    lever-down spot is only 2 cm from Task C's path, so it drives straight
    to where Task C's 300 mm drive ends (almost the same line, about 3
    degrees different), then Task C's 150 mm drive - each one to the spot
    where it ends, from wherever the robot actually is."""

    async def raise_arm_once_clear():
        # Wait until the robot has turned away from the levers, then raise
        # the arm to the height it drove to challenge 2 with before.
        await wait(800)
        await lift_to_height(C1_ARM_AFTER_HEIGHT, LIFT_FREE_SPEED)

    await multitask(
        position.go_to("drive_300", C2_WAYPOINT_1[0], C2_WAYPOINT_1[1],
                       GYRO_HEADINGS["turn7"]),
        raise_arm_once_clear(),
    )
    await position.go_to("drive_150", C2_WAYPOINT_2[0], C2_WAYPOINT_2[1],
                         GYRO_HEADINGS["turn8"])


async def run_m():
    robot.settings(straight_speed=200, turn_rate=90)
    home = robot.distance()

    # Same start as Tasks A, B and C: reset the lift while driving forward
    # until the right color sensor sees GREEN then WHITE.
    _, found = await multitask(
        reset_lift_safely(),
        find_pattern(Color.GREEN, Color.WHITE, max_distance=PATTERN_MAX_MM)
    )
    if not found:
        # Don't carry on blind - go back to where we started.
        print("going home")
        await robot.straight(home - robot.distance())
        return

    # From here on, keep track of where the robot is on the mat (the
    # pattern stop is (0, 0)), alongside the rest of the run.
    position.start()
    await multitask(run_after_pattern(), position.keep_track(), race=True)


async def run_after_pattern():
    """Everything after the pattern stop: challenge 1, Task Q's two levers,
    then challenge 2."""
    # Out to challenge 1. The lift resets during the long first drive.
    await turn("turn1", -75)
    await multitask(
        drive("out", OUT_DRIVE_MM),
        reset_lift_safely(),
    )
    await turn("turn2", -60)
    # Arm right down to the bottom (and held there) during this drive - well
    # before the robot squeezes between the two models to challenge 1.
    await multitask(
        robot.straight(100),
        lower_to_bottom(C1_BOTTOM_FORCE),
    )
    await turn("turn3", -5)
    await robot.straight(90)
    # The last drive in: slowly, stopping if it bumps the model on the right.
    c1_heading = path["target"]
    driven = await headings.drive_until_bump("C1_in", c1_heading, C1_LAST_MM,
                                             BUMP_TWIST, CAREFUL_SPEED,
                                             BUMP_TILT)
    # How far short of Task C's 55 mm it is. The rest of the path was worked
    # out for 55, so the drives after challenge 1 make up for the difference.
    c1_short = 55 - driven
    # Should still point at turn3's heading. If it's way off, the robot ran
    # into the model on the way in (batch 1: twisted 16 to 29 degrees).
    headings.check("C1_arrive", path["target"])

    # Turn a little left, ready to scoop up the lever (the arm is already
    # down at the bottom).
    await headings.turn_to(path["target"] + C1_TURN_BEFORE_DIP)
    headings.check("C1_dip", path["target"] + C1_TURN_BEFORE_DIP)

    # Challenge 1: turn in and lift high (scoops up the lever). The turn
    # still ends at the same heading as before - it just has 5 less to go.
    await turn("C1", -15)
    await lift_to_height(2200)
    # Back out of challenge 1 FIRST, then turn. Turning straight after the
    # lift swung the robot (fork up high) into the model with the seed on
    # it - the motors strained (load 102-120) during that turn.
    await robot.straight(-50)
    await turn("turn5", C1_TURN_AFTER_LIFT)
    await lift_to_height(C1_ARM_AFTER_HEIGHT)
    await drive("back_C1", -C1_BACK_MM)
    print_csv("POS", "after_C1", one_decimal(position.x),
              one_decimal(position.y), one_decimal(headings.heading()))

    if DO_LEVERS:
        # Task Q's lever up and lever down, then back onto Task C's path.
        # (The position tracker drives each hop from wherever the robot
        # really is, so it also makes up for a short stop at challenge 1.)
        await do_levers()
        if STOP_AFTER_LEVERS:
            print("STOP_AFTER_LEVERS is on - stopping after the levers")
            return
        await back_to_c2_route()
        path["heading"] = GYRO_HEADINGS["turn7"]   # keep the turn count right
    else:
        # Straight over to challenge 2, making up for backing out at a
        # different angle than Task C did (and any short stop at C1).
        extra_300, extra_150 = drives_after_back_up(path["target"], c1_short,
                                                    c1_heading)
        print_csv("BACKUP_FIX", round(extra_300), round(extra_150),
                  round(c1_short))
        await turn("turn6", 50)
        await drive("drive_300", 300 + extra_300)
        await turn("turn7", 45)
        await robot.straight(150 + extra_150)
    await turn("turn8", 65)
    # The 340 mm drive, shortened a little to make up for the shorter
    # back-up, and ended using the RED/WHITE edge on the mat.
    adjust = keep_c2_lined_up(C2_BACK_MM)
    print_csv("C2_FIX", round(adjust))
    if USE_GYRO:
        # Watch for the edge only if C2_USE_LINE is on. Otherwise start the
        # search past the end of the drive, so it never looks - it just
        # drives the exact distance, still stopping if it bumps a model.
        search_from = C2_LINE_SEARCH_FROM
        if not C2_USE_LINE:
            search_from = 10000
        await headings.drive_to_color_edge(
            "drive_340", path["target"], "R", Color.RED, Color.WHITE,
            search_from, 330, C2_LINE_EXTRA_MM + adjust, 340 + adjust,
            200, C2_DRIVE_MAX_TWIST, BUMP_TILT)
        if headings.last_bumped and C2_SKIP_AFTER_BUMP:
            # It hit a model on the way. From the wrong spot challenge 2
            # would miss anyway, and pushing on is how pieces break off.
            print("bumped a model on the way to challenge 2 - stopping here")
            return
    else:
        await drive("drive_340", 340 + adjust)
    await turn("turn9", 45)
    # Back up slowly, stopping if it backs into the model on the left.
    await headings.drive_until_bump("C2_back", path["target"], -C2_BACK_MM,
                                    BUMP_TWIST, CAREFUL_SPEED, BUMP_TILT)

    # Challenge 2: turn in, set the arm height, drive in, and keep pressing.
    await turn("C2", 45)
    await lift_to_height(C2_ARM_HEIGHT)
    await robot.straight(C2_PUSH_MM)
    await press_forward(C2_PRESS_MM, C2_PRESS_MS, C2_PRESS_SPEED)
    best_hit = 0   # the hardest any fast push hit the lever
    if C2_JAM:
        # Back up, then drive back in FAST to hit the lever and jam it in,
        # then keep pressing.
        await robot.straight(-C2_JAM_BACK_MM)
        if C2_JAM_TURN:
            await headings.turn_to(GYRO_HEADINGS["C2"] + C2_JAM_TURN)
            headings.check("C2_jam", GYRO_HEADINGS["C2"] + C2_JAM_TURN)
        best_hit = max(best_hit, await jam_in("jam"))
        await press_forward(C2_PRESS_MM, C2_PRESS_MS, C2_PRESS_SPEED)

    if C2_THIRD_PUSH_RAISE:
        # Third push: back up while raising the arm a little, then drive in
        # fast again and keep pressing.
        print("third push - arm at", C2_ARM_HEIGHT + C2_THIRD_PUSH_RAISE)
        await multitask(
            robot.straight(-C2_JAM_BACK_MM),
            lift_to_height(C2_ARM_HEIGHT + C2_THIRD_PUSH_RAISE),
        )
        best_hit = max(best_hit, await jam_in("third"))
        await press_forward(C2_PRESS_MM, C2_PRESS_MS, C2_PRESS_SPEED)

    # Retry while no fast push has hit the lever.
    retry = 0
    while C2_JAM and best_hit < C2_HIT_LOAD and retry < C2_RETRIES:
        retry += 1
        print("lever not hit - retry", retry)
        await multitask(
            robot.straight(-C2_JAM_BACK_MM),
            lift_to_height(C2_ARM_HEIGHT),
        )
        best_hit = max(best_hit, await jam_in(
            "retry" + str(retry), C2_JAM_EXTRA_MM + C2_RETRY_EXTRA_MM * retry))
        await press_forward(C2_PRESS_MM, C2_PRESS_MS, C2_PRESS_SPEED)

    if GO_HOME:
        await go_home()


async def go_home():
    """Back out of challenge 2, put the arm down, and back into the left
    home area along the team's route (see HOME_BACK_OUT_MM and below)."""
    print("going home (left)")
    # The home turns count from the challenge 2 heading (not from the
    # jam turn).
    path["heading"] = GYRO_HEADINGS["C2"]
    await multitask(
        robot.straight(-HOME_BACK_OUT_MM),
        lift_to_height(0, LIFT_FREE_SPEED),
    )
    await turn("home_turn1", HOME_TURN_1)
    await drive("home_back1", -HOME_BACK_1_MM)
    await turn("home_turn2", HOME_TURN_2)
    await drive("home_back2", -HOME_BACK_2_MM)
    position.update()
    print_csv("POS", "home", one_decimal(position.x), one_decimal(position.y),
              one_decimal(headings.heading()))


async def run_m_with_setup():
    """Start check and gyro first, then the run. Always leaves the robot
    driving like the other tasks afterwards."""
    if CHECK_START:
        await check_start_position()
    headings.start()
    path["heading"] = 0
    path["target"] = 0
    gyro_on = False
    if USE_GYRO:
        gyro_on = await wait_for_gyro()
    robot.use_gyro(gyro_on)
    try:
        await run_m()
    finally:
        robot.use_gyro(False)


async def task():
    """Menu C: the run (pick Cd to run it with the recorder)."""
    await run_m_with_setup()
