# Task Q: Task B (as of Nov 11, 2026), plus changes to make it more reliable.
#
# Four challenges: two seeds lifted off branches, then a lever lifted up,
# then the arm drops down to push a lever down.
# Run it with the recorder from menu T (see taskT.py) to log the sensors.
# taskB.py is not changed - you can still run B to compare.
#
# It drives the same path with the same numbers as Task B.
# Search for "CHANGE" to find what's new:
#   1. Uses the gyro to steer. Task B's turns already match the gyro within
#      about 2 degrees (checked in the logged run), so this keeps B's path but
#      stops it drifting a different amount each run. In 5 of the first 6
#      runs, the robot ended up off to the side (mostly too far left).
#   2. After finding the GREEN/WHITE pattern, it moves to the same spot every
#      time, instead of wherever the brakes happened to stop it.
#   3. If it doesn't find the pattern where it always is, it drives home
#      instead of carrying on.
#   4. Checks it is in the right starting spot before it moves.
#   5. Prints the real heading at each challenge ("CHECK" lines), and every
#      line the color sensors cross on the approach drives ("LINES").
#      These don't change the driving - they tell us what to tune next.
#   6. Seed 1: shifts the robot a little to the RIGHT. In 4 of 5 runs it was
#      too far left, even though it was pointing the right way (heading 0
#      every time) - so it needs to slide over, not turn.
#   7. Seed 2: aims a little further right. Seed 2 worked when pointing at
#      -40 to -43; the misses were at -46 to -49.
#   8. Both levers: aims at the headings where they worked (or, for the
#      second lever, a little right of where it missed).
#   9. Turns to EXACT gyro headings at each challenge. robot.turn() stops
#      when it's within a few degrees, so every challenge was ending up
#      2 to 4 degrees too far left. turn_to() nudges until it's within 1.
#  10. Drives a little further in at seed 1 and the first lever.
#  11. Drives in more slowly at seed 1, so the arm doesn't bump the seed.
#  12. Goes into seed 1 from the FRONT (facing right) instead of the side.
#      Set C1_FROM_FRONT = False to go back to the side approach.
#  13. The arm drops onto the second lever 5 mm further forward.
#  14. Second lever: always pushes twice - the second time turned a little
#      more clockwise. Logs the lift effort on each push.
#  15. Holds the first lever up for half a second instead of 2 seconds.
#  16. After seed 1, shifts the rest of the run a little to the right.

from init import robot, lift, pressure, colorRight
from utilities.lift import reset_lift, lift_to_height
from utilities.readings import get
from utilities.run_helpers import (print_csv, wait_for_gyro,
                                   check_start_position, HeadingChecker,
                                   straight_and_watch)
from pybricks.tools import multitask, wait
from pybricks.parameters import Color
from umath import atan2, sqrt, pi, sin, cos


# ---------------- Settings you can change ----------------

USE_GYRO = True       # CHANGE 1: steer with the gyro. False = drive like Task B

# CHANGE 2: how far past the pattern edge to stop, in mm. Task B's path was
# tuned with wherever the brakes stopped it - about 5 mm past the edge on
# average (measured in Task Y, which starts from the same spot). So 5 keeps
# Task B's tuning, just without the run-to-run wobble.
PATTERN_STOP_MM = 5

CHECK_START = True    # CHANGE 4: check the start spot before moving

# CHANGE 3: in every logged run the pattern was found 146 to 152 mm from the
# start. If it isn't found by this distance, something is wrong (once it
# drove 572 mm), so the robot goes home instead of carrying on.
PATTERN_MAX_MM = 250

# CHANGE 10: how far to drive in before lifting, in mm.
#   Seed 1: "hit the seed but didn't knock it off" 3 times out of 4,
#   so drive in a bit further (Task B: 120).
C1_APPROACH_MM = 125
#   Lever up: "didn't go forward enough" (Task B: 113).
C3_APPROACH_MM = 118

# CHANGE 15: how long to hold the first lever up, in ms (Task B: 2000).
C3_WAIT_MS = 500

# CHANGE 11: be gentler with seed 1. The arm kept touching the seed and then
# flicking it or slipping off, or bumping it on the way in. The path and arm
# are the same - only the speeds change.
C1_APPROACH_SPEED = 100   # mm/s on the last drive in (the rest of the run: 200)
C1_LIFT_SPEED = 600       # lift speed for seed 1 (back to full speed - notes
                          # said "maybe we need to lift faster")

# CHANGE 12: go into seed 1 from the FRONT (robot turned right, facing +90)
# instead of from the side. False = the old side approach.
C1_FROM_FRONT = True
# Measure this on the robot: from the middle point between the two wheels
# to the tip of the fork, in mm, straight forward. Everything else is worked
# out from it, so it matters!
ARM_REACH_MM = 120
# How far the robot drives in (facing the seed) before lifting, in mm.
#   Fork stops short of the seed?   Make it bigger.
#   Pushes the seed / hits it hard?  Make it smaller.
C1_FRONT_IN_MM = 70
# Slides the lifting point along the seed, in mm.
#   Fork hits too close to home (the start side)?   Make it bigger.
#   Fork hits too far from home?                    Make it smaller (can go
#                                                   below 0).
C1_FRONT_ALONG_MM = -30   # was 0: "pushes too far into the map by about 3 cm"
C1_FRONT_HEADING = 90     # facing the seed: turned right about 90 degrees

# CHANGE 16: after seed 1, move the rest of the run this many mm to the RIGHT
# (as seen by the robot at seed 2 and both levers). Seed 2 missed too far
# left in 5 of 6 runs, and the lever and C4 failed in those same runs - the
# whole second half of the run was sitting too far left.
#   Seed 2 / levers still too far LEFT?   Make it bigger (by about 5).
#   Now too far RIGHT?                    Make it smaller.   0 = off.
AFTER_C1_SHIFT_MM = 10

# Test mode for tuning seed 1 at the table: True = stop right after seed 1
# (lift, back out, stop). Set back to False for full runs!
TEST_C1_ONLY = False
# The back-ups afterwards are made longer by the same amount, so the rest
# of the path doesn't move.

# CHANGE 6: how far to slide the robot RIGHT for seed 1, in mm.
# 8 mm is one Lego stud. 0 = same as Task B.
#   Still too far LEFT at seed 1?   Make it bigger (by about 4).
#   Now too far RIGHT?              Make it smaller.
# The robot slides back the same amount on the way out, so seed 2 and the
# levers are not affected.
C1_SHIFT_RIGHT_MM = 12   # 8 still missed too far LEFT in 4 of 9 runs

# CHANGES 7, 8, 9: which way the robot points (gyro heading, in degrees) at
# each challenge. 0 = the way it pointed at the start. Negative = left.
# The robot turns until the gyro reads this, within 1 degree (CHANGE 9).
#   Too far LEFT at a challenge?   ADD 2  (e.g. -40 becomes -38).
#   Too far RIGHT?                 SUBTRACT 2  (e.g. -40 becomes -42).
C1_HEADING = 0      # seed 1: worked at 0
C2_HEADING = -40    # seed 2: worked at -40 (Task B pointed at about -46)
C3_HEADING = -71    # lever up: worked at -71 both times; missed at -72 to -75
# CHANGE 13: how far to back up from the first lever before the arm drops
# onto the second lever, in mm. Was 20; "needs to go forward a little more".
C4_BACKUP_MM = 15
# CHANGE 14: always try a second push on the second lever. The arm lifts
# back up, the robot turns a little right and moves a little forward, and
# the arm pushes down again. (In the logs the lift motor felt exactly the
# same whether it pushed the lever or missed it, so the robot can't tell a
# miss yet - it just tries twice. If the first push worked, the second one
# just presses a lever that is already down.)
C4_RETRY = True
C4_RETRY_TURN = 3          # degrees MORE clockwise for the second push
C4_RETRY_FORWARD_MM = 0    # mm further forward for the second push
                           # (was 10 - now just the clockwise turn)
C4_DROP_SPEED = 300        # lift speed for pushing down (was 600) - slower
                           # makes it easier to feel the lever
# Lift load that means "the arm is pushing the lever". Not known yet, so
# None = always do the second push. Each push prints "C4PUSH,<try>,<load>".
C4_PUSH_LOAD = None
C4_HEADING = -62    # lever down: at -64 to -66 it missed too far LEFT 14 times
                    # (right only twice), so turn more clockwise.
                    # Now too far RIGHT?  Try -63.  Still LEFT?  Try -60.

headings = HeadingChecker()


def diagonal(forward_mm, right_mm):
    """The turn (degrees) and drive (mm) that end up forward_mm ahead and
    right_mm to the right. Task B's diagonal to seed 1 was turn 45 and
    drive 110, which is about 78 mm forward and 78 mm right."""
    angle = atan2(right_mm, forward_mm) * 180 / pi
    length = sqrt(forward_mm * forward_mm + right_mm * right_mm)
    return angle, length


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
    # Same as Task B, plus CHANGE 2 at the end.
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

    # CHANGE 2: the brakes don't stop in exactly the same place every time,
    # so move to exactly PATTERN_STOP_MM past where we saw the pattern.
    await wait(100)  # let the robot settle
    overshoot = robot.distance() - seen_at
    print_csv("PATTERN", round(seen_at - start_distance), round(overshoot))
    await robot.straight(PATTERN_STOP_MM - overshoot)
    return True


async def push_lever_down(attempt):
    """Drops the arm down to push the second lever (challenge 4), and
    measures how hard the lift motor works while it does.
    Returns True if it felt the lever (only once C4_PUSH_LOAD is set).
    Prints:  C4PUSH,<attempt>,<biggest lift load>,<felt it?>"""
    biggest = {"load": 0}

    async def watch_lift():
        while True:
            load = abs(await get(lift.load()))
            biggest["load"] = max(biggest["load"], load)
            await wait(10)

    # race=True: stop watching as soon as the arm is all the way down.
    await multitask(lift_to_height(50, C4_DROP_SPEED), watch_lift(), race=True)
    felt = C4_PUSH_LOAD is not None and biggest["load"] >= C4_PUSH_LOAD
    print_csv("C4PUSH", attempt, round(biggest["load"]), felt)
    return felt


async def seed1_from_side():
    """The old way (changes 6, 9, 10, 11): slide over, drive straight in
    from the side, lift. Starts and ends facing 0, like Task B."""
    # CHANGE 6: Task B did turn(45), straight(110), turn(-45) here. This is
    # the same diagonal, but it ends C1_SHIFT_RIGHT_MM further right.
    angle, length = diagonal(77.8, 77.8 + C1_SHIFT_RIGHT_MM)
    await robot.turn(angle)
    await robot.straight(length)
    await robot.turn(-angle)
    await headings.turn_to(C1_HEADING)  # CHANGE 9
    headings.check("C1_seed", C1_HEADING)
    robot.settings(straight_speed=C1_APPROACH_SPEED)         # CHANGE 11
    await straight_and_watch("C1_approach", C1_APPROACH_MM)  # CHANGE 10
    robot.settings(straight_speed=200)
    await lift_to_height(1200, C1_LIFT_SPEED)                # CHANGE 11
    await lift_to_height(900, C1_LIFT_SPEED)
    await robot.straight(-140 - (C1_APPROACH_MM - 120))      # CHANGE 10
    # CHANGE 6: Task B did turn(45), straight(-100), turn(-45) here. Same
    # diagonal back, but it slides C1_SHIFT_RIGHT_MM further left, to undo
    # the shift.
    angle, length = diagonal(70.7, 70.7 + C1_SHIFT_RIGHT_MM)
    await robot.turn(angle)
    await robot.straight(-length)
    await robot.turn(-angle)


async def seed1_from_front():
    """CHANGE 12: go into seed 1 from the front.

    The fork tip ends up at the SAME spot it lifted the seed from in the old
    side approach - but the robot comes in facing right (+90) instead of
    facing forward (0). Then it backs out and ends at exactly the same place
    as the old way, so seed 2 and the levers don't move.

    Map, in mm, from where the robot stopped after the pattern (x = right,
    y = forward, at heading 0). It is now at (0, 210)."""
    # Where the fork tip lifted the seed in the old side approach:
    lift_x = 77.8 + C1_SHIFT_RIGHT_MM
    lift_y = 210 + 77.8 + C1_APPROACH_MM + ARM_REACH_MM + C1_FRONT_ALONG_MM

    # 1. Diagonal to a spot to the LEFT of the seed. The robot's middle must
    #    be ARM_REACH_MM + C1_FRONT_IN_MM left of the lifting point, so that
    #    after turning right the fork tip is C1_FRONT_IN_MM away from it.
    side_x = lift_x - ARM_REACH_MM - C1_FRONT_IN_MM
    angle, length = diagonal(77.8, side_x)
    await robot.turn(angle)
    await robot.straight(length)
    await robot.turn(-angle)
    await headings.turn_to(0)

    # 2. Drive forward until level with the lifting point.
    forward = lift_y - (210 + 77.8)
    await straight_and_watch("C1_front_fwd", forward)

    # 3. Turn right to face the seed, then drive in slowly and lift.
    await headings.turn_to(C1_FRONT_HEADING)
    headings.check("C1_seed", C1_FRONT_HEADING)
    robot.settings(straight_speed=C1_APPROACH_SPEED)
    await straight_and_watch("C1_front_in", C1_FRONT_IN_MM)
    robot.settings(straight_speed=200)
    await lift_to_height(1200, C1_LIFT_SPEED)
    await lift_to_height(900, C1_LIFT_SPEED)

    # 4. Back out the same way.
    await robot.straight(-C1_FRONT_IN_MM)
    if TEST_C1_ONLY:
        return
    await headings.turn_to(0)
    await robot.straight(-forward)

    # 5. Diagonal back to where the old path ended up before its last
    #    straight(-210): 7.1 mm right and 197.1 mm forward of the start spot.
    #    CHANGE 16: plus AFTER_C1_SHIFT_MM in the direction that is "right"
    #    for the robot at seed 2 (facing -40) AND the levers (facing -71):
    #    about 35 degrees right of straight ahead. Everything after this
    #    moves over by the same amount.
    shift_x = AFTER_C1_SHIFT_MM * sin(35 * pi / 180)
    shift_y = AFTER_C1_SHIFT_MM * cos(35 * pi / 180)
    end_x = 7.1 + shift_x
    end_y = 197.1 + shift_y
    angle, length = diagonal(287.8 - end_y, side_x - end_x)
    await robot.turn(angle)
    await robot.straight(-length)
    await robot.turn(-angle)
    await headings.turn_to(0)


async def doChallenge1():
    robot.settings(straight_speed=200,turn_rate=90)
    home = robot.distance()

    _, found = await multitask(
      reset_lift(),
      find_pattern(Color.GREEN, Color.WHITE, max_distance=PATTERN_MAX_MM)
    )
    if not found:
        # CHANGE 3: don't carry on blind - go back to where we started.
        print("going home")
        await robot.straight(home - robot.distance())
        return

    # Challenge 1: first seed.
    await straight_and_watch("to_C1", 210)
    if C1_FROM_FRONT:
        await seed1_from_front()        # CHANGE 12
        if TEST_C1_ONLY:
            print("TEST_C1_ONLY is on - stopping after seed 1")
            return
    else:
        await seed1_from_side()
    await robot.straight(-210)

    # Challenge 2: second seed.
    await robot.turn(-64)
    await straight_and_watch("to_C2", 400)
    await headings.turn_to(C2_HEADING)  # CHANGE 7: Task B did turn(20)
    headings.check("C2_seed", C2_HEADING)
    await lift_to_height(2000)

    # Challenges 3 and 4: lever up, then push the lever down.
    await headings.turn_to(-89)         # same direction as Task B's path
    await robot.straight(100)
    await headings.turn_to(C3_HEADING)  # CHANGE 8: Task B did turn(17)
    await lift_to_height(600)
    headings.check("C3_lever", C3_HEADING)
    await straight_and_watch("C3_approach", C3_APPROACH_MM)  # CHANGE 10
    await lift_to_height(1500)
    await wait(C3_WAIT_MS)                                   # CHANGE 15
    await lift_to_height(1100)
    await robot.straight(-C4_BACKUP_MM)                      # CHANGE 13
    await headings.turn_to(C4_HEADING)  # CHANGE 8: aim a little right
    headings.check("C4_lever", C4_HEADING)
    felt = await push_lever_down(1)
    # CHANGE 14: second push, a little to the right and a little forward.
    if C4_RETRY and not felt:
        await lift_to_height(1100)      # lift the arm back up, off the lever
        await headings.turn_to(C4_HEADING + C4_RETRY_TURN)
        headings.check("C4_retry", C4_HEADING + C4_RETRY_TURN)
        if C4_RETRY_FORWARD_MM != 0:
            await robot.straight(C4_RETRY_FORWARD_MM)
        await push_lever_down(2)
        if C4_RETRY_FORWARD_MM != 0:
            await robot.straight(-C4_RETRY_FORWARD_MM)
    await headings.turn_to(-72)         # back to Task B's direction for home

    # Home.
    await robot.straight(-60)
    await robot.turn(-30)
    headings.check("home", -102)
    # Turned off for practice: the long drive to the far home base. Another
    # group is working there. Remove the # to put it back for competition.
    # await robot.straight(1300)
    await reset_lift()


async def task():
    if CHECK_START:
        await check_start_position()   # CHANGE 4
    headings.start()
    gyro_on = False
    if USE_GYRO:
        gyro_on = await wait_for_gyro()
    robot.use_gyro(gyro_on)            # CHANGE 1
    try:
        await doChallenge1()
    finally:
        # Put things back so the other tasks drive exactly like before.
        robot.use_gyro(False)
