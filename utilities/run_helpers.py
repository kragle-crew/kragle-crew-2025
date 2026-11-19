# Helpers that make a run more reliable, shared by the tasks.
# (The same ideas that made Task Y work: gyro, start check, heading checks.)

from init import hub, robot, lift, left_motor, right_motor, colorRight, colorLeft
from utilities.readings import get, color_name
from pybricks.tools import multitask, wait, StopWatch
from pybricks.parameters import Color, Button
from umath import sin, cos, atan2, sqrt, pi


def print_csv(*values):
    print(",".join(str(v) for v in values))


# Menu tasks that run with the recorder. The recorder already handles
# stopping a run with the buttons (and logs it), so run_with_abort()
# leaves these alone.
RECORDED_TASKS = ("Ad", "Bd", "Cd", "Dd")


async def run_with_abort(task_function):
    """Runs a task. Press LEFT and RIGHT together (or CENTER) to stop it
    early: the robot stops and the program carries on - so the computer
    keeps logging, instead of the whole program ending.
    Returns True if the run was stopped early."""
    # CENTER would normally end the whole program, so move that job to
    # the BLUETOOTH button while the task runs.
    try:
        hub.system.set_stop_button(Button.BLUETOOTH)
    except Exception as e:
        print("could not move the stop button:", e)

    async def watch_buttons():
        while True:
            pressed = hub.buttons.pressed()
            if Button.CENTER in pressed:
                return True
            if Button.LEFT in pressed and Button.RIGHT in pressed:
                return True
            await wait(20)

    # race=True: whichever finishes first (the task, or a button) wins.
    results = await multitask(task_function(), watch_buttons(), race=True)
    if results[1] is True:
        print("Run stopped with a button")
        robot.stop()
        robot.use_gyro(False)
        lift.stop()
        while hub.buttons.pressed():  # wait for the buttons to be let go
            await wait(10)
        return True
    return False


def one_decimal(number):
    """Rounds to one decimal place, e.g. 12.5306 -> 12.5.
    (The hub's Python can't do round(number, 1) on a whole number like 90 -
    it crashes - so we do it this way instead.)"""
    return int(round(number * 10)) / 10


async def wait_for_gyro(timeout_ms=2000):
    """The gyro needs the robot to sit still for a moment before it's ready.
    Returns True if the gyro is ready to use."""
    timer = StopWatch()
    while not hub.imu.ready():
        if timer.time() > timeout_ms:
            print("gyro not ready - driving without it this run")
            return False
        await wait(50)
    return True


async def check_start_position(right_color=Color.WHITE, left_color=Color.YELLOW):
    """Make sure the robot is in its starting spot before moving, by checking
    what the color sensors see. In every logged run from home, the right
    sensor saw WHITE and the left saw YELLOW.
    If the colors look wrong: fix the robot, then press RIGHT to check again,
    or press LEFT to go anyway."""
    while True:
        right = await get(colorRight.color())
        left = await get(colorLeft.color())
        ok = right == right_color and left == left_color
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


class HeadingChecker:
    """Keeps track of the robot's real heading (from the gyro), measured from
    where it pointed at the start of the run. Call start() at the start."""

    def __init__(self):
        self.start_heading = 0
        self.last_bumped = False   # did the last careful drive hit something?

    def start(self):
        self.start_heading = hub.imu.heading()

    def heading(self):
        """Which way the robot points now, in degrees. 0 = the way it pointed
        at the start. Clockwise (right) is positive."""
        return hub.imu.heading() - self.start_heading

    def check(self, name, target):
        """Print the real heading next to the heading it should have.
        Prints:  CHECK,<name>,<heading>,<target>,<off by>"""
        heading = self.heading()
        print_csv("CHECK", name, round(heading), target, round(heading - target))

    async def turn_to(self, target, tolerance=1, timeout_ms=2000):
        """Turn until the gyro says the robot points at `target` degrees.

        robot.turn() calls a turn "done" when it is within a few degrees,
        so small turns often stop 2 to 4 degrees short. This does a normal
        turn first (if it is a big one), then turns SLOWLY on the spot until
        the gyro is within `tolerance` degrees. It gives up after timeout_ms,
        so it can never get stuck if something blocks the robot.

        About 1 degree is as exact as a standing robot can turn - ask for
        less and the wheels stick and jump, and it wastes 2 seconds. For a
        LONG drive that must be exact, use drive_on_heading() instead: it
        steers while moving, which is easy for the robot.
        Prints:  AIM,<target>,<heading before>,<heading after>"""
        before = self.heading()
        if abs(target - before) > 8:
            await robot.turn(target - before)

        timer = StopWatch()
        rate = 15          # degrees per second to nudge at
        turning = 0        # which way we last told it to turn: -1, 0 or 1
        last_heading = self.heading()
        last_moved = timer.time()
        while timer.time() < timeout_ms:
            now = self.heading()
            error = target - now
            if abs(error) <= tolerance:
                # Looks right - but the robot can still drift a little after
                # the brakes go on. Stop, wait, and look again.
                robot.brake()
                await wait(60)
                if abs(target - self.heading()) <= tolerance:
                    break
                turning = 0
                rate = 15
                last_heading = self.heading()
                last_moved = timer.time()
                continue

            # Is the robot actually turning? If it hasn't moved for 0.3
            # seconds the wheels are stuck, so push a bit harder.
            if abs(now - last_heading) >= 0.3:
                last_heading = now
                last_moved = timer.time()
            elif timer.time() - last_moved > 300 and rate < 45:
                rate += 10
                turning = 0   # send the command again with the new rate
                last_moved = timer.time()

            # Tell it to turn ONCE, not every time round the loop - sending
            # the command over and over keeps restarting the motors.
            direction = 1
            if error < 0:
                direction = -1
            if direction != turning:
                robot.drive(0, rate * direction)
                turning = direction
            await wait(10)
        robot.brake()
        await wait(50)  # let it settle
        # One decimal place, so we can see small misses in the logs.
        print_csv("AIM", one_decimal(target), one_decimal(before),
                  one_decimal(self.heading()))

    async def drive_on_heading(self, name, target, distance, speed=200):
        """Drives `distance` mm (negative = backward) while STEERING toward
        the gyro heading `target` the whole way.

        robot.straight() keeps whatever heading the robot had when it set
        off - so if the turn before it was 1.5 degrees off, a 580 mm drive
        ends 15 mm to the side. This keeps steering back to `target`, so a
        small turn error is gone in the first 10 cm.

        The last 120 mm is handed to robot.straight(), so it still stops at
        exactly the right distance. Also prints the lines the color sensors
        cross, like straight_and_watch().
        Prints:  DRIVE,<name>,<heading at start>,<heading at end>,
                       <average heading while steering>,<target>,
                       <worst heading error>,<highest drive motor load>
                 LINES,<name>,R,<mm>:<color> ...   and the same for L
        A big "worst heading error" together with a high load = the robot
        bumped something on this drive."""
        start = robot.distance()
        before = self.heading()
        lines = {"R": [], "L": []}
        bumps = {"worst": 0, "load": 0}

        async def watch_for_bumps():
            while True:
                bumps["worst"] = max(bumps["worst"], abs(target - self.heading()))
                load = max(abs(await get(left_motor.load())),
                           abs(await get(right_motor.load())))
                bumps["load"] = max(bumps["load"], load)
                await wait(10)

        async def watch_sensors():
            while True:
                for side, sensor in (("R", colorRight), ("L", colorLeft)):
                    color = color_name(await get(sensor.color()))
                    seen = lines[side]
                    if (len(seen) == 0 or seen[-1][1] != color) and len(seen) < 30:
                        seen.append((round(abs(robot.distance() - start)), color))
                await wait(5)

        steering = {"total": 0, "count": 0}

        async def steer_and_drive():
            direction = 1          # 1 = forward, -1 = backward
            if distance < 0:
                direction = -1
            # Steer toward the target heading until 120 mm is left...
            while abs(distance) - abs(robot.distance() - start) > 120:
                now = self.heading()
                steering["total"] += now
                steering["count"] += 1
                error = target - now
                # How fast to turn back toward the heading (turning right is
                # positive whether we drive forward or backward):
                #  - less than 0.3 degrees off: close enough, go straight
                #  - otherwise 5 degrees per second for each degree off,
                #    but at least 8 (the robot ignores smaller commands - it
                #    used to sit about 1 degree to one side the whole drive)
                #    and never more than 30.
                if abs(error) < 0.3:
                    turn_rate = 0
                else:
                    turn_rate = 5 * abs(error)
                    if turn_rate < 8:
                        turn_rate = 8
                    if turn_rate > 30:
                        turn_rate = 30
                    if error < 0:
                        turn_rate = -turn_rate
                robot.drive(direction * speed, turn_rate)
                await wait(10)
            # ...then let robot.straight() finish, so it stops exactly.
            await robot.straight(distance - (robot.distance() - start))

        # race=True: stop watching as soon as the drive is done.
        await multitask(steer_and_drive(), watch_sensors(), watch_for_bumps(),
                        race=True)
        # The average heading shows whether the robot really ran on its line.
        average = before
        if steering["count"] > 0:
            average = steering["total"] / steering["count"]
        print_csv("DRIVE", name, one_decimal(before), one_decimal(self.heading()),
                  one_decimal(average), one_decimal(target),
                  one_decimal(bumps["worst"]), round(bumps["load"]))
        for side in ("R", "L"):
            print_csv("LINES", name, side,
                      " ".join(str(mm) + ":" + color for mm, color in lines[side]))

    async def drive_until_bump(self, name, target, distance, max_twist=3,
                               speed=100, max_tilt=3.5):
        """Drives `distance` mm (negative = backward) SLOWLY, steering toward
        gyro heading `target` - and STOPS straight away if the robot bumps
        into something. Pushing on is how pieces get knocked off models.
        It counts as a bump if the robot gets:
          - TWISTED more than max_twist degrees off its heading, or
          - TILTED more than max_tilt degrees (one side or the front riding
            up onto something - normal driving stays under about 1.5).
        The first 15 mm don't count (the robot jolts a little as it sets
        off), and if the robot was already a bit off `target` when it
        started, that much extra twist is allowed.
        Returns how many mm it actually drove (always positive), so the
        caller can make up for stopping short. self.last_bumped says whether
        it stopped because of a bump.
        Prints:  BUMP,<name>,<mm driven>,<mm it was meant to drive>,
                      <worst twist>,<highest drive load>,<worst tilt>
                 (or CAREFUL,... with the same numbers if nothing was hit)"""
        start = robot.distance()
        direction = 1
        if distance < 0:
            direction = -1
        bumped = False
        worst = 0
        worst_tilt = 0
        biggest_load = 0
        allowed_twist = max_twist + abs(target - self.heading())
        # Steer most of the way; the last 8 mm is done by robot.straight()
        # below so it still stops at exactly the right distance.
        while abs(robot.distance() - start) < abs(distance) - 8:
            error = target - self.heading()
            pitch, roll = hub.imu.tilt()
            if abs(robot.distance() - start) > 15:
                worst = max(worst, abs(error))
                worst_tilt = max(worst_tilt, abs(pitch), abs(roll))
                if abs(error) > allowed_twist or worst_tilt > max_tilt:
                    bumped = True
                    break
            if abs(error) < 0.3:
                turn_rate = 0
            else:
                turn_rate = 5 * abs(error)
                if turn_rate < 8:
                    turn_rate = 8
                if turn_rate > 30:
                    turn_rate = 30
                if error < 0:
                    turn_rate = -turn_rate
            robot.drive(direction * speed, turn_rate)
            load = max(abs(await get(left_motor.load())),
                       abs(await get(right_motor.load())))
            biggest_load = max(biggest_load, load)
            await wait(5)
        if bumped:
            robot.brake()
        else:
            await robot.straight(distance - (robot.distance() - start))
        tag = "CAREFUL"
        if bumped:
            tag = "BUMP"
        driven = abs(robot.distance() - start)
        print_csv(tag, name, round(driven), abs(distance), one_decimal(worst),
                  round(biggest_load), one_decimal(worst_tilt))
        self.last_bumped = bumped
        return driven

    async def drive_to_color_edge(self, name, target, side, from_color,
                                  to_color, search_from_mm, search_to_mm,
                                  extra_mm, normal_mm, speed=200,
                                  max_twist=1000, max_tilt=1000):
        """Drives forward, steering toward gyro heading `target`, and uses a
        line on the mat to end up in EXACTLY the same place every run.

        It watches one color sensor (`side` = "R" or "L") for the edge where
        the color changes from `from_color` to `to_color`, somewhere between
        search_from_mm and search_to_mm into the drive. (It still counts if
        another color shows for a moment in between - the sensor often reads
        YELLOW for a few mm going from RED to WHITE.) Once it sees the edge,
        it drives extra_mm more and stops. If it never sees the edge, it
        drives exactly normal_mm, like a plain drive.

        If max_twist or max_tilt is given, it also STOPS if it bumps into
        something (same test as drive_until_bump), and self.last_bumped
        becomes True.
        Prints:  EDGE,<name>,<mm where it saw the edge, -1 = not seen>,
                      <mm driven in total>,<worst heading error>,<highest load>,
                      <worst tilt>,<bumped?>"""
        sensor = colorRight
        if side == "L":
            sensor = colorLeft
        start = robot.distance()
        from_seen_at = -1000      # where it last saw from_color
        edge_at = -1
        worst = 0
        worst_tilt = 0
        biggest_load = 0
        bumped = False
        allowed_twist = max_twist + abs(target - self.heading())
        while True:
            travelled = robot.distance() - start
            if edge_at < 0 and travelled >= normal_mm - 30:
                # Never saw the edge: finish the normal distance exactly.
                await robot.straight(normal_mm - travelled)
                break
            if edge_at >= 0 and travelled >= edge_at + extra_mm:
                break                     # saw it, and drove extra_mm past it
            color = await get(sensor.color())
            if color == from_color:
                from_seen_at = travelled
            elif (edge_at < 0 and color == to_color
                    and search_from_mm <= travelled <= search_to_mm
                    and travelled - from_seen_at < 15):
                edge_at = travelled

            # Steer toward the heading, the same way as drive_on_heading().
            error = target - self.heading()
            pitch, roll = hub.imu.tilt()
            if travelled > 15:
                worst = max(worst, abs(error))
                worst_tilt = max(worst_tilt, abs(pitch), abs(roll))
                if abs(error) > allowed_twist or worst_tilt > max_tilt:
                    bumped = True
                    break
            if abs(error) < 0.3:
                turn_rate = 0
            else:
                turn_rate = 5 * abs(error)
                if turn_rate < 8:
                    turn_rate = 8
                if turn_rate > 30:
                    turn_rate = 30
                if error < 0:
                    turn_rate = -turn_rate
            # Slow down once the edge is found, so it stops in the same place.
            if edge_at >= 0:
                robot.drive(100, turn_rate)
            else:
                robot.drive(speed, turn_rate)
            load = max(abs(await get(left_motor.load())),
                       abs(await get(right_motor.load())))
            biggest_load = max(biggest_load, load)
            await wait(5)
        robot.brake()
        await wait(50)
        self.last_bumped = bumped
        print_csv("EDGE", name, round(edge_at), round(robot.distance() - start),
                  one_decimal(worst), round(biggest_load),
                  one_decimal(worst_tilt), bumped)


def nearest_angle(angle, reference):
    """The same direction as `angle`, written within 180 degrees of
    `reference`. (The gyro heading keeps counting past 180 - so -238 and 122
    point the same way, and this picks whichever is the shorter turn.)"""
    return reference + ((angle - reference + 180) % 360 - 180)


class Position:
    """Keeps track of WHERE the robot is on the mat: it adds up every mm the
    wheels drive, in the direction the gyro says the robot is pointing.

    The map is in mm from where the robot stops after the GREEN/WHITE
    pattern at the start (the same spot for every task that starts from
    home): x = mm to the right, y = mm forward (the way it faced at the start).

    Run keep_track() alongside the run (multitask), then go_to(x, y) drives
    to a spot from wherever the robot ACTUALLY is - so a short stop or a
    small slip earlier doesn't carry on through the rest of the run."""

    def __init__(self, headings):
        self.headings = headings
        self.x = 0
        self.y = 0
        self.last = 0

    def start(self):
        """Call this at the pattern stop: that spot is (0, 0)."""
        self.x = 0
        self.y = 0
        self.last = robot.distance()

    def update(self):
        now = robot.distance()
        step = now - self.last
        self.last = now
        h = self.headings.heading() * pi / 180
        self.x += step * sin(h)
        self.y += step * cos(h)

    async def keep_track(self):
        while True:
            self.update()
            await wait(10)

    async def go_to(self, name, x, y, next_heading=None, careful=False,
                    speed=200, max_twist=3, max_tilt=3.5):
        """Turns toward (x, y) and drives there in a straight line.

        It drives forward or backward, whichever needs less turning -
        counting the turn to next_heading afterwards too, if given.
        careful=True drives slowly and stops on a bump (drive_until_bump).
        Prints:  GOTO,<name>,<x now>,<y now>,<x to>,<y to>,<heading>,<mm>
                 POS,<name>,<x after>,<y after>,<heading after>"""
        self.update()
        dx = x - self.x
        dy = y - self.y
        dist = sqrt(dx * dx + dy * dy)
        now = self.headings.heading()
        forward = nearest_angle(atan2(dx, dy) * 180 / pi, now)
        backward = nearest_angle(forward + 180, now)

        def turning(h):
            total = abs(h - now)
            if next_heading is not None:
                total += abs(nearest_angle(next_heading, h) - h)
            return total

        heading = forward
        distance = dist
        if turning(backward) < turning(forward):
            heading = backward
            distance = -dist
        print_csv("GOTO", name, one_decimal(self.x), one_decimal(self.y),
                  round(x), round(y), one_decimal(heading), round(distance))
        if dist >= 5:
            await self.headings.turn_to(heading)
            if careful:
                await self.headings.drive_until_bump(name, heading, distance,
                                                     max_twist, 100, max_tilt)
            else:
                await self.headings.drive_on_heading(name, heading, distance,
                                                     speed)
        self.update()
        print_csv("POS", name, one_decimal(self.x), one_decimal(self.y),
                  one_decimal(self.headings.heading()))


async def straight_and_watch(name, distance):
    """Drives exactly like robot.straight(distance), but also prints every
    color change each sensor sees, with how far into the drive (mm).
    Prints:  LINES,<name>,R,<mm>:<color> ...   and the same for L"""
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
