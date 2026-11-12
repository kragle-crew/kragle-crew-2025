# Helpers that make a run more reliable, shared by the tasks.
# (The same ideas that made Task Y work: gyro, start check, heading checks.)

from init import hub, robot, colorRight, colorLeft
from utilities.readings import get, color_name
from pybricks.tools import multitask, wait, StopWatch
from pybricks.parameters import Color, Button


def print_csv(*values):
    print(",".join(str(v) for v in values))


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
                       <average heading while steering>,<target>
                 LINES,<name>,R,<mm>:<color> ...   and the same for L"""
        start = robot.distance()
        before = self.heading()
        lines = {"R": [], "L": []}

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

        # race=True: stop watching the sensors as soon as the drive is done.
        await multitask(steer_and_drive(), watch_sensors(), race=True)
        # The average heading shows whether the robot really ran on its line.
        average = before
        if steering["count"] > 0:
            average = steering["total"] / steering["count"]
        print_csv("DRIVE", name, one_decimal(before), one_decimal(self.heading()),
                  one_decimal(average), one_decimal(target))
        for side in ("R", "L"):
            print_csv("LINES", name, side,
                      " ".join(str(mm) + ":" + color for mm, color in lines[side]))


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
