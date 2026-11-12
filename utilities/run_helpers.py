# Helpers that make a run more reliable, shared by the tasks.
# (The same ideas that made Task Y work: gyro, start check, heading checks.)

from init import hub, robot, colorRight, colorLeft
from utilities.readings import get, color_name
from pybricks.tools import multitask, wait, StopWatch
from pybricks.parameters import Color, Button


def print_csv(*values):
    print(",".join(str(v) for v in values))


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
        Prints:  AIM,<target>,<heading before>,<heading after>"""
        before = self.heading()
        if abs(target - before) > 8:
            await robot.turn(target - before)

        timer = StopWatch()
        while timer.time() < timeout_ms:
            error = target - self.heading()
            if abs(error) <= tolerance:
                break
            # Turn slowly (15 degrees per second) toward the target.
            if error > 0:
                robot.drive(0, 15)
            else:
                robot.drive(0, -15)
            await wait(10)
        robot.brake()
        await wait(50)  # let it settle
        print_csv("AIM", target, round(before), round(self.heading()))


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
