from init import robot,lift,pressure,colorRight
from utilities.lift import reset_lift, lift_to_height
from pybricks.tools import multitask,wait
from pybricks.parameters import Color

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

async def find_not_color(color: Color):
    robot.drive(100,0)
    while await colorRight.color() != color:
        print(await colorRight.color())
    print(await colorRight.color())
    robot.brake()

async def find_pattern(*pattern: Color, max_distance=1000):
    # We only record color changes, so repeats in the pattern could never match.
    # Collapse them (e.g. WHITE, WHITE -> WHITE).
    pattern = [c for i, c in enumerate(pattern) if i == 0 or c != pattern[i - 1]]
    start_distance = robot.distance()
    robot.drive(100,0)
    print("looking for pattern", pattern)
    recently_seen = CircularBuffer(maxlen=len(pattern))
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
    print("pattern found!")
    robot.brake()
    return True



async def doChallenge1():
    robot.settings(straight_speed=200,turn_rate=90)
    
    

   
    _, found = await multitask(
      reset_lift(),
      find_pattern(Color.GREEN, Color.WHITE)
    )
    # Don't move on if we never found the pattern
    if not found:
      pass

    await robot.straight(210)
    await robot.turn(45)
    await robot.straight(110)
    await robot.turn(-45)
    await robot.straight(120)
    await lift_to_height(1200)
    await lift_to_height(900)
    await robot.straight(-140)
    await robot.turn(45)
    await robot.straight(-100)
    await robot.turn(-45)
    await robot.straight(-210)
    await robot.turn(-64)
    await robot.straight(400)
    await robot.turn(20)
    await lift_to_height(2000)
    await robot.turn(-45)
    await robot.straight(100)
    await robot.turn(17)
    await lift_to_height(600)
    await robot.straight(113)
    await lift_to_height(1500)
    await wait(2000)
    await lift_to_height(1100)
    await robot.straight(-15)
    await lift_to_height(50)
    await robot.straight(-60)
    await robot.turn(-30)
    await robot.straight(1300)
    await reset_lift()

async def task():
    #await robot.turn(360)
    await doChallenge1()