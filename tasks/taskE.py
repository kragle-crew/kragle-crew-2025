# Task E: lift the tree, and shake it so the seed pops off.
# Starts on the LEFT side of the field, the same way as Task D (drives
# forward until the right color sensor sees RED).
#
# Turns: positive = right, negative = left.
# Drives: positive = forward, negative = backward (mm).
# robot.settings(straight_speed=...) sets the speed for the drives after it.
#
# Pick E to run it, or Ed (flashing E) to run it with the recorder.

from pybricks.parameters import Color

from pybricks.tools import multitask,wait
from init import robot,lift,pressure,colorRight
from utilities.lift import reset_lift, lift_to_height


async def task():
  start = robot.distance()

  # ---- Start: same as Task D ----
  robot.drive(100,0)

  while await colorRight.color() != Color.RED:
    print(await colorRight.color())

  robot.brake()
  robot.settings(straight_speed=400)   # mm/s

  # ---- Drive to the tree ----
  # 690 mm forward OVERALL - counting the drive to the RED line, which can
  # be a little different each time. The lift resets on the way.
  already = robot.distance() - start
  print("drove", already, "mm to RED - now", 690 - already, "more")
  await multitask(
    robot.straight(690 - already),
    reset_lift()
  )
  await robot.turn(55)                  # right
  robot.settings(straight_speed=200)    # slower going in to the tree
  await robot.straight(195)             # was 210, then 200 (arm caught in the tree)
  robot.settings(straight_speed=400)

  # ---- Lift the tree ----
  await lift_to_height(2000)

  # ---- Rock the tree gently so the seed falls off ----
  # (Lifting and lowering didn't shake it loose.) While holding the tree
  # up, the robot turns a little left and right, then rocks forward and
  # back. The turns add up to 0 and the drives add up to 0, so it ends
  # where it started.
  #   Seed still not falling?  bigger turns (6) or drives (25), or faster.
  #   Too rough?               smaller numbers, or slower.
  normal_settings = robot.settings()
  robot.settings(turn_rate=40, straight_speed=50)   # slow = gentle
  # More rocks = more lines. Keep them in pairs (-8, 8 or -15, 15).
  await robot.turn(-4)
  await robot.turn(8)
  await robot.turn(-8)
  await robot.turn(8)
  await robot.turn(-8)
  await robot.turn(8)
  await robot.turn(-8)
  await robot.turn(8)
  await robot.turn(-4)                  # back to facing the tree
  await robot.straight(-15)
  await robot.straight(15)
  await robot.straight(-15)
  await robot.straight(15)
  await robot.straight(-15)
  await robot.straight(15)
  await robot.straight(-15)
  await robot.straight(15)
  robot.settings(*normal_settings)      # normal speeds again

  # ---- Lower the lift ----
  await lift_to_height(0)

  # ---- Home: the same way we came, backward ----
  # If you change the drives or the turn above, change these to match.
  robot.settings(straight_speed=200)    # slowly back out from the tree
  await robot.straight(-195)
  robot.settings(straight_speed=400)
  await robot.turn(-55)                 # undo the turn right
  await robot.straight(-690)            # back to where it started (690 overall)
