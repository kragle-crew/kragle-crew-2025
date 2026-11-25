# Task D: two challenges on the left side of the field.
#   Challenge 1 - the model in the middle-left of the field. SKIPPED for
#                 now, while we work on challenge 2 (the code is kept below,
#                 switched off with #).
#   Challenge 2 - the root arm model at the top-left, near the far wall. Two
#                 parts: lift and drop the first lever (front left of the
#                 model), which frees the second one; then get the arm right
#                 under it and lift.
#
# Turns: positive = right, negative = left.
# Drives: positive = forward, negative = backward (mm).
# robot.settings(straight_speed=...) sets the speed for the drives after it.

from pybricks.parameters import Color

from pybricks.tools import multitask,wait
from init import robot,lift,pressure,colorRight
from utilities.lift import reset_lift, lift_to_height, lower_to_bottom


async def task():
  robot.drive(100,0)

  while await colorRight.color() != Color.RED:
    print(await colorRight.color())

  robot.brake()
  robot.settings(straight_speed=800)   # mm/s (was 1000)

  # ---- Challenge 1 (skipped for now) ----
  # To put it back: remove the # from these lines, and change challenge 2
  # back to turn(9), straight(590), turn(-10), and home turn(10),
  # straight(-740).
  # await reset_lift()
  # await robot.turn(58)
  # await multitask(
  #   lift_to_height(1800),
  #   robot.straight(340)
  # )
  # await robot.straight(-300)
  # await robot.turn(-58)

  # ---- Challenge 2: drive to the root arm model ----
  # Without challenge 1, the robot starts this about 3.4 cm further left and
  # 2 cm further back, so these aim at the SAME spot as the tested numbers
  # (turn 9, drive 590 + 100 after challenge 1).
  # Turn right before the long drive. 1 degree = about 1.2 cm sideways by
  # the end of the drive.
  await robot.turn(11)
  # The drive in: fast, then the last 100 slow so it doesn't bang into
  # anything. The lift resets on the way.
  await multitask(
    robot.straight(617),
    reset_lift()
  )
  await robot.turn(-10)
  robot.settings(straight_speed=150)
  await robot.straight(100)

  # Part 1: lift the first lever, then drop it (frees the second lever).
  # -12 here faces the same way as -10 did after challenge 1.
  await lift_to_height(1200)
  await lift_to_height(200)

  # Part 2: back up, arm right down, lift slowly, then push in while lifting.
  await robot.straight(-50)
  await lower_to_bottom(3)          # press the bottom with 3 N (like Task C)
  await robot.straight(50)
  await lift_to_height(1000, 500)   # lift speed 500 = about 2 seconds
  # Push 50 mm forward WHILE lifting to 2000 (1500 wasn't high enough).
  # Push speed 30 mm/s, so the push lasts about as long as the lift.
  robot.settings(straight_speed=30)
  await multitask(
    lift_to_height(2000),
    robot.straight(50)
  )
  robot.settings(straight_speed=800)

  # ---- Home ----
  # Undo the turn at the model (the opposite of the turn in part 1)...
  await robot.turn(12)
  # ...then back up as far as we drove in: 617 + 100 - 50 + 50 + 50 = 767.
  # If you change those drives, change this too.
  await robot.straight(-767)
