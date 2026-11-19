from pybricks.parameters import Color

from pybricks.tools import multitask,wait
from init import robot,lift,pressure,colorRight
from utilities.lift import reset_lift, lift_to_height

# ---- Settings you can change ----
DRIVE_SPEED = 800      # mm/s for the drives (was 1000)
EXTRA_RIGHT = 5        # extra turn right at the second model, before lifting
PUSH_MM = 100          # first lift: push forward this far WHILE lifting
PUSH_SPEED = 150       # slow, so the push lasts about as long as the lift
FIRST_LIFT = 1500      # first lift height
BACK_UP_MM = 50        # then back up this far...
SECOND_LIFT = 2000     # ...and lift again to here to finish extending it

async def task():
  robot.drive(100,0)

  while await colorRight.color() != Color.RED:
    print(await colorRight.color())

  robot.brake()
  robot.settings(straight_speed=DRIVE_SPEED)
  await reset_lift()
  # await multitask(
  #   reset_lift(),
  #   robot.turn(90)
  # )

  # await robot.straight(550)
  # await robot.turn(50)
  # await robot.straight(55)
  # await lift_to_height(2100)
  # await robot.straight(-100)

  await robot.turn(58)
  await multitask(
    lift_to_height(1800),
    robot.straight(340)
  )
  await robot.straight(-300)
  await robot.turn(-58)
  await robot.turn(7)
  await multitask(
    robot.straight(700),
    reset_lift()
  )

  # Second model: turn a bit more right, then lift and push forward at the
  # same time.
  await robot.turn(EXTRA_RIGHT)
  robot.settings(straight_speed=PUSH_SPEED)
  await multitask(
    lift_to_height(FIRST_LIFT, 2000),
    robot.straight(PUSH_MM)
  )
  # Back up a little and lift again to finish extending the model.
  await robot.straight(-BACK_UP_MM)
  await lift_to_height(SECOND_LIFT, 2000)
  robot.settings(straight_speed=DRIVE_SPEED)

  # Home: turn back to the old heading, then back up the same line as before
  # (it used to back up 790 from the end of the push).
  await robot.turn(-EXTRA_RIGHT)
  await robot.straight(-(790 - BACK_UP_MM))
  # await robot.straight(50)
  # await lift_to_height(750)
  # robot.settings(straight_speed=40)
  # await multitask(
  #   robot.straight(-200),
  # lift_to_height(2000)
  # )
