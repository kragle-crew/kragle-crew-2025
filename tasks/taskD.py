from pybricks.parameters import Color

from pybricks.tools import multitask,wait
from init import robot,lift,pressure,colorRight
from utilities.lift import reset_lift, lift_to_height

async def task():
  robot.drive(100,0)
  
  while await colorRight.color() != Color.RED:
    print(await colorRight.color())

  robot.brake()
  robot.settings(straight_speed=1000)
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
  await lift_to_height(1500, 2000)
  await robot.straight(100)

  await robot.straight(-790)
  # await robot.straight(50)
  # await lift_to_height(750)
  # robot.settings(straight_speed=40)
  # await multitask(
  #   robot.straight(-200),
  # lift_to_height(2000)
  # )