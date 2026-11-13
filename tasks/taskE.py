from pybricks.parameters import Color

from pybricks.tools import multitask,wait
from init import robot,lift,pressure,colorRight
from utilities.lift import reset_lift, lift_to_height

async def task():
  robot.drive(100,0)
  
  while await colorRight.color() != Color.RED:
    print(await colorRight.color())

  robot.brake()
  await reset_lift()
  await multitask( 
    lift_to_height(1400),
    robot.turn(90)
  )

  await robot.straight(265)
  await robot.turn(90)
  await robot.straight(95)
  await robot.turn(-90)
  robot.settings(straight_speed=20)
  await robot.straight(800)