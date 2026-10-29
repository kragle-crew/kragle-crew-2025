from pybricks.parameters import Color

from pybricks.tools import multitask,wait
from init import robot,lift,pressure,colorRight
from utilities.lift import reset_lift, lift_to_height

async def task():
  robot.drive(100,0)
  
  while await colorRight.color() != Color.RED:
    print(await colorRight.color())

  robot.brake()
  
  await multitask( 
    reset_lift(),
    robot.turn(90)
  )

  await robot.straight(550)
  await robot.turn(50)
  await robot.straight(55)
  await lift_to_height(2100)
  await robot.curve(50, -60)
  
