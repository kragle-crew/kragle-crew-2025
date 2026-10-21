from init import robot,lift,pressure,colorRight
from utilities.lift import reset_lift, lift_to_height
from pybricks.tools import multitask,wait
from pybricks.parameters import Color   


async def task():
    robot.settings(straight_speed=200,turn_rate=90)
    #take off
    robot.drive(100,0)
    while await colorRight.color() != Color.YELLOW:
      print(await colorRight.color())

    robot.brake()

    
   