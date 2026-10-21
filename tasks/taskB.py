from init import robot,lift,pressure,colorRight
from utilities.lift import reset_lift, lift_to_height
from pybricks.tools import multitask,wait
from pybricks.parameters import Color

async def doChallenge1():
    
    robot.settings(straight_speed=200,turn_rate=90)
    robot.drive(100,0)

    while await colorRight.color() != Color.RED:
        print(await colorRight.color())

    robot.brake()
  

async def task():
    #await robot.turn(360)
    await doChallenge1()