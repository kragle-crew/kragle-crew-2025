from init import robot,lift,pressure,colorRight,colorLeft
from utilities.lift import reset_lift, lift_to_height
from pybricks.tools import multitask,wait
from pybricks.parameters import Color   


async def task():
    robot.settings(straight_speed=350)
    await multitask(reset_lift(),robot.straight(10000))









    