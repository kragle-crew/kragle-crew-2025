from init import robot
from utilities.lift import reset_lift, lift_to_height
from pybricks.tools import multitask


async def task():
    await multitask(
      robot.straight(300), 
      reset_lift()
      )
    robot.settings(straight_speed=328)
    await robot.turn(-45)
    await multitask(
      robot.straight(290), 
      lift_to_height(98)
      )
    await robot.turn(-98)
    await robot.straight(150)
    await robot.curve(-140, 140)
    await robot.straight(-128)
    
    
    
    # await robot.turn(-106)
    # await robot.straight(58)
    # await robot.turn(90)
    # await robot.straight(90)
    # await robot.turn(-2)
