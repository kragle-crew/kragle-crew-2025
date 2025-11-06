from init import robot,lift,pressure,colorRight,colorLeft
from utilities.lift import reset_lift, lift_to_height
from pybricks.tools import multitask,wait
from pybricks.parameters import Color   


async def task():
    robot.settings(straight_speed=200,turn_rate=90)
    robot.drive(100,0)
    while await colorRight.color() != Color.YELLOW:
      print(await colorRight.color())

    robot.brake()
    await multitask(
      robot.straight(289.5), 
      reset_lift()
      )
    
    robot.settings(straight_speed=328)
    await robot.turn(-45)
    await multitask(
      robot.straight(328), 
      lift_to_height(98)
      )
    await robot.turn(-98)
    await robot.straight(133)
    await robot.curve(-140, 140)
    await robot.straight(-165)
    await robot.straight(28)
    await robot.turn(42)
    await reset_lift()
    robot.settings(straight_speed=75)
    await robot.straight(450)()
    robot.settings(straight_speed=313)
    await robot.straight(-528)
    await robot.turn(-42)
    
    
    
    # await robot.turn(-106)
    # await robot.straight(58)
    # await robot.turn(90)
    # await robot.straight(90)
    # await robot.turn(-2