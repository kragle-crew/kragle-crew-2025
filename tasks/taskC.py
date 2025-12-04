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
    lift_to_height(108)
    )
    # Dinnertime Lever
    robot.settings(straight_speed=500)
    await robot.turn(-98)
    await lift_to_height(108)
    await robot.straight(122)
    robot.settings(straight_acceleration=383)
    await robot.curve(-140, 135)
    # table setter
    await robot.straight(-165)
    await robot.straight(28)
    await robot.turn(44)
    await reset_lift()
    robot.settings(straight_speed=500)
    await robot.straight(-25)
    await robot.straight(428)
    robot.settings(straight_speed=313)
    await robot.straight(-140)
    await robot.turn(-52)
    await robot.turn(-8)
    await robot.straight(285)
    await robot.straight(-150)
    await robot.turn(45)
    robot.settings(straight_speed=500)
    await robot.straight(200)
    # await robot.straight(58)
    await robot.turn(15)
    # await robot.straight(90)
    # await robot.turn(-20)
    await robot.straight(800)
    # await robot.turn(-15)
    # await robot.straight(200
    