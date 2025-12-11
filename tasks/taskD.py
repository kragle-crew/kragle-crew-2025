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
    
    robot.settings(straight_speed=328)
    await reset_lift()
    await robot.straight(300)
    # Rock Fall
    robot.settings(straight_speed=350)
    await robot.turn(-40)
    #await lift_to_height(108)
    await robot.straight(360)
    #robot.settings(straight_acceleration=383)
    #await robot.curve(-140, 135)
    await robot.turn(75)
    await robot.straight(110)
    await robot.turn(35)
    await robot.turn(-35)
    await robot.straight(-100)
    await robot.turn(-90)
    await multitask(lift_to_height(2100),robot.straight(200))
    await robot.turn(-80)
    await robot.straight(130)
    await lift_to_height(1500)
    await robot.turn(50)
    await robot.straight(310)
    await robot.turn(-85)
    await robot.straight(450)
    await robot.turn(70)
    await multitask(reset_lift(),robot.straight(1700))









    