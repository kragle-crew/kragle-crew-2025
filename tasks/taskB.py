from init import robot,lift,pressure,colorRight,colorLeft
from utilities.lift import reset_lift, lift_to_height
from pybricks.tools import multitask,wait
from pybricks.parameters import Color

async def doChallenge1():
    
    robot.settings(straight_speed=200,turn_rate=90)
    robot.drive(100,0)

    while await colorRight.color() != Color.RED:
        print(await colorRight.color())

    robot.brake()
    
    # await lift.run_angle(360,360)
    await multitask(reset_lift(), lift_to_height(100))
    #await robot.curve(260,179)
    await robot.straight(200)
    await robot.turn(90)
    await robot.straight(480)
    await robot.turn(85)
    await robot.straight(210)
    await robot.turn(45)
    await robot.turn(-35)
    await robot.straight(-70)
    await robot.curve(-80,249)
    await robot.turn(155)
    await lift_to_height(50)
    await robot.straight(200)
    await robot.straight(-150)
    await robot.turn(-45)
    await robot.straight(430)
    await robot.turn(40)
    await multitask(robot.straight(175), lift_to_height(260))
    await reset_lift()
    await robot.turn(60)


   
                                                        
    #await robot.turn(-45)
    #await lift_to_height(0)
    #await robot.straight(-200)


async def task():
    #await robot.turn(360)
    await doChallenge1()