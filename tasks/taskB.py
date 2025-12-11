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
    await multitask(
        reset_lift(),
        lift_to_height(100)
    )
    await robot.curve(260,174)
    await robot.turn(30)
    await robot.turn(-20)
    await robot.straight(-70)
    await robot.curve(-80,250)
    await robot.turn(155)
    await lift_to_height(50)
    await robot.straight(200)
    await robot.straight(-150)
    await robot.turn(-45)
    await robot.straight(430)
    await robot.turn(40)
    await lift_to_height(260)
    await robot.straight(175)
    await reset_lift()
    await robot.turn(60)


   
                                                        
    #await robot.turn(-45)
    #await lift_to_height(0)
    #await robot.straight(-200)


async def task():
    #await robot.turn(360)
    await doChallenge1()