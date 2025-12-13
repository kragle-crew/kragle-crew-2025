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
    
    await multitask(
      reset_lift(),
      robot.straight(200)
    )

    await robot.curve(400,57)
    await robot.turn(-145)
    await multitask(lift_to_height(967), robot.straight(110))   
    await robot.straight(67)
    await robot.turn(40)
    await robot.turn(-70)
    await robot.turn(30)
    await robot.straight(-70)
    #turn toward minecart
    await multitask(reset_lift(),robot.turn(150))
    await robot.straight(360)
    await lift_to_height(2300)
    await wait(1000)
    await reset_lift()
    await robot.straight(-110)
    #turn toward plants
    await robot.turn(-121)
    await multitask(lift_to_height(750),robot.straight(200))
    await robot.straight(120)
    await lift_to_height(2067)
    await robot.straight(-50)
    await robot.turn(-110)
    await multitask(reset_lift(),robot.straight(1067))

async def liftGoDown():
    await lift.run_target(400,0)
async def task():
    #await robot.turn(360)
    await doChallenge1()

async def drive2():
    await robot.curve(355,-15)
    await robot.straight(200)
    
