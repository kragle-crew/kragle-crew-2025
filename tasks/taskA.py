from init import robot,lift,pressure,colorRight,colorLeft
from utilities.lift import reset_lift, lift_to_height
from pybricks.tools import multitask,wait
from pybricks.parameters import Color   



# async def square():
#     await robot.straight(400)
#     await robot.turn(-90)
#     await robot.straight(400) 
    
#     await robot.turn(-90)

#     await robot.straight(400)
#     await robot.turn(-90)
   
        
#     await robot.straight(400)
#     await robot.turn(-90)

async def doChallenge1():
    robot.settings(straight_speed=200,turn_rate=90)
    robot.drive(100,0)

    while await colorRight.color() != Color.RED:
        print(await colorRight.color())

    robot.brake()
    
    # await lift.run_angle(360,360)
    await multitask(
      reset_lift(),
      robot.straight(200)
    )
        # await reset_lift()
    # await robot .straight(200)
    await robot.curve(400,57)
    await robot.turn(-145)
    await multitask(
        lift_to_height(967),
        robot.straight(110)
    )   
    await robot.straight(67)
    await robot.turn(40)
    await robot.turn(-70)
    await robot.turn(30)
    await robot.straight(-70)
    await reset_lift()
    await robot.turn(147)
    await robot.straight(370)
    await robot.turn(-15)
    await lift_to_height(2300)
    await wait(1000)
    await reset_lift()
    await robot.straight(-100)
    await robot.turn(-95)
    await lift_to_height(670)
    await robot.straight(367)
    await lift_to_height(2067)
    await robot.straight(-20)
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
    
