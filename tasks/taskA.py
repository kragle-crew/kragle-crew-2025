from init import robot,lift,pressure
from utilities.lift import reset_lift, lift_to_height
from pybricks.tools import multitask


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
    robot.settings(straight_speed=200)
    
    # await lift.run_angle(360,360)
    await multitask(
        reset_lift(),
        robot.straight(200)
    )
    # await reset_lift()
    # await robot .straight(200)
    await robot.curve(400,57)
    await robot.turn (-140)
    await robot .straight(120)
    await lift_to_height(850)
    await robot .straight(70)
    await robot.turn (30)
    await robot.turn (-60)
    await robot.turn (30)
    await robot.straight (-60)
    await reset_lift()

async def liftGoDown():
    await lift.run_target (400,0)
async def task():
    #await robot.turn(360)
    await doChallenge1()

async def drive2():
    await robot.curve(350,-15)
    
