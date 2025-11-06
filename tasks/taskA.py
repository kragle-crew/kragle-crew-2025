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

    while await colorRight.color() != Color.YELLOW:
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
    await robot.turn (-140)
    await multitask(
        lift_to_height(700),
        robot.straight(120)
    )   
    await robot.straight(67)
    await robot.turn (35)
    await robot.turn (-60)
    await robot.turn (30)
    await robot.straight (-70)
    await reset_lift()
    await robot.turn(150)
    await robot.straight (480)
    await robot.turn (-80)
    await lift_to_height(2000)
    #await robot.straight(20)
    
async def liftGoDown():
    await lift.run_target (400,0)
async def task():
    #await robot.turn(360)
    await doChallenge1()

async def drive2():
    await robot.curve(350,-15)
    
