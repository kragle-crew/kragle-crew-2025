from init import robot,lift,pressure

# async def square():
#     await robot.straight(400)
#     await robot.turn(-90)
#     await robot.straight(400) 
    
#     await robot.turn(-90)

#     await robot.straight(400)
#     await robot.turn(-90)
   
        
#     await robot.straight(400)
#     await robot.turn(-90)

async def liftGoDown():
    await lift.run_target (400,0)
async def task():
    #await robot.turn(360)
    robot.settings(straight_speed=200)
    
    # await lift.run_angle(360,360)

    await lift.run_target(400,-950,)
    await robot .straight(60)
    await robot.turn (30)
    await robot.turn (-60)
    await robot.turn (30)
    await robot.straight (-60)
    await liftGoDown()
    # await robot.curve(300,-80)
    # await square()

    # await square()
    # await square()
    
