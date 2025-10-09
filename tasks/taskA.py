from init import robot

# async def square():
#     await robot.straight(400)
#     await robot.turn(-90)
#     await robot.straight(400) 
    
#     await robot.turn(-90)

#     await robot.straight(400)
#     await robot.turn(-90)
   
        
#     await robot.straight(400)
#     await robot.turn(-90)


async def task():
    await robot.turn(360)
    robot.settings(straight_speed=500)
    # await robot.curve(300,-80)
    # await square()
    # await square()
    # await square()

