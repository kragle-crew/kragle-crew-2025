from init import robot


async def task():

    robot.settings(straight_speed=535)
    for value in range(0,2):
      await robot.curve(250, 360) 
      await robot.straight(125)
      await robot.turn (-90) 
      await robot.straight(250)
      await robot.turn (-90) 
      await robot.straight(250)
      await robot.turn (-90) 
      await robot.straight(250)
      await robot.turn (-90) 
      await robot.straight(125)
    # await robot.straight(100)
    # await robot.turn (90)
    # await robot.straight(100)
    # await robot.turn (90)
    # await robot.straight(100)
    # radius = 280/3
    # angle = 180
    # for var in range(0, 2):
    #   await robot.curve(radius, angle * -1)
    #   await robot.curve(radius, angle)
    #   await robot.curve(radius, angle)
    #   await robot.curve(radius, angle * -1)
    # await robot.curve(200,-180)
    