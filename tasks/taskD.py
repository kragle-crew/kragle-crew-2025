from init import robot, lift
from utilities.lift import reset_lift, lift_to_height


async def task():
    await lift_to_height(950)
    await reset_lift()
    # await drawN()
    # await drawJ()
    # robot.settings(straight_speed=280)
    # sideLength=300
    # while sideLength>10:
    # await square(sideLength)
    # sideLength=sideLength/2
    # for value in range(0, 20):
    # await robot.curve(100, 360)
    # await robot.curve(100, -360)
    # print(value)
    # await robot.straight(382.28)


async def drawN():
    #print(sideLength)
    #for value in range(0, 4):
        #await robot.straight(sideLength)
        #await robot.turn(90)
  await robot.straight(228)
  await robot.turn(135)
  await robot.straight(322.44)
  await robot.turn(-45)
  await robot.straight(228)
  await robot.turn(90)
  
  
async def drawJ():
  await robot.straight(20)
  await robot.straight(200)
  await robot.turn(180)
  await robot.straight(100)
  await robot.turn(-90)
  await robot.straight(200)
  await robot.curve(100,180)