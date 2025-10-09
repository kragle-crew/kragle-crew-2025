from init import robot


async def task():
  robot.settings(speed=500)
  for value in range(0,2000):
    await robot.straight(1000)
    await robot.turn(90)