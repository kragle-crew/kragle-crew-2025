from init import robot
from utilities.lift import reset_lift, lift_to_height
from tasks.taskB import find_pattern
from pybricks.tools import multitask
from pybricks.parameters import Color


async def task():
    robot.settings(straight_speed=200, turn_rate=90)

    # Same start as Task A and Task B: reset the lift while driving forward
    # until the right color sensor sees GREEN then WHITE.
    _, found = await multitask(
        reset_lift(),
        find_pattern(Color.GREEN, Color.WHITE)
    )
    # If the pattern never showed up, stop here instead of driving blind.
    if not found:
        return

    await robot.turn(-75)       # rotate left 75 degrees
    await robot.straight(480)   # was 450 - came up short and hit a model
    await robot.turn(-60)       # rotate left 60 degrees
    await robot.straight(100)
    await robot.turn(-5)       # rotate left 10 degrees
    await robot.straight(90)
    # The last 50 mm while the lift resets at the same time.
    await multitask(
        robot.straight(85),
        reset_lift()
    )
    await robot.turn(-15)     
    await lift_to_height(2200)
    await robot.turn(-15)        
    await robot.straight(-50)
    await lift_to_height(600)
    await robot.straight(-270)
    await robot.turn(50)
    await robot.straight(300)  
    await robot.turn(45)
    await robot.straight(150)
    await robot.turn(65)
    await robot.straight(340)
    await robot.turn(45)
    await robot.straight(-140)
    await robot.turn(45)
    await lift_to_height(900)
    await robot.straight(200)
