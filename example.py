from pybricks.hubs import PrimeHub
from pybricks.tools import multitask, wait, run_task
from pybricks.robotics import DriveBase
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction

hub=PrimeHub()


left_motor = Motor(Port.E,Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.F,Direction.CLOCKWISE)

robot = DriveBase(left_motor, right_motor, 49, 100) 
print("init complete")
hub.speaker.beep(frequency=500, duration=200)

# right_motor.run_time(100,10000)


async def display_name():
    hub.display.text("Kragle Krew!")

async def rotate():
    print("starting rotation")
    await robot.turn(360)

async def main_program():
    await multitask(display_name(), rotate())
    
    print("program complete")
    await hub.speaker.beep(frequency=700, duration=200)
    

run_task(main_program())
wait(100)

