# Task T: runs Task Q with the diagnostic recorder, which logs the gyro,
# both color sensors, the drive motors, and the lift motor.
#
# When Task Q finishes, the hub shows "?":
#   press RIGHT if the run WORKED, press LEFT if it FAILED.
#
# To stop a run early, press CENTER (or LEFT and RIGHT together). The robot
# stops and goes back to the menu, and the computer keeps logging.
# To end the whole program, press the BLUETOOTH button.

from tasks.diagnostic import diagnose
from tasks.taskQ import task as task_q


async def task():
    await diagnose(task_q, "Q")
