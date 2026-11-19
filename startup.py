from pybricks.tools import multitask, wait, run_task
from pybricks.parameters import Icon, Color
from init import hub
from menu import menu
from utilities.run_helpers import run_with_abort, RECORDED_TASKS

print("starting up")
hub.speaker.beep(frequency=500, duration=200)

print(f"battery voltage {hub.battery.voltage()}mV")
print(f"battery current {hub.battery.current()}mA")
print(f"hub info {hub.system.info()}")
print("entering menu loop")

option="A"

while True:
    print("presenting menu")
    hub.light.on(Color.GREEN)
    option, task = menu(option)
    print(f"selected {option}. Running function")
    print(f"battery voltage {hub.battery.voltage()}mV")
    hub.speaker.beep(frequency=500, duration=200)
    hub.light.on(Color.ORANGE)
    if option in RECORDED_TASKS:
        # The recorder handles the stop buttons itself, and logs the run.
        run_task(task())
    else:
        # LEFT+RIGHT (or CENTER) stops the run and comes back to the menu,
        # instead of ending the whole program.
        run_task(run_with_abort(task))
    print(f"completed {option}")
    hub.speaker.beep(frequency=700, duration=200)
