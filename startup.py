from pybricks.tools import multitask, wait, run_task
from pybricks.parameters import Icon, Color
from init import hub
from menu import menu

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
    hub.speaker.beep(frequency=500, duration=200)
    hub.light.on(Color.ORANGE)
    run_task(task())
    print(f"completed {option}")
    hub.speaker.beep(frequency=700, duration=200)
