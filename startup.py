from pybricks.tools import multitask, wait, run_task
from init import hub
from menu import menu

print("starting up")
hub.speaker.beep(frequency=500, duration=200)

print("entering menu loop")

while True:
    print("presenting menu")
    option, task = menu()
    print(f"selected {option}. Running function")
    hub.speaker.beep(frequency=500, duration=200)
    run_task(task())
    print(f"completed {option}")
    hub.speaker.beep(frequency=700, duration=200)
