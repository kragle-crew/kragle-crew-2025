# Quick testing: shows the menu (starting on A) and runs the task you pick.
#   - The run FINISHES: the program ends, so the hub is straight away ready
#     for new code. (startup.py never ends, so you have to stop it first.)
#   - You STOP the run early (LEFT and RIGHT together, or CENTER): the menu
#     comes back, and the computer keeps logging - so you can reset the
#     board and try again.
#
# Run it with:   pybricksdev run ble --name "not bob" run_once.py
# To save a log:  ... run_once.py | Tee-Object -FilePath taskC-runs-1.txt
# or in VS Code: pick "Menu, run once" in the Run and Debug list, press F5.
#
# For competition, keep using startup.py.

from pybricks.tools import run_task
from pybricks.parameters import Color
from init import hub
from menu import menu
from utilities.run_helpers import run_with_abort, RECORDED_TASKS

option = "A"
while True:
    print("menu: pick a task - it runs once, then the program ends")
    hub.light.on(Color.GREEN)
    option, task = menu(option)
    print(f"selected {option}. Running function")
    print(f"battery voltage {hub.battery.voltage()}mV")
    hub.speaker.beep(frequency=500, duration=200)
    hub.light.on(Color.ORANGE)
    if option in RECORDED_TASKS:
        # The recorder handles the stop buttons itself, and logs the run.
        run_task(task())
        stopped_early = False
    else:
        stopped_early = run_task(run_with_abort(task))
    if stopped_early:
        print(f"stopped {option} early - back to the menu")
        hub.speaker.beep(frequency=300, duration=300)
        continue
    hub.speaker.beep(frequency=700, duration=200)
    print(f"completed {option} - the hub is ready for new code")
    break
