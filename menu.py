from init import hub
from tasks.taskA import task as a
from tasks.taskB import task as b
from tasks.taskC import task as c
from tasks.taskD import task as d
from tasks.diagnostic import record_a, record_b, record_c, record_d

from pybricks.tools import wait, StopWatch
from pybricks.parameters import Button

# A, B, C, D = the four runs.
# Ad, Bd, Cd, Dd = the same runs with the diagnostic recorder (logs the
# sensors). The hub can only show one letter, so a "d" option shows its
# letter BLINKING.
menu_options = {"A": a, "B": b, "C": c, "D": d,
                "Ad": record_a, "Bd": record_b, "Cd": record_c, "Dd": record_d}
menu_option_keys = ["A", "B", "C", "D", "Ad", "Bd", "Cd", "Dd"]

BLINK_MS = 300   # how fast a "d" option blinks


def menu(starting_selection="A"):
    menu_index = menu_option_keys.index(starting_selection)

    def navigate(direction, old_index):
        print(f"navigating {direction}")
        return (old_index - direction) % len(menu_option_keys)

    print("Starting menu")
    print(f"menu options: {menu_option_keys}")

    hub.system.set_stop_button(None)
    print("beginning menu loop")
    while True:
        key = menu_option_keys[menu_index]
        print(f"current menu selection: {key}")
        hub.display.char(key[0])
        recorded = key.endswith("d")

        pressed = ()
        print("waiting for button press")
        blink = StopWatch()
        while not pressed:
            pressed = hub.buttons.pressed()
            if recorded and blink.time() > BLINK_MS:
                # Blink: letter on, then off, then on ...
                if blink.time() > 2 * BLINK_MS:
                    hub.display.char(key[0])
                    blink.reset()
                else:
                    hub.display.off()
            wait(10)

        print(f"button pressed: {pressed}. waiting for release")
        while hub.buttons.pressed():
            wait(10)

        print("buttons released")
        if Button.CENTER in pressed:
            print(f"{key} chosen")
            hub.display.char(key[0])
            hub.system.set_stop_button(Button.CENTER)
            return (key, menu_options[key])

        elif Button.LEFT in pressed:
            menu_index = navigate(-1, menu_index)
        elif Button.RIGHT in pressed:
            menu_index = navigate(1, menu_index)
