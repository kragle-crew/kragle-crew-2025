from init import hub
from tasks.taskA import task as a
from tasks.taskB import task as b
from tasks.taskC import task as c
from tasks.taskD import task as d
from tasks.taskE import task as e

from pybricks.tools import wait
from pybricks.parameters import Button

menu_options = {"A": a, "B": b, "C": c, "D": d, "E": e}
menu_option_keys = list(menu_options.keys())


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
        print(f"current menu selection: {menu_option_keys[menu_index]}")
        hub.display.char(menu_option_keys[menu_index])

        pressed = ()
        print("waiting for button press")
        while not pressed:
            pressed = hub.buttons.pressed()
            wait(10)

        print(f"button pressed: {pressed}. waiting for release")
        while hub.buttons.pressed():
            wait(10)

        print("buttons released")
        if Button.CENTER in pressed:
            print(f"{menu_option_keys[menu_index]} chosen")
            hub.system.set_stop_button(Button.CENTER)
            return (
                menu_option_keys[menu_index],
                menu_options[menu_option_keys[menu_index]],
            )

        elif Button.LEFT in pressed:
            menu_index = navigate(-1, menu_index)
        elif Button.RIGHT in pressed:
            menu_index = navigate(1, menu_index)
