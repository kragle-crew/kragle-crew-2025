from init import lift, pressure
from pybricks.tools import multitask, wait, StopWatch

lift_speed=600

# True once reset_lift() has run since the program started. Before that the
# lift motor doesn't know where the bottom is, so lift_to_height() would go
# to the wrong place (and can jam the lift).
lift_was_reset = False

def lift_height_is_known():
    """True if the lift has been reset since the program started, so
    lift_to_height() goes to the right place."""
    return lift_was_reset

async def reset_lift():
    global lift_was_reset
    lift.run(lift_speed)
    while not await pressure.pressed(force=.5):
      pass
    print("button pressed. stopping lift")
    lift.stop()
    print("moving to zero")
    await lift.run_angle(-lift_speed, 90)
    print("lift at zero")
    lift.reset_angle(0)
    lift_was_reset = True

async def lift_to_height(height_to_go_to,robotLiftSpeed=lift_speed):
    print("lifting to height:", height_to_go_to)
    await lift.run_target(robotLiftSpeed, -height_to_go_to)


async def lower_to_bottom(force=3, speed=300, timeout_ms=3000):
    """Lowers the lift until it presses the force sensor at the bottom with
    `force` newtons, then HOLDS it there - as low as the arm goes.

    reset_lift() does the same at first, but then lifts the arm back up 90
    degrees. Use this when the arm must stay right down (for example to get
    under a lever). It gives up after timeout_ms so it can never get stuck."""
    timer = StopWatch()
    lift.run(speed)   # positive = down, toward the force sensor
    while not await pressure.pressed(force=force):
        if timer.time() > timeout_ms:
            print("lift never pressed the sensor - holding where it is")
            break
    lift.hold()
    print("lift at the bottom")


async def reset_lift_safely(timeout_ms=5000):
    """reset_lift(), but it gives up after timeout_ms. reset_lift() waits for
    the force sensor - if that never gets pressed, it would wait forever."""
    finished = {"ok": False}

    async def do_reset():
        await reset_lift()
        finished["ok"] = True

    # race=True: whichever finishes first (the reset or the timer) wins.
    await multitask(do_reset(), wait(timeout_ms), race=True)
    if not finished["ok"]:
        lift.stop()
        print("lift reset took too long - stopped it")
