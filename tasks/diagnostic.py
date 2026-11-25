# Diagnostic recorder: menu Ad, Bd, Cd, Dd and Ed run Task A, B, C, D or E
# with the recorder (see record_a ... record_e at the bottom).
#
# This runs the task exactly like normal, and at the same time records what
# the gyro, the motors, and both color sensors see. The task is NOT changed.
#
# The goal: collect a "fingerprint" of what a successful run looks like, so
# later we can compare a live run against it and send the robot home early
# when things look wrong.
#
# Every line it prints starts with a tag, so it is easy to pull out later:
#   INFO    - once per run: battery, gyro status, which sensors were found
#   RAW     - a snapshot of every reading, every RAW_EVERY_MS milliseconds
#   SEG     - one line per move (each straight, each turn, and long stops).
#             Step 0 is "START": the drive to the GREEN/WHITE pattern.
#   PICKUP  - printed if the robot tilts a lot (someone picked it up)
#   RESULT  - one line at the end, with the success/fail label you enter
#
# When the task finishes, the hub shows "?":
#   press RIGHT if the run WORKED, press LEFT if it FAILED.
#
# To stop a run early (e.g. the board wasn't reset), press CENTER, or LEFT and
# RIGHT together. The robot stops and goes back to the menu, and logging keeps
# going. To end the whole program, press the BLUETOOTH button.

from pybricks.tools import multitask, wait, StopWatch
from pybricks.parameters import Color, Port, Button, Axis
from pybricks.pupdevices import ColorSensor

from init import hub, robot, lift, pressure, left_motor, right_motor, colorRight
from init import WHEEL_DIAMETER, AXLE_TRACK
from utilities.readings import get, color_name
from tasks.taskA import task as task_a
from tasks.taskB import task as task_b
from tasks.taskC import task as task_c
from tasks.taskD import task as task_d
from tasks.taskE import task as task_e

# The left color sensor is commented out in init.py right now.
# If init.py makes it, use that one. Otherwise try to make it here.
LEFT_SENSOR_PORT = Port.E
try:
    from init import colorLeft
except ImportError:
    try:
        colorLeft = ColorSensor(LEFT_SENSOR_PORT)
    except OSError:
        colorLeft = None  # nothing plugged into that port


# ---------------- Settings you can change ----------------

SAMPLE_MS = 20          # how often we take a reading
RAW_LOG = True          # print RAW lines? Set False if Task A acts differently
RAW_EVERY_MS = 100      # how often to print a RAW line
SPEED_LIMIT = 15        # mm/s - slower than this counts as "not driving"
TURN_LIMIT = 10         # deg/s - slower than this counts as "not turning"
LIFT_LIMIT = 30         # deg/s - lift motor slower than this = "lift not moving"
DEBOUNCE = 3            # readings in a row before we believe a new move started
MIN_STOP_MS = 300       # only report stops longer than this (e.g. lift moves)
LABEL_TIMEOUT_MS = 30000  # how long to wait for the success/fail button
PICKUP_TILT = 10        # degrees - tilting more than this means "picked up"
                        # (normal driving stays under about 3 degrees)


# ---------------- Shared run information ----------------

run = {
    "number": 0,        # counts runs since the program started
    "finished": False,  # set True when Task A is done
    "error": "",        # set if Task A crashed
    "summary": None,    # filled in by the recorder at the end
    "pickup_ms": "",    # when the robot was first picked up (blank = never)
    "pickup_step": "",  # which step it was on when picked up
}


# ---------------- Small helpers ----------------

def whole(number):
    """Round to a whole number so the output is easy to read."""
    return int(round(number))


def print_csv(*values):
    print(",".join(str(v) for v in values))


def print_headers():
    """Column names, using the same tags so they survive filtering."""
    print_csv("INFO", "run", "task", "battery_mV", "gyro_ready", "left_sensor",
              "start_heading")
    print_csv("RAW", "run", "ms", "distance", "speed", "odo_angle",
              "turn_rate", "heading", "pitch", "roll", "lift", "force",
              "R_color", "R_h", "R_s", "R_v", "R_reflect",
              "L_color", "L_h", "L_s", "L_v", "L_reflect", "lift_load")
    print_csv("SEG", "run", "step", "kind", "start_ms", "ms", "distance",
              "odo_turn", "gyro_turn", "slip", "end_heading",
              "max_tilt", "max_bump", "max_load", "lift_change", "max_force",
              "R_end_color", "R_end_reflect", "R_colors_seen",
              "L_end_color", "L_end_reflect", "L_colors_seen",
              "max_lift_load")
    print_csv("RESULT", "run", "task", "label", "total_ms", "moves",
              "end_heading",
              "end_odo_angle", "total_slip", "max_tilt", "max_bump",
              "R_end_color", "L_end_color", "pickup_ms", "pickup_step",
              "error")
    print_csv("PICKUP", "run", "ms", "step")
    # Task Y also prints these lines itself:
    print_csv("PATTERN", "seen_at_mm", "overshoot_mm")
    print_csv("HOOK", "attempt", "max_load", "backed_up_mm", "hooked")
    print_csv("CHECK", "point", "heading", "target", "off_by")
    print_csv("AIM", "target", "heading_before", "heading_after")
    print_csv("DRIVE", "drive", "heading_at_start", "heading_at_end",
              "average_heading", "target", "worst_heading_error", "max_load")
    print_csv("EDGE", "drive", "edge_seen_at_mm", "mm_driven",
              "worst_heading_error", "max_load", "worst_tilt", "bumped")
    print_csv("PRESS", "mm_moved", "ms", "max_load")
    print_csv("BUMP", "drive", "mm_driven", "mm_meant", "worst_twist",
              "max_load", "worst_tilt")
    print_csv("CAREFUL", "drive", "mm_driven", "mm_meant", "worst_twist",
              "max_load", "worst_tilt")
    print_csv("C2_FIX", "change_to_340_drive_mm")
    print_csv("GOTO", "name", "x_now", "y_now", "x_to", "y_to", "heading", "mm")
    print_csv("POS", "name", "x", "y", "heading")
    print_csv("C4FEEL", "push", "max_lift_load", "stop_height", "sensor_pressed",
              "felt_lever")
    print_csv("BACKUP_FIX", "extra_mm_on_300_drive", "extra_mm_on_150_drive",
              "c1_stopped_short_mm")
    print_csv("START", "right_color", "left_color", "ok")
    print_csv("LINES", "drive", "sensor", "mm_into_drive:color ...")


# ---------------- Reading the sensors ----------------

def wheel_angle():
    """How far the robot has turned according to the WHEELS only, in degrees
    (clockwise = positive). We work it out from the two motors ourselves,
    because when the gyro is turned on, robot.angle() uses the gyro instead."""
    difference = left_motor.angle() - right_motor.angle()
    return difference * WHEEL_DIAMETER / (2 * AXLE_TRACK)


async def read_color_sensor(sensor):
    """Returns (color name, hue, saturation, value, reflection)."""
    if sensor is None:
        return ("-", 0, 0, 0, 0)
    hsv = await get(sensor.hsv())
    reflection = await get(sensor.reflection())
    color = await get(sensor.color())
    return (color_name(color), hsv.h, hsv.s, hsv.v, whole(reflection))


async def take_reading(clock, zero):
    """One snapshot of everything. Angles and distance start at 0."""
    time = clock.time()
    distance, speed, _, turn_rate = robot.state()
    odo_angle = wheel_angle()
    pitch, roll = hub.imu.tilt()
    right = await read_color_sensor(colorRight)
    left = await read_color_sensor(colorLeft)
    force = await get(pressure.force())
    left_load = await get(left_motor.load())
    right_load = await get(right_motor.load())
    lift_load = await get(lift.load())
    return {
        "time": time,
        "distance": distance - zero["distance"],
        "speed": speed,
        # odo_angle comes from the wheels, heading comes from the gyro.
        # If they disagree, the wheels slipped or the robot got pushed.
        "odo_angle": odo_angle - zero["odo_angle"],
        "turn_rate": turn_rate,
        "heading": hub.imu.heading() - zero["heading"],
        "pitch": pitch,
        "roll": roll,
        "tilt": max(abs(pitch), abs(roll)),
        # Sideways/forward shake in mm/s^2. Big numbers = we hit something.
        "bump": max(abs(hub.imu.acceleration(Axis.X)),
                    abs(hub.imu.acceleration(Axis.Y))),
        # How hard the drive motors are working. High = pushing something.
        "load": max(abs(left_load), abs(right_load)),
        "lift": lift.angle(),
        "lift_speed": lift.speed(),
        # How hard the lift motor is working. High = lifting something heavy
        # or pressing against something.
        "lift_load": abs(lift_load),
        "force": force,
        "right": right,
        "left": left,
    }


def print_raw(reading):
    r = reading["right"]
    l = reading["left"]
    print_csv("RAW", run["number"], reading["time"],
              whole(reading["distance"]), whole(reading["speed"]),
              whole(reading["odo_angle"]), whole(reading["turn_rate"]),
              whole(reading["heading"]), reading["pitch"], reading["roll"],
              reading["lift"], whole(reading["force"] * 10) / 10,
              r[0], r[1], r[2], r[3], r[4],
              l[0], l[1], l[2], l[3], l[4],
              whole(reading["lift_load"]))


# ---------------- Splitting the run into moves ----------------

def classify(speed, turn_rate, lift_speed):
    """What is the robot doing right now?"""
    if speed > SPEED_LIMIT:
        return "FWD"
    if speed < -SPEED_LIMIT:
        return "BACK"
    if turn_rate > TURN_LIMIT:
        return "RIGHT"
    if turn_rate < -TURN_LIMIT:
        return "LEFT"
    # Standing still, but the lift is moving. lift_to_height() uses negative
    # motor angles for higher heights, so a negative speed means going UP.
    if lift_speed < -LIFT_LIMIT:
        return "LIFT_UP"
    if lift_speed > LIFT_LIMIT:
        return "LIFT_DOWN"
    return "STOP"


def remember_color(seen, name):
    """Keep a list of color changes, like GREEN>WHITE>BLACK."""
    if name == "-":
        return
    if len(seen) == 0 or seen[-1] != name:
        if len(seen) < 10:
            seen.append(name)


class Segment:
    """Collects the numbers for one move (a straight, a turn, or a stop)."""

    def __init__(self, kind, step, first_reading):
        self.kind = kind
        self.step = step
        self.first = first_reading
        self.last = first_reading
        self.max_tilt = 0
        self.max_bump = 0
        self.max_load = 0
        self.max_force = 0
        self.max_lift_load = 0
        self.right_seen = []
        self.left_seen = []

    def add(self, reading):
        self.last = reading
        self.max_tilt = max(self.max_tilt, reading["tilt"])
        self.max_bump = max(self.max_bump, reading["bump"])
        self.max_load = max(self.max_load, reading["load"])
        self.max_force = max(self.max_force, reading["force"])
        self.max_lift_load = max(self.max_lift_load, reading["lift_load"])
        remember_color(self.right_seen, reading["right"][0])
        remember_color(self.left_seen, reading["left"][0])

    def duration(self):
        return self.last["time"] - self.first["time"]

    def report(self):
        # Skip short stops - they are just the pause between two moves.
        if self.kind == "STOP" and self.duration() < MIN_STOP_MS:
            return
        a = self.first
        b = self.last
        odo_turn = b["odo_angle"] - a["odo_angle"]
        gyro_turn = b["heading"] - a["heading"]
        print_csv("SEG", run["number"], self.step, self.kind,
                  a["time"], self.duration(),
                  whole(b["distance"] - a["distance"]),
                  whole(odo_turn), whole(gyro_turn),
                  whole(gyro_turn - odo_turn), whole(b["heading"]),
                  self.max_tilt, whole(self.max_bump), whole(self.max_load),
                  b["lift"] - a["lift"], whole(self.max_force * 10) / 10,
                  b["right"][0], b["right"][4], ">".join(self.right_seen),
                  b["left"][0], b["left"][4], ">".join(self.left_seen),
                  whole(self.max_lift_load))


async def recorder(clock, zero, start_reading):
    """Takes readings until Task A is done, and prints one SEG per move."""
    # Task A's pattern search (find_pattern) keeps the hub so busy that the
    # recorder can't take readings while it runs. So step 0 is called START
    # and covers everything from the start reading until the first move we
    # can actually see - usually the whole drive to the GREEN/WHITE pattern.
    reading = start_reading
    step = 0                 # counts moves (stops don't count)
    segment = Segment("START", step, reading)
    segment.add(reading)
    pending = []             # readings that might be the start of a new move
    pending_kind = None
    last_raw_time = -RAW_EVERY_MS
    max_tilt = 0
    max_bump = 0

    while not run["finished"]:
        reading = await take_reading(clock, zero)
        max_tilt = max(max_tilt, reading["tilt"])
        max_bump = max(max_bump, reading["bump"])

        # Did someone pick the robot up? Everything after this point is not
        # a real run, so we note when it happened.
        if reading["tilt"] > PICKUP_TILT and run["pickup_ms"] == "":
            run["pickup_ms"] = reading["time"]
            run["pickup_step"] = step
            print_csv("PICKUP", run["number"], reading["time"], step)

        kind = classify(reading["speed"], reading["turn_rate"],
                        reading["lift_speed"])

        if kind == segment.kind:
            # Still doing the same move. Any "maybe new move" readings
            # were just noise, so they belong to this move.
            for r in pending:
                segment.add(r)
            pending = []
            segment.add(reading)
        else:
            if kind != pending_kind:
                for r in pending:
                    segment.add(r)
                pending = []
                pending_kind = kind
            pending.append(reading)

            if len(pending) >= DEBOUNCE:
                # It's a real new move. Finish the old one and start fresh.
                segment.report()
                if kind != "STOP":
                    step += 1
                segment = Segment(kind, step, segment.last)
                for r in pending:
                    segment.add(r)
                pending = []
                pending_kind = None

        if RAW_LOG and reading["time"] - last_raw_time >= RAW_EVERY_MS:
            print_raw(reading)
            last_raw_time = reading["time"]

        await wait(SAMPLE_MS)

    for r in pending:
        segment.add(r)
    segment.report()

    final = segment.last
    run["summary"] = {
        "moves": step,
        "final": final,
        "max_tilt": max_tilt,
        "max_bump": max_bump,
    }


# ---------------- Running the task ----------------

async def watch_for_abort():
    """Press CENTER (or LEFT and RIGHT together) to stop a run early. The run
    stops and goes back to the menu, so the computer keeps logging."""
    while True:
        pressed = hub.buttons.pressed()
        if Button.CENTER in pressed:
            return True
        if Button.LEFT in pressed and Button.RIGHT in pressed:
            return True
        await wait(20)


async def run_the_task(task_function):
    try:
        # race=True: whichever finishes first (the task, or someone pressing
        # LEFT+RIGHT) stops the other one.
        results = await multitask(task_function(), watch_for_abort(),
                                  race=True)
        if results[1] is True:
            run["error"] = "aborted"
            print("Run stopped with a button - back to the menu")
            robot.stop()
            lift.stop()
            robot.use_gyro(False)
            while hub.buttons.pressed():  # wait for the buttons to be let go
                await wait(10)
    except Exception as e:
        # Keep the diagnostic going so we still get the RESULT line.
        run["error"] = repr(e).replace(",", ";")
        print("Task crashed:", e)
        robot.stop()
    run["finished"] = True


async def ask_for_label():
    """After the run: RIGHT button = success, LEFT button = fail."""
    hub.display.char("?")
    hub.light.on(Color.BLUE)
    await hub.speaker.beep(frequency=900, duration=150)
    print("Press RIGHT if the run worked, LEFT if it failed")

    label = "UNLABELED"
    timer = StopWatch()
    while timer.time() < LABEL_TIMEOUT_MS:
        pressed = hub.buttons.pressed()
        if Button.RIGHT in pressed:
            label = "SUCCESS"
            break
        if Button.LEFT in pressed:
            label = "FAIL"
            break
        await wait(10)

    # Wait for the button to be let go, so the menu doesn't see it.
    while hub.buttons.pressed():
        await wait(10)

    if label == "SUCCESS":
        hub.display.char("Y")
        await hub.speaker.beep(frequency=1200, duration=300)
    elif label == "FAIL":
        hub.display.char("N")
        await hub.speaker.beep(frequency=300, duration=300)
    return label


async def record_a():
    """Menu Ad: Task A with the recorder."""
    await diagnose(task_a, "A")


async def record_b():
    """Menu Bd: Task B with the recorder."""
    await diagnose(task_b, "B")


async def record_c():
    """Menu Cd: Task C with the recorder."""
    await diagnose(task_c, "C")


async def record_d():
    """Menu Dd: Task D with the recorder."""
    await diagnose(task_d, "D")


async def record_e():
    """Menu Ed: Task E with the recorder."""
    await diagnose(task_e, "E")


async def diagnose(task_function, task_name):
    run["number"] += 1
    run["finished"] = False
    run["error"] = ""
    run["summary"] = None
    run["pickup_ms"] = ""
    run["pickup_step"] = ""

    if run["number"] == 1:
        print_headers()

    # CENTER now stops just this run (see watch_for_abort), so it must not
    # end the whole program - that stops the logging too. The Bluetooth
    # button becomes the emergency stop instead.
    try:
        hub.system.set_stop_button(Button.BLUETOOTH)
    except Exception as e:
        print("could not move the stop button:", e)

    gyro_ready = hub.imu.ready()
    if not gyro_ready:
        print("WARNING: gyro not ready. Keep the robot still for a few"
              " seconds before starting.")

    # Everything is measured from where the robot is right now.
    zero = {
        "distance": robot.distance(),
        "odo_angle": wheel_angle(),
        "heading": hub.imu.heading(),
    }
    print_csv("INFO", run["number"], task_name, hub.battery.voltage(),
              gyro_ready, colorLeft is not None, whole(zero["heading"]))

    clock = StopWatch()
    # Take the first reading BEFORE the task starts moving.
    start_reading = await take_reading(clock, zero)
    await multitask(run_the_task(task_function),
                    recorder(clock, zero, start_reading))

    if run["error"] == "aborted":
        label = "ABORTED"  # no need to ask - we know it didn't finish
    else:
        label = await ask_for_label()

    s = run["summary"]
    f = s["final"]
    print_csv("RESULT", run["number"], task_name, label, f["time"], s["moves"],
              whole(f["heading"]), whole(f["odo_angle"]),
              whole(f["heading"] - f["odo_angle"]),
              s["max_tilt"], whole(s["max_bump"]),
              f["right"][0], f["left"][0],
              run["pickup_ms"], run["pickup_step"], run["error"])
