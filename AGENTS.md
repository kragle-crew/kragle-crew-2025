# Agent notes

## Context

This is the robot code for **Kragle Crew**, a FIRST LEGO League team (2025 season).
The code is written and changed by team members, often at practice, so expect quick
experimental commits and several people pushing to `main`. Always pull/rebase before
pushing, and keep changes easy for students to read.

## Platform

- Runs on a LEGO SPIKE Prime hub flashed with [Pybricks](https://pybricks.com/) firmware.
- The code is **MicroPython**, not desktop CPython. Use only what Pybricks provides
  (`pybricks.*` modules and a subset of the standard library).
- Load onto the hub with `pybricksdev run ble <file>.py` (see `readme.md`).
- Nothing here can be tested off the robot; say so rather than claiming a change works.

## Layout

- `startup.py` — entry point; loops the menu and runs the chosen task.
- `menu.py` — hub-button menu that picks a task by letter.
- `init.py` — hardware setup (hub, drive base `robot`, motors, sensors). Check which
  sensors are actually enabled here before importing them (e.g. `colorLeft` may be
  commented out).
- `tasks/taskA.py` … `taskE.py` — one mission run each, as `async def task()`.
- `utilities/` — shared helpers such as `lift.py` (`reset_lift`, `lift_to_height`).
