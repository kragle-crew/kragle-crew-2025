from pybricks.parameters import Color


async def get(result):
    """Inside a task, a sensor sometimes hands back the answer right away,
    and sometimes hands back a "wait" object that we must await first.
    This handles both, so we always end up with the real value.

    Use it like:  force = await get(pressure.force())"""
    if result is None or isinstance(result, (int, float, Color)):
        return result  # already a real value
    return await result


COLOR_NAMES = [
    (Color.NONE, "NONE"),
    (Color.BLACK, "BLACK"),
    (Color.WHITE, "WHITE"),
    (Color.RED, "RED"),
    (Color.ORANGE, "ORANGE"),
    (Color.YELLOW, "YELLOW"),
    (Color.GREEN, "GREEN"),
    (Color.CYAN, "CYAN"),
    (Color.BLUE, "BLUE"),
    (Color.VIOLET, "VIOLET"),
    (Color.MAGENTA, "MAGENTA"),
]


def color_name(color):
    """Turns a Color into a short name like "GREEN" for printing."""
    if color is None:
        return "NONE"
    for known, name in COLOR_NAMES:
        if color == known:
            return name
    return "OTHER"
