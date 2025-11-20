import svgpathtools


class Point:
    """Helper class to store 2D coordinates"""

    def __init__(self, x, y):
        self.x = x
        self.y = y


def get_points_from_path(path: svgpathtools.Path, step_mm: float = 5.0) -> list[Point]:
    """
    Discretizes a Bezier curve (SVG Path) into a list of Points
    based on a specific resolution (step_mm).
    """
    points = []
    try:
        path_len = path.length()
    except:
        return []

    if not path_len:
        return []

    if path_len < 0.1:
        return []

    num_steps = int(path_len / step_mm)
    if num_steps == 0:
        num_steps = 1

    for i in range(num_steps + 1):
        t = i / num_steps
        c_point = path.point(t)
        points.append(Point(c_point.real, c_point.imag))

    return points
