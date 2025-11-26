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

    if path_len < 0.5:
        return []

    num_steps = int(path_len / step_mm)
    if num_steps == 0:
        num_steps = 1

    for i in range(num_steps + 1):
        t = i / num_steps
        c_point = path.point(t)
        points.append(Point(c_point.real, c_point.imag))

    return points


def perpendicular_distance(point: Point, start: Point, end: Point) -> float:
    if start.x == end.x and start.y == end.y:
        return ((point.x - start.x) ** 2 + (point.y - start.y) ** 2) ** 0.5

    num = abs((end.y - start.y) * point.x - (end.x - start.x) * point.y + end.x * start.y - end.y * start.x)
    den = ((end.y - start.y) ** 2 + (end.x - start.x) ** 2) ** 0.5
    return num / den


def simplify_points(points: list[Point], epsilon: float) -> list[Point]:
    """
    Simplifies a list of points using the Ramer-Douglas-Peucker algorithm.
    """
    if len(points) < 3:
        return points

    dmax = 0.0
    index = 0
    end = len(points) - 1

    for i in range(1, end):
        d = perpendicular_distance(points[i], points[0], points[end])
        if d > dmax:
            index = i
            dmax = d

    if dmax > epsilon:
        rec_results1 = simplify_points(points[: index + 1], epsilon)
        rec_results2 = simplify_points(points[index:], epsilon)
        return rec_results1[:-1] + rec_results2
    else:
        return [points[0], points[end]]
