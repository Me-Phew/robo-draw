import svgpathtools


class Point:
    """Helper class to store 2D coordinates"""
    def __init__(self, x, y):
        self.x = x
        self.y = y


# ------------------------------------------------------------
# 1. Discretization (your code)
# ------------------------------------------------------------
def get_points_from_path(path: svgpathtools.Path, step_mm: float = 5.0) -> list[Point]:
    """Discretizes a Bezier curve (SVG Path) into a list of Points."""
    points = []
    try:
        path_len = path.length()
    except:
        return []

    if not path_len or path_len < 0.5:
        return []

    num_steps = max(1, int(path_len / step_mm))

    for i in range(num_steps + 1):
        t = i / num_steps
        c_point = path.point(t)
        points.append(Point(c_point.real, c_point.imag))

    return points


# ------------------------------------------------------------
# 2. Noise removal
# ------------------------------------------------------------
def clean_noise(points: list[Point], min_dist: float = 0.05) -> list[Point]:
    """Remove points that are too close together (noise / oversampling)."""
    if not points:
        return []

    cleaned = [points[0]]
    min_dist_sq = min_dist * min_dist

    for p in points[1:]:
        dx = p.x - cleaned[-1].x
        dy = p.y - cleaned[-1].y
        if dx * dx + dy * dy >= min_dist_sq:
            cleaned.append(p)

    return cleaned


# ------------------------------------------------------------
# 3. Ramer–Douglas–Peucker (your function)
# ------------------------------------------------------------
def perpendicular_distance(point: Point, start: Point, end: Point) -> float:
    if start.x == end.x and start.y == end.y:
        return ((point.x - start.x)**2 + (point.y - start.y)**2)**0.5

    num = abs(
        (end.y - start.y) * point.x -
        (end.x - start.x) * point.y +
        end.x * start.y -
        end.y * start.x
    )
    den = ((end.y - start.y)**2 + (end.x - start.x)**2)**0.5
    return num / den


def simplify_points(points: list[Point], epsilon: float) -> list[Point]:
    """Simplifies a list of points using the Ramer-Douglas-Peucker algorithm."""
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


# ------------------------------------------------------------
# 4. Visvalingam–Whyatt
# ------------------------------------------------------------
def triangle_area(a: Point, b: Point, c: Point) -> float:
    """2x area of triangle (absolute value)."""
    return abs((a.x - c.x) * (b.y - a.y) - (a.x - b.x) * (c.y - a.y)) / 2.0


def visvalingam_simplify(points: list[Point], area_threshold: float) -> list[Point]:
    """Simplify using Visvalingam–Whyatt algorithm."""
    if len(points) <= 2:
        return points

    # Compute area for each triplet
    areas = []
    for i in range(1, len(points) - 1):
        area = triangle_area(points[i - 1], points[i], points[i + 1])
        areas.append((area, i))

    # Filter points based on area threshold
    keep = {0, len(points) - 1}
    for area, i in areas:
        if area > area_threshold:
            keep.add(i)

    keep = sorted(list(keep))
    return [points[i] for i in keep]


# ------------------------------------------------------------
# 5. PIPELINE (final function you call)
# ------------------------------------------------------------
def process_path(path, step_mm=1.0, epsilon=0.2, min_noise=0.05, area=0.1):
    points = get_points_from_path(path, step_mm)
    points = clean_noise(points, min_dist=min_noise)
    points = simplify_points(points, epsilon=epsilon)
    points = visvalingam_simplify(points, area_threshold=area)
    return points
