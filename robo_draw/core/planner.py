"""Plan rysowania: ścieżki SVG → koło, łuk, odcinek albo punkt, potem kolejność ruchów."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import svgpathtools

from .utils import Point, simplify_points

JOIN_MM = 3.0
POINT_MM = 1.2
FIT_MM = 0.45
LIFT_PENALTY_MM = 40.0
MESH_MIN_TRIANGLES = 12
MESH_FRACTION = 0.8


@dataclass
class Stroke:
    kind: str
    start: complex
    end: complex
    via: complex | None = None
    points: tuple[complex, ...] = ()
    center: complex | None = None
    radius: float = 0.0
    clockwise: bool = False

    def oriented(self, reverse: bool) -> Stroke:
        if not reverse or self.kind in ("point", "circle"):
            return self
        if self.kind == "polyline":
            pts = tuple(reversed(self.points))
            return Stroke("polyline", pts[0], pts[-1], points=pts)
        if self.kind == "arc":
            return Stroke("arc", self.end, self.start, via=self.via, center=self.center, radius=self.radius)
        return Stroke("line", self.end, self.start)


@dataclass
class Motion:
    """Ruch w jednostkach SVG. travel i retract trzymają pisak w powietrzu."""

    kind: str
    target: complex
    via: complex | None = None
    pen_down: bool = False


@dataclass
class Plan:
    strokes: list[Stroke] = field(default_factory=list)
    motions: list[Motion] = field(default_factory=list)

    def counts(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for stroke in self.strokes:
            counts[stroke.kind] = counts.get(stroke.kind, 0) + 1
        return counts


def plan_drawing(
    paths: list[svgpathtools.Path],
    scale: float = 1.0,
    resolution: float = 1.0,
    join_mm: float = JOIN_MM,
) -> Plan:
    """Zbuduj kolejność kresek i ruchy pisaka."""
    scale = scale if scale else 1.0
    paths = _drop_internal_mesh_edges(paths)
    strokes = [_classify_path(path, scale, resolution) for path in paths]
    strokes = [stroke for stroke in strokes if stroke is not None]
    strokes = _order_strokes(strokes, scale, join_mm)
    return Plan(strokes=strokes, motions=_motions_from_strokes(strokes, scale, join_mm))


def _drop_internal_mesh_edges(paths: list[svgpathtools.Path]) -> list[svgpathtools.Path]:
    """Z siatki trójkątów zostaw krawędzie brzegu i zepnij je w łańcuchy."""
    triangles = []
    others = []
    for path in paths:
        if _is_triangle(path):
            triangles.append(path)
        else:
            others.append(path)

    if len(triangles) < MESH_MIN_TRIANGLES or len(triangles) < MESH_FRACTION * max(len(paths), 1):
        return paths

    counts: dict[tuple[complex, complex], int] = {}
    endpoints: dict[tuple[complex, complex], tuple[complex, complex]] = {}
    for path in triangles:
        corners = [path[i].start for i in range(3)]
        for start, end in zip(corners, corners[1:] + corners[:1]):
            key = _edge_key(start, end)
            counts[key] = counts.get(key, 0) + 1
            endpoints[key] = (start, end)

    boundary = [endpoints[key] for key, count in counts.items() if count == 1]
    chains = _chain_segments(boundary, tol=0.05)
    chained = []
    for chain in chains:
        if len(chain) < 2:
            continue
        path = svgpathtools.Path(*[svgpathtools.Line(a, b) for a, b in zip(chain, chain[1:])])
        chained.append(path)
    return others + chained


def _is_triangle(path: svgpathtools.Path) -> bool:
    if len(path) != 3:
        return False
    if not all(isinstance(seg, svgpathtools.Line) for seg in path):
        return False
    return abs(path[0].start - path[-1].end) < 1e-3


def _edge_key(start: complex, end: complex, quantum: float = 0.01) -> tuple[complex, complex]:
    a = complex(round(start.real / quantum) * quantum, round(start.imag / quantum) * quantum)
    b = complex(round(end.real / quantum) * quantum, round(end.imag / quantum) * quantum)
    return (a, b) if (a.real, a.imag) <= (b.real, b.imag) else (b, a)


def _chain_segments(segments: list[tuple[complex, complex]], tol: float) -> list[list[complex]]:
    unused = set(range(len(segments)))
    buckets: dict[tuple[int, int], list[int]] = {}

    def bucket(point: complex) -> tuple[int, int]:
        return (round(point.real / tol), round(point.imag / tol))

    for index, (start, end) in enumerate(segments):
        buckets.setdefault(bucket(start), []).append(index)
        buckets.setdefault(bucket(end), []).append(index)

    def near(a: complex, b: complex) -> bool:
        return abs(a - b) <= tol

    chains = []
    while unused:
        index = unused.pop()
        start, end = segments[index]
        chain = [start, end]
        for forward in (True, False):
            while True:
                tip = chain[-1] if forward else chain[0]
                found = None
                for candidate in buckets.get(bucket(tip), []):
                    if candidate not in unused:
                        continue
                    a, b = segments[candidate]
                    if near(a, tip):
                        nxt = b
                    elif near(b, tip):
                        nxt = a
                    else:
                        continue
                    found = (candidate, nxt)
                    break
                if found is None:
                    break
                unused.remove(found[0])
                if forward:
                    chain.append(found[1])
                else:
                    chain.insert(0, found[1])
        chains.append(chain)
    return chains


def _classify_path(path: svgpathtools.Path, scale: float, resolution: float) -> Stroke | None:
    samples = _samples(path, scale, resolution)
    if len(samples) < 2:
        return None

    length_mm = _length_mm(samples, scale)
    if length_mm <= POINT_MM:
        center = sum(samples) / len(samples)
        return Stroke("point", center, center)

    circle = _fit_circle(samples)
    fit_svg = FIT_MM / scale
    if circle is not None:
        center, radius, deviation = circle
        if deviation <= fit_svg and radius * scale > POINT_MM / 2:
            start = _project(center, radius, samples[0])
            end = _project(center, radius, samples[-1])
            clockwise = _is_clockwise(samples)
            closed = abs(samples[0] - samples[-1]) <= fit_svg or length_mm >= 0.85 * 2 * math.pi * radius * scale
            if closed:
                return Stroke("circle", start, start, center=center, radius=radius, clockwise=clockwise)
            via = _project(center, radius, samples[len(samples) // 2])
            return Stroke("arc", start, end, via=via, center=center, radius=radius, clockwise=clockwise)

    if _max_line_deviation(samples) <= fit_svg:
        return Stroke("line", samples[0], samples[-1])

    simplified = simplify_points([Point(p.real, p.imag) for p in samples], fit_svg)
    points = tuple(complex(p.x, p.y) for p in simplified)
    if len(points) < 2:
        return None
    if len(points) == 2:
        return Stroke("line", points[0], points[1])
    return Stroke("polyline", points[0], points[-1], points=points)


def _samples(path: svgpathtools.Path, scale: float, resolution: float) -> list[complex]:
    try:
        length = path.length()
    except Exception:
        return []
    if not length:
        return []
    step = max(resolution, 0.2) / (scale if scale else 1.0)
    count = max(4, min(64, int(length / step) + 1))
    return [path.point(i / (count - 1)) for i in range(count)]


def _length_mm(samples: list[complex], scale: float) -> float:
    return sum(abs(b - a) for a, b in zip(samples, samples[1:])) * scale


def _fit_circle(samples: list[complex]) -> tuple[complex, float, float] | None:
    if len(samples) < 3:
        return None
    a, b, c = samples[0], samples[len(samples) // 2], samples[-1]
    if abs(a - c) < 1e-6:
        a, b, c = samples[0], samples[len(samples) // 3], samples[(2 * len(samples)) // 3]
    fitted = _circle_from_three(a, b, c)
    if fitted is None:
        return None
    center, radius = fitted
    if radius < 1e-6:
        return None
    deviation = max(abs(abs(point - center) - radius) for point in samples)
    return center, radius, deviation


def _circle_from_three(p1: complex, p2: complex, p3: complex) -> tuple[complex, float] | None:
    x1, y1 = p1.real, p1.imag
    x2, y2 = p2.real, p2.imag
    x3, y3 = p3.real, p3.imag
    det = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)
    if abs(det) < 1e-9:
        return None
    b1 = 0.5 * ((x2 * x2 + y2 * y2) - (x1 * x1 + y1 * y1))
    b2 = 0.5 * ((x3 * x3 + y3 * y3) - (x1 * x1 + y1 * y1))
    cx = ((y3 - y1) * b1 - (y2 - y1) * b2) / det
    cy = ((x2 - x1) * b2 - (x3 - x1) * b1) / det
    center = complex(cx, cy)
    return center, abs(center - p1)


def _project(center: complex, radius: float, point: complex) -> complex:
    offset = point - center
    magnitude = abs(offset)
    if magnitude < 1e-9:
        return center + radius
    return center + offset / magnitude * radius


def _is_clockwise(samples: list[complex]) -> bool:
    area = 0.0
    for a, b in zip(samples, samples[1:]):
        area += a.real * b.imag - b.real * a.imag
    return area < 0


def _max_line_deviation(samples: list[complex]) -> float:
    start, end = samples[0], samples[-1]
    span = end - start
    length = abs(span)
    if length < 1e-9:
        return max(abs(point - start) for point in samples)
    return max(abs(((point - start).conjugate() * span).imag) / length for point in samples)


def _order_strokes(strokes: list[Stroke], scale: float, join_mm: float) -> list[Stroke]:
    if len(strokes) < 2:
        return strokes

    remaining = set(range(len(strokes)))
    current = 0
    remaining.remove(current)
    order: list[tuple[int, bool]] = [(current, False)]
    cursor = strokes[current].end

    while remaining:
        best = None
        best_cost = float("inf")
        for index in remaining:
            stroke = strokes[index]
            for reverse, start, end in (
                (False, stroke.start, stroke.end),
                (True, stroke.end, stroke.start),
            ):
                cost = _link_cost(cursor, start, scale, join_mm)
                if cost < best_cost:
                    best_cost = cost
                    best = (index, reverse, end)
        index, reverse, end = best
        remaining.remove(index)
        order.append((index, reverse))
        cursor = end

    if len(order) <= 600:
        order = _two_opt(order, strokes, scale, join_mm)

    return [strokes[index].oriented(reverse) for index, reverse in order]


def _link_cost(a: complex, b: complex, scale: float, join_mm: float) -> float:
    distance = abs(b - a) * scale
    if distance <= join_mm:
        return distance
    return distance + LIFT_PENALTY_MM


def _route_cost(order: list[tuple[int, bool]], strokes: list[Stroke], scale: float, join_mm: float) -> float:
    total = 0.0
    for (index_a, rev_a), (index_b, rev_b) in zip(order, order[1:]):
        end = strokes[index_a].start if rev_a else strokes[index_a].end
        start = strokes[index_b].end if rev_b else strokes[index_b].start
        total += _link_cost(end, start, scale, join_mm)
    return total


def _two_opt(
    order: list[tuple[int, bool]], strokes: list[Stroke], scale: float, join_mm: float
) -> list[tuple[int, bool]]:
    window = 48 if len(order) > 250 else len(order)
    best_cost = _route_cost(order, strokes, scale, join_mm)
    for _ in range(8):
        improved = False
        for i in range(len(order) - 1):
            last = min(len(order), i + window)
            for j in range(i + 1, last):
                flipped = [(index, not rev) for index, rev in reversed(order[i : j + 1])]
                candidate = order[:i] + flipped + order[j + 1 :]
                cost = _route_cost(candidate, strokes, scale, join_mm)
                if cost + 1e-6 < best_cost:
                    order = candidate
                    best_cost = cost
                    improved = True
                    break
            if improved:
                break
        if not improved:
            break
    return order


def _motions_from_strokes(strokes: list[Stroke], scale: float, join_mm: float) -> list[Motion]:
    motions: list[Motion] = []
    cursor: complex | None = None
    for stroke in strokes:
        if cursor is None or abs(stroke.start - cursor) * scale > join_mm:
            motions.append(Motion("travel", stroke.start, pen_down=False))
        motions.extend(_draw_motions(stroke))
        cursor = stroke.end
    if cursor is not None:
        motions.append(Motion("retract", cursor, pen_down=False))
    return motions


def _draw_motions(stroke: Stroke) -> list[Motion]:
    if stroke.kind == "point":
        return []
    if stroke.kind == "line":
        return [Motion("line", stroke.end, pen_down=True)]
    if stroke.kind == "polyline":
        return [Motion("line", point, pen_down=True) for point in stroke.points[1:]]
    if stroke.kind == "arc" and stroke.via is not None:
        return [Motion("arc", stroke.end, via=stroke.via, pen_down=True)]
    if stroke.kind == "circle" and stroke.center is not None:
        return _circle_motions(stroke)
    return []


def _circle_motions(stroke: Stroke) -> list[Motion]:
    center = stroke.center
    radius = stroke.radius
    assert center is not None
    start = _project(center, radius, stroke.start)
    half_turn = -math.pi if stroke.clockwise else math.pi
    base = math.atan2(start.imag - center.imag, start.real - center.real)

    def at(delta: float) -> complex:
        angle = base + delta
        return complex(center.real + radius * math.cos(angle), center.imag + radius * math.sin(angle))

    opposite = at(half_turn)
    return [
        Motion("arc", opposite, via=at(half_turn / 2), pen_down=True),
        Motion("arc", start, via=at(half_turn + half_turn / 2), pen_down=True),
    ]
