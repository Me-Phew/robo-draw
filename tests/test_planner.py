import math

import svgpathtools

from robo_draw.core.planner import plan_drawing


def _circle_path(radius: float, center: complex = 0j) -> svgpathtools.Path:
    """Pełne koło z czterech łuków, tak jak typowa emotka."""
    arcs = []
    for i in range(4):
        a0 = i * math.pi / 2
        a1 = (i + 1) * math.pi / 2
        start = center + complex(radius * math.cos(a0), radius * math.sin(a0))
        end = center + complex(radius * math.cos(a1), radius * math.sin(a1))
        arcs.append(svgpathtools.Arc(start, radius + radius * 1j, 0, False, True, end))
    return svgpathtools.Path(*arcs)


def _smile() -> svgpathtools.Path:
    return svgpathtools.Path(svgpathtools.Arc(complex(-10, 0), 10 + 10j, 0, False, True, complex(10, 0)))


def test_emoji_primitives_and_order():
    eye = _circle_path(4, complex(-12, 8))
    other = _circle_path(4, complex(12, 8))
    face = _circle_path(30)
    smile = _smile()
    speck = svgpathtools.Path(svgpathtools.Line(0j, 0.2 + 0.1j))
    plan = plan_drawing([face, eye, smile, speck, other], scale=1.0, resolution=1.0)

    kinds = [stroke.kind for stroke in plan.strokes]
    assert kinds.count("circle") == 3
    assert "arc" in kinds
    assert "point" in kinds

    circle_moves = [motion for motion in plan.motions if motion.kind == "arc"]
    assert len(circle_moves) >= 6

    travels = [motion for motion in plan.motions if motion.kind == "travel"]
    assert travels
    assert plan.motions[-1].kind == "retract"


def test_mesh_keeps_boundary_only():
    triangles = []
    for i in range(6):
        for j in range(4):
            x, y = float(i * 10), float(j * 10)
            a, b, c, d = complex(x, y), complex(x + 10, y), complex(x + 10, y + 10), complex(x, y + 10)
            triangles.append(svgpathtools.Path(svgpathtools.Line(a, b), svgpathtools.Line(b, c), svgpathtools.Line(c, a)))
            triangles.append(svgpathtools.Path(svgpathtools.Line(a, c), svgpathtools.Line(c, d), svgpathtools.Line(d, a)))

    plan = plan_drawing(triangles, scale=1.0, resolution=1.0)
    assert len(plan.strokes) < 20
    assert all(stroke.kind in ("line", "polyline") for stroke in plan.strokes)
    drawn = sum(1 for motion in plan.motions if motion.pen_down)
    assert drawn < 40


def test_sheet_frame_points_up():
    from robo_draw.core.sheet_setup import frame_matrix

    flat = frame_matrix((0, 0, 10), (100, 0, 10), (0, 80, 10))
    assert abs(flat[0][3]) < 1e-6
    assert abs(flat[2][3] - 10) < 1e-6
    assert flat[0][0] > 0.99
    assert flat[2][2] > 0.99

    mirrored = frame_matrix((0, 0, 0), (50, 0, 0), (0, -40, 0))
    assert mirrored[2][2] > 0.99


def test_sheet_size_and_fit():
    from robo_draw.core.sheet_setup import fit_paths, sheet_bounds, sheet_extents

    width, height, sign = sheet_extents((0, 0, 10), (200, 0, 10), (0, 100, 10))
    assert abs(width - 200) < 1e-6
    assert abs(height - 100) < 1e-6
    assert sign == 1.0

    width, height, sign = sheet_extents((0, 0, 0), (50, 0, 0), (0, -40, 0))
    assert abs(width - 50) < 1e-6
    assert abs(height - 40) < 1e-6
    assert sign == -1.0

    stored = sheet_bounds(
        {"points_mm": {"origin": [0, 0, 10], "x": [200, 0, 10], "y": [0, 100, 10]}}
    )
    assert stored is not None
    assert abs(stored[0] - 200) < 1e-6

    drawing = svgpathtools.Path(svgpathtools.Line(10 + 20j, 110 + 70j))
    fitted, scale, inset, svg_w, svg_h = fit_paths([drawing], 200, 100, None, margin=0)
    assert abs(svg_w - 100) < 1e-6
    assert abs(svg_h - 50) < 1e-6
    assert abs(scale - 2.0) < 1e-6
    assert inset == 0
    assert abs(fitted[0].start) < 1e-6

    _, capped, _, _, _ = fit_paths([drawing], 200, 100, 5.0, margin=0)
    assert abs(capped - 2.0) < 1e-6

    _, smaller, _, _, _ = fit_paths([drawing], 200, 100, 0.5, margin=0)
    assert abs(smaller - 0.5) < 1e-6


def test_straight_stroke_is_one_line():
    path = svgpathtools.Path(svgpathtools.Line(0j, 40 + 0j))
    plan = plan_drawing([path], scale=1.0)
    assert [stroke.kind for stroke in plan.strokes] == ["line"]
    assert [motion.kind for motion in plan.motions] == ["travel", "line", "retract"]
