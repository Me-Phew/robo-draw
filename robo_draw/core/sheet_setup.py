"""Układ kartki i końcówka pisaka zapisane do JSON."""

from __future__ import annotations

import json
import math
from pathlib import Path

from robodk import robomath


def frame_matrix(origin: tuple[float, float, float], x_point: tuple[float, float, float], y_point: tuple[float, float, float]) -> list[list[float]]:
    """Macierz układu: oś X w stronę drugiego punktu, Z w górę stacji."""
    x_axis = _normalize(_sub(x_point, origin))
    y_raw = _sub(y_point, origin)
    z_axis = _cross(x_axis, y_raw)
    length = math.sqrt(sum(component * component for component in z_axis))
    if length < 1e-4:
        raise ValueError("Punkty kartki leżą na jednej linii. Rozsuń punkt X i punkt Y.")
    z_axis = tuple(component / length for component in z_axis)
    y_axis = _cross(z_axis, x_axis)
    if z_axis[2] < 0:
        y_axis = tuple(-component for component in y_axis)
        z_axis = tuple(-component for component in z_axis)
    return [
        [x_axis[0], y_axis[0], z_axis[0], origin[0]],
        [x_axis[1], y_axis[1], z_axis[1], origin[1]],
        [x_axis[2], y_axis[2], z_axis[2], origin[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


SHEET_MARGIN_MM = 2.0


def sheet_extents(
    origin: tuple[float, float, float],
    x_point: tuple[float, float, float],
    y_point: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Szerokość, wysokość i znak Y. Znak -1 znaczy, że kartka leży po ujemnym Y układu (Z i tak w górę)."""
    rows = frame_matrix(origin, x_point, y_point)
    x_axis = (rows[0][0], rows[1][0], rows[2][0])
    y_axis = (rows[0][1], rows[1][1], rows[2][1])
    width = _dot(_sub(x_point, origin), x_axis)
    along_y = _dot(_sub(y_point, origin), y_axis)
    if width < 1.0 or abs(along_y) < 1.0:
        raise ValueError("Kartka jest mniejsza niż 1 mm. Dotknij końców krawędzi, nie punktów obok rogu.")
    return width, abs(along_y), (1.0 if along_y >= 0.0 else -1.0)


def sheet_bounds(sheet: dict) -> tuple[float, float, float] | None:
    size = sheet.get("sheet_mm")
    if size is not None:
        return float(size["width"]), float(size["height"]), float(sheet.get("y_sign", 1.0))
    points = sheet.get("points_mm")
    if not points:
        return None
    return sheet_extents(tuple(points["origin"]), tuple(points["x"]), tuple(points["y"]))


def fit_paths(paths: list, width: float, height: float, requested_scale: float | None, margin: float = SHEET_MARGIN_MM):
    """Przesuwa rysunek do rogu kartki i dobiera skalę, która mieści go w prostokącie."""
    inset = min(margin, width * 0.05, height * 0.05)
    usable_w = width - 2.0 * inset
    usable_h = height - 2.0 * inset
    xmin, ymin, xmax, ymax = _paths_bbox(paths)
    svg_w = max(xmax - xmin, 1e-6)
    svg_h = max(ymax - ymin, 1e-6)
    fit = min(usable_w / svg_w, usable_h / svg_h)
    scale = fit if requested_scale is None else min(abs(requested_scale), fit)
    delta = complex(-xmin, -ymin)
    return [path.translated(delta) for path in paths], scale, inset, svg_w, svg_h


def pose_to_xyzrpw(pose) -> list[float]:
    values = robomath.pose_2_xyzrpw(pose)
    return [round(float(value), 4) for value in values]


def rows_to_pose(rows: list[list[float]]):
    return robomath.Mat(rows)


def save_sheet(path: Path, sheet: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(sheet, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def load_sheet(path: Path) -> dict:
    sheet = json.loads(path.read_text(encoding="utf-8"))
    if "frame_pose" not in sheet or "tool_pose_xyzrpw" not in sheet:
        raise ValueError(f"Plik {path} nie jest setupem kartki.")
    return sheet


def _paths_bbox(paths: list) -> tuple[float, float, float, float]:
    xmin = ymin = math.inf
    xmax = ymax = -math.inf
    for path in paths:
        if not path:
            continue
        x0, x1, y0, y1 = path.bbox()
        xmin, xmax = min(xmin, x0), max(xmax, x1)
        ymin, ymax = min(ymin, y0), max(ymax, y1)
    if xmin is math.inf:
        return 0.0, 0.0, 0.0, 0.0
    return xmin, ymin, xmax, ymax


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(sum(component * component for component in vector))
    if length < 1.0:
        raise ValueError("Punkty kartki są bliżej niż 1 mm. Odsuń punkt X od rogu.")
    return tuple(component / length for component in vector)
