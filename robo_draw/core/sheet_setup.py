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
