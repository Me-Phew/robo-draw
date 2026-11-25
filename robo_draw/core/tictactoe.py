"""Kółko i krzyżyk: kamera (OpenCV) widzi znak gracza, robot dorysowuje swój."""

from __future__ import annotations

import math
from pathlib import Path

import cv2
import numpy as np
import svgpathtools

from .draw_options import DrawOptions

LINES = (
    (0, 1, 2),
    (3, 4, 5),
    (6, 7, 8),
    (0, 3, 6),
    (1, 4, 7),
    (2, 5, 8),
    (0, 4, 8),
    (2, 4, 6),
)
WINDOW = "kolko-krzyzyk"


def outcome(board: list[str]) -> str | None:
    for a, b, c in LINES:
        if board[a] != "." and board[a] == board[b] == board[c]:
            return board[a]
    if "." not in board:
        return "draw"
    return None


def best_move(board: list[str], player: str = "O") -> int:
    move, _score = _search(list(board), player)
    if move is None:
        raise ValueError("Brak wolnego pola.")
    return move


def format_board(board: list[str]) -> str:
    """Róg kartki (pole 1) jest na dole po lewej, oś X w prawo, oś Y w górę."""
    rows = []
    for row in (2, 1, 0):
        cells = []
        for col in range(3):
            index = row * 3 + col
            cells.append(board[index] if board[index] != "." else str(index + 1))
        rows.append(" ".join(cells))
    return "\n".join(rows)


def grid_square(width: float, height: float, margin: float = 12.0) -> tuple[float, float, float]:
    size = min(width, height) - 2.0 * margin
    if size < 30.0:
        raise ValueError("Kartka jest za mała na planszę 3x3.")
    return (width - size) / 2.0, (height - size) / 2.0, size


def grid_paths(x0: float, y0: float, size: float) -> list:
    step = size / 3.0
    paths = []
    for step_index in (1, 2):
        paths.append(_line(x0 + step_index * step, y0, x0 + step_index * step, y0 + size))
        paths.append(_line(x0, y0 + step_index * step, x0 + size, y0 + step_index * step))
    return paths


def mark_paths(x0: float, y0: float, size: float, index: int, kind: str) -> list:
    col, row = index % 3, index // 3
    cell = size / 3.0
    pad = cell * 0.22
    left = x0 + col * cell + pad
    right = x0 + (col + 1) * cell - pad
    bottom = y0 + row * cell + pad
    top = y0 + (row + 1) * cell - pad
    if kind == "X":
        return [_line(left, bottom, right, top), _line(left, top, right, bottom)]
    radius = cell * 0.28
    return [_circle((left + right) / 2.0, (bottom + top) / 2.0, radius)]


def occupied_cells(warped: np.ndarray, ink_ratio: float = 0.045) -> tuple[list[bool], list[float]]:
    """Pola od rogu obrazu odpowiadającego początkowi kartki. True znaczy: w środku pola jest tusz."""
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    _threshold, ink = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    size = warped.shape[0]
    cell = size / 3.0
    pad = cell * 0.28
    flags: list[bool] = []
    ratios: list[float] = []
    for row in range(3):
        for col in range(3):
            x0 = int(col * cell + pad)
            x1 = int((col + 1) * cell - pad)
            y_bottom = int((3 - row) * cell - pad)
            y_top = int((2 - row) * cell + pad)
            roi = ink[y_top:y_bottom, x0:x1]
            ratio = 0.0 if roi.size == 0 else float(roi.mean()) / 255.0
            ratios.append(ratio)
            flags.append(ratio >= ink_ratio)
    return flags, ratios


def warp_board(frame: np.ndarray, corners: np.ndarray, size: int = 600) -> np.ndarray:
    """Narożniki: róg XY, koniec X, róg przeciwny, koniec Y."""
    destination = np.float32(
        [[0, size - 1], [size - 1, size - 1], [size - 1, 0], [0, 0]]
    )
    matrix = cv2.getPerspectiveTransform(np.float32(corners), destination)
    return cv2.warpPerspective(frame, matrix, (size, size))


def open_capture(source: str) -> cv2.VideoCapture:
    capture = cv2.VideoCapture(int(source) if source.isdigit() else source)
    if not capture.isOpened():
        raise RuntimeError(
            f"Nie udało się otworzyć kamery '{source}'. "
            "OnRobot Eyes jest kamerą USB-C podłączoną do Eye Box. OpenCV zobaczy obraz, "
            "gdy ten komputer dostaje strumień USB (numer kamery) albo adres obrazu (URL). "
            "Wybór detali w Web Client Eyes nie wystawia planszy do tego programu."
        )
    return capture


def play(drawer, draw_options: DrawOptions, camera: str, ink_ratio: float) -> None:
    width = draw_options.sheet_width_mm
    height = draw_options.sheet_height_mm
    if width is None or height is None:
        raise ValueError("Gra potrzebuje pliku setup z wymiarami kartki.")

    x0, y0, size = grid_square(width, height)
    print("Rysuję planszę. Pole 1 leży przy rogu kartki, numery idą wzdłuż osi X, potem w głąb osi Y.")
    print(format_board(["."] * 9))
    drawer.draw_paths(grid_paths(x0, y0, size), draw_options)

    capture = open_capture(camera)
    try:
        corners = _calibrate(capture)
        board = ["."] * 9
        print("Zaczynasz ty, znakiem X. Narysuj jeden krzyżyk i naciśnij Enter. q kończy grę, cyfra 1-9 stawia X bez kamery.")
        while outcome(board) is None:
            print(format_board(board))
            answer = input("Twój ruch. Enter = zdjęcie z kamery: ").strip().lower()
            if answer == "q":
                print("Koniec gry.")
                return
            if answer.isdigit() and answer in "123456789":
                index = int(answer) - 1
                if board[index] != ".":
                    print("To pole jest zajęte.")
                    continue
            else:
                frame = _grab(capture)
                seen, ratios = occupied_cells(warp_board(frame, corners), ink_ratio)
                _show_reading(warp_board(frame, corners), seen)
                fresh = [cell for cell, filled in enumerate(seen) if filled and board[cell] == "."]
                print("Tusz w polach: " + ", ".join(f"{cell + 1}:{ratio:.2f}" for cell, ratio in enumerate(ratios)))
                if len(fresh) != 1:
                    print(f"Potrzebny jest jeden nowy znak, a widzę {len(fresh)}. Popraw krzyżyk albo wpisz numer pola.")
                    continue
                index = fresh[0]
            board[index] = "X"
            if _finish_if_over(board):
                return
            robot_index = best_move(board, "O")
            print(f"Rysuję kółko w polu {robot_index + 1}.")
            drawer.draw_paths(mark_paths(x0, y0, size, robot_index, "O"), draw_options)
            board[robot_index] = "O"
            if _finish_if_over(board):
                return
        print(format_board(board))
    finally:
        capture.release()
        cv2.destroyAllWindows()


def _finish_if_over(board: list[str]) -> bool:
    result = outcome(board)
    if result is None:
        return False
    print(format_board(board))
    if result == "draw":
        print("Remis.")
    elif result == "X":
        print("Wygrywasz.")
    else:
        print("Wygrywa robot.")
    return True


def _search(board: list[str], player: str) -> tuple[int | None, int]:
    done = outcome(board)
    if done == "O":
        return None, 1
    if done == "X":
        return None, -1
    if done == "draw":
        return None, 0
    best_score = -2 if player == "O" else 2
    best_index = None
    opponent = "X" if player == "O" else "O"
    for index, cell in enumerate(board):
        if cell != ".":
            continue
        board[index] = player
        _move, score = _search(board, opponent)
        board[index] = "."
        if player == "O" and score > best_score:
            best_score, best_index = score, index
        elif player == "X" and score < best_score:
            best_score, best_index = score, index
    return best_index, best_score


def _line(x0: float, y0: float, x1: float, y1: float):
    return svgpathtools.Path(svgpathtools.Line(complex(x0, y0), complex(x1, y1)))


def _circle(cx: float, cy: float, radius: float):
    arcs = []
    for step in range(4):
        a0 = step * math.pi / 2.0
        a1 = (step + 1) * math.pi / 2.0
        start = complex(cx + radius * math.cos(a0), cy + radius * math.sin(a0))
        end = complex(cx + radius * math.cos(a1), cy + radius * math.sin(a1))
        arcs.append(svgpathtools.Arc(start, complex(radius, radius), 0, False, True, end))
    return svgpathtools.Path(*arcs)


def _grab(capture: cv2.VideoCapture) -> np.ndarray:
    frame = None
    for _ in range(5):
        ok, frame = capture.read()
        if not ok:
            frame = None
    if frame is None:
        raise RuntimeError("Kamera nie oddała klatki.")
    return frame


def _calibrate(capture: cv2.VideoCapture) -> np.ndarray:
    clicks: list[tuple[int, int]] = []

    def on_mouse(event, x, y, _flags, _param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 4:
            clicks.append((x, y))

    cv2.namedWindow(WINDOW)
    cv2.setMouseCallback(WINDOW, on_mouse)
    print(
        "W oknie kamery kliknij cztery rogi planszy: 1 róg przy początku kartki, "
        "2 róg na końcu osi X, 3 róg przeciwny, 4 róg na końcu osi Y. "
        "Enter zatwierdza, r czyści, q wychodzi."
    )
    while True:
        frame = _grab(capture)
        view = frame.copy()
        for number, (x, y) in enumerate(clicks, start=1):
            cv2.circle(view, (x, y), 6, (0, 180, 0), -1)
            cv2.putText(view, str(number), (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 180, 0), 2)
        if len(clicks) == 4:
            cv2.polylines(view, [np.array(clicks, dtype=np.int32)], True, (0, 180, 0), 2)
        cv2.imshow(WINDOW, view)
        key = cv2.waitKey(30) & 0xFF
        if key in (13, 32) and len(clicks) == 4:
            cv2.destroyWindow(WINDOW)
            return np.float32(clicks)
        if key == ord("r"):
            clicks.clear()
        if key == ord("q"):
            raise SystemExit("Przerwano kalibrację kamery.")


def _show_reading(warped: np.ndarray, seen: list[bool]) -> None:
    view = warped.copy()
    cell = view.shape[0] // 3
    for index, filled in enumerate(seen):
        col, row = index % 3, index // 3
        x0, y0 = col * cell, (2 - row) * cell
        color = (0, 0, 255) if filled else (0, 180, 0)
        cv2.rectangle(view, (x0 + 4, y0 + 4), (x0 + cell - 4, y0 + cell - 4), color, 2)
        cv2.putText(view, str(index + 1), (x0 + 12, y0 + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.imshow(WINDOW, view)
    cv2.waitKey(1)


def sheet_draw_options(sheet: dict, bounds: tuple[float, float, float]) -> DrawOptions:
    width, height, y_sign = bounds
    return DrawOptions(
        svg_path=Path("tictactoe"),
        frame_item_name=sheet["frame_item_name"],
        tool_item_name=sheet["tool_item_name"],
        use_visual_simulation=False,
        pixel_item_name=None,
        board_item_name=None,
        approach_dist=int(sheet["approach_dist"]),
        scale=1.0,
        resolution=1.0,
        sheet_width_mm=width,
        sheet_height_mm=height,
        sheet_y_sign=y_sign,
        requested_scale=1.0,
        fit_to_sheet=False,
    )
