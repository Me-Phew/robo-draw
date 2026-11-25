from pathlib import Path

from pydantic import BaseModel


class DrawOptions(BaseModel):
    svg_path: Path

    frame_item_name: str
    tool_item_name: str

    use_visual_simulation: bool

    # Optional items for visual simulation
    pixel_item_name: str | None
    board_item_name: str | None

    approach_dist: int
    scale: float
    resolution: float

    # Prostokąt kartki z pliku setup. Puste pola znaczą: rysuj w skali `scale`, bez dopasowania.
    sheet_width_mm: float | None = None
    sheet_height_mm: float | None = None
    sheet_y_sign: float = 1.0
    requested_scale: float | None = None
    fit_to_sheet: bool = True
