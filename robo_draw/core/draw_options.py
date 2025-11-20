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
