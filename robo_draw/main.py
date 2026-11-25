from pathlib import Path

import typer
from mephew_python_commons import LoggerFactory

from . import DrawOptions, RoboDrawer, RoboDrawerSettings
from .core.sheet_setup import load_sheet, sheet_bounds

app = typer.Typer(help="RoboDraw - SVG drawing application for robots")


@app.command()
def draw(
    svg_path: Path = typer.Option(..., help="Path to the SVG file to draw"),
    run_on_robot: bool = typer.Option(False, help="Whether to run the drawing on the robot"),
    force_robot: bool = typer.Option(False, help="Whether to force runnning on the robot"),
    robot_name: str = typer.Option("Hanwha HCR-3A", help="Name of the robot in RoboDK"),
    frame_item_name: str = typer.Option("Drawing Frame", help="Name of the refrence frame item for drawing"),
    tool_item_name: str = typer.Option("Drawing Tool", help="Name of the drawing tool item"),
    use_visual_simulation: bool = typer.Option(
        False, help="Whether to use visual simulation (requires additional items)"
    ),
    pixel_item_name: str = typer.Option("Pixel", help="Name of the pixel item for visual simulation"),
    board_item_name: str = typer.Option("Drawing Board", help="Name of the board item for visual simulation"),
    approach_dist: int = typer.Option(
        10, help="How high the pen lifts above the sheet, in mm. Positive Z of the drawing frame."
    ),
    scale: float | None = typer.Option(
        None, help="Skala rysunku. Bez tej opcji i z --setup skala dopasowuje rysunek do kartki."
    ),
    resolution: float = typer.Option(1.0, help="Resolution in mm per point"),
    setup: Path | None = typer.Option(None, help="JSON z setupu kartki i pisaka"),
):
    """Draw an SVG file using the robot."""
    settings: RoboDrawerSettings = RoboDrawerSettings(_env_file=".env", _env_file_encoding="utf-8")

    logger = LoggerFactory(log_files_prefix="RoboDraw").get_logger(__name__, level=settings.LOG_LEVEL)

    logger.info("Starting RoboDraw application.")

    logger.debug("Creating RoboDrawer instance.")
    drawer: RoboDrawer = RoboDrawer(settings)

    logger.debug("Connecting to robot.")
    drawer.connect_robot(run_on_robot, robot_name, force_robot)

    sheet_width = sheet_height = None
    sheet_y_sign = 1.0
    if setup is not None:
        sheet = load_sheet(setup)
        drawer.apply_sheet(sheet)
        frame_item_name = sheet["frame_item_name"]
        tool_item_name = sheet["tool_item_name"]
        approach_dist = int(sheet["approach_dist"])
        bounds = sheet_bounds(sheet)
        if bounds is not None:
            sheet_width, sheet_height, sheet_y_sign = bounds

    logger.debug("Preparing draw options.")
    draw_options: DrawOptions = DrawOptions(
        svg_path=svg_path,
        frame_item_name=frame_item_name,
        tool_item_name=tool_item_name,
        use_visual_simulation=use_visual_simulation,
        pixel_item_name=pixel_item_name,
        board_item_name=board_item_name,
        approach_dist=approach_dist,
        scale=1.0 if scale is None else scale,
        resolution=resolution,
        sheet_width_mm=sheet_width,
        sheet_height_mm=sheet_height,
        sheet_y_sign=sheet_y_sign,
        requested_scale=scale,
    )

    logger.debug("Starting drawing process.")
    return drawer.draw(draw_options)


@app.command()
def setup(
    output: Path = typer.Option(Path("sheet.json"), help="Gdzie zapisać JSON"),
    robot_name: str = typer.Option("Hanwha HCR-3A", help="Name of the robot in RoboDK"),
    frame_item_name: str = typer.Option("Drawing Frame", help="Name of the reference frame"),
    tool_item_name: str = typer.Option("Drawing Tool", help="Name of the pen tool"),
    approach_dist: int = typer.Option(10, help="Pen lift above the sheet, in mm"),
    apply: bool = typer.Option(False, help="Wstaw istniejący JSON do stacji, bez uczenia"),
):
    """Naucz róg kartki i kierunki X/Y, albo wstaw zapisany JSON do RoboDK."""
    settings: RoboDrawerSettings = RoboDrawerSettings(_env_file=".env", _env_file_encoding="utf-8")
    drawer = RoboDrawer(settings)
    drawer.connect_robot(False, robot_name, False)
    if apply:
        drawer.apply_sheet(load_sheet(output))
        print(f"Wstawiono {output} do stacji.")
        return
    drawer.teach_sheet(output, frame_item_name, tool_item_name, approach_dist)


@app.command()
def serve(
    host: str = typer.Option("0.0.0.0", help="Host address to bind to"),
    port: int = typer.Option(8000, help="Port to listen on"),
):
    """Start the API server to receive SVG files remotely."""
    from .api import run_server

    print(f"Starting RoboDraw API server on {host}:{port}")
    print(f"Documentation available at http://{host}:{port}/docs")
    run_server(host=host, port=port)


def main():
    app()


if __name__ == "__main__":
    main()
