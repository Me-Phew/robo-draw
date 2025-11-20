import sys
from pathlib import Path

import click
import typer
from mephew_python_commons import LoggerFactory

from . import DrawOptions, RoboDrawer, RoboDrawerSettings


def main(
    svg_path: Path = typer.Option(..., help="Path to the SVG file to draw"),
    run_on_robot: bool = typer.Option(False, help="Whether to run the drawing on the robot"),
    force_robot: bool = typer.Option(False, help="Whether to force runnning on the robot"),
    robot_name: str = typer.Option("Hanwha HCR-3A", help="Name of the robot in RoboDK"),
    frame_item_name: str = typer.Option("Drawing Frame", help="Name of the refrence frame item for drawing"),
    tool_item_name: str = typer.Option("Drawing Tool", help="Name of the drawing tool item"),
    use_visual_simulation: bool = typer.Option(
        False, help="Whether to use visual simulation (requires additional items)"
    ),
    pixel_item_name: str = typer.Option(None, help="Name of the pixel item for visual simulation"),
    board_item_name: str = typer.Option(None, help="Name of the board item for visual simulation"),
    approach_dist: int = typer.Option(50, help="Approach distance in mm"),
    scale: float = typer.Option(1.0, help="Scaling factor for the drawing"),
    resolution: float = typer.Option(2.0, help="Resolution in mm per point"),
):
    settings: RoboDrawerSettings = RoboDrawerSettings(_env_file=".env", _env_file_encoding="utf-8")

    logger = LoggerFactory(log_files_prefix="RoboDraw").get_logger(__name__, level=settings.LOG_LEVEL)

    logger.info("Starting RoboDraw application.")

    logger.debug("Creating RoboDrawer instance.")
    drawer: RoboDrawer = RoboDrawer(settings)

    logger.debug("Connecting to robot.")
    drawer.connect_robot(run_on_robot, robot_name, force_robot)

    logger.debug("Preparing draw options.")
    draw_options: DrawOptions = DrawOptions(
        svg_path=svg_path,
        frame_item_name=frame_item_name,
        tool_item_name=tool_item_name,
        use_visual_simulation=use_visual_simulation,
        pixel_item_name=pixel_item_name,
        board_item_name=board_item_name,
        approach_dist=approach_dist,
        scale=scale,
        resolution=resolution,
    )

    logger.debug("Starting drawing process.")
    return drawer.draw(draw_options)


if __name__ == "__main__":
    try:
        sys.exit(typer.run(main))
    except SystemExit:
        print("Program exited.")

        sys.exit(0)
