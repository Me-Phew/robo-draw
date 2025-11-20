import svgpathtools
from mephew_python_commons import LoggerFactory
from robodk import robolink, robomath

from .draw_options import DrawOptions
from .settings import RoboDrawerSettings
from .utils import get_points_from_path


class RoboDrawer:
    _RDK: robolink.Robolink

    _robot: robolink.Item
    _frame: robolink.Item
    _tool: robolink.Item

    _pixel: robolink.Item | None
    _board: robolink.Item | None

    def __init__(self, robo_drawer_settings: RoboDrawerSettings):
        self._settings: RoboDrawerSettings = robo_drawer_settings

        self._logger = LoggerFactory(log_files_prefix="RoboDraw").get_logger(
            self.__class__.__name__, level=self._settings.LOG_LEVEL
        )

        self._logger.debug(f"RoboDrawer initialized with settings: {self._settings}")

    def connect_robot(self, run_on_robot: bool, robot_name: str, force_robot: bool) -> None:
        self._logger.info(
            f"Connecting to robot '{robot_name}', with run_on_robot={run_on_robot}, force_robot={force_robot}..."
        )
        self._RDK = robolink.Robolink()

        self._robot = self._RDK.Item(robot_name, robolink.ITEM_TYPE_ROBOT)
        self._logger.info(f"Robot item obtained: {self._robot.Name()}")

        if not self._robot.Valid():
            self._logger.error(f"Robot '{robot_name}' not found in RoboDK station.")
            raise ValueError(f"Robot '{robot_name}' not found in RoboDK station.")

        if not run_on_robot:
            self._RDK.setRunMode(robolink.RUNMODE_SIMULATE)
            self._logger.info("Set RoboDK to simulation mode.")

            return

        if self._RDK.RunMode() != robolink.RUNMODE_SIMULATE:
            if run_on_robot and not force_robot:
                self._logger.error(
                    "RoboDK is not in simulation mode, refusing to run on robot. Use --force-robot to override."
                )
                raise ValueError(
                    "RoboDK is not in simulation mode, refusing to run on robot. Use --force-robot to override."
                )

        self._logger.info("Connecting to robot...")

        success = self._robot.Connect()

        if not success:
            self._logger.error("Robot connection attempt failed.")

        status, status_msg = self._robot.ConnectedState()

        if status != robolink.ROBOTCOM_READY:
            self._logger.error(f"Robot connection failed: {status_msg}")
            raise ConnectionError(f"Failed to connect to robot: {status_msg}")

        self._logger.info("Robot connected successfully.")

        self._RDK.setRunMode(robolink.RUNMODE_RUN_ROBOT)
        self._logger.info("Set RoboDK to run on robot mode.")

    def draw(self, draw_options: DrawOptions) -> None:
        if not draw_options.svg_path.exists():
            self._logger.error(f"SVG file not found: {draw_options.svg_path}")
            raise FileNotFoundError(f"SVG file not found: {draw_options.svg_path}")

        self._init_drawing_items(draw_options)

        self._logger.info(f"Starting drawing process for SVG: {draw_options.svg_path}")

        paths, attributes, _ = self._load_svg(str(draw_options.svg_path))

        self._draw_svg(draw_options, paths, attributes)

        self._logger.info("Drawing process completed.")

    def _init_drawing_items(self, draw_options: DrawOptions) -> None:
        self._logger.info("Initializing drawing items...")

        self._frame = self._RDK.Item(draw_options.frame_item_name, robolink.ITEM_TYPE_FRAME)
        if not self._frame.Valid():
            self._logger.error(f"Frame item '{draw_options.frame_item_name}' not found.")
            raise ValueError(f"Frame item '{draw_options.frame_item_name}' not found.")

        self._logger.info(f"Frame item '{draw_options.frame_item_name}' obtained.")

        self._tool = self._RDK.Item(draw_options.tool_item_name, robolink.ITEM_TYPE_TOOL)
        if not self._tool.Valid():
            self._logger.error(f"Tool item '{draw_options.tool_item_name}' not found.")
            raise ValueError(f"Tool item '{draw_options.tool_item_name}' not found.")

        self._logger.info(f"Tool item '{draw_options.tool_item_name}' obtained.")

        self._robot.setTool(self._tool)

        if not draw_options.use_visual_simulation:
            self._pixel = None
            self._board = None

            self._logger.info("Visual simulation disabled; skipping pixel and board item initialization.")
            self._logger.info("Drawing items initialized successfully.")
            return

        if draw_options.pixel_item_name is None:
            self._logger.error("Pixel item name must be provided for visual simulation.")
            raise ValueError("Pixel item name must be provided for visual simulation.")

        self._pixel = self._RDK.Item(draw_options.pixel_item_name)
        if not self._pixel.Valid():
            self._logger.error(f"Pixel item '{draw_options.pixel_item_name}' not found.")
            raise ValueError(f"Pixel item '{draw_options.pixel_item_name}' not found.")

        self._logger.info(f"Pixel item '{draw_options.pixel_item_name}' obtained.")

        if draw_options.board_item_name is None:
            self._logger.error("Board item name must be provided for visual simulation.")
            raise ValueError("Board item name must be provided for visual simulation.")

        self._board = self._RDK.Item(draw_options.board_item_name)
        if not self._board.Valid():
            self._logger.error(f"Board item '{draw_options.board_item_name}' not found.")
            raise ValueError(f"Board item '{draw_options.board_item_name}' not found.")

        self._logger.info(f"Board item '{draw_options.board_item_name}' obtained.")

        self._logger.info("Drawing items initialized successfully.")

    def _load_svg(self, svg_path: str):
        self._logger.info(f"Loading SVG file: {svg_path}")

        paths, attributes, *rest = svgpathtools.svg2paths(svg_path)

        self._logger.info(f"Loaded {len(paths)} paths from SVG.")

        return paths, attributes, rest

    def _draw_svg(
        self, draw_options: DrawOptions, paths: list[svgpathtools.Path], attributes: list[dict[str, str]]
    ) -> None:
        self._logger.info("Starting SVG drawing routine.")

        self._logger.debug(f"Drawing with options: {draw_options}")

        self._robot.setPoseFrame(self._frame)
        self._logger.debug(f"Set robot frame to '{self._frame.Name()}'.")

        self._robot.setPoseTool(self._tool)
        self._logger.debug(f"Set robot tool to '{self._tool.Name()}'.")

        orient_tool = robomath.rotx(180 * robomath.pi / 180)
        self._logger.debug(f"Calculated tool orientation matrix: {orient_tool}")

        total_paths = len(paths)
        self._logger.info(f"Total paths to draw: {total_paths}")

        self._logger.info("Moving robot to home position.")
        self._robot.MoveJ(self._robot.JointsHome())
        self._logger.info("Moved robot to home position.")

        for i, (path, attr) in enumerate(zip(paths, attributes)):
            points_2d = get_points_from_path(path, step_mm=draw_options.resolution / draw_options.scale)

            if not points_2d:
                self._logger.warning(f"Path {i+1} has no points after discretization; skipping.")
                continue

            self._logger.info(f"Drawing path {i+1}/{total_paths} with {len(points_2d)} points.")

            p0 = points_2d[0]
            self._logger.debug(f"Starting point of path: {p0}")

            target_pose = robomath.transl(p0.x * draw_options.scale, p0.y * draw_options.scale, 0) * orient_tool
            self._logger.debug(f"Calculated target pose: {target_pose}")

            approach_pose = target_pose * robomath.transl(0, 0, draw_options.approach_dist)
            self._logger.debug(f"Calculated approach pose: {approach_pose}")

            try:
                self._logger.info(f"Moving to approach pose for path {i+1}.")
                self._robot.MoveJ(approach_pose)
                self._logger.info(f"Moved to approach pose for path {i+1}.")
            except Exception as e:
                self._logger.exception(f"Robot cannot reach start of path {i+1}. Exception: {e}")
                self._logger.error("Ensure 'Frame draw' is within reach (approx X=300mm, Y=0mm).")
                raise

            self._logger.info(f"Moving down to target pose for path {i+1}.")
            self._robot.MoveL(target_pose)
            self._logger.info(f"Moved down to target pose for path {i+1}.")

            self._logger.info(f"Moving down to touch the drawing surface for path {i+1}.")
            self._robot.MoveL(target_pose)
            self._logger.info(f"Moved down to touch the drawing surface for path {i+1}.")

            self._logger.info(f"Tracing the curve for path {i+1}.")
            for p in points_2d:
                self._logger.debug(f"Drawing point: {p}")
                target_pose = robomath.transl(p.x * draw_options.scale, p.y * draw_options.scale, 0) * orient_tool
                self._logger.debug(f"Calculated target pose for point: {target_pose}")

                self._logger.info(f"Moving to point at X={p.x}, Y={p.y}.")
                self._robot.MoveL(target_pose)
                self._logger.info(f"Moved to point at X={p.x}, Y={p.y}.")

                if draw_options.use_visual_simulation:
                    assert self._board is not None and self._pixel is not None

                    self._board.AddGeometry(self._pixel, target_pose)
                    self._logger.debug("Added pixel geometry to board for visual simulation.")

            self._logger.info(f"Retracting after completing path {i+1}.")
            self._robot.MoveL(target_pose * robomath.transl(0, 0, -draw_options.approach_dist))
            self._logger.info(f"Retracted after completing path {i+1}.")

        self._logger.info("Returning robot to home position.")
        self._robot.MoveJ(self._robot.JointsHome())
        self._logger.info("Robot returned to home position.")

        self._logger.info("SVG drawing routine completed.")
