import svgpathtools
from mephew_python_commons import LoggerFactory
from robodk import robolink, robomath
from tqdm import tqdm

from .draw_options import DrawOptions
from .settings import RoboDrawerSettings
from .utils import get_points_from_path, simplify_points


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

        # Try connecting with retries to handle transient communication issues
        attempts = 3
        success = False
        for attempt in range(1, attempts + 1):
            self._logger.debug(f"Attempt {attempt}/{attempts} to connect to robot...")
            success = self._robot.Connect()
            if success:
                self._logger.info(f"Robot connection attempt succeeded on attempt {attempt}.")
                break
            else:
                self._logger.warning(f"Robot connection attempt {attempt} failed.")
                if attempt < attempts:
                    import time
                    time.sleep(1.5 * attempt)

        status, status_msg = self._robot.ConnectedState()

        # Handle known non-fatal controller message "Unknown SETROUNDING"
        if status != robolink.ROBOTCOM_READY:
            msg = str(status_msg)
            if "Unknown SETROUNDING" in msg:
                # Allow proceeding only when explicitly forced by user
                if force_robot:
                    self._logger.warning(
                        f"Non-fatal connection warning from robot controller: {msg}. "
                        "Proceeding because force_robot=True."
                    )
                else:
                    self._logger.error(
                        f"Robot connection returned warning: {msg}. "
                        "Use --force-robot to proceed if you understand the risk, "
                        "or check/update your RoboDK/robot plugin."
                    )
                    raise ConnectionError(f"Failed to connect to robot: {msg}. Use --force-robot to override if safe.")
            else:
                self._logger.error(f"Robot connection failed: {msg}")
                raise ConnectionError(f"Failed to connect to robot: {msg}")

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

        self._logger.info("Optimizing path order...")
        paths, attributes = self._optimize_path_order(paths, attributes, draw_options)
        self._logger.info("Path optimization completed.")

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

        # self._robot.setTool(self._tool)

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

    def _optimize_path_order(
        self, paths: list[svgpathtools.Path], attributes: list[dict[str, str]], draw_options: DrawOptions
    ) -> tuple[list[svgpathtools.Path], list[dict[str, str]]]:
        """
        Reorders paths using a greedy nearest-neighbor strategy to minimize joint travel distance.
        Also reverses paths if starting from the end is closer in joint space.
        """
        if not paths:
            return [], []

        try:
            current_joints = self._robot.Joints().list()
        except Exception:
            self._logger.warning("Could not get robot joints, assuming all zeros.")
            current_joints = [0.0] * 6

        orient_tool = robomath.rotx(180 * robomath.pi / 180)
        frame_pose = self._frame.Pose()

        remaining = list(zip(paths, attributes))
        ordered_paths = []
        ordered_attributes = []

        while remaining:
            best_idx = -1
            best_dist = float("inf")
            should_reverse = False

            for i, (p, _) in enumerate(remaining):
                # Check start
                try:
                    pose_start = robomath.transl(p.start.real * draw_options.scale, p.start.imag * draw_options.scale, 0) * orient_tool
                    joints_start_mat = self._robot.SolveIK(pose_start, joints_approx=current_joints, reference=frame_pose)
                    joints_start = joints_start_mat.list()

                    if len(joints_start) > 0:
                        dist_start = max([abs(j1 - j2) for j1, j2 in zip(current_joints, joints_start)])

                        if dist_start < best_dist:
                            best_dist = dist_start
                            best_idx = i
                            should_reverse = False
                except Exception:
                    pass

                # Check end (reverse)
                try:
                    pose_end = robomath.transl(p.end.real * draw_options.scale, p.end.imag * draw_options.scale, 0) * orient_tool
                    joints_end_mat = self._robot.SolveIK(pose_end, joints_approx=current_joints, reference=frame_pose)
                    joints_end = joints_end_mat.list()

                    if len(joints_end) > 0:
                        dist_end = max([abs(j1 - j2) for j1, j2 in zip(current_joints, joints_end)])

                        if dist_end < best_dist:
                            best_dist = dist_end
                            best_idx = i
                            should_reverse = True
                except Exception:
                    pass

            if best_idx == -1:
                self._logger.warning("No reachable path found among remaining paths. Appending rest as is.")
                for p, attr in remaining:
                    ordered_paths.append(p)
                    ordered_attributes.append(attr)
                break

            p, attr = remaining.pop(best_idx)

            # Update current_joints to the end of the selected path
            if should_reverse:
                pose_move_start = robomath.transl(p.end.real * draw_options.scale, p.end.imag * draw_options.scale, 0) * orient_tool
                joints_move_start_mat = self._robot.SolveIK(pose_move_start, joints_approx=current_joints, reference=frame_pose)
                joints_move_start = joints_move_start_mat.list()

                if len(joints_move_start) > 0:
                    pose_move_end = robomath.transl(p.start.real * draw_options.scale, p.start.imag * draw_options.scale, 0) * orient_tool
                    joints_move_end_mat = self._robot.SolveIK(pose_move_end, joints_approx=joints_move_start, reference=frame_pose)
                    joints_move_end = joints_move_end_mat.list()
                    if len(joints_move_end) > 0:
                        current_joints = joints_move_end
                
                try:
                    p = p.reversed()
                except Exception:
                    pass
            else:
                pose_move_start = robomath.transl(p.start.real * draw_options.scale, p.start.imag * draw_options.scale, 0) * orient_tool
                joints_move_start_mat = self._robot.SolveIK(pose_move_start, joints_approx=current_joints, reference=frame_pose)
                joints_move_start = joints_move_start_mat.list()

                if len(joints_move_start) > 0:
                    pose_move_end = robomath.transl(p.end.real * draw_options.scale, p.end.imag * draw_options.scale, 0) * orient_tool
                    joints_move_end_mat = self._robot.SolveIK(pose_move_end, joints_approx=joints_move_start, reference=frame_pose)
                    joints_move_end = joints_move_end_mat.list()
                    if len(joints_move_end) > 0:
                        current_joints = joints_move_end

            ordered_paths.append(p)
            ordered_attributes.append(attr)

        return ordered_paths, ordered_attributes

    def _draw_svg(
        self, draw_options: DrawOptions, paths: list[svgpathtools.Path], attributes: list[dict[str, str]]
    ) -> None:
        self._logger.info("Starting SVG drawing routine.")

        self._logger.debug(f"Drawing with options: {draw_options}")

        self._robot.setPoseFrame(self._frame)
        self._logger.debug(f"Set robot frame to '{self._frame.Name()}'.")

        # self._robot.setPoseTool(self._tool)
        # self._logger.debug(f"Set robot tool to '{self._tool.Name()}'.")

        orient_tool = robomath.rotx(180 * robomath.pi / 180)
        self._logger.debug(f"Calculated tool orientation matrix: {orient_tool}")
        
        frame_pose = self._frame.Pose()

        total_paths = len(paths)
        self._logger.info(f"Total paths to draw: {total_paths}")

        self._logger.info("Moving robot to home position.")
        self._go_home()
        self._logger.info("Moved robot to home position.")

        try:
            last_joints = self._robot.Joints().list()
        except Exception:
            self._logger.warning("Could not get robot joints, assuming all zeros.")
            last_joints = [0.0] * 6

        last_end_point = None
        pending_retract_pose = None
        
        execution_queue = []
        self._logger.info("Planning movements...")

        # Disable rendering to speed up IK calculations
        self._RDK.Render(False)

        for i, (path, attr) in enumerate(zip(tqdm(paths, desc="Planning"), attributes)):
            points_2d = get_points_from_path(path, step_mm=draw_options.resolution / draw_options.scale)

            if not points_2d:
                self._logger.warning(f"Path {i+1} has no points after discretization; skipping.")
                continue

            # Optimization: Simplify path using Ramer-Douglas-Peucker algorithm
            original_count = len(points_2d)
            # 0.1mm tolerance converted to SVG units
            epsilon = 0.5 / draw_options.scale
            points_2d = simplify_points(points_2d, epsilon)
            self._logger.info(f"Path {i+1} simplified from {original_count} to {len(points_2d)} points.")

            self._logger.info(f"Planning path {i+1}/{total_paths} with {len(points_2d)} points.")

            p0 = points_2d[0]
            self._logger.debug(f"Starting point of path: {p0}")

            # Check continuity with previous path
            is_continuous = False
            if last_end_point is not None:
                dist = ((p0.x - last_end_point.x) ** 2 + (p0.y - last_end_point.y) ** 2) ** 0.5
                if dist < 1.0:  # 1mm tolerance for continuity
                    is_continuous = True

            target_pose = robomath.transl(p0.x * draw_options.scale, p0.y * draw_options.scale, 0) * orient_tool
            self._logger.debug(f"Calculated target pose: {target_pose}")

            approach_pose = target_pose * robomath.transl(0, 0, draw_options.approach_dist)
            self._logger.debug(f"Calculated approach pose: {approach_pose}")

            if not is_continuous:
                # Execute pending retract from previous path if we are not continuous
                if pending_retract_pose is not None:
                    self._logger.info(f"Retracting after completing previous path.")
                    last_joints = self._solve_ik(pending_retract_pose, last_joints, frame_pose)
                    execution_queue.append(("MOVE", last_joints))
                    self._logger.info(f"Retracted.")
                    pending_retract_pose = None

                try:
                    self._logger.info(f"Moving to approach pose for path {i+1}.")
                    last_joints = self._solve_ik(approach_pose, last_joints, frame_pose)
                    execution_queue.append(("MOVE", last_joints))
                    self._logger.info(f"Moved to approach pose for path {i+1}.")
                except Exception as e:
                    self._logger.exception(f"Robot cannot reach start of path {i+1}. Exception: {e}")
                    self._logger.error("Ensure 'Frame draw' is within reach (approx X=300mm, Y=0mm).")
                    raise
            else:
                self._logger.debug("Path is continuous with previous one. Skipping retract/approach.")

            self._logger.info(f"Moving down to target pose for path {i+1}.")
            last_joints = self._solve_ik(target_pose, last_joints, frame_pose)
            execution_queue.append(("MOVE", last_joints))
            self._logger.info(f"Moved down to target pose for path {i+1}.")

            self._logger.info(f"Tracing the curve for path {i+1}.")
            for p in points_2d[1:]:
                self._logger.debug(f"Drawing point: {p}")
                target_pose = robomath.transl(p.x * draw_options.scale, p.y * draw_options.scale, 0) * orient_tool
                self._logger.debug(f"Calculated target pose for point: {target_pose}")

                self._logger.info(f"Moving to point at X={p.x}, Y={p.y}.")
                last_joints = self._solve_ik(target_pose, last_joints, frame_pose)
                execution_queue.append(("MOVE", last_joints))
                self._logger.info(f"Moved to point at X={p.x}, Y={p.y}.")

                if draw_options.use_visual_simulation:
                    execution_queue.append(("DRAW", target_pose))

            last_end_point = points_2d[-1]
            end_pose = robomath.transl(last_end_point.x * draw_options.scale, last_end_point.y * draw_options.scale, 0) * orient_tool
            pending_retract_pose = end_pose * robomath.transl(0, 0, draw_options.approach_dist)

        self._logger.info("All paths planned. Retracting drawing tool.")
        if pending_retract_pose is not None:
            last_joints = self._solve_ik(pending_retract_pose, last_joints, frame_pose)
            execution_queue.append(("MOVE", last_joints))
        self._logger.info("Drawing tool retracted.")
        
        self._logger.info(f"Planning completed. {len(execution_queue)} actions queued.")
        self._logger.info("Executing movements...")
        
        # Enable rendering again
        self._RDK.Render(True)
        
        for action, data in tqdm(execution_queue, desc="Executing"):
            if action == "MOVE":
                self._robot.MoveJ(data)
            elif action == "DRAW":
                if self._board and self._pixel:
                    self._board.AddGeometry(self._pixel, data)
        
        # self._RDK.Render(True)

        self._logger.info("Returning robot to home position.")
        self._go_home()
        self._logger.info("Robot returned to home position.")

        self._logger.info("SVG drawing routine completed.")

    def _solve_ik(self, pose, last_joints: list[float], frame_pose) -> list[float]:
        """
        Calculates the inverse kinematics for the specified pose using the previous joints as approximation.
        Returns the new joints.
        """
        ik_result = self._robot.SolveIK(pose, joints_approx=last_joints, reference=frame_pose)
        joints = ik_result.list()
        
        if len(joints) > 0:
            return joints
        else:
            self._logger.warning("SolveIK failed to find a solution during planning. Using last known joints.")
            return last_joints

    def _go_home(self) -> None:
        self._logger.debug(f"Default home position: {self._robot.JointsHome()}")

        custom_home = [0, -70, -95, -104, 90, -20]
        self._logger.debug(f"Custom home position: {custom_home}")

        self._logger.info("Moving to custom home position.")
        self._robot.MoveJ(custom_home)
