import math
import svgpathtools
from mephew_python_commons import LoggerFactory
from robodk import robolink, robomath

from .sheet_setup import fit_paths, frame_matrix, pose_to_xyzrpw, rows_to_pose, save_sheet, sheet_extents
from tqdm import tqdm
import numpy as np

from .draw_options import DrawOptions
from .settings import RoboDrawerSettings
from .planner import plan_drawing


def _distance_mm(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


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

        # cache wyników SolveIK aby unikać powtarzających się, kosztownych wywołań
        self._ik_cache: dict[str, list[float]] = {}
        # zbiór póz dla których już raz zalogowano brak rozwiązania (ograniczenie spamu)
        self._ik_failure_logged: set[str] = set()

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

    def teach_sheet(self, output, frame_name: str, tool_name: str, approach_dist: int) -> None:
        """Trzy dotknięcia pisakiem: róg kartki, plus X, plus Y. Zapisuje JSON i układy w RoboDK."""
        tool = self._ensure_tool(tool_name)
        self._robot.setTool(tool)
        print("Aktywna końcówka to czubek pisaka. Dojazd rób tym czubkiem.")
        origin = self._teach_point("1/3  Róg kartki, początek XY. Przesuń robota i naciśnij Enter.")
        x_point = self._teach_point(
            "2/3  Koniec dolnej krawędzi, to jest szerokość kartki. Przesuń robota i naciśnij Enter.",
            (origin,),
        )
        y_point = self._teach_point(
            "3/3  Koniec lewej krawędzi, to jest wysokość kartki. Przesuń robota i naciśnij Enter.",
            (origin, x_point),
        )

        rows = [[round(value, 4) for value in row] for row in frame_matrix(origin, x_point, y_point)]
        width, height, y_sign = sheet_extents(origin, x_point, y_point)
        frame = self._ensure_frame(frame_name)
        frame.setPoseAbs(rows_to_pose(rows))
        tool_pose = self._robot.PoseTool()
        tool.setPoseTool(tool_pose)

        sheet = {
            "version": 1,
            "robot_name": self._robot.Name(),
            "frame_item_name": frame_name,
            "tool_item_name": tool_name,
            "approach_dist": approach_dist,
            "frame_pose": rows,
            "frame_xyzrpw": pose_to_xyzrpw(rows_to_pose(rows)),
            "tool_pose_xyzrpw": pose_to_xyzrpw(tool_pose),
            "points_mm": {
                "origin": [round(value, 3) for value in origin],
                "x": [round(value, 3) for value in x_point],
                "y": [round(value, 3) for value in y_point],
            },
            "sheet_mm": {"width": round(width, 3), "height": round(height, 3)},
            "y_sign": y_sign,
        }
        save_sheet(output, sheet)
        print(f"Kartka {width:.1f} x {height:.1f} mm. Układ {frame_name} i pisak {tool_name} zapisane w {output}")

    def apply_sheet(self, sheet: dict) -> None:
        """Wstaw zapisany układ kartki i końcówkę pisaka do otwartej stacji."""
        frame = self._ensure_frame(sheet["frame_item_name"])
        frame.setPoseAbs(rows_to_pose(sheet["frame_pose"]))
        tool = self._ensure_tool(sheet["tool_item_name"])
        tool.setPoseTool(robomath.xyzrpw_2_pose(sheet["tool_pose_xyzrpw"]))
        self._robot.setTool(tool)
        self._robot.setPoseFrame(frame)
        self._logger.info(
            f"Wczytano setup: układ {sheet['frame_item_name']}, pisak {sheet['tool_item_name']}."
        )

    def _teach_point(
        self,
        prompt: str,
        blocked: tuple[tuple[float, float, float], ...] = (),
    ) -> tuple[float, float, float]:
        while True:
            input(prompt + " ")
            point = self._tcp_in_station()
            if any(_distance_mm(point, earlier) < 1.0 for earlier in blocked):
                print("   To ten sam punkt. Przesuń robota w symulatorze i naciśnij Enter jeszcze raz.")
                continue
            print(f"   {point[0]:.1f}, {point[1]:.1f}, {point[2]:.1f} mm")
            return point

    def _tcp_in_station(self) -> tuple[float, float, float]:
        """Czubek aktywnego narzędzia w układzie stacji. PoseAbs robota to tylko baza."""
        pose = self._robot.PoseAbs() * self._robot.PoseFrame() * self._robot.Pose()
        return (float(pose[0, 3]), float(pose[1, 3]), float(pose[2, 3]))

    def _ensure_frame(self, name: str):
        item = self._RDK.Item(name, robolink.ITEM_TYPE_FRAME)
        if item.Valid():
            return item
        return self._RDK.AddFrame(name)

    def _ensure_tool(self, name: str):
        item = self._RDK.Item(name, robolink.ITEM_TYPE_TOOL)
        if item.Valid():
            return item
        return self._robot.AddTool(robomath.eye(4), name)

    def draw(self, draw_options: DrawOptions) -> None:
        if not draw_options.svg_path.exists():
            self._logger.error(f"SVG file not found: {draw_options.svg_path}")
            raise FileNotFoundError(f"SVG file not found: {draw_options.svg_path}")

        self._init_drawing_items(draw_options)

        self._logger.info(f"Starting drawing process for SVG: {draw_options.svg_path}")

        paths, _, _ = self._load_svg(str(draw_options.svg_path))
        self._draw_paths(paths, draw_options)

    def draw_paths(self, paths, draw_options: DrawOptions) -> None:
        """Rysuje gotowe ścieżki. Przy fit_to_sheet=False współrzędne są milimetrami układu kartki."""
        self._init_drawing_items(draw_options)
        self._draw_paths(paths, draw_options)

    def _prepare_paths(self, paths, draw_options: DrawOptions):
        scale = draw_options.scale
        margin = 0.0
        y_sign = 1.0
        if (
            draw_options.fit_to_sheet
            and draw_options.sheet_width_mm is not None
            and draw_options.sheet_height_mm is not None
        ):
            paths, scale, margin, svg_w, svg_h = fit_paths(
                paths,
                draw_options.sheet_width_mm,
                draw_options.sheet_height_mm,
                draw_options.requested_scale,
            )
            y_sign = draw_options.sheet_y_sign
            self._logger.info(
                f"Kartka {draw_options.sheet_width_mm:.1f} x {draw_options.sheet_height_mm:.1f} mm, "
                f"rysunek {svg_w:.1f} x {svg_h:.1f}, skala {scale:.4f}."
            )
        elif draw_options.sheet_width_mm is not None:
            y_sign = draw_options.sheet_y_sign
        return paths, scale, margin, y_sign

    def _draw_paths(self, paths, draw_options: DrawOptions) -> None:
        paths, scale, margin, y_sign = self._prepare_paths(paths, draw_options)
        self._logger.info("Planning motions...")
        plan = plan_drawing(paths, scale=scale, resolution=draw_options.resolution)
        counts = ", ".join(f"{kind}={count}" for kind, count in sorted(plan.counts().items()))
        self._logger.info(
            f"Plan: {len(plan.strokes)} strokes ({counts}), {len(plan.motions)} motions."
        )

        self._execute_plan(plan, draw_options, scale, margin, y_sign)

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

        split_paths = []
        split_attributes = []
        for path, attr in zip(paths, attributes):
            for subpath in self._split_discontinuous_path(path):
                split_paths.append(subpath)
                split_attributes.append(attr)

        self._logger.info(
            f"Loaded {len(paths)} paths from SVG (split into {len(split_paths)} continuous segments)."
        )

        return split_paths, split_attributes, rest

    def _split_discontinuous_path(self, path: svgpathtools.Path) -> list[svgpathtools.Path]:
        """Split a path where a later segment does not start at the previous segment's end."""
        if not path:
            return []

        subpaths = []
        current = svgpathtools.Path()
        current.append(path[0])

        for i in range(1, len(path)):
            if abs(path[i - 1].end - path[i].start) > 1e-5:
                subpaths.append(current)
                current = svgpathtools.Path()
            current.append(path[i])

        subpaths.append(current)
        return subpaths


    def _execute_plan(
        self,
        plan,
        draw_options: DrawOptions,
        scale: float | None = None,
        margin: float = 0.0,
        y_sign: float = 1.0,
    ) -> None:
        """Wykonaj plan: nad kartką MoveL na stałej wysokości, na papierze MoveL albo MoveC."""
        self._robot.setPoseFrame(self._frame)
        orient_tool = robomath.rotx(180 * robomath.pi / 180)
        if scale is None:
            scale = draw_options.scale

        self._logger.info("Moving robot to home position.")
        self._go_home()

        pen_down = False
        last_xy = None
        fast = None
        # Dodatnie Z układu kartki jest nad papierem. Starsze -10 wjeżdżało w kartkę.
        clearance = abs(float(draw_options.approach_dist))

        def pace(travel: bool) -> None:
            nonlocal fast
            if fast == travel:
                return
            fast = travel
            if travel:
                self._robot.setRounding(0)
                self._robot.setSpeed(500, 180, 2500, 800)
            else:
                self._robot.setRounding(1.0)
                self._robot.setSpeed(150, 90, 1000, 400)

        for motion in tqdm(plan.motions, desc="Drawing"):
            if motion.kind == "travel":
                self._robot.WaitMove()
                pace(True)
                if pen_down and last_xy is not None:
                    self._move_linear(self._pen_pose(last_xy, clearance, scale, orient_tool, margin, y_sign))
                self._move_linear(self._pen_pose(motion.target, clearance, scale, orient_tool, margin, y_sign))
                self._move_linear(self._pen_pose(motion.target, 0, scale, orient_tool, margin, y_sign))
                pen_down = True
                last_xy = motion.target
            elif motion.kind == "line":
                pace(False)
                pose = self._pen_pose(motion.target, 0, scale, orient_tool, margin, y_sign)
                self._move_linear(pose, blocking=False)
                self._paint(pose, draw_options)
                last_xy = motion.target
            elif motion.kind == "arc":
                pace(False)
                via = self._pen_pose(motion.via, 0, scale, orient_tool, margin, y_sign)
                end = self._pen_pose(motion.target, 0, scale, orient_tool, margin, y_sign)
                try:
                    self._robot.MoveC(via, end, blocking=False)
                except Exception as exc:
                    self._logger.warning(f"MoveC failed ({exc}); drawing the arc as two lines.")
                    self._move_linear(via, blocking=False)
                    self._move_linear(end, blocking=False)
                self._paint(end, draw_options)
                last_xy = motion.target
            elif motion.kind == "retract":
                self._robot.WaitMove()
                pace(True)
                if pen_down and last_xy is not None:
                    self._move_linear(self._pen_pose(last_xy, clearance, scale, orient_tool, margin, y_sign))
                pen_down = False

        self._robot.WaitMove()

        self._logger.info("Returning robot to home position.")
        self._go_home()
        self._logger.info("SVG drawing routine completed.")

    def _pen_pose(self, point: complex, z_mm: float, scale: float, orient_tool, margin: float = 0.0, y_sign: float = 1.0):
        x_mm = margin + point.real * scale
        y_mm = y_sign * (margin + point.imag * scale)
        return robomath.transl(x_mm, y_mm, z_mm) * orient_tool

    def _move_joints(self, pose, last_joints: list[float], frame_pose) -> list[float]:
        joints = self._solve_ik_stable(pose, last_joints, frame_pose)
        try:
            self._robot.MoveJ(joints, blocking=True)
        except Exception as exc:
            self._logger.warning(f"MoveJ failed ({exc}); trying MoveL.")
            self._move_linear(pose)
        return joints

    def _move_linear(self, pose, blocking: bool = True) -> None:
        self._robot.MoveL(pose, blocking=blocking)

    def _paint(self, pose, draw_options: DrawOptions) -> None:
        if draw_options.use_visual_simulation and self._board and self._pixel:
            self._board.AddGeometry(self._pixel, pose)


    def _solve_ik_stable(self, pose, last_joints: list[float], frame_pose, max_attempts: int = 5) -> list[float]:
        """
        Stabilne rozwiązanie IK z cache, rozszerzonym zestawem przybliżeń i ograniczeniem warningów.
        """
        try:
            pose_key = str(pose)
        except Exception:
            pose_key = repr(pose)

        if pose_key in self._ik_cache:
            return list(self._ik_cache[pose_key])

        best_joints = None
        best_score = float('inf')

        candidates = []
        if last_joints:
            candidates.append(list(last_joints))
        candidates.append([0.0, -70.0, -95.0, -104.0, 90.0, -20.0])
        candidates.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        if last_joints:
            for delta in (5.0, -5.0):
                candidates.append([j + delta for j in last_joints])
        candidates = candidates[:max_attempts]

        for attempt_idx, joints_approx in enumerate(candidates):
            try:
                ik_result = self._robot.SolveIK(pose, joints_approx=joints_approx, reference=frame_pose)
                if not ik_result:
                    continue
                joints = ik_result.list()
                if not joints:
                    continue

                if self._is_solution_valid(joints):
                    score = self._joint_distance(last_joints, joints)
                    try:
                        if abs(joints[0] - last_joints[0]) > 150:
                            score += 1000.0
                    except Exception:
                        pass

                    if score < best_score:
                        best_score = score
                        best_joints = joints
            except Exception as e:
                self._logger.debug(f"IK attempt {attempt_idx + 1} failed: {e}")
                continue

        if best_joints is not None:
            try:
                self._ik_cache[pose_key] = list(best_joints)
            except Exception:
                pass
            self._logger.debug(f"Found stable IK solution with score {best_score}")
            return best_joints
        else:
            if pose_key not in self._ik_failure_logged:
                self._logger.warning("No valid IK solution found after multiple attempts; using last known joints")
                self._ik_failure_logged.add(pose_key)
            try:
                self._ik_cache[pose_key] = list(last_joints)
            except Exception:
                pass
            return last_joints

    def _is_solution_valid(self, joints: list[float]) -> bool:
        """Sprawdza czy rozwiązanie IK jest w dopuszczalnych granicach i realistyczne."""
        joint_limits = [
            (-170, 170),
            (-180, 180),
            (-180, 180),
            (-180, 180),
            (-180, 180),
            (-180, 180)
        ]

        for i, (j, limits) in enumerate(zip(joints, joint_limits)):
            if j < limits[0] or j > limits[1]:
                self._logger.debug(f"Joint {i+1} out of limits: {j} not in {limits}")
                return False

        try:
            if abs(joints[1]) < 20 and joints[2] > -20:
                return False
        except Exception:
            pass

        return True

    def _joint_distance(self, joints1: list[float], joints2: list[float]) -> float:
        """Oblicza 'odległość' między konfiguracjami jointów, uwzględnia okresowość 360°. Wersja numpy."""
        try:
            a = np.asarray(joints1, dtype=float)
            b = np.asarray(joints2, dtype=float)
            if a.size == 0 or b.size == 0 or a.shape != b.shape:
                return float("inf")
            diff = np.abs(a - b)
            diff = np.minimum(diff, 360.0 - diff)
            return float(np.linalg.norm(diff))
        except Exception:
            return float("inf")

    def _go_home(self) -> None:
        """Przejście do pozycji domowej (bezpieczne MoveJ z obsługą wyjątków)."""
        custom_home = [0, -70, -95, -104, 90, -20]
        self._logger.debug(f"Moving to custom home position: {custom_home}")
        try:
            self._robot.MoveJ(custom_home, blocking=True)
        except Exception as e:
            self._logger.exception(f"Failed to MoveJ to home: {e}")

