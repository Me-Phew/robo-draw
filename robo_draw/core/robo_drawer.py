import svgpathtools
from mephew_python_commons import LoggerFactory
from robodk import robolink, robomath
from tqdm import tqdm
import math
from typing import Optional, Tuple, List
import numpy as np

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
                    joints_start = self._solve_ik_stable(pose_start, current_joints, frame_pose)

                    if joints_start:
                        dist_start = self._joint_distance(current_joints, joints_start)

                        if dist_start < best_dist:
                            best_dist = dist_start
                            best_idx = i
                            should_reverse = False
                except Exception:
                    pass

                # Check end (reverse)
                try:
                    pose_end = robomath.transl(p.end.real * draw_options.scale, p.end.imag * draw_options.scale, 0) * orient_tool
                    joints_end = self._solve_ik_stable(pose_end, current_joints, frame_pose)

                    if joints_end:
                        dist_end = self._joint_distance(current_joints, joints_end)

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
                joints_move_start = self._solve_ik_stable(pose_move_start, current_joints, frame_pose)

                if joints_move_start:
                    pose_move_end = robomath.transl(p.start.real * draw_options.scale, p.start.imag * draw_options.scale, 0) * orient_tool
                    joints_move_end = self._solve_ik_stable(pose_move_end, joints_move_start, frame_pose)
                    if joints_move_end:
                        current_joints = joints_move_end
                
                try:
                    p = p.reversed()
                except Exception:
                    pass
            else:
                pose_move_start = robomath.transl(p.start.real * draw_options.scale, p.start.imag * draw_options.scale, 0) * orient_tool
                joints_move_start = self._solve_ik_stable(pose_move_start, current_joints, frame_pose)

                if joints_move_start:
                    pose_move_end = robomath.transl(p.end.real * draw_options.scale, p.end.imag * draw_options.scale, 0) * orient_tool
                    joints_move_end = self._solve_ik_stable(pose_move_end, joints_move_start, frame_pose)
                    if joints_move_end:
                        current_joints = joints_move_end

            ordered_paths.append(p)
            ordered_attributes.append(attr)

        return ordered_paths, ordered_attributes

    def _collect_arc_sequence(self, path, start_idx: int, draw_options: DrawOptions):
        """
        Zbiera kolejne segmenty zaczynając od start_idx, które można traktować jako ciągły łuk.
        Zwraca (end_idx_exclusive, segments_list).
        """
        seq = []
        continuity_tol = 1.0 / draw_options.scale  # w jednostkach SVG przed skalowaniem
        center_tol = 1.0 / draw_options.scale

        if start_idx >= len(path):
            return start_idx, []

        seg0 = path[start_idx]
        is_arc0 = isinstance(seg0, svgpathtools.Arc) or self._is_bezier_arc(seg0, tolerance_mm=1.0 / draw_options.scale)
        if not is_arc0:
            return start_idx, []

        fit0 = self._fit_circle_from_three_points(seg0.start, seg0.point(0.5), seg0.end)
        if fit0 is None:
            return start_idx, []
        
        base_center, base_radius = fit0
        seq.append(seg0)
        prev_end = seg0.end

        for i in range(start_idx + 1, len(path)):
            seg = path[i]
            is_arc = isinstance(seg, svgpathtools.Arc) or self._is_bezier_arc(seg, tolerance_mm=1.0 / draw_options.scale)
            if not is_arc:
                break

            start = seg.start
            end = seg.end
            mid = seg.point(0.5)

            # continuity check with previous
            d = abs(start - prev_end)
            if d > continuity_tol:
                break

            # center/radius check
            fit = self._fit_circle_from_three_points(start, mid, end)
            if fit is not None:
                center, radius = fit
                if abs(center - base_center) > center_tol or abs(radius - base_radius) > center_tol:
                    break
            else:
                break

            seq.append(seg)
            prev_end = end

        end_idx = start_idx + len(seq)
        return end_idx, seq

    def _merge_continuous_arcs(self, paths: list[svgpathtools.Path], draw_options: DrawOptions) -> list[svgpathtools.Path]:
        """
        Łączy ciągłe łuki w pojedyncze ścieżki, aby można je było narysować jednym ruchem.
        """
        if not paths:
            return paths

        merged_paths = []
        current_arc_group = []

        def is_arc_continuous(prev_arc, next_arc, tolerance=0.1):
            """Sprawdza czy łuki są ciągłe i współśrodkowe."""
            if prev_arc is None or next_arc is None:
                return False

            # Sprawdź ciągłość geometryczną
            end_to_start_dist = abs(prev_arc.end - next_arc.start)
            if end_to_start_dist > tolerance:
                return False

            # Dla Arc - sprawdź czy mają te same parametry (prosty warunek)
            if isinstance(prev_arc, svgpathtools.Arc) and isinstance(next_arc, svgpathtools.Arc):
                try:
                    return (prev_arc.radius == next_arc.radius and 
                            prev_arc.rotation == next_arc.rotation and
                            prev_arc.large_arc == next_arc.large_arc and
                            prev_arc.sweep == next_arc.sweep)
                except Exception:
                    return False

            # Dla Bézier-łuków - sprawdź dopasowanie do tego samego okręgu
            if self._is_bezier_arc(prev_arc) and self._is_bezier_arc(next_arc):
                prev_fit = self._fit_circle_from_three_points(prev_arc.start, prev_arc.point(0.5), prev_arc.end)
                next_fit = self._fit_circle_from_three_points(next_arc.start, next_arc.point(0.5), next_arc.end)

                if prev_fit and next_fit:
                    prev_center, prev_radius = prev_fit
                    next_center, next_radius = next_fit
                    center_tol = 1.0 / draw_options.scale
                    radius_tol = 1.0 / draw_options.scale

                    return (abs(prev_center - next_center) < center_tol and 
                            abs(prev_radius - next_radius) < radius_tol)

            return False

        def create_merged_arc_path(arc_group):
            """Tworzy pojedynczą ścieżkę z grupy ciągłych łuków."""
            if not arc_group:
                return None

            merged_segments = []
            for item in arc_group:
                if isinstance(item, svgpathtools.Path):
                    for s in item:
                        merged_segments.append(s)
                else:
                    merged_segments.append(item)

            if not merged_segments:
                return None
            return svgpathtools.Path(*merged_segments)

        for path in paths:
            if len(path) == 1 and (isinstance(path[0], svgpathtools.Arc) or self._is_bezier_arc(path[0], tolerance_mm=1.0 / draw_options.scale)):
                seg = path[0]
                if current_arc_group:
                    last_seg = current_arc_group[-1][-1] if isinstance(current_arc_group[-1], svgpathtools.Path) else current_arc_group[-1]
                    if is_arc_continuous(last_seg, seg):
                        current_arc_group.append(path)
                        continue

                if current_arc_group:
                    merged = create_merged_arc_path([p[0] if isinstance(p, svgpathtools.Path) else p for p in current_arc_group])
                    if merged:
                        merged_paths.append(merged)
                    current_arc_group = []

                current_arc_group.append(path)
            else:
                if current_arc_group:
                    merged = create_merged_arc_path([p[0] if isinstance(p, svgpathtools.Path) else p for p in current_arc_group])
                    if merged:
                        merged_paths.append(merged)
                    current_arc_group = []

                merged_paths.append(path)

        if current_arc_group:
            merged = create_merged_arc_path([p[0] if isinstance(p, svgpathtools.Path) else p for p in current_arc_group])
            if merged:
                merged_paths.append(merged)

        self._logger.info(f"Połączono łuki: {len(paths)} -> {len(merged_paths)} ścieżek")
        return merged_paths

    def _plan_arc_sequence(self, seq, last_joints, frame_pose, orient_tool, draw_options):
        """
        Zoptymalizowana wersja planowania sekwencji łuków z lepszą detekcją ciągłości.
        """
        if not seq:
            return None, None, None

        if len(seq) == 1:
            seg = seq[0]
            start = seg.start
            via = seg.point(0.5)
            end = seg.end

            p_start = robomath.transl(start.real * draw_options.scale, start.imag * draw_options.scale, 0) * orient_tool
            p_via = robomath.transl(via.real * draw_options.scale, via.imag * draw_options.scale, 0) * orient_tool
            p_end = robomath.transl(end.real * draw_options.scale, end.imag * draw_options.scale, 0) * orient_tool

            j_start = self._solve_ik_stable(p_start, last_joints, frame_pose)
            if not j_start:
                return None, None, None

            j_via = self._solve_ik_stable(p_via, j_start, frame_pose)
            j_end = self._solve_ik_stable(p_end, j_via if j_via else j_start, frame_pose)

            if j_via and j_end:
                if (self._are_joints_consistent(j_start, j_via, threshold=120.0) and 
                    self._are_joints_consistent(j_via, j_end, threshold=120.0)):

                    actions = []
                    actions.append(("MOVE", j_start))
                    actions.append(("MOVEC_J", (j_via, j_end)))

                    last_end_point = type("P",(object,),{"x":end.real,"y":end.imag})()
                    return actions, j_end, last_end_point

        p_start_svg = seq[0].start
        p_end_svg = seq[-1].end

        total_len = sum(s.length() for s in seq)
        mid_len = total_len / 2.0

        cur_len = 0.0
        p_via_svg = seq[0].point(0.5)

        for s in seq:
            l = s.length()
            if cur_len + l >= mid_len:
                remain = mid_len - cur_len
                t = remain / l if l > 1e-9 else 0.5
                p_via_svg = s.point(t)
                break
            cur_len += l

        def to_pose(p):
            return robomath.transl(p.real * draw_options.scale, p.imag * draw_options.scale, 0) * orient_tool

        pose_start = to_pose(p_start_svg)
        pose_via = to_pose(p_via_svg)
        pose_end = to_pose(p_end_svg)

        j_start = self._solve_ik_stable(pose_start, last_joints, frame_pose)
        if not j_start:
            return None, None, None

        j_via = self._solve_ik_stable(pose_via, j_start, frame_pose)
        if not j_via:
            return None, None, None

        j_end = self._solve_ik_stable(pose_end, j_via, frame_pose)
        if not j_end:
            return None, None, None

        if not (self._are_joints_consistent(j_start, j_via, threshold=150.0) and 
                self._are_joints_consistent(j_via, j_end, threshold=150.0)):
            return None, None, None

        actions = []
        actions.append(("MOVE", j_start))
        actions.append(("MOVEC_J", (j_via, j_end)))

        last_end_point = type("P",(object,),{"x":p_end_svg.real,"y":p_end_svg.imag})()
        return actions, j_end, last_end_point

    def _is_bezier_line(self, seg, tolerance_mm: float = 0.5) -> bool:
        """Sprawdza czy Bézier jest praktycznie linią (maksymalna odchyłka od odcinka start-end)."""
        typ_name = seg.__class__.__name__
        if typ_name not in ("CubicBezier", "QuadraticBezier"):
            return False

        start = seg.start
        end = seg.end
        ts = np.array([0.0, 0.25, 0.5, 0.75, 1.0], dtype=float)
        pts = np.array([seg.point(t) for t in ts])
        v = end - start
        v_len = abs(v)
        if v_len < 1e-9:
            return False

        def point_line_dist(p):
            return abs(np.imag(np.conj(p - start) * v)) / v_len

        dists = np.array([point_line_dist(p) for p in pts])
        max_dev = float(np.max(dists))

        return max_dev <= tolerance_mm

    def _collect_linear_sequence(self, path, start_idx: int, draw_options: DrawOptions):
        """
        Zbiera kolejne segmenty liniowe (Line lub Bézier bliski linii) zaczynając od start_idx.
        Zwraca (end_idx_exclusive, segments_list).
        """
        seq = []
        continuity_tol = 1.0 / draw_options.scale

        if start_idx >= len(path):
            return start_idx, []

        seg0 = path[start_idx]
        is_line0 = isinstance(seg0, svgpathtools.Line) or self._is_bezier_line(seg0, tolerance_mm=1.0 / draw_options.scale)
        if not is_line0:
            return start_idx, []

        seq.append(seg0)
        prev_end = seg0.end

        for i in range(start_idx + 1, len(path)):
            seg = path[i]
            is_line = isinstance(seg, svgpathtools.Line) or self._is_bezier_line(seg, tolerance_mm=1.0 / draw_options.scale)
            if not is_line:
                break

            start = seg.start
            if abs(start - prev_end) > continuity_tol:
                break

            seq.append(seg)
            prev_end = seg.end

        end_idx = start_idx + len(seq)
        return end_idx, seq

    def _draw_svg(
        self, draw_options: DrawOptions, paths: list[svgpathtools.Path], attributes: list[dict[str, str]]
    ) -> None:
        self._logger.info("Starting SVG drawing routine.")

        self._logger.debug(f"Drawing with options: {draw_options}")

        self._robot.setPoseFrame(self._frame)
        self._logger.debug(f"Set robot frame to '{self._frame.Name()}'.")

        orient_tool = robomath.rotx(180 * robomath.pi / 180)
        self._logger.debug(f"Calculated tool orientation matrix: {orient_tool}")

        frame_pose = self._frame.Pose()

        total_paths = len(paths)
        self._logger.info(f"Total paths to draw: {total_paths}")

        # OPTYMALIZACJA: Połącz ciągłe łuki przed rozpoczęciem rysowania
        self._logger.info("Merging continuous arcs...")
        paths = self._merge_continuous_arcs(paths, draw_options)
        self._logger.info("Arc merging completed.")

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

        for i, (path, attr) in enumerate(zip(tqdm(paths, desc="Planning"), attributes)):
            # Sprawdź czy ścieżka zawiera tylko segmenty liniowe (bez łuków)
            is_purely_linear = True
            for seg in path:
                if isinstance(seg, svgpathtools.Arc) or self._is_bezier_arc(seg, tolerance_mm=1.0 / draw_options.scale):
                    is_purely_linear = False
                    break
                if not (isinstance(seg, svgpathtools.Line) or self._is_bezier_line(seg, tolerance_mm=1.0 / draw_options.scale)):
                    is_purely_linear = False
                    break

            if is_purely_linear:
                # Uproszczona obsługa ścieżek liniowych (z drugiego kodu)
                points_2d = get_points_from_path(path, step_mm=draw_options.resolution / draw_options.scale)

                if not points_2d:
                    self._logger.warning(f"Path {i+1} has no points after discretization; skipping.")
                    continue

                original_count = len(points_2d)
                epsilon = 0.5 / draw_options.scale
                points_2d = simplify_points(points_2d, epsilon)
                self._logger.info(f"Path {i+1} simplified from {original_count} to {len(points_2d)} points.")

                p0 = points_2d[0]
                is_continuous = False
                if last_end_point is not None:
                    dist = ((p0.x - last_end_point.x) ** 2 + (p0.y - last_end_point.y) ** 2) ** 0.5
                    if dist < 1.0:
                        is_continuous = True

                target_pose = robomath.transl(p0.x * draw_options.scale, p0.y * draw_options.scale, 0) * orient_tool
                approach_pose = target_pose * robomath.transl(0, 0, draw_options.approach_dist)

                if not is_continuous:
                    if pending_retract_pose is not None:
                        self._logger.info(f"Retracting after completing previous path.")
                        last_joints = self._solve_ik_stable(pending_retract_pose, last_joints, frame_pose)
                        execution_queue.append(("MOVE", last_joints))
                        pending_retract_pose = None

                    try:
                        self._logger.info(f"Moving to approach pose for path {i+1}.")
                        last_joints = self._solve_ik_stable(approach_pose, last_joints, frame_pose)
                        execution_queue.append(("MOVE", last_joints))
                    except Exception as e:
                        self._logger.exception(f"Robot cannot reach start of path {i+1}. Exception: {e}")
                        raise

                self._logger.info(f"Moving down to target pose for path {i+1}.")
                last_joints = self._solve_ik_stable(target_pose, last_joints, frame_pose)
                execution_queue.append(("MOVE", last_joints))

                for p in points_2d[1:]:
                    target_pose = robomath.transl(p.x * draw_options.scale, p.y * draw_options.scale, 0) * orient_tool
                    last_joints = self._solve_ik_stable(target_pose, last_joints, frame_pose)
                    execution_queue.append(("MOVE", last_joints))
                    if draw_options.use_visual_simulation:
                        execution_queue.append(("DRAW", target_pose))

                last_end_point = points_2d[-1]
                end_pose = robomath.transl(last_end_point.x * draw_options.scale, last_end_point.y * draw_options.scale, 0) * orient_tool
                pending_retract_pose = end_pose * robomath.transl(0, 0, draw_options.approach_dist)

            else:
                # Zaawansowana obsługa ścieżek z łukami (z pierwszego kodu)
                idx = 0
                while idx < len(path):
                    seg = path[idx]

                    if isinstance(seg, svgpathtools.Arc) or self._is_bezier_arc(seg, tolerance_mm=1.0 / draw_options.scale):
                        end_idx, seq = self._collect_arc_sequence(path, idx, draw_options)
                        if len(seq) >= 1:
                            planned, new_last_joints, new_last_end_point = self._plan_arc_sequence(seq, last_joints, frame_pose, orient_tool, draw_options)
                            if planned:
                                for act in planned:
                                    execution_queue.append(act)
                                last_joints = new_last_joints
                                last_end_point = new_last_end_point
                                pending_retract_pose = robomath.transl(new_last_end_point.x * draw_options.scale, 
                                                                     new_last_end_point.y * draw_options.scale, 0) * orient_tool * robomath.transl(0, 0, draw_options.approach_dist)
                                idx = end_idx
                                continue
                            else:
                                combined_path = svgpathtools.Path(*seq)
                                points_2d = get_points_from_path(combined_path, step_mm=draw_options.resolution / draw_options.scale)
                                if points_2d:
                                    original_count = len(points_2d)
                                    epsilon = 0.5 / draw_options.scale
                                    points_2d = simplify_points(points_2d, epsilon)
                                    self._logger.debug(f"Arc-to-line fallback: simplified from {original_count} to {len(points_2d)} points.")
                                    
                                    p0 = points_2d[0]
                                    is_continuous = False
                                    if last_end_point is not None:
                                        dist = ((p0.x - last_end_point.x) ** 2 + (p0.y - last_end_point.y) ** 2) ** 0.5
                                        if dist < 1.0:
                                            is_continuous = True

                                    target_pose = robomath.transl(p0.x * draw_options.scale, p0.y * draw_options.scale, 0) * orient_tool
                                    approach_pose = target_pose * robomath.transl(0, 0, draw_options.approach_dist)

                                    if not is_continuous:
                                        if pending_retract_pose is not None:
                                            self._logger.info("Retracting after previous segment.")
                                            last_joints = self._solve_ik_stable(pending_retract_pose, last_joints, frame_pose)
                                            execution_queue.append(("MOVE", last_joints))
                                            pending_retract_pose = None

                                        last_joints = self._solve_ik_stable(approach_pose, last_joints, frame_pose)
                                        execution_queue.append(("MOVE", last_joints))

                                    last_joints = self._solve_ik_stable(target_pose, last_joints, frame_pose)
                                    execution_queue.append(("MOVE", last_joints))

                                    for p in points_2d[1:]:
                                        target_pose = robomath.transl(p.x * draw_options.scale, p.y * draw_options.scale, 0) * orient_tool
                                        last_joints = self._solve_ik_stable(target_pose, last_joints, frame_pose)
                                        execution_queue.append(("MOVE", last_joints))
                                        if draw_options.use_visual_simulation:
                                            execution_queue.append(("DRAW", target_pose))

                                    last_end_point = points_2d[-1]
                                    end_pose = robomath.transl(last_end_point.x * draw_options.scale, last_end_point.y * draw_options.scale, 0) * orient_tool
                                    pending_retract_pose = end_pose * robomath.transl(0, 0, draw_options.approach_dist)
                                idx = end_idx
                                continue

                    if isinstance(seg, svgpathtools.Line) or self._is_bezier_line(seg, tolerance_mm=1.0 / draw_options.scale):
                        end_idx, lin_seq = self._collect_linear_sequence(path, idx, draw_options)
                        if len(lin_seq) >= 1:
                            combined_path = svgpathtools.Path(*lin_seq)
                            points_2d = get_points_from_path(combined_path, step_mm=draw_options.resolution / draw_options.scale)
                            if points_2d:
                                original_count = len(points_2d)
                                epsilon = 0.5 / draw_options.scale
                                points_2d = simplify_points(points_2d, epsilon)
                                self._logger.debug(f"Linear sequence simplified from {original_count} to {len(points_2d)} points.")

                                p0 = points_2d[0]
                                is_continuous = False
                                if last_end_point is not None:
                                    dist = ((p0.x - last_end_point.x) ** 2 + (p0.y - last_end_point.y) ** 2) ** 0.5
                                    if dist < 1.0:
                                        is_continuous = True

                                target_pose = robomath.transl(p0.x * draw_options.scale, p0.y * draw_options.scale, 0) * orient_tool
                                approach_pose = target_pose * robomath.transl(0, 0, draw_options.approach_dist)

                                if not is_continuous:
                                    if pending_retract_pose is not None:
                                        self._logger.info("Retracting after previous segment.")
                                        last_joints = self._solve_ik_stable(pending_retract_pose, last_joints, frame_pose)
                                        execution_queue.append(("MOVE", last_joints))
                                        pending_retract_pose = None

                                    last_joints = self._solve_ik_stable(approach_pose, last_joints, frame_pose)
                                    execution_queue.append(("MOVE", last_joints))

                                last_joints = self._solve_ik_stable(target_pose, last_joints, frame_pose)
                                execution_queue.append(("MOVE", last_joints))

                                for p in points_2d[1:]:
                                    target_pose = robomath.transl(p.x * draw_options.scale, p.y * draw_options.scale, 0) * orient_tool
                                    last_joints = self._solve_ik_stable(target_pose, last_joints, frame_pose)
                                    execution_queue.append(("MOVE", last_joints))
                                    if draw_options.use_visual_simulation:
                                        execution_queue.append(("DRAW", target_pose))

                                last_end_point = points_2d[-1]
                                end_pose = robomath.transl(last_end_point.x * draw_options.scale, last_end_point.y * draw_options.scale, 0) * orient_tool
                                pending_retract_pose = end_pose * robomath.transl(0, 0, draw_options.approach_dist)

                                idx = end_idx
                                continue

                    # Fallback do oryginalnej logiki dla pozostałych pojedynczych segmentów
                    seg_path = svgpathtools.Path(seg)
                    points_2d = get_points_from_path(seg_path, step_mm=draw_options.resolution / draw_options.scale)

                    if not points_2d:
                        self._logger.debug("Segment has no points after discretization; skipping.")
                        idx += 1
                        continue

                    original_count = len(points_2d)
                    epsilon = 0.5 / draw_options.scale
                    points_2d = simplify_points(points_2d, epsilon)
                    self._logger.debug(f"Segment simplified from {original_count} to {len(points_2d)} points.")

                    p0 = points_2d[0]
                    is_continuous = False
                    if last_end_point is not None:
                        dist = ((p0.x - last_end_point.x) ** 2 + (p0.y - last_end_point.y) ** 2) ** 0.5
                        if dist < 1.0:
                            is_continuous = True

                    target_pose = robomath.transl(p0.x * draw_options.scale, p0.y * draw_options.scale, 0) * orient_tool
                    approach_pose = target_pose * robomath.transl(0, 0, draw_options.approach_dist)

                    if not is_continuous:
                        if pending_retract_pose is not None:
                            self._logger.info("Retracting after previous segment.")
                            last_joints = self._solve_ik_stable(pending_retract_pose, last_joints, frame_pose)
                            execution_queue.append(("MOVE", last_joints))
                            pending_retract_pose = None

                        try:
                            last_joints = self._solve_ik_stable(approach_pose, last_joints, frame_pose)
                            execution_queue.append(("MOVE", last_joints))
                        except Exception as e:
                            self._logger.exception(f"Cannot reach segment start: {e}")
                            raise

                    last_joints = self._solve_ik_stable(target_pose, last_joints, frame_pose)
                    execution_queue.append(("MOVE", last_joints))

                    for p in points_2d[1:]:
                        target_pose = robomath.transl(p.x * draw_options.scale, p.y * draw_options.scale, 0) * orient_tool
                        last_joints = self._solve_ik_stable(target_pose, last_joints, frame_pose)
                        execution_queue.append(("MOVE", last_joints))
                        if draw_options.use_visual_simulation:
                            execution_queue.append(("DRAW", target_pose))

                    last_end_point = points_2d[-1]
                    end_pose = robomath.transl(last_end_point.x * draw_options.scale, last_end_point.y * draw_options.scale, 0) * orient_tool
                    pending_retract_pose = end_pose * robomath.transl(0, 0, draw_options.approach_dist)

                    idx += 1

        self._logger.info("All paths planned. Retracting drawing tool.")
        if pending_retract_pose is not None:
            last_joints = self._solve_ik_stable(pending_retract_pose, last_joints, frame_pose)
            execution_queue.append(("MOVE", last_joints))
        self._logger.info("Drawing tool retracted.")

        self._logger.info(f"Planning completed. {len(execution_queue)} actions queued.")

        # Precomputation i wykonanie kolejki
        fk_cache: dict[str, object] = {}
        resolved_queue = []

        for action, data in execution_queue:
            if action == "MOVE":
                joints = list(data)
                key = repr(joints)
                if key not in fk_cache:
                    try:
                        fk_cache[key] = self._robot.SolveFK(joints)
                    except Exception as e:
                        self._logger.debug(f"SolveFK failed for MOVE joints {joints}: {e}")
                        fk_cache[key] = None
                resolved_queue.append(("MOVE", joints, fk_cache[key]))
            elif action == "DRAW":
                resolved_queue.append(("DRAW", data))
            elif action == "MOVEC_J":
                j_via, j_end = data
                key_v = repr(list(j_via))
                key_e = repr(list(j_end))
                if key_v not in fk_cache:
                    try:
                        fk_cache[key_v] = self._robot.SolveFK(j_via)
                    except Exception as e:
                        self._logger.debug(f"SolveFK failed for MOVEC via {j_via}: {e}")
                        fk_cache[key_v] = None
                if key_e not in fk_cache:
                    try:
                        fk_cache[key_e] = self._robot.SolveFK(j_end)
                    except Exception as e:
                        self._logger.debug(f"SolveFK failed for MOVEC end {j_end}: {e}")
                        fk_cache[key_e] = None
                resolved_queue.append(("MOVEC_J", (j_via, j_end, fk_cache[key_v], fk_cache[key_e])))
            else:
                resolved_queue.append((action, data))

        self._logger.info("Precomputation completed. Executing precomputed queue...")

        for item in tqdm(resolved_queue, desc="Executing"):
            action = item[0]
            if action == "MOVE":
                _, joints, fk_pose = item
                try:
                    self._robot.MoveJ(joints, blocking=True)
                except Exception as e:
                    self._logger.warning(f"MoveJ failed: {e}, trying precomputed MoveL fallback")
                    try:
                        if fk_pose is not None:
                            self._robot.MoveL(fk_pose, blocking=True)
                        else:
                            self._logger.debug("No precomputed FK pose available; trying SolveFK on-the-fly")
                            fk_try = None
                            try:
                                fk_try = self._robot.SolveFK(joints)
                            except Exception as e2:
                                self._logger.error(f"SolveFK also failed: {e2}")
                            if fk_try is not None:
                                self._robot.MoveL(fk_try, blocking=True)
                    except Exception as e2:
                        self._logger.error(f"Both MoveJ and MoveL failed: {e2}")
            elif action == "DRAW":
                _, pose = item
                if self._board and self._pixel:
                    self._board.AddGeometry(self._pixel, pose)
            elif action == "MOVEC_J":
                _, (j_via, j_end, fk_via, fk_end) = item
                try:
                    if self._are_joints_consistent_for_movement(j_via, j_end):
                        try:
                            self._robot.MoveC(j_via, j_end, blocking=True)
                        except Exception:
                            if fk_via is not None and fk_end is not None:
                                self._robot.MoveC(fk_via, fk_end, blocking=True)
                            else:
                                self._discretize_arc_fallback(j_via, j_end)
                    else:
                        self._logger.warning("MOVEC joints inconsistent, using discretization fallback")
                        self._discretize_arc_fallback(j_via, j_end)
                except Exception as e:
                    self._logger.exception(f"MOVEC_J failed: {e}")
                    self._discretize_arc_fallback(j_via, j_end)
            else:
                self._logger.debug(f"Unknown action in resolved queue: {action}")

        self._logger.info("Returning robot to home position.")
        self._go_home()
        self._logger.info("Robot returned to home position.")

        self._logger.info("SVG drawing routine completed.")

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

    def _are_joints_consistent(self, joints1: list[float], joints2: list[float], threshold: float = 90.0) -> bool:
        """Sprawdza czy dwie konfiguracje jointów są spójne (bez dużych skoków). Wersja numpy."""
        try:
            a = np.asarray(joints1, dtype=float)
            b = np.asarray(joints2, dtype=float)
            diff = np.abs(a - b)
            diff = np.minimum(diff, 360.0 - diff)
            if np.any(diff > threshold):
                self._logger.debug(f"Large joint change detected: {float(np.max(diff))} degrees")
                return False
            return True
        except Exception:
            return False

    def _are_joints_consistent_for_movement(self, joints1: list[float], joints2: list[float]) -> bool:
        """Bardziej restrykcyjny test spójności dla MOVEC (lista max zmian per joint). Wersja numpy."""
        try:
            a = np.asarray(joints1, dtype=float)
            b = np.asarray(joints2, dtype=float)
            max_allowed_changes = np.asarray([45, 45, 45, 90, 90, 90], dtype=float)
            diff = np.abs(a - b)
            diff = np.minimum(diff, 360.0 - diff)
            too_large = diff > max_allowed_changes
            if np.any(too_large):
                idx = int(np.argmax(too_large))
                self._logger.warning(f"Joint {idx+1} change too large for MOVEC: {float(diff[idx])} > {float(max_allowed_changes[idx])}")
                return False
            return True
        except Exception:
            return False

    def _discretize_arc_fallback(self, j_via: list[float], j_end: list[float]):
        """Fallback - dyskretyzacja łuku na mniejsze segmenty wykonywane jako MoveJ."""
        self._logger.info("Using discretization fallback for arc")
        try:
            j_v = np.asarray(j_via, dtype=float)
            j_e = np.asarray(j_end, dtype=float)
            steps = 5
            fs = np.linspace(1.0/steps, 1.0, steps)
            for f in fs:
                interp = (1.0 - f) * j_v + f * j_e
                interp_joints = interp.tolist()
                try:
                    self._robot.MoveJ(interp_joints, blocking=True)
                except Exception as e:
                    self._logger.exception(f"Fallback MoveJ step failed: {e}")
                    break
        except Exception as e:
            self._logger.error(f"Arc discretization fallback also failed: {e}")

    def _go_home(self) -> None:
        """Przejście do pozycji domowej (bezpieczne MoveJ z obsługą wyjątków)."""
        custom_home = [0, -70, -95, -104, 90, -20]
        self._logger.debug(f"Moving to custom home position: {custom_home}")
        try:
            self._robot.MoveJ(custom_home, blocking=True)
        except Exception as e:
            self._logger.exception(f"Failed to MoveJ to home: {e}")

    def _fit_circle_from_three_points(self, p1: complex, p2: complex, p3: complex) -> Optional[Tuple[complex, float]]:
        """Fit circle through three complex points. Uses numpy linear solve for stability."""
        x1, y1 = p1.real, p1.imag
        x2, y2 = p2.real, p2.imag
        x3, y3 = p3.real, p3.imag

        A = np.array([[x2 - x1, y2 - y1], [x3 - x1, y3 - y1]], dtype=float)
        b = 0.5 * np.array([x2**2 + y2**2 - x1**2 - y1**2, x3**2 + y3**2 - x1**2 - y1**2], dtype=float)

        try:
            if abs(np.linalg.det(A)) < 1e-12:
                return None
            ux, uy = np.linalg.solve(A, b)
            center = complex(ux, uy)
            radius = abs(center - p1)
            return center, float(radius)
        except Exception:
            return None

    def _is_bezier_arc(self, seg, tolerance_mm: float = 0.5) -> bool:
        """
        Sprawdza, czy segment Béziera można potraktować jako łuk okręgu.
        Wersja wektoryzowana próbkowania dla szybkości.
        """
        typ_name = seg.__class__.__name__
        if typ_name not in ("CubicBezier", "QuadraticBezier"):
            return False

        start = seg.start
        mid = seg.point(0.5)
        end = seg.end

        res = self._fit_circle_from_three_points(start, mid, end)
        if res is None:
            return False
        center, radius = res

        ts = np.array([0.0, 0.25, 0.5, 0.75, 1.0], dtype=float)
        pts = np.array([seg.point(t) for t in ts])
        dists = np.abs(np.abs(pts - center) - radius)
        max_dev = float(np.max(dists))

        return max_dev <= tolerance_mm