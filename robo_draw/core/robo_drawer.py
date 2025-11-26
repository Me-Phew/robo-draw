import svgpathtools
from mephew_python_commons import LoggerFactory
from robodk import robolink, robomath
from tqdm import tqdm

from .draw_options import DrawOptions
from .settings import RoboDrawerSettings
from .utils import get_points_from_path, simplify_points, Point


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

        # Split discontinuous paths (e.g. paths with 'M' commands inside)
        split_paths = []
        split_attributes = []

        for path, attr in zip(paths, attributes):
            subpaths = self._split_discontinuous_path(path)
            for subpath in subpaths:
                split_paths.append(subpath)
                split_attributes.append(attr)

        self._logger.info(f"Loaded {len(paths)} paths from SVG (split into {len(split_paths)} continuous segments).")

        return split_paths, split_attributes, rest

    def _split_discontinuous_path(self, path: svgpathtools.Path) -> list[svgpathtools.Path]:
        """
        Splits a path into multiple paths if there are discontinuities (gaps).
        svgpathtools stores discontinuous subpaths in a single Path object.
        """
        if not path:
            return []

        subpaths = []
        current_subpath = svgpathtools.Path()
        current_subpath.append(path[0])

        for i in range(1, len(path)):
            prev_seg = path[i - 1]
            curr_seg = path[i]

            # Check if the end of the previous segment matches the start of the current one
            if abs(prev_seg.end - curr_seg.start) > 1e-5:
                subpaths.append(current_subpath)
                current_subpath = svgpathtools.Path()

            current_subpath.append(curr_seg)

        subpaths.append(current_subpath)
        return subpaths

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

        # Grupowanie ścieżek w ciągłe segmenty z zaawansowanym łączeniem
        self._logger.info("Grouping paths into continuous segments...")
        path_segments = self._group_continuous_paths_advanced(paths, attributes, draw_options)
        self._logger.info(f"Created {len(path_segments)} continuous segments from {total_paths} paths.")

        execution_queue = []
        self._logger.info("Planning all movements in advance...")

        # Przetwarzanie każdego segmentu ciągłego
        for segment_idx, segment in enumerate(path_segments):
            self._logger.info(f"Processing segment {segment_idx+1}/{len(path_segments)} with {len(segment['points'])} points")
            
            points_2d = segment['points']
            segment_type = segment['type']  # 'draw', 'travel', 'continuous'

            if not points_2d:
                continue

            self._logger.info(f"Segment {segment_idx+1} has {len(points_2d)} points, type: {segment_type}")

            if segment_type == 'travel':
                # Segment przemieszczenia - podnieś pisak i przemieszczaj się
                self._logger.info(f"Travel segment {segment_idx+1} - moving without drawing")
                
                # Przejdź do pierwszego punktu z podniesionym pisakiem
                p0 = points_2d[0]
                target_pose = robomath.transl(p0.x * draw_options.scale, p0.y * draw_options.scale, 0) * orient_tool
                approach_pose = target_pose * robomath.transl(0, 0, draw_options.approach_dist)
                
                last_joints = self._solve_ik(approach_pose, last_joints, frame_pose)
                execution_queue.append(("MOVE", approach_pose, last_joints))
                
                # Przejdź przez pozostałe punkty z podniesionym pisakiem
                for p in points_2d[1:]:
                    travel_pose = robomath.transl(p.x * draw_options.scale, p.y * draw_options.scale, 0) * orient_tool
                    travel_approach_pose = travel_pose * robomath.transl(0, 0, draw_options.approach_dist)
                    last_joints = self._solve_ik(travel_approach_pose, last_joints, frame_pose)
                    execution_queue.append(("MOVE", travel_approach_pose, last_joints))
                    
            else:
                # Segment rysowania - pisak na kartce
                p0 = points_2d[0]
                self._logger.debug(f"Starting point of segment: {p0}")

                target_pose = robomath.transl(p0.x * draw_options.scale, p0.y * draw_options.scale, 0) * orient_tool
                self._logger.debug(f"Calculated target pose: {target_pose}")

                approach_pose = target_pose * robomath.transl(0, 0, draw_options.approach_dist)
                self._logger.debug(f"Calculated approach pose: {approach_pose}")

                if segment_type == 'draw':
                    # Nowy segment rysowania - wykonaj podejście i zejście
                    try:
                        self._logger.info(f"Moving to approach pose for drawing segment {segment_idx+1}.")
                        last_joints = self._solve_ik(approach_pose, last_joints, frame_pose)
                        execution_queue.append(("MOVE", approach_pose, last_joints))
                        self._logger.info(f"Moved to approach pose for segment {segment_idx+1}.")
                    except Exception as e:
                        self._logger.exception(f"Robot cannot reach start of segment {segment_idx+1}. Exception: {e}")
                        self._logger.error("Ensure 'Frame draw' is within reach (approx X=300mm, Y=0mm).")
                        raise

                    self._logger.info(f"Moving down to target pose for drawing segment {segment_idx+1}.")
                    last_joints = self._solve_ik(target_pose, last_joints, frame_pose)
                    execution_queue.append(("MOVE", target_pose, last_joints))
                    self._logger.info(f"Moved down to target pose for segment {segment_idx+1}.")
                    
                elif segment_type == 'continuous':
                    # Kontynuacja poprzedniego segmentu - przejdź bezpośrednio do pierwszego punktu
                    self._logger.info(f"Continuing to next point in continuous segment.")
                    last_joints = self._solve_ik(target_pose, last_joints, frame_pose)
                    execution_queue.append(("MOVE", target_pose, last_joints))
                    self._logger.info(f"Continued to next point.")

                # Śledź krzywą dla segmentu rysowania
                self._logger.info(f"Tracing the curve for segment {segment_idx+1}.")
                for p in points_2d[1:]:
                    self._logger.debug(f"Drawing point: {p}")
                    target_pose = robomath.transl(p.x * draw_options.scale, p.y * draw_options.scale, 0) * orient_tool
                    self._logger.debug(f"Calculated target pose for point: {target_pose}")

                    self._logger.info(f"Moving to point at X={p.x}, Y={p.y}.")
                    last_joints = self._solve_ik(target_pose, last_joints, frame_pose)
                    execution_queue.append(("MOVE", target_pose, last_joints))
                    self._logger.info(f"Moved to point at X={p.x}, Y={p.y}.")

                    if draw_options.use_visual_simulation:
                        execution_queue.append(("DRAW", target_pose, None))

                # Podnieś pisak tylko jeśli następny segment nie jest ciągły
                if segment_idx < len(path_segments) - 1:
                    next_segment = path_segments[segment_idx + 1]
                    if next_segment['type'] != 'continuous':
                        # Podnieś pisak na koniec segmentu
                        last_point = points_2d[-1]
                        end_pose = robomath.transl(last_point.x * draw_options.scale, last_point.y * draw_options.scale, 0) * orient_tool
                        retract_pose = end_pose * robomath.transl(0, 0, draw_options.approach_dist)
                        
                        self._logger.info("Retracting drawing tool after segment.")
                        last_joints = self._solve_ik(retract_pose, last_joints, frame_pose)
                        execution_queue.append(("MOVE", retract_pose, last_joints))
                        self._logger.info("Drawing tool retracted.")
                else:
                    # Ostatni segment - podnieś pisak
                    last_point = points_2d[-1]
                    end_pose = robomath.transl(last_point.x * draw_options.scale, last_point.y * draw_options.scale, 0) * orient_tool
                    retract_pose = end_pose * robomath.transl(0, 0, draw_options.approach_dist)
                    
                    self._logger.info("Retracting drawing tool after final segment.")
                    last_joints = self._solve_ik(retract_pose, last_joints, frame_pose)
                    execution_queue.append(("MOVE", retract_pose, last_joints))
                    self._logger.info("Drawing tool retracted.")

        self._logger.info(f"Planning completed. {len(execution_queue)} actions queued.")
        
        # Precompute all joint positions for the entire execution queue
        self._logger.info("Precomputing all joint positions...")
        precomputed_joints_queue = self._precompute_all_joints(execution_queue, frame_pose)
        self._logger.info(f"Precomputed {len(precomputed_joints_queue)} joint positions.")
        
        self._logger.info("Executing precomputed movements...")
        
        # Optional: Disable rendering for faster execution if needed
        # self._RDK.Render(False)
        
        # Execute all precomputed movements
        for action, data in tqdm(precomputed_joints_queue, desc="Executing"):
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

    def _group_continuous_paths_advanced(self, paths: list[svgpathtools.Path], attributes: list[dict[str, str]], draw_options: DrawOptions) -> list:
        """
        Zaawansowane grupowanie ścieżek w ciągłe segmenty.
        Optymalizuje kolejność ścieżek aby maksymalizować ciągłość rysowania.
        """
        if not paths:
            return []

        # Konwertuj wszystkie ścieżki na punkty
        path_points = []
        for i, (path, attr) in enumerate(zip(paths, attributes)):
            points_2d = get_points_from_path(path, step_mm=draw_options.resolution / draw_options.scale)
            if points_2d:
                # Simplify points
                epsilon = 0.5 / draw_options.scale
                points_2d = simplify_points(points_2d, epsilon)
                path_points.append({
                    'points': points_2d,
                    'start': points_2d[0],
                    'end': points_2d[-1],
                    'original_index': i,
                    'attribute': attr
                })

        if not path_points:
            return []

        # Zbuduj graf połączeń - krawędź z i->j jeśli koniec i jest blisko początku j
        tolerance = 3.0  # mm - tolerancja snapowania

        def euclid(a: Point, b: Point) -> float:
            return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5

        adjacency = {i: [] for i in range(len(path_points))}
        scale = draw_options.scale

        for i in range(len(path_points)):
            for j in range(len(path_points)):
                if i == j:
                    continue
                d = euclid(path_points[i]['end'], path_points[j]['start']) * scale
                if d <= tolerance:
                    adjacency[i].append(j)

        # Greedy budowa łańcuchów (chain) korzystając z grafu
        visited = set()
        chains = []

        def build_chain(start_idx):
            chain = [start_idx]
            cur = start_idx
            while True:
                visited.add(cur)
                nbrs = [n for n in adjacency.get(cur, []) if n not in visited]
                if not nbrs:
                    break
                # wybierz najbliższe dalej (możesz tu dać heurystykę)
                next_idx = nbrs[0]
                chain.append(next_idx)
                cur = next_idx
            return chain

        for i in range(len(path_points)):
            if i in visited:
                continue
            chains.append(build_chain(i))

        # Konwersja chain -> segmenty punktów
        segments = []
        for chain in chains:
            pts_total = []
            for k, idx in enumerate(chain):
                p = path_points[idx]['points']
                if k > 0 and pts_total:
                    # jeśli pierwszy punkt pokrywa się z ostatnim - usuń duplikat
                    if euclid(pts_total[-1], p[0]) * scale <= tolerance:
                        p = p[1:]
                pts_total.extend(p)
            if pts_total:
                segments.append({'points': pts_total, 'type': 'draw'})

        # Dodaj segmenty przemieszczenia pomiędzy nieciągłymi segmentami
        final_segments = []
        if not segments:
            return final_segments

        final_segments.append(segments[0])
        for i in range(1, len(segments)):
            prev_end = final_segments[-1]['points'][-1]
            curr_start = segments[i]['points'][0]
            dist = euclid(prev_end, curr_start) * scale
            travel_tolerance = 2.0
            if dist <= travel_tolerance:
                # połącz jako ciągły
                segments[i]['type'] = 'continuous'
                final_segments.append(segments[i])
            else:
                # dodaj travel
                final_segments.append({'points': [prev_end, curr_start], 'type': 'travel'})
                final_segments.append(segments[i])

        self._logger.info(f"Grouped {len(paths)} paths into {len(final_segments)} segments")
        total_points = sum(len(segment['points']) for segment in final_segments)
        self._logger.info(f"Total points in all segments: {total_points}")

        return final_segments

    def _find_optimal_path_order(self, path_points: list, draw_options: DrawOptions) -> list:
        """
        Znajduje optymalną kolejność ścieżek używając algorytmu najbliższego sąsiada
        z uwzględnieniem możliwości odwracania ścieżek.
        """
        if not path_points:
            return []

        remaining = path_points.copy()
        ordered = []
        
        # Rozpocznij od ścieżki najbliższej do punktu startowego (domyślnie (0,0))
        start_point = Point(0, 0)
        current_point = start_point
        
        while remaining:
            best_path = None
            best_distance = float('inf')
            should_reverse = False
            
            for path in remaining:
                # Sprawdź odległość do początku ścieżki
                dist_start = self._calculate_distance(current_point, path['start'])
                if dist_start < best_distance:
                    best_distance = dist_start
                    best_path = path
                    should_reverse = False
                
                # Sprawdź odległość do końca ścieżki (odwrócona ścieżka)
                dist_end = self._calculate_distance(current_point, path['end'])
                if dist_end < best_distance:
                    best_distance = dist_end
                    best_path = path
                    should_reverse = True
            
            if best_path:
                remaining.remove(best_path)
                
                if should_reverse:
                    # Odwróć ścieżkę
                    best_path['points'] = list(reversed(best_path['points']))
                    best_path['start'], best_path['end'] = best_path['end'], best_path['start']
                
                ordered.append(best_path)
                current_point = best_path['end']
            else:
                break
        
        return ordered

    def _calculate_distance(self, p1: Point, p2: Point) -> float:
        """Oblicza odległość euklidesową między dwoma punktami."""
        return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5

    def _add_travel_segments(self, segments: list, draw_options: DrawOptions) -> list:
        """
        Dodaje segmenty przemieszczenia między nieciągłymi segmentami rysowania.
        """
        if len(segments) <= 1:
            return segments

        final_segments = []
        
        # Pierwszy segment zawsze jest segmentem rysowania
        final_segments.append(segments[0])
        
        for i in range(1, len(segments)):
            prev_segment = segments[i-1]
            curr_segment = segments[i]
            
            prev_end = prev_segment['points'][-1]
            curr_start = curr_segment['points'][0]
            
            # Sprawdź czy segmenty są ciągłe
            dist = self._calculate_distance(prev_end, curr_start) * draw_options.scale
            tolerance = 2.0  # Tolerancja dla ciągłości
            
            if dist <= tolerance:
                # Segmenty są ciągłe - oznacz jako ciągły
                curr_segment['type'] = 'continuous'
                final_segments.append(curr_segment)
            else:
                # Dodaj segment przemieszczenia
                travel_points = [prev_end, curr_start]
                final_segments.append({
                    'points': travel_points,
                    'type': 'travel'
                })
                final_segments.append(curr_segment)
        
        return final_segments

    def _precompute_all_joints(self, execution_queue, frame_pose) -> list:
        """
        Precomputes all joint positions for the entire execution queue in advance.
        This eliminates IK solving delays during execution.
        """
        precomputed_queue = []
        last_joints = None
        
        for i, (action, pose, joints) in enumerate(tqdm(execution_queue, desc="Precomputing joints")):
            if action == "MOVE":
                if last_joints is None:
                    # For the first movement, use current robot joints
                    try:
                        last_joints = self._robot.Joints().list()
                    except Exception:
                        self._logger.warning("Could not get robot joints, assuming all zeros.")
                        last_joints = [0.0] * 6
                
                # Use SolveIK to compute the joint positions
                computed_joints = self._solve_ik_for_precomputation(pose, last_joints, frame_pose)
                precomputed_queue.append(("MOVE", computed_joints))
                last_joints = computed_joints
            elif action == "DRAW":
                # For DRAW actions, just pass the pose data through
                precomputed_queue.append(("DRAW", pose))
        
        return precomputed_queue

    def _solve_ik_for_precomputation(self, pose, last_joints: list[float], frame_pose) -> list[float]:
        """
        Calculates the inverse kinematics for the specified pose using the previous joints as approximation.
        This version is optimized for precomputation.
        """
        ik_result = self._robot.SolveIK(pose, joints_approx=last_joints, reference=frame_pose)
        joints = ik_result.list()
        
        if len(joints) > 0:
            return joints
        else:
            self._logger.warning("SolveIK failed to find a solution during precomputation. Using last known joints.")
            return last_joints

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
