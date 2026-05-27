from __future__ import annotations

import collections
import itertools
import json
import math
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import tkinter as tk
from tkinter import messagebox, ttk


# ---------------------------------------------------------------------------
# Adjustable mission parameters
# ---------------------------------------------------------------------------

SERVER_STATE_URL = "http://192.168.0.74:8090/api/state"
GRID_SIZE = 9
ROW_LABELS = "ABCDEFGHI"

START_CELL = "C9"
FINISH_CELL = "G9"

SEEDS_TO_PLANT = 5
RESERVE_TARGET_COUNT = 7

# Numbered columns 1-5 are treated as double-score planting cells.
HIGH_VALUE_MAX_COL = 5
CANDIDATE_POOL_SIZE = 22

OBSTACLE_CONFIRM_SECONDS = 5.0
AUTO_STEP_DELAY_MS = 450

# Route cost priority:
# 1) choose higher scoring seed cells, 2) fewer turns, 3) shorter travel.
TURN_COST_FOR_STATUS_ONLY = 1

CELL_SIZE = 54
CANVAS_MARGIN = 2


Cell = Tuple[int, int]  # (row 1..9, col 1..9)


@dataclass
class CellState:
    tag_id: str = ""
    fertile: bool = False
    planted: bool = False
    explored: bool = False
    debug: bool = False
    label: Optional[str] = None


@dataclass
class RoutePlan:
    targets: List[Cell]
    rescue_targets: List[Cell]
    waypoints: List[Cell]
    path: List[Cell]
    turns: int
    steps: int
    seed_score: int
    wall_score: int


def cell_from_label(label: str) -> Cell:
    label = label.strip().upper()
    if len(label) < 2:
        raise ValueError(f"Bad cell label: {label}")
    row = ROW_LABELS.index(label[0]) + 1
    col = int(label[1:])
    if not (1 <= row <= GRID_SIZE and 1 <= col <= GRID_SIZE):
        raise ValueError(f"Cell outside map: {label}")
    return (row, col)


def label_from_cell(cell: Cell) -> str:
    row, col = cell
    return f"{ROW_LABELS[row - 1]}{col}"


START = cell_from_label(START_CELL)
FINISH = cell_from_label(FINISH_CELL)


def manhattan(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def wall_distance(cell: Cell) -> int:
    row, col = cell
    return min(row - 1, GRID_SIZE - row, col - 1, GRID_SIZE - col)


def wall_closeness_score(cell: Cell) -> int:
    return GRID_SIZE - wall_distance(cell)


def is_high_value_cell(cell: Cell) -> bool:
    return cell[1] <= HIGH_VALUE_MAX_COL


def seed_cell_score(cell: Cell) -> int:
    return 2 if is_high_value_cell(cell) else 1


def neighbors(cell: Cell) -> Iterable[Cell]:
    row, col = cell
    for dr, dc in ((-1, 0), (0, 1), (1, 0), (0, -1)):
        nr, nc = row + dr, col + dc
        if 1 <= nr <= GRID_SIZE and 1 <= nc <= GRID_SIZE:
            yield (nr, nc)


def direction_between(a: Cell, b: Cell) -> Tuple[int, int]:
    return (b[0] - a[0], b[1] - a[1])


def count_turns(path: Sequence[Cell]) -> int:
    if len(path) < 3:
        return 0
    turns = 0
    last_direction = direction_between(path[0], path[1])
    for i in range(1, len(path) - 1):
        new_direction = direction_between(path[i], path[i + 1])
        if new_direction != last_direction:
            turns += 1
        last_direction = new_direction
    return turns


def bfs_path(start: Cell, goal: Cell, blocked: Set[Cell]) -> Optional[List[Cell]]:
    if start == goal:
        return [start]
    if goal in blocked:
        return None

    queue = collections.deque([start])
    came_from: Dict[Cell, Optional[Cell]] = {start: None}

    while queue:
        current = queue.popleft()
        for nxt in neighbors(current):
            if nxt in blocked and nxt != goal:
                continue
            if nxt in came_from:
                continue
            came_from[nxt] = current
            if nxt == goal:
                path = [goal]
                while path[-1] != start:
                    prev = came_from[path[-1]]
                    if prev is None:
                        break
                    path.append(prev)
                path.reverse()
                return path
            queue.append(nxt)

    return None


def concatenate_paths(segments: Sequence[List[Cell]]) -> List[Cell]:
    result: List[Cell] = []
    for segment in segments:
        if not segment:
            continue
        if result and result[-1] == segment[0]:
            result.extend(segment[1:])
        else:
            result.extend(segment)
    return result


def load_cells_from_payload(payload: dict) -> Dict[Cell, CellState]:
    cells: Dict[Cell, CellState] = {}
    for tag in payload.get("rfidTags", []):
        row = int(tag.get("row", tag.get("y")))
        col = int(tag.get("col", tag.get("x")))
        if not (1 <= row <= GRID_SIZE and 1 <= col <= GRID_SIZE):
            continue
        cells[(row, col)] = CellState(
            tag_id=str(tag.get("tagId", "")),
            fertile=bool(tag.get("fertile", False)),
            planted=bool(tag.get("planted", False)),
            explored=bool(tag.get("explored", False)),
            debug=bool(tag.get("debug", False)),
            label=tag.get("label"),
        )

    for row in range(1, GRID_SIZE + 1):
        for col in range(1, GRID_SIZE + 1):
            cells.setdefault((row, col), CellState())

    return cells


def default_snapshot_path() -> Path:
    # Script lives at Robotic_Challenge/tools/path_planner_visualizer.py.
    # The old saved dashboard state is in the workspace root.
    return Path(__file__).resolve().parents[2] / "_mini_messenger_state_snapshot.json"


def generate_fallback_cells() -> Dict[Cell, CellState]:
    cells: Dict[Cell, CellState] = {}
    for row in range(1, GRID_SIZE + 1):
        for col in range(1, GRID_SIZE + 1):
            cells[(row, col)] = CellState(
                tag_id=f"SIM-{row}-{col}",
                fertile=((row + col) % 2 == 0 or row <= 2 or col in (1, 9)),
                planted=False,
                explored=False,
            )
    return cells


def fetch_server_cells() -> Tuple[Dict[Cell, CellState], str]:
    try:
        with urllib.request.urlopen(SERVER_STATE_URL, timeout=5) as response:
            payload = json.loads(response.read().decode("utf-8"))
        return load_cells_from_payload(payload), f"server: {SERVER_STATE_URL}"
    except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
        snapshot = default_snapshot_path()
        if snapshot.exists():
            payload = json.loads(snapshot.read_text(encoding="utf-8-sig"))
            return load_cells_from_payload(payload), f"offline snapshot: {snapshot.name} ({exc})"
        return generate_fallback_cells(), f"generated fallback map ({exc})"


def line_alignment_score(cells: Iterable[Cell]) -> int:
    cells = list(cells)
    if not cells:
        return 0

    row_counts = collections.Counter(cell[0] for cell in cells)
    col_counts = collections.Counter(cell[1] for cell in cells)
    pair_count = 0
    for count in list(row_counts.values()) + list(col_counts.values()):
        pair_count += count * (count - 1) // 2
    return pair_count + 3 * max(max(row_counts.values()), max(col_counts.values()))


def candidate_score(cell: Cell, cells: Dict[Cell, CellState], available: Set[Cell]) -> float:
    same_row_or_col = sum(
        1
        for other in available
        if other != cell and (other[0] == cell[0] or other[1] == cell[1])
    )

    score = 0.0
    score += 10000.0 if is_high_value_cell(cell) else 0.0
    score += 140.0 * wall_closeness_score(cell)
    score += 55.0 * same_row_or_col
    score -= 2.0 * manhattan(START, cell)
    score -= 1.2 * manhattan(cell, FINISH)

    # Do not make the start/finish cells planting targets unless they are the
    # only option, because physically they are mission control locations.
    if cell in (START, FINISH):
        score -= 60.0

    # A tiny deterministic tie-breaker keeps the route stable.
    score -= 0.01 * (cell[0] * 10 + cell[1])
    return score


def available_seed_cells(
    cells: Dict[Cell, CellState],
    known_obstacles: Set[Cell],
    planted_by_robot: Set[Cell],
) -> List[Cell]:
    available = []
    for cell, state in cells.items():
        if cell in known_obstacles:
            continue
        if cell in planted_by_robot:
            continue
        if state.fertile and not state.planted:
            available.append(cell)
    return available


def choose_reserve_targets(
    cells: Dict[Cell, CellState],
    known_obstacles: Set[Cell],
    planted_by_robot: Set[Cell],
    count: int = RESERVE_TARGET_COUNT,
) -> List[Cell]:
    available = set(available_seed_cells(cells, known_obstacles, planted_by_robot))
    if not available:
        return []

    high_value = {cell for cell in available if is_high_value_cell(cell)}
    primary = high_value if len(high_value) >= count else available

    scored_primary = sorted(
        primary,
        key=lambda cell: candidate_score(cell, cells, available),
        reverse=True,
    )
    pool = scored_primary[: min(CANDIDATE_POOL_SIZE, len(scored_primary))]

    if len(primary) < count:
        secondary = sorted(
            available - primary,
            key=lambda cell: candidate_score(cell, cells, available),
            reverse=True,
        )
        pool = [*pool, *secondary[: count - len(pool)]]

    if len(pool) <= count:
        return pool

    best_combo: Optional[Tuple[Cell, ...]] = None
    best_key: Optional[Tuple[int, int, int, int, int]] = None
    for combo in itertools.combinations(pool, count):
        seed_score = sum(seed_cell_score(cell) for cell in combo)
        alignment = line_alignment_score(combo)
        wall_score = sum(wall_closeness_score(cell) for cell in combo)
        rough_travel = (
            min(manhattan(START, cell) for cell in combo)
            + min(manhattan(cell, FINISH) for cell in combo)
            + sum(manhattan(a, b) for a, b in itertools.combinations(combo, 2)) // max(1, len(combo) - 1)
        )
        center_penalty = sum(wall_distance(cell) for cell in combo)

        # Priority: double-score cells first, then collinearity, then wall
        # following convenience, then rough travel distance.
        key = (-seed_score, -alignment, -wall_score, rough_travel, center_penalty)
        if best_key is None or key < best_key:
            best_key = key
            best_combo = combo

    if best_combo is None:
        return scored_primary[:count]

    return sorted(
        best_combo,
        key=lambda cell: candidate_score(cell, cells, available),
        reverse=True,
    )


def optimize_route(
    current: Cell,
    reserve_targets: Sequence[Cell],
    finish: Cell,
    known_obstacles: Set[Cell],
    seeds_needed: int,
    required_waypoints: Sequence[Cell] = (),
) -> Optional[RoutePlan]:
    required = list(dict.fromkeys(required_waypoints))

    if seeds_needed <= 0 and not required:
        path = bfs_path(current, finish, known_obstacles)
        if path is None:
            return None
        return RoutePlan([], [], [], path, count_turns(path), max(0, len(path) - 1), 0, 0)

    usable_targets = [target for target in reserve_targets if target not in known_obstacles]
    if seeds_needed > 0 and not usable_targets and not required:
        return None

    choose_count = min(seeds_needed, len(usable_targets))
    best: Optional[RoutePlan] = None
    best_key: Optional[Tuple[int, int, int, int]] = None
    path_cache: Dict[Tuple[Cell, Cell], Optional[List[Cell]]] = {}

    def cached_bfs_path(start_cell: Cell, goal_cell: Cell) -> Optional[List[Cell]]:
        key = (start_cell, goal_cell)
        if key not in path_cache:
            path_cache[key] = bfs_path(start_cell, goal_cell, known_obstacles)
        return path_cache[key]

    for combo in itertools.combinations(usable_targets, choose_count):
        waypoint_pool = list(dict.fromkeys([*combo, *required]))
        for order in itertools.permutations(waypoint_pool):
            points = [current, *order, finish]
            segments: List[List[Cell]] = []
            impossible = False

            for start, goal in zip(points, points[1:]):
                segment = cached_bfs_path(start, goal)
                if segment is None:
                    impossible = True
                    break
                segments.append(segment)

            if impossible:
                continue

            path = concatenate_paths(segments)
            seed_score = sum(seed_cell_score(cell) for cell in combo)
            wall_score = sum(wall_closeness_score(cell) for cell in combo)
            turns = count_turns(path)
            steps = max(0, len(path) - 1)
            route_seed_targets = [cell for cell in order if cell in combo]
            route_rescue_targets = [cell for cell in order if cell in required]

            # Higher seed score first, then fewer turns, then shorter path,
            # then closer-to-wall planting points.
            key = (-seed_score, turns, steps, -wall_score)
            if best_key is None or key < best_key:
                best_key = key
                best = RoutePlan(
                    route_seed_targets,
                    route_rescue_targets,
                    list(order),
                    path,
                    turns,
                    steps,
                    seed_score,
                    wall_score,
                )

    return best


class PlannerApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("9x9 Seed Planting Path Planner")

        self.cells: Dict[Cell, CellState] = {}
        self.hidden_obstacles: Set[Cell] = set()
        self.known_obstacles: Set[Cell] = set()
        self.planted_by_robot: Set[Cell] = set()

        self.robot_pos: Cell = START
        self.reserve_targets: List[Cell] = []
        self.active_targets: List[Cell] = []
        self.active_rescue_targets: List[Cell] = []
        self.rescue_targets: Set[Cell] = set()
        self.rescued_targets: Set[Cell] = set()
        self.full_path: List[Cell] = []
        self.planned_turns = 0
        self.planned_steps = 0
        self.planned_seed_score = 0

        self.planted_count = 0
        self.emergency_return = False
        self.auto_running = False
        self.pending_obstacle_check: Optional[Cell] = None
        self.last_plan_reason = "not planned yet"

        self.edit_mode = tk.StringVar(value="obstacle")
        self.status_var = tk.StringVar(value="Loading map...")
        self.summary_var = tk.StringVar(value="")
        self.source_var = tk.StringVar(value="")

        self._build_ui()
        self.load_state_from_server()

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        controls = ttk.Frame(self.root, padding=8)
        controls.grid(row=0, column=0, columnspan=2, sticky="ew")

        ttk.Button(controls, text="Refresh /api/state", command=self.load_state_from_server).grid(row=0, column=0, padx=4)
        ttk.Button(controls, text="Replan", command=lambda: self.replan("manual")).grid(row=0, column=1, padx=4)
        ttk.Button(controls, text="Step", command=self.step_once).grid(row=0, column=2, padx=4)
        self.auto_button = ttk.Button(controls, text="Auto Run", command=self.toggle_auto)
        self.auto_button.grid(row=0, column=3, padx=4)
        ttk.Button(controls, text=f"Emergency Return {FINISH_CELL}", command=self.trigger_emergency_return).grid(row=0, column=4, padx=4)
        ttk.Button(controls, text="Reset Robot Memory", command=self.reset_robot_memory).grid(row=0, column=5, padx=4)

        edit = ttk.LabelFrame(controls, text="World edit mode", padding=4)
        edit.grid(row=0, column=6, padx=10)
        ttk.Radiobutton(edit, text="Obstacle", value="obstacle", variable=self.edit_mode).grid(row=0, column=0)
        ttk.Radiobutton(edit, text="Planted", value="planted", variable=self.edit_mode).grid(row=0, column=1)
        ttk.Radiobutton(edit, text="Fertile", value="fertile", variable=self.edit_mode).grid(row=0, column=2)
        ttk.Radiobutton(edit, text="Rescue", value="rescue", variable=self.edit_mode).grid(row=0, column=3)

        map_frame = ttk.Frame(self.root, padding=8)
        map_frame.grid(row=1, column=0, columnspan=2)

        ttk.Label(map_frame, text="World map: server state + hidden obstacles you edit").grid(row=0, column=0)
        ttk.Label(map_frame, text="Robot map: known state + planned route").grid(row=0, column=1)

        canvas_size = CELL_SIZE * (GRID_SIZE + 1) + CANVAS_MARGIN * 2
        self.world_canvas = tk.Canvas(map_frame, width=canvas_size, height=canvas_size, bg="white")
        self.robot_canvas = tk.Canvas(map_frame, width=canvas_size, height=canvas_size, bg="white")
        self.world_canvas.grid(row=1, column=0, padx=10)
        self.robot_canvas.grid(row=1, column=1, padx=10)
        self.world_canvas.bind("<Button-1>", self.on_world_click)

        bottom = ttk.Frame(self.root, padding=8)
        bottom.grid(row=2, column=0, columnspan=2, sticky="ew")
        ttk.Label(bottom, textvariable=self.status_var, anchor="w").grid(row=0, column=0, sticky="ew")
        ttk.Label(bottom, textvariable=self.source_var, anchor="w").grid(row=1, column=0, sticky="ew")
        ttk.Label(bottom, textvariable=self.summary_var, anchor="w", justify="left").grid(row=2, column=0, sticky="ew")
        bottom.columnconfigure(0, weight=1)

    # ------------------------------------------------------------------
    # State and planning
    # ------------------------------------------------------------------

    def load_state_from_server(self) -> None:
        old_hidden = set(self.hidden_obstacles)
        had_plan = bool(self.full_path)
        try:
            self.cells, source = fetch_server_cells()
            self.hidden_obstacles = old_hidden
            self.source_var.set(f"Map source: {source}")
            self.status_var.set("Loaded 81-cell map state. Obstacles remain hidden from the robot until detected.")
            stolen_target = self.planned_seed_planted_by_others()
            if not had_plan:
                self.replan("initial server load")
            elif stolen_target is not None:
                self.status_var.set(
                    f"Planned seed target {label_from_cell(stolen_target)} was planted by another robot. "
                    "Replanning with reserve targets."
                )
                self.replan("planned seed planted by others")
            else:
                self.draw_all()
                self.update_summary()
        except Exception as exc:  # defensive UI guard
            messagebox.showerror("Map load failed", str(exc))
            self.cells = generate_fallback_cells()
            self.source_var.set("Map source: generated fallback after unexpected error")
            if not had_plan:
                self.replan("fallback")
            else:
                self.draw_all()
                self.update_summary()

    def reset_robot_memory(self) -> None:
        self.known_obstacles.clear()
        self.planted_by_robot.clear()
        self.planted_count = 0
        self.robot_pos = START
        self.rescued_targets.clear()
        self.emergency_return = False
        self.pending_obstacle_check = None
        self.replan("reset robot memory")
        self.status_var.set("Robot memory reset. Hidden world obstacles are still hidden.")

    def trigger_emergency_return(self) -> None:
        self.emergency_return = True
        self.replan("emergency return")
        self.status_var.set(f"Emergency return active: robot will skip planting and route to {FINISH_CELL}.")
        if not self.auto_running:
            self.toggle_auto()

    def replan(self, reason: str) -> None:
        self.last_plan_reason = reason
        self.reserve_targets = choose_reserve_targets(
            self.cells,
            self.known_obstacles,
            self.planted_by_robot,
            RESERVE_TARGET_COUNT,
        )

        seeds_needed = max(0, SEEDS_TO_PLANT - self.planted_count)
        required_waypoints: List[Cell] = []
        if self.emergency_return:
            seeds_needed = 0
        else:
            required_waypoints.extend(self.pending_rescue_targets())

        plan = optimize_route(
            self.robot_pos,
            self.reserve_targets,
            FINISH,
            self.known_obstacles,
            seeds_needed,
            required_waypoints,
        )

        if plan is None:
            self.active_targets = []
            self.active_rescue_targets = []
            self.full_path = [self.robot_pos]
            self.planned_turns = 0
            self.planned_steps = 0
            self.planned_seed_score = 0
            self.status_var.set("No route found with current known obstacles.")
        else:
            self.active_targets = plan.targets
            self.active_rescue_targets = plan.rescue_targets
            self.full_path = plan.path
            self.planned_turns = plan.turns
            self.planned_steps = plan.steps
            self.planned_seed_score = plan.seed_score

        self.draw_all()
        self.update_summary()

    def pending_rescue_targets(self) -> List[Cell]:
        return [cell for cell in sorted(self.rescue_targets) if cell not in self.rescued_targets]

    def planned_seed_planted_by_others(self) -> Optional[Cell]:
        for target in self.active_targets:
            if target in self.planted_by_robot:
                continue
            state = self.cells.get(target, CellState())
            if state.planted:
                return target
        return None

    def blocked_planned_waypoint(self) -> Optional[Cell]:
        for target in [*self.active_targets, *self.active_rescue_targets]:
            if target in self.known_obstacles:
                return target
        return None

    def target_became_invalid(self) -> bool:
        if self.blocked_planned_waypoint() is not None:
            return True
        if self.planned_seed_planted_by_others() is not None:
            return True
        return False

    # ------------------------------------------------------------------
    # Movement simulation
    # ------------------------------------------------------------------

    def toggle_auto(self) -> None:
        self.auto_running = not self.auto_running
        self.auto_button.configure(text="Pause" if self.auto_running else "Auto Run")
        if self.auto_running:
            self.schedule_auto_step()

    def schedule_auto_step(self) -> None:
        if not self.auto_running:
            return
        if self.pending_obstacle_check is not None:
            self.root.after(250, self.schedule_auto_step)
            return
        self.step_once()
        self.root.after(AUTO_STEP_DELAY_MS, self.schedule_auto_step)

    def step_once(self) -> None:
        if self.pending_obstacle_check is not None:
            self.status_var.set(
                f"Waiting {OBSTACLE_CONFIRM_SECONDS:.0f}s to confirm obstacle at "
                f"{label_from_cell(self.pending_obstacle_check)}."
            )
            return

        if self.robot_pos == FINISH and (self.emergency_return or self.planted_count >= SEEDS_TO_PLANT):
            self.status_var.set(f"Robot is already at {FINISH_CELL}. Mission return complete.")
            if self.auto_running:
                self.toggle_auto()
            return

        if self.target_became_invalid():
            blocked_target = self.blocked_planned_waypoint()
            stolen_target = self.planned_seed_planted_by_others()
            if blocked_target is not None:
                self.status_var.set(
                    f"Planned waypoint {label_from_cell(blocked_target)} is blocked. Replanning."
                )
                self.replan("planned waypoint blocked")
            elif stolen_target is not None:
                self.status_var.set(
                    f"Planned seed target {label_from_cell(stolen_target)} was planted by another robot. "
                    "Replanning with reserve targets."
                )
                self.replan("planned seed planted by others")

        if len(self.full_path) < 2 or self.full_path[0] != self.robot_pos:
            self.replan("path exhausted")

        if len(self.full_path) < 2:
            self.status_var.set("No next step available.")
            return

        next_cell = self.full_path[1]
        if next_cell in self.hidden_obstacles:
            self.begin_obstacle_confirmation(next_cell)
            return

        self.move_into(next_cell)

    def begin_obstacle_confirmation(self, cell: Cell) -> None:
        self.pending_obstacle_check = cell
        self.status_var.set(
            f"Front sensor detected possible obstacle at {label_from_cell(cell)}. "
            f"Stopping for {OBSTACLE_CONFIRM_SECONDS:.0f}s before deciding."
        )
        self.draw_all()
        self.root.after(int(OBSTACLE_CONFIRM_SECONDS * 1000), self.finish_obstacle_confirmation)

    def finish_obstacle_confirmation(self) -> None:
        cell = self.pending_obstacle_check
        self.pending_obstacle_check = None
        if cell is None:
            return

        if cell in self.hidden_obstacles:
            self.known_obstacles.add(cell)
            self.status_var.set(
                f"Obstacle at {label_from_cell(cell)} is still present. "
                "Robot marks it as blocked and reruns BFS."
            )
            self.replan("obstacle discovered")
        else:
            self.status_var.set(
                f"Obstacle at {label_from_cell(cell)} disappeared during the wait. Continuing."
            )
            self.move_into(cell)

    def move_into(self, cell: Cell) -> None:
        self.robot_pos = cell

        if self.full_path and self.full_path[0] != cell:
            # Keep the path anchored at the robot after manual replans or edits.
            try:
                idx = self.full_path.index(cell)
                self.full_path = self.full_path[idx:]
            except ValueError:
                self.full_path = [cell]
        elif len(self.full_path) > 1 and self.full_path[1] == cell:
            self.full_path = self.full_path[1:]

        state = self.cells.get(cell, CellState())
        messages: List[str] = []

        if (
            not self.emergency_return
            and cell in self.rescue_targets
            and cell not in self.rescued_targets
        ):
            self.rescued_targets.add(cell)
            self.active_rescue_targets = [target for target in self.active_rescue_targets if target != cell]
            messages.append(f"Visited rescue waypoint at {label_from_cell(cell)}.")

        if (
            not self.emergency_return
            and self.planted_count < SEEDS_TO_PLANT
            and cell in self.active_targets
            and state.fertile
            and not state.planted
            and cell not in self.planted_by_robot
        ):
            self.cells[cell].planted = True
            self.planted_by_robot.add(cell)
            self.active_targets = [target for target in self.active_targets if target != cell]
            self.planted_count += 1
            messages.append(f"Planted seed {self.planted_count}/{SEEDS_TO_PLANT} at {label_from_cell(cell)}.")
            if self.planted_count >= SEEDS_TO_PLANT:
                self.emergency_return = True
                messages.append(f"Planted 5 seeds. Continue along the current route back to {FINISH_CELL}.")

        if messages:
            self.status_var.set(" ".join(messages))
            self.draw_all()
            self.update_summary()
        elif cell == FINISH and (self.emergency_return or self.planted_count >= SEEDS_TO_PLANT):
            self.status_var.set(f"Arrived at {FINISH_CELL}.")
            if self.auto_running:
                self.toggle_auto()
            self.draw_all()
            self.update_summary()
        else:
            self.draw_all()
            self.update_summary()

    # ------------------------------------------------------------------
    # Editing and drawing
    # ------------------------------------------------------------------

    def on_world_click(self, event: tk.Event) -> None:
        row = int((event.y - CANVAS_MARGIN) // CELL_SIZE)
        col = int((event.x - CANVAS_MARGIN) // CELL_SIZE)
        if row == 0 or col == 0:
            return
        if not (1 <= row <= GRID_SIZE and 1 <= col <= GRID_SIZE):
            return
        cell = (row, col)

        mode = self.edit_mode.get()
        if mode == "obstacle":
            if cell in self.hidden_obstacles:
                self.hidden_obstacles.remove(cell)
                self.status_var.set(f"World obstacle removed at {label_from_cell(cell)}.")
            else:
                if cell in (START, FINISH):
                    messagebox.showwarning("Protected cell", f"Start {START_CELL} and finish {FINISH_CELL} are protected.")
                    return
                self.hidden_obstacles.add(cell)
                self.status_var.set(
                    f"Hidden world obstacle added at {label_from_cell(cell)}. "
                    "Robot does not know it yet."
                )
        elif mode == "planted":
            self.cells[cell].planted = not self.cells[cell].planted
            if self.cells[cell].planted:
                self.planted_by_robot.discard(cell)
            self.status_var.set(
                f"Server/world planted state at {label_from_cell(cell)} = {self.cells[cell].planted}."
            )
            if cell in self.active_targets and self.cells[cell].planted:
                self.replan("planned seed planted by others")
        elif mode == "fertile":
            self.cells[cell].fertile = not self.cells[cell].fertile
            self.status_var.set(
                f"Server/world fertile state at {label_from_cell(cell)} = {self.cells[cell].fertile}. "
                "Route is unchanged unless you press Replan."
            )
        elif mode == "rescue":
            if cell in self.rescue_targets:
                self.rescue_targets.remove(cell)
                self.rescued_targets.discard(cell)
                self.active_rescue_targets = [target for target in self.active_rescue_targets if target != cell]
                self.status_var.set(f"Rescue waypoint removed from {label_from_cell(cell)}.")
                self.replan("rescue waypoint removed")
            else:
                self.rescue_targets.add(cell)
                self.rescued_targets.discard(cell)
                self.status_var.set(
                    f"New rescue robot added at {label_from_cell(cell)}. "
                    "Replanning to include all pending rescue waypoints."
                )
                self.replan("new rescue robot")

        self.draw_all()
        self.update_summary()

    def draw_all(self) -> None:
        self.draw_world_map()
        self.draw_robot_map()

    def draw_grid_headers(self, canvas: tk.Canvas) -> None:
        for row in range(0, GRID_SIZE + 1):
            for col in range(0, GRID_SIZE + 1):
                x0 = CANVAS_MARGIN + col * CELL_SIZE
                y0 = CANVAS_MARGIN + row * CELL_SIZE
                x1 = x0 + CELL_SIZE
                y1 = y0 + CELL_SIZE
                fill = "#e6e6e6" if row == 0 or col == 0 else "#ffffff"
                canvas.create_rectangle(x0, y0, x1, y1, fill=fill, outline="#9a9a9a")
                if row == 0 and col > 0:
                    canvas.create_text((x0 + x1) / 2, (y0 + y1) / 2, text=str(col), font=("Arial", 12, "bold"))
                elif col == 0 and row > 0:
                    canvas.create_text((x0 + x1) / 2, (y0 + y1) / 2, text=ROW_LABELS[row - 1], font=("Arial", 12, "bold"))

    def cell_rect(self, cell: Cell) -> Tuple[int, int, int, int]:
        row, col = cell
        x0 = CANVAS_MARGIN + col * CELL_SIZE
        y0 = CANVAS_MARGIN + row * CELL_SIZE
        return x0, y0, x0 + CELL_SIZE, y0 + CELL_SIZE

    def cell_center(self, cell: Cell) -> Tuple[float, float]:
        x0, y0, x1, y1 = self.cell_rect(cell)
        return (x0 + x1) / 2, (y0 + y1) / 2

    def draw_cell_text(self, canvas: tk.Canvas, cell: Cell, text: str, fill: str = "#111111", bold: bool = False) -> None:
        x, y = self.cell_center(cell)
        font = ("Arial", 10, "bold" if bold else "normal")
        canvas.create_text(x, y, text=text, fill=fill, font=font)

    def draw_special_marks(self, canvas: tk.Canvas) -> None:
        for cell, color, label in ((START, "#2b6cb0", START_CELL), (FINISH, "#6b46c1", FINISH_CELL)):
            x0, y0, x1, y1 = self.cell_rect(cell)
            canvas.create_rectangle(x0 + 3, y0 + 3, x1 - 3, y1 - 3, outline=color, width=3)
            canvas.create_text(x0 + 14, y0 + 12, text=label, fill=color, font=("Arial", 8, "bold"))

    def draw_world_map(self) -> None:
        canvas = self.world_canvas
        canvas.delete("all")
        self.draw_grid_headers(canvas)

        for row in range(1, GRID_SIZE + 1):
            for col in range(1, GRID_SIZE + 1):
                cell = (row, col)
                state = self.cells.get(cell, CellState())
                if cell in self.hidden_obstacles:
                    fill, text, text_fill = "#30343b", "X", "#ffffff"
                elif state.planted:
                    fill, text, text_fill = "#d7a86e", "P", "#2d1d08"
                elif state.fertile:
                    fill, text, text_fill = "#bfe7b7", "F", "#124016"
                else:
                    fill, text, text_fill = "#f8f8f2", ".", "#777777"

                x0, y0, x1, y1 = self.cell_rect(cell)
                canvas.create_rectangle(x0, y0, x1, y1, fill=fill, outline="#9a9a9a")
                self.draw_cell_text(canvas, cell, text, text_fill, bold=(text != "."))

        if self.pending_obstacle_check is not None:
            x0, y0, x1, y1 = self.cell_rect(self.pending_obstacle_check)
            canvas.create_rectangle(x0 + 5, y0 + 5, x1 - 5, y1 - 5, outline="#ffcc00", width=4)

        for rescue_cell in sorted(self.rescue_targets):
            done = rescue_cell in self.rescued_targets
            x0, y0, x1, y1 = self.cell_rect(rescue_cell)
            color = "#7a4b58" if done else "#d7263d"
            label = "OK" if done else "RES"
            canvas.create_rectangle(x0 + 4, y0 + 4, x1 - 4, y1 - 4, outline=color, width=4)
            canvas.create_text(
                (x0 + x1) / 2,
                y1 - 12,
                text=label,
                fill=color,
                font=("Arial", 8, "bold"),
            )

        self.draw_special_marks(canvas)

    def draw_robot_map(self) -> None:
        canvas = self.robot_canvas
        canvas.delete("all")
        self.draw_grid_headers(canvas)

        path_cells = set(self.full_path)
        active = set(self.active_targets)
        rescue_active = set(self.active_rescue_targets)
        rescue_pending = set(self.pending_rescue_targets())
        rescue_done = set(self.rescued_targets)
        reserve = set(self.reserve_targets)

        for row in range(1, GRID_SIZE + 1):
            for col in range(1, GRID_SIZE + 1):
                cell = (row, col)
                state = self.cells.get(cell, CellState())

                if cell in self.known_obstacles:
                    fill, text, text_fill = "#30343b", "X", "#ffffff"
                elif cell == self.robot_pos:
                    fill, text, text_fill = "#ffb347", "BOT", "#2d1d08"
                elif cell in rescue_done:
                    fill, text, text_fill = "#e9e0e3", "OK", "#7a4b58"
                elif cell in rescue_active:
                    fill, text, text_fill = "#ffd6de", "RES", "#a3162b"
                elif cell in rescue_pending:
                    fill, text, text_fill = "#ffeaf0", "RES", "#a3162b"
                elif cell in self.planted_by_robot or state.planted:
                    fill, text, text_fill = "#d7a86e", "P", "#2d1d08"
                elif cell in active:
                    fill, text, text_fill = "#ffe680", "T", "#3f3000"
                elif cell in reserve:
                    fill, text, text_fill = "#d7e9ff", "R", "#174a7c"
                elif cell in path_cells:
                    fill, text, text_fill = "#e5f6ff", "+", "#2b6cb0"
                elif state.fertile:
                    fill, text, text_fill = "#edf8ea", "F", "#59845d"
                else:
                    fill, text, text_fill = "#f8f8f2", ".", "#777777"

                x0, y0, x1, y1 = self.cell_rect(cell)
                canvas.create_rectangle(x0, y0, x1, y1, fill=fill, outline="#9a9a9a")
                self.draw_cell_text(canvas, cell, text, text_fill, bold=(text in {"BOT", "T", "R", "X", "P", "RES", "OK"}))

        if len(self.full_path) >= 2:
            coords: List[float] = []
            for cell in self.full_path:
                coords.extend(self.cell_center(cell))
            canvas.create_line(*coords, fill="#1677b8", width=3, smooth=False, arrow=tk.LAST)

        if self.pending_obstacle_check is not None:
            x0, y0, x1, y1 = self.cell_rect(self.pending_obstacle_check)
            canvas.create_rectangle(x0 + 5, y0 + 5, x1 - 5, y1 - 5, outline="#ffcc00", width=4)

        self.draw_special_marks(canvas)

    def update_summary(self) -> None:
        fertile_count = sum(1 for state in self.cells.values() if state.fertile)
        planted_count = sum(1 for state in self.cells.values() if state.planted)
        available_count = len(available_seed_cells(self.cells, self.known_obstacles, self.planted_by_robot))

        reserve_labels = ", ".join(label_from_cell(cell) for cell in self.reserve_targets) or "none"
        active_labels = ", ".join(label_from_cell(cell) for cell in self.active_targets) or "none"
        pending_rescue_labels = ", ".join(label_from_cell(cell) for cell in self.pending_rescue_targets()) or "none"
        done_rescue_labels = ", ".join(label_from_cell(cell) for cell in sorted(self.rescued_targets)) or "none"
        active_rescue_labels = ", ".join(label_from_cell(cell) for cell in self.active_rescue_targets) or "none"

        mode = "EMERGENCY RETURN" if self.emergency_return else "PLANTING"
        self.summary_var.set(
            f"Mode: {mode} | Robot: {label_from_cell(self.robot_pos)} | "
            f"Planted by robot: {self.planted_count}/{SEEDS_TO_PLANT} | "
            f"World fertile: {fertile_count}, world planted: {planted_count}, available: {available_count}\n"
            f"Reserve targets ({len(self.reserve_targets)}): {reserve_labels}\n"
            f"Active seed targets: {active_labels} | Pending rescue: {pending_rescue_labels} | "
            f"Done rescue: {done_rescue_labels} | Active rescue waypoints: {active_rescue_labels}\n"
            f"Path steps: {self.planned_steps}, "
            f"turns: {self.planned_turns}, seed score: {self.planned_seed_score} | "
            f"Known obstacles: {len(self.known_obstacles)} | Hidden obstacles: {len(self.hidden_obstacles)} | "
            f"Plan reason: {self.last_plan_reason}"
        )


def dump_plan_to_console() -> None:
    cells, source = fetch_server_cells()
    known: Set[Cell] = set()
    planted: Set[Cell] = set()
    reserve = choose_reserve_targets(cells, known, planted, RESERVE_TARGET_COUNT)
    plan = optimize_route(START, reserve, FINISH, known, SEEDS_TO_PLANT)

    print(f"Map source: {source}")
    print(f"Start: {START_CELL}, Finish: {FINISH_CELL}")
    print("Reserve targets:", ", ".join(label_from_cell(cell) for cell in reserve))
    if plan is None:
        print("No route found.")
        return
    print("Active targets:", ", ".join(label_from_cell(cell) for cell in plan.targets))
    print(f"Steps={plan.steps}, turns={plan.turns}, seed_score={plan.seed_score}")
    print("Path:", " -> ".join(label_from_cell(cell) for cell in plan.path))


def main() -> None:
    if "--dump-plan" in sys.argv:
        dump_plan_to_console()
        return

    root = tk.Tk()
    app = PlannerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
