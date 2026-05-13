"""A* path planner for VacuSim.

This node accepts a start and goal pose in tile coordinates, plans over a
numeric occupancy grid, and returns a queue of waypoint poses that the robot
controller can follow.
"""

from __future__ import annotations

import heapq
import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from vacusim_robot_interfaces.srv import PlanPath

GridCell = Tuple[int, int]
GridMap = List[List[int]]

TRAVERSABLE_VALUES = set(range(1, 9))
BLOCKED_VALUES = {0, 9}


class PlannerNode(Node):
    """ROS 2 service node that calculates A* plans over a numeric map."""

    def __init__(self) -> None:
        super().__init__('vacusim_planner')

        default_map_path = Path(
            get_package_share_directory('vacusim_robot_planning')
        ) / 'maps' / 'default_map.txt'

        self.declare_parameter('map_path', str(default_map_path))
        self.declare_parameter('service_name', 'plan_path')
        self.declare_parameter('robot_buffer', 0.5)

        map_path = Path(self.get_parameter('map_path').value).expanduser()
        if not map_path.is_absolute():
            map_path = (Path.cwd() / map_path).resolve()
        else:
            map_path = map_path.resolve()

        self._grid = self._load_grid(map_path)
        self._height = len(self._grid)
        self._width = len(self._grid[0]) if self._grid else 0
        self._robot_buffer = float(self.get_parameter('robot_buffer').value)
        self._traversable = self._build_traversability_grid(self._robot_buffer)
        self._service = self.create_service(
            PlanPath,
            self.get_parameter('service_name').value,
            self._handle_plan_request,
        )

        self.get_logger().info(
            f'Loaded planning map {map_path} with size {self._width}x{self._height}.'
        )
        self.get_logger().info(
            f'Using obstacle buffer of {self._robot_buffer:.2f} tile(s).'
        )
        self.get_logger().info(
            f'Serving A* plans on "/{self.get_parameter("service_name").value}".'
        )

    def _build_traversability_grid(self, buffer_tiles: float) -> List[List[bool]]:
        traversable: List[List[bool]] = [
            [False for _ in range(self._width)] for _ in range(self._height)
        ]

        for y in range(self._height):
            for x in range(self._width):
                traversable[y][x] = self._is_cell_traversable_with_buffer(x, y, buffer_tiles)

        return traversable

    def _is_cell_traversable_with_buffer(self, x: int, y: int, buffer_tiles: float) -> bool:
        if self._grid[y][x] in BLOCKED_VALUES:
            return False

        center_x = x + 0.5
        center_y = y + 0.5

        # Only nearby blocked cells can violate a small buffer.
        min_x = max(0, x - 1)
        max_x = min(self._width - 1, x + 1)
        min_y = max(0, y - 1)
        max_y = min(self._height - 1, y + 1)

        for ny in range(min_y, max_y + 1):
            for nx in range(min_x, max_x + 1):
                if self._grid[ny][nx] not in BLOCKED_VALUES:
                    continue

                # Distance from cell center to blocked tile rectangle [nx, nx+1] x [ny, ny+1].
                nearest_x = min(max(center_x, nx), nx + 1.0)
                nearest_y = min(max(center_y, ny), ny + 1.0)
                distance = math.hypot(center_x - nearest_x, center_y - nearest_y)

                if distance <= buffer_tiles:
                    return False

        return True

    def _load_grid(self, map_path: Path) -> GridMap:
        if not map_path.exists():
            raise FileNotFoundError(f'Map file does not exist: {map_path}')

        rows: GridMap = []
        with map_path.open('r', encoding='utf-8') as map_file:
            for line_number, raw_line in enumerate(map_file, start=1):
                line = raw_line.strip()
                if not line:
                    continue

                row = self._parse_row(line, map_path, line_number)
                rows.append(row)

        if not rows:
            raise ValueError(f'Map file is empty: {map_path}')

        width = len(rows[0])
        for row in rows:
            if len(row) != width:
                raise ValueError(
                    f'Map file {map_path} is not rectangular: expected width {width}'
                )

        return rows

    def _parse_row(self, line: str, map_path: Path, line_number: int) -> List[int]:
        row: List[int] = []
        for column_number, char in enumerate(line, start=1):
            if not char.isdigit():
                raise ValueError(
                    f'Invalid character {char!r} in {map_path} at line {line_number}, '
                    f'column {column_number}'
                )
            row.append(int(char))
        return row

    def _handle_plan_request(self, request: PlanPath.Request, response: PlanPath.Response):
        start_cell = self._pose_to_cell(request.start)
        goal_cell = self._pose_to_cell(request.goal)

        # Replanning can legitimately start from a currently occupied/buffered cell.
        validation_error = self._validate_cell(start_cell, 'start', allow_occupied=True)
        if validation_error is not None:
            response.success = False
            response.message = validation_error
            response.waypoints = []
            return response

        validation_error = self._validate_cell(goal_cell, 'goal')
        if validation_error is not None:
            response.success = False
            response.message = validation_error
            response.waypoints = []
            return response

        cell_path = self._find_path(start_cell, goal_cell)
        if not cell_path:
            response.success = False
            response.message = (
                f'No path found between {start_cell} and {goal_cell} on the loaded map.'
            )
            response.waypoints = []
            return response

        waypoints = self._cells_to_waypoints(cell_path)
        response.success = True
        response.message = f'Planned {len(waypoints)} waypoint(s).'
        response.waypoints = waypoints
        return response

    def _pose_to_cell(self, pose: Pose2D) -> GridCell:
        return int(round(pose.x)), int(round(pose.y))

    def _validate_cell(
        self,
        cell: GridCell,
        label: str,
        allow_occupied: bool = False,
    ) -> Optional[str]:
        x, y = cell
        if x < 0 or y < 0 or x >= self._width or y >= self._height:
            return f'{label.capitalize()} pose {cell} is outside the loaded map bounds.'

        if not allow_occupied and not self._traversable[y][x]:
            return f'{label.capitalize()} pose {cell} is located on an occupied cell.'

        return None

    def _find_path(self, start: GridCell, goal: GridCell) -> List[GridCell]:
        if start == goal:
            return [start]

        open_heap: List[Tuple[float, int, GridCell]] = []
        heap_counter = 0
        heapq.heappush(open_heap, (self._heuristic(start, goal), heap_counter, start))

        came_from: Dict[GridCell, GridCell] = {}
        g_costs: Dict[GridCell, float] = {start: 0.0}
        closed_set = set()

        while open_heap:
            _, _, current = heapq.heappop(open_heap)

            if current in closed_set:
                continue
            closed_set.add(current)

            if current == goal:
                return self._reconstruct_path(came_from, current)

            for neighbor in self._neighbors(current):
                if neighbor in closed_set:
                    continue

                tentative_cost = g_costs[current] + 1.0
                if tentative_cost >= g_costs.get(neighbor, math.inf):
                    continue

                came_from[neighbor] = current
                g_costs[neighbor] = tentative_cost
                heap_counter += 1
                priority = tentative_cost + self._heuristic(neighbor, goal)
                heapq.heappush(open_heap, (priority, heap_counter, neighbor))

        return []

    def _reconstruct_path(
        self,
        came_from: Dict[GridCell, GridCell],
        current: GridCell,
    ) -> List[GridCell]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _neighbors(self, cell: GridCell) -> Iterable[GridCell]:
        x, y = cell
        candidates = (
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1),
        )

        for candidate_x, candidate_y in candidates:
            if not self._is_in_bounds(candidate_x, candidate_y):
                continue
            if not self._traversable[candidate_y][candidate_x]:
                continue
            yield candidate_x, candidate_y

    def _is_in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self._width and 0 <= y < self._height

    def _heuristic(self, cell: GridCell, goal: GridCell) -> float:
        return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

    def _cells_to_waypoints(self, cells: Sequence[GridCell]) -> List[Pose2D]:
        if not cells:
            return []

        # For single cell (start == goal), return the goal position as a waypoint
        if len(cells) == 1:
            pose = Pose2D()
            pose.x = cells[0][0] + 0.5
            pose.y = cells[0][1] + 0.5
            pose.theta = 0.0
            return [pose]

        # Compress path to turning points only
        turning_points = self._compress_cells(cells)
        if not turning_points:
            return []

        waypoints: List[Pose2D] = []
        for index, cell in enumerate(turning_points):
            pose = Pose2D()
            pose.x = cell[0] + 0.5
            pose.y = cell[1] + 0.5
            if index + 1 < len(turning_points):
                next_cell = turning_points[index + 1]
                pose.theta = math.atan2(
                    -(next_cell[1] - cell[1]),
                    next_cell[0] - cell[0],
                )
            else:
                pose.theta = 0.0
            waypoints.append(pose)

        return waypoints

    def _compress_cells(self, cells: Sequence[GridCell]) -> List[GridCell]:
        # For very short paths, return as-is (skipping start)
        if len(cells) <= 2:
            return list(cells[1:])  # Return just goal(s)

        # For longer paths, skip start and find turning points
        turning_points: List[GridCell] = [cells[1]]
        previous_direction = self._direction(cells[0], cells[1])

        for index in range(2, len(cells)):
            current_direction = self._direction(cells[index - 1], cells[index])
            if current_direction != previous_direction:
                turning_points.append(cells[index - 1])
                previous_direction = current_direction

        # Always include the goal
        if turning_points[-1] != cells[-1]:
            turning_points.append(cells[-1])

        return turning_points

    def _direction(self, start: GridCell, end: GridCell) -> GridCell:
        return end[0] - start[0], end[1] - start[1]


def main() -> None:
    rclpy.init()
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
