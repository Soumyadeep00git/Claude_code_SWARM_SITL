"""
Simplified Hybrid A* path planner for collision-free navigation among peer drones.

Operates in 2D NED (north-east plane). Treats peer drones as circular obstacles.
Short-circuits to direct path when distance is small or path is clear.
"""

import numpy as np
import heapq
import logging
from math import pi

log = logging.getLogger(__name__)


class HybridAStarPlanner:
    """Grid-based Hybrid A* with continuous heading and peer avoidance."""

    def __init__(self, config: dict):
        self.cell_size = config.get("GRID_CELL_M", 2.0)
        self.obstacle_radius = config.get("OBSTACLE_RADIUS_M", 5.0)
        self.grid_extent = config.get("GRID_EXTENT_M", 80.0)
        self.num_headings = config.get("NUM_HEADINGS", 8)
        self.step_length = config.get("STEP_LENGTH_M", 3.0)
        self.max_nodes = config.get("MAX_NODES", 2000)

        # Steering options: sharp left, left, straight, right, sharp right
        self._steer_options = np.array([-pi / 4, -pi / 8, 0, pi / 8, pi / 4])

    def plan(self, start_ne, goal_ne, peer_positions_ne, start_heading=0.0):
        """
        Plan a path from start to goal avoiding peer obstacles.

        Args:
            start_ne: (2,) [north, east] in meters
            goal_ne: (2,) [north, east] in meters
            peer_positions_ne: (P, 2) peer positions [north, east]
            start_heading: current heading in radians

        Returns:
            List of (north, east) waypoints from near start to goal.
            Returns [goal] if direct path is feasible or distance < 10m.
        """
        start = np.asarray(start_ne, dtype=np.float64)
        goal = np.asarray(goal_ne, dtype=np.float64)

        dist = np.linalg.norm(goal - start)

        # Short-circuit: close enough for MPPI to handle directly
        if dist < 10.0:
            return [tuple(goal)]

        # Short-circuit: direct path is clear of obstacles
        if self._is_path_clear(start, goal, peer_positions_ne):
            return [tuple(goal)]

        # Run full A* search
        return self._astar(start, goal, peer_positions_ne, start_heading)

    def _is_path_clear(self, start, goal, peers):
        """Check if straight line from start to goal avoids all peer obstacles."""
        if peers is None or len(peers) == 0:
            return True

        direction = goal - start
        length = np.linalg.norm(direction)
        if length < 1e-3:
            return True
        direction /= length

        peers = np.asarray(peers)
        for p in peers:
            pt = p[:2]
            # Project peer onto line segment
            t = np.dot(pt - start, direction)
            t = np.clip(t, 0.0, length)
            closest = start + t * direction
            dist = np.linalg.norm(pt - closest)
            if dist < self.obstacle_radius:
                return False
        return True

    def _astar(self, start, goal, peers, start_heading):
        """Hybrid A* search. Returns list of (north, east) waypoints."""
        peers_arr = np.asarray(peers) if peers is not None and len(peers) > 0 else None

        def discretize(pos, heading):
            ci = int(round(pos[0] / self.cell_size))
            cj = int(round(pos[1] / self.cell_size))
            ch = int(round(heading / (2 * pi / self.num_headings))) % self.num_headings
            return (ci, cj, ch)

        def heuristic(pos):
            return np.linalg.norm(pos - goal)

        def is_valid(pos):
            if abs(pos[0]) > self.grid_extent or abs(pos[1]) > self.grid_extent:
                return False
            if peers_arr is not None:
                dists = np.linalg.norm(peers_arr[:, :2] - pos, axis=1)
                if np.any(dists < self.obstacle_radius):
                    return False
            return True

        # Priority queue: (f_cost, counter, position, heading, discrete_key)
        counter = 0
        open_set = []
        g_cost = {}
        came_from = {}  # key -> (continuous_pos, parent_key)

        start_key = discretize(start, start_heading)
        g_cost[start_key] = 0.0
        came_from[start_key] = (start.copy(), None)
        h = heuristic(start)
        heapq.heappush(open_set, (h, counter, start.copy(), start_heading, start_key))
        counter += 1

        goal_key = None
        nodes_explored = 0

        while open_set and nodes_explored < self.max_nodes:
            f, _, pos, heading, key = heapq.heappop(open_set)
            nodes_explored += 1

            # Already found a better path to this cell
            if g_cost.get(key, float('inf')) < f - heuristic(pos) - 1e-6:
                continue

            # Goal reached: within 1.5× step length
            if np.linalg.norm(pos - goal) < self.step_length * 1.5:
                goal_key = key
                break

            # Expand neighbors
            for steer in self._steer_options:
                new_heading = (heading + steer) % (2 * pi)
                dx = self.step_length * np.cos(new_heading)
                dy = self.step_length * np.sin(new_heading)
                new_pos = pos + np.array([dx, dy])

                if not is_valid(new_pos):
                    continue

                new_key = discretize(new_pos, new_heading)
                step_cost = self.step_length + abs(steer) * 0.5
                new_g = g_cost[key] + step_cost

                if new_key not in g_cost or new_g < g_cost[new_key]:
                    g_cost[new_key] = new_g
                    came_from[new_key] = (new_pos.copy(), key)
                    f_new = new_g + heuristic(new_pos)
                    heapq.heappush(open_set, (f_new, counter, new_pos, new_heading, new_key))
                    counter += 1

        # Reconstruct path
        if goal_key is None:
            log.debug("Hybrid A* found no path in %d nodes, using direct", nodes_explored)
            return [tuple(goal)]

        path = []
        key = goal_key
        while key is not None:
            pos, parent_key = came_from[key]
            path.append(tuple(pos))
            key = parent_key
        path.reverse()
        path.append(tuple(goal))

        # Simplify: remove waypoints too close together
        simplified = [path[0]]
        for wp in path[1:]:
            if np.linalg.norm(np.array(wp) - np.array(simplified[-1])) > self.cell_size:
                simplified.append(wp)
        if simplified[-1] != path[-1]:
            simplified.append(path[-1])

        log.debug("Hybrid A* path: %d waypoints (%d nodes explored)",
                  len(simplified), nodes_explored)
        return simplified
