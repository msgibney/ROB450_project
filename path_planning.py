import heapq
import numpy as np
from scipy.ndimage import distance_transform_edt


class Node:
    def __init__(self, position):
        self.position = position
        self.g = np.inf
        self.h = 0
        self.f = np.inf
        self.parent = None

    def __lt__(self, other):
        return self.f < other.f


class AStar:
    def __init__(self, grid, proximity_weight=3.0, safety_radius=0.04, world_size=0.3048):
        """
        grid: 2D numpy array (0 = free, 1 = obstacle)
        proximity_weight: how strongly to avoid walls (increase for more avoidance)
        safety_radius: in meters; how much clearance around obstacles
        world_size: physical size of the whole world (same scale as grid mapping)
        """
        self.grid = np.array(grid, dtype=np.uint8)
        self.rows, self.cols = self.grid.shape
        self.nodes = {}

        # Compute distance (in grid cells) from nearest obstacle
        self.distance_from_wall = distance_transform_edt(self.grid == 0)

        # Convert safety radius to equivalent number of grid cells
        self.cell_size = world_size / self.rows
        self.safety_cells = safety_radius / self.cell_size

        self.proximity_weight = proximity_weight

    def get_node(self, position):
        if position not in self.nodes:
            self.nodes[position] = Node(position)
        return self.nodes[position]

    def heuristic(self, pos, goal):
        # Manhattan distance (good for grid movement)
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])

    def get_neighbors(self, pos):
        row, col = pos
        moves = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        neighbors = []
        for dr, dc in moves:
            r, c = row + dr, col + dc
            if 0 <= r < self.rows and 0 <= c < self.cols and self.grid[r, c] == 0:
                neighbors.append((r, c))
        return neighbors

    def reconstruct_path(self, node):
        path = []
        while node:
            path.append(node.position)
            node = node.parent
        return path[::-1]

    def find_path(self, start, goal):
        self.nodes = {}
        open_set = []
        closed_set = set()

        start_node = self.get_node(start)
        start_node.g = 0
        start_node.h = self.heuristic(start, goal)
        start_node.f = start_node.g + start_node.h

        heapq.heappush(open_set, (start_node.f, start_node))

        while open_set:
            _, current = heapq.heappop(open_set)

            if current.position == goal:
                return self.reconstruct_path(current)

            closed_set.add(current.position)

            for neighbor_pos in self.get_neighbors(current.position):
                if neighbor_pos in closed_set:
                    continue

                neighbor = self.get_node(neighbor_pos)
                base_cost = 1.0

                wall_distance = self.distance_from_wall[neighbor_pos]

                # Apply a strong penalty if within the safety radius
                if wall_distance < self.safety_cells:
                    proximity_penalty = self.proximity_weight * (
                        (self.safety_cells - wall_distance) / self.safety_cells
                    ) ** 2  # quadratic increase near walls
                else:
                    proximity_penalty = 0.0

                tentative_g = current.g + base_cost + proximity_penalty

                if tentative_g < neighbor.g:
                    neighbor.parent = current
                    neighbor.g = tentative_g
                    neighbor.h = self.heuristic(neighbor.position, goal)
                    neighbor.f = neighbor.g + neighbor.h
                    heapq.heappush(open_set, (neighbor.f, neighbor))

        return None