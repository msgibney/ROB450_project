import pybullet as p
import pybullet_data
import time
import random
import math
import numpy as np
import path_planning as plan
import sys, os
import contextlib
import scipy.ndimage as nd
from scipy.spatial import KDTree
from collections import deque
import matplotlib.pyplot as plt

grid_size_const = 100

def world_to_grid(x, y, grid_size=grid_size_const, world_size=0.3048):
    # Map world position to nearest **grid center**
    cell_size = world_size / grid_size
    gx = int(x / cell_size)
    gy = int(y / cell_size)
    # Clamp to valid range
    gx = min(max(gx, 0), grid_size - 1)
    gy = min(max(gy, 0), grid_size - 1)
    return (gy, gx)

def grid_to_world(row, col, grid_size=grid_size_const, world_size=0.3048):
    # Map grid index to **cell center**
    cell_size = world_size / grid_size
    x = (col + 0.5) * cell_size
    y = (row + 0.5) * cell_size
    return (x, y)


def clamp(val, lo, hi):
    return max(lo, min(val, hi))


def generate_thick_maze(grid_size=grid_size_const, path_width=3, seed=None, visualize=False):
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    coarse_size = grid_size // path_width
    if coarse_size % 2 == 0:
        coarse_size -= 1

    # ---- STEP 0: solid walls (for visualization only) ----
    coarse_solid = np.ones((coarse_size, coarse_size), dtype=np.int8)

    # ---- STEP 1: carve maze ----
    coarse = coarse_solid.copy()
    carve_maze(coarse)
    coarse_carved = coarse.copy()

    # ---- STEP 2: add holes ONLY ----
    holes = np.zeros_like(coarse, dtype=np.int8)

    for _ in range((coarse_size * coarse_size) // 50):
        r = random.randint(1, coarse_size - 2)
        c = random.randint(1, coarse_size - 2)
        if coarse[r, c] == 1 and random.random() < 0.2:
            coarse[r, c] = 0
            holes[r, c] = 1   # mark hole only

    coarse_with_holes = coarse.copy()

    # ---- STEP 3: upscale (thin paths) ----
    thin_paths = np.kron(coarse_with_holes,
                         np.ones((path_width, path_width), dtype=np.int8))

    # ---- STEP 4: widen paths ----
    inverted = 1 - thin_paths
    widened = nd.binary_dilation(inverted, iterations=path_width // 3)
    wide_paths = 1 - widened.astype(np.int8)

    if visualize:
        return {
            "coarse_solid": coarse_solid,
            "holes": holes,
            "coarse_carved": coarse_carved,
            "thin_paths": thin_paths,
            "wide_paths": wide_paths
        }

    return wide_paths


def plot_maze(ax, maze, title, vmin=0, vmax=1, cmap="gray_r"):
    ax.imshow(maze, cmap=cmap, vmin=vmin, vmax=vmax)
    ax.set_title(title)
    ax.set_xticks([])
    ax.set_yticks([])


def carve_maze(m):
    rows, cols = m.shape
    m[1:rows-1, 1:cols-1] = 1
    stack = [(1, 1)]
    m[1, 1] = 0
    dirs = [(-2, 0), (2, 0), (0, -2), (0, 2)]
    while stack:
        r, c = stack[-1]
        neigh = []
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if 1 <= nr < rows - 1 and 1 <= nc < cols - 1 and m[nr, nc] == 1:
                neigh.append((nr, nc))
        if neigh:
            nr, nc = random.choice(neigh)
            m[(r + nr)//2, (c + nc)//2] = 0
            m[nr, nc] = 0
            stack.append((nr, nc))
        else:
            stack.pop()


def cluster_links(links, map, cluster_radius=0.02):
    positions = np.array([p.getBasePositionAndOrientation(l)[0][:2] for l in links])

    if len(positions) == 0:
        return []

    tree = KDTree(positions)
    clusters = []
    visited = set()

    for i in range(len(positions)):
        if i in visited:
            continue
        idxs = tree.query_ball_point(positions[i], cluster_radius)
        clusters.append([links[j] for j in idxs])
        visited.update(idxs)

    return clusters


def closest_cluster(ee_pos, clusters):
    if not clusters:
        return None
    min_dist = float('inf')
    closest = None
    for c in clusters:
        cluster_positions = np.array([p.getBasePositionAndOrientation(l)[0][:2] for l in c])
        dist = np.min(np.linalg.norm(cluster_positions - ee_pos[:2], axis=1))
        if dist < min_dist:
            min_dist = dist
            closest = c
    return closest

def cluster_centroid(cluster):
    positions = np.array([p.getBasePositionAndOrientation(l)[0][:2] for l in cluster])
    return np.mean(positions, axis=0)

def find_nearest_free_cell(start, grid):
    rows, cols = grid.shape
    visited = set()
    queue = deque([start])
    while queue:
        r, c = queue.popleft()
        if (r, c) in visited:
            continue
        visited.add((r, c))
        if 0 <= r < rows and 0 <= c < cols:
            if grid[r, c] == 0:
                return (r, c)
            for dr, dc in [(1,0), (-1,0), (0,1), (0,-1)]:
                queue.append((r+dr, c+dc))
    return start

def cluster_near_ee(ee_pos, cluster, threshold):
    for link_id in cluster:
        link_pos = p.getBasePositionAndOrientation(link_id)[0]
        if np.linalg.norm(np.array(link_pos[:2]) - ee_pos[:2]) > threshold:
            return False
    return True


path_width = 7  # each open corridor will be about 10 cells thick
# map, start_cell, goal_cell = generate_thick_maze(grid_size_const, path_width, seed=75)

stages = generate_thick_maze(
    grid_size=100,
    path_width=7,
    seed=75,
    visualize=True
)

fig, axs = plt.subplots(1, 3, figsize=(16, 4))


plot_maze(
    axs[0],
    stages["coarse_solid"],
    "Initial Grid\n(All Walls)",
    vmin=1,
    vmax=1,
    cmap="gray"
)


plot_maze(
    axs[1],
    stages["coarse_carved"],
    "Carved Maze\n(Thin Paths)"
)


plot_maze(
    axs[2],
    stages["wide_paths"],
    "Final Maze\n(Wide Paths)"
)

plt.tight_layout()
plt.show()



# import re
#
# with open("array.txt", "r") as f:
#     txt = f.read()
#
# # Extract everything that looks like a row: "[ ... ]"
# rows = re.findall(r"\[([^\]]+)\]", txt)
#
# matrix = []
# for row in rows:
#     # extract all numbers in that row
#     nums = re.findall(r"[-+]?\d*\.?\d+", row)
#     nums = list(map(float, nums))
#     matrix.append(nums)
#
# map = np.array(matrix, dtype=float)
#
# start_cell = (80, 27)
# goal_cell = (117, 240)

# Convert start/goal to world coordinates
start_world = grid_to_world(start_cell[0], start_cell[1])
goal_world = grid_to_world(goal_cell[0], goal_cell[1])

start_grid = start_cell
goal_grid = goal_cell

# ----------------------------
# Setup
# ----------------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(numSolverIterations=200)
p.setGravity(0, 0, -9.81)
p.setTimeStep(1./240.)


# ----------------------------
# Dimensions
# ----------------------------
one_foot = 0.3048     # 1 ft
rail_thickness = 0.01
rail_height = 0.01
y_rail_length = one_foot
ee_length = 0.02      # cylinder height

inner_length = 0.3048
inner_width = 0.3048
wall_thickness = 0.01
wall_height = 0.1

# Gantry Z shift to avoid collision with container
gantry_z_shift = -0.1

# Container shift to align inner area from 0→1 ft in X and Y
shift_x = inner_length / 2
shift_y = inner_width / 2

# ----------------------------
# Camera
# ----------------------------
center_x = shift_x
center_y = shift_y
center_z = 0.05  # a bit above container for better view
p.resetDebugVisualizerCamera(
    cameraDistance=0.5,
    cameraYaw=0,
    cameraPitch=-89,
    cameraTargetPosition=[center_x, center_y, center_z]
)

# ----------------------------
# Gantry shapes
# ----------------------------
base_shape = p.createCollisionShape(p.GEOM_BOX,
    halfExtents=[one_foot/2, rail_thickness/2, rail_height/2])
base_visual = p.createVisualShape(p.GEOM_BOX,
    halfExtents=[one_foot/2, rail_thickness/2, rail_height/2],
    rgbaColor=[0, 0, 1, 1])

y_rail_shape = p.createCollisionShape(p.GEOM_BOX,
    halfExtents=[rail_thickness/2, y_rail_length/2, rail_height/2])
y_rail_visual = p.createVisualShape(p.GEOM_BOX,
    halfExtents=[rail_thickness/2, y_rail_length/2, rail_height/2],
    rgbaColor=[0, 1, 0, 1])

ee_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.01, height=ee_length)
ee_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.01, length=ee_length, rgbaColor=[1, 0, 0, 1])

# ----------------------------
# Create Gantry
# ----------------------------
gantry = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=base_shape,
    baseVisualShapeIndex=base_visual,
    basePosition=[one_foot/2, 0, gantry_z_shift],

    linkMasses=[1, 0.1],
    linkCollisionShapeIndices=[y_rail_shape, ee_collision],
    linkVisualShapeIndices=[y_rail_visual, ee_visual],

    linkPositions=[
        [-one_foot/2, y_rail_length/2, 0],
        [0, -y_rail_length/2, 0]
    ],
    linkOrientations=[[0,0,0,1]]*2,
    linkInertialFramePositions=[[0,0,0]]*2,
    linkInertialFrameOrientations=[[0,0,0,1]]*2,
    linkParentIndices=[0,1],
    linkJointTypes=[p.JOINT_PRISMATIC, p.JOINT_PRISMATIC],
    linkJointAxis=[[1,0,0],[0,1,0]]
)

# ----------------------------
# Container base
# ----------------------------
container_base_collision = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[inner_length/2, inner_width/2, wall_thickness/2])
container_base_visual = p.createVisualShape(
    p.GEOM_BOX, halfExtents=[inner_length/2, inner_width/2, wall_thickness/2],
    rgbaColor=[0, 0, 1, 0.3])

container_base = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=container_base_collision,
    baseVisualShapeIndex=container_base_visual,
    basePosition=[shift_x, shift_y, wall_thickness/2]
)

# ----------------------------
# Container walls (overlapping corners)
# ----------------------------
# X walls (along length)
x_wall_collision = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[inner_length/2, (wall_thickness + wall_thickness)/2, wall_height/2])
x_wall_visual = p.createVisualShape(
    p.GEOM_BOX, halfExtents=[inner_length/2, (wall_thickness + wall_thickness)/2, wall_height/2],
    rgbaColor=[0,0,1,0.3])
x_wall_y = inner_width/2 + wall_thickness/2
x_wall_top = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=x_wall_collision,
                               baseVisualShapeIndex=x_wall_visual, basePosition=[shift_x, shift_y + x_wall_y, wall_height/2])
x_wall_bottom = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=x_wall_collision,
                                  baseVisualShapeIndex=x_wall_visual, basePosition=[shift_x, shift_y - x_wall_y, wall_height/2])

# Y walls (along width)
y_wall_collision = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[(wall_thickness + wall_thickness)/2, inner_width/2, wall_height/2])
y_wall_visual = p.createVisualShape(
    p.GEOM_BOX, halfExtents=[(wall_thickness + wall_thickness)/2, inner_width/2, wall_height/2],
    rgbaColor=[0,0,1,0.3])
y_wall_right = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=y_wall_collision,
                                 baseVisualShapeIndex=y_wall_visual, basePosition=[shift_x + inner_length/2 + wall_thickness/2, shift_y, wall_height/2])
y_wall_left = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=y_wall_collision,
                                baseVisualShapeIndex=y_wall_visual, basePosition=[shift_x - inner_length/2 - wall_thickness/2, shift_y, wall_height/2])

wall_height_visual = 0.05
cell_size = 0.3048 / grid_size_const
half_cell = cell_size / 2.0

wall_col_shape = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[cell_size/2, cell_size/2, wall_height_visual/2]
)
wall_visual = p.createVisualShape(
    p.GEOM_BOX, halfExtents=[cell_size/2, cell_size/2, wall_height_visual/2],
    rgbaColor=[0, 0, 0, 1]
)

# Create horizontal strips (merge contiguous wall segments in each row)
for r in range(map.shape[0]):
    c = 0
    while c < map.shape[1]:
        if map[r, c] == 1:
            c_start = c
            while c < map.shape[1] and map[r, c] == 1:
                c += 1
            c_end = c - 1
            # Compute world-space center and width
            width = (c_end - c_start + 1) * cell_size
            x_center = (c_start + c_end + 1) / 2 * cell_size
            y_center = (r + 0.5) * cell_size
            # One long wall segment instead of many cubes
            long_col_shape = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[width/2, cell_size/2, wall_height_visual/2]
            )
            long_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[width/2, cell_size/2, wall_height_visual/2],
                rgbaColor=[0, 0, 0, 1]
            )
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=long_col_shape,
                baseVisualShapeIndex=long_visual,
                basePosition=[x_center, y_center, wall_height_visual/2]
            )
        c += 1

# ----------------------------
# Floating link spawn function (fixed inside container)
# ----------------------------
def spawn_floating_sphere(radius=0.003):
    margin = 0.005
    x_min = shift_x - inner_length/2 + margin
    x_max = shift_x + inner_length/2 - margin
    y_min = shift_y - inner_width/2 + margin
    y_max = shift_y + inner_width/2 - margin
    x = random.uniform(x_min, x_max)
    y = random.uniform(y_min, y_max)
    z = wall_thickness + radius + 0.005  # center above base

    # Collision and visual shape
    link_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    link_visual = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=[1,0,0,1])

    link_id = p.createMultiBody(
        baseMass=0.01,
        baseCollisionShapeIndex=link_collision,
        baseVisualShapeIndex=link_visual,
        basePosition=[x, y, z]
    )

    # Dynamics properties
    p.changeDynamics(
        link_id,
        lateralFriction=0.1,
        spinningFriction=0.05,
        rollingFriction=0.05,
        contactStiffness=1000,
        contactDamping=0.5,
        contactProcessingThreshold=0,
        ccdSweptSphereRadius=radius,
        linearDamping=0.05,
        angularDamping=0.05
    )


    return link_id


def spawn_floating_sphere_near_start(radius=0.003, spawn_radius=0.05, max_attempts=50):
    """Spawn a floating sphere near the start position but not inside walls."""
    for _ in range(max_attempts):
        angle = random.uniform(0, 2 * math.pi)
        dist = random.uniform(0, spawn_radius)
        x = start_world[0] + dist * math.cos(angle)
        y = start_world[1] + dist * math.sin(angle)
        grid_r, grid_c = world_to_grid(x, y)
        # Only spawn in free space
        if map[grid_r, grid_c] == 0:
            z = wall_thickness + radius + 0.005
            link_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
            link_visual = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=[1, 0, 0, 1])
            link_id = p.createMultiBody(
                baseMass=0.01,
                baseCollisionShapeIndex=link_collision,
                baseVisualShapeIndex=link_visual,
                basePosition=[x, y, z]
            )
            p.changeDynamics(link_id, -1,
                             lateralFriction=0.1, spinningFriction=0.05,
                             rollingFriction=0.05, contactStiffness=1000,
                             contactDamping=0.5, linearDamping=0.05, angularDamping=0.05)
            return link_id
    return None  # failed to find spot

def spawn_floating_sphere_in_empty_cell(map, radius=0.003):
    free_cells = np.argwhere(map == 0)  # all empty grid cells
    if len(free_cells) == 0:
        raise ValueError("No free cells found in map!")

    # Choose a random free cell
    r, c = random.choice(free_cells)
    x, y = grid_to_world(r, c)
    z = wall_thickness + radius + 0.005

    # Create sphere
    link_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    link_visual = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=[1, 0, 0, 1])
    link_id = p.createMultiBody(
        baseMass=0.01,
        baseCollisionShapeIndex=link_collision,
        baseVisualShapeIndex=link_visual,
        basePosition=[x, y, z]
    )

    # Physics properties
    p.changeDynamics(link_id, -1,
                     lateralFriction=0.1, spinningFriction=0.05,
                     rollingFriction=0.05, contactStiffness=1000,
                     contactDamping=0.5, linearDamping=0.05, angularDamping=0.05)
    return link_id


def pull_links_toward_ee(ee_pos, links, max_force=0.4, falloff_distance=0.1):
    for link_id in links:
        link_pos, _ = p.getBasePositionAndOrientation(link_id)
        vec = np.array(ee_pos) - np.array(link_pos)
        distance = np.linalg.norm(vec)
        if distance > 1e-6:
            direction = vec / distance
            force_magnitude = max_force * math.exp(-(distance / falloff_distance)**2)
            force = force_magnitude * direction
        else:
            force = np.zeros(3)
        p.applyExternalForce(link_id, -1, force, [0,0,0], p.WORLD_FRAME)

def links_near_ee(ee_pos, links, threshold):
    positions = [p.getBasePositionAndOrientation(l)[0] for l in links]
    return all(np.linalg.norm(np.array(pos[:2]) - ee_pos[:2]) < threshold for pos in positions)

def closest_cluster(ee_pos, clusters, astar, map):
    """
    Finds the cluster whose centroid is *closest by A* path distance* from the EE.
    Falls back to Euclidean distance if no valid path exists.
    """
    if not clusters:
        return None

    ee_grid = world_to_grid(ee_pos[0], ee_pos[1])
    min_dist = float('inf')
    closest = None

    for c in clusters:
        centroid = cluster_centroid(c)
        centroid_grid = world_to_grid(centroid[0], centroid[1])

        path = astar.find_path(ee_grid, centroid_grid)
        if path is not None and len(path) > 0:
            dist = len(path)
        else:
            # fallback to Euclidean if A* fails
            dist = np.linalg.norm(np.array(centroid) - ee_pos[:2])

        if dist < min_dist:
            min_dist = dist
            closest = c

    return closest

num_links = 50
link_size = 0.003
links = [spawn_floating_sphere_in_empty_cell(map, radius=link_size / 2) for _ in range(num_links)]

# ----------------------------
# Move EE in square
# ----------------------------
limit = one_foot
positions = [(0,0),(limit,0),(limit,limit),(0,limit)]

#
ee_target_x = p.getJointState(gantry, 0)[0]
ee_target_y = p.getJointState(gantry, 1)[0]
ee_speed = 0.001

with open(os.devnull, "w") as f, contextlib.redirect_stdout(f), contextlib.redirect_stderr(f):
    video_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "maze_clusters_faster_seed75.mp4")

target_x = start_world[0]
target_y = start_world[1]

while True:
    joint_x = p.getJointState(gantry, 0)[0]
    joint_y = p.getJointState(gantry, 1)[0]

    error_x = target_x - joint_x
    error_y = target_y - joint_y

    if abs(error_x) < 1e-4 and abs(error_y) < 1e-4:
        break

    p.setJointMotorControl2(gantry, 0, p.POSITION_CONTROL, targetPosition=target_x, force=100)
    p.setJointMotorControl2(gantry, 1, p.POSITION_CONTROL, targetPosition=target_y, force=100)

    p.stepSimulation()
    time.sleep(1./240.)

astar = plan.AStar(map)

corner_margin = 0.05

# goal_positions = [
#     (corner_margin, corner_margin),
#     (one_foot - corner_margin, one_foot - corner_margin),
#     (one_foot - corner_margin, corner_margin),
#     (corner_margin, one_foot - corner_margin)
# ]

current_goal_idx = 0
# goal = goal_positions[current_goal_idx]

threshold = 0.005

current_waypoint_idx = 1

ee_state = p.getLinkState(gantry, 1)
ee_pos = np.array(ee_state[0])

start_grid = world_to_grid(ee_pos[0], ee_pos[1])
goal_grid = world_to_grid(goal_world[0], goal_world[1])

path = astar.find_path(start_grid, goal_grid)

print(path)

waypoint_grid = path[current_waypoint_idx]
waypoint_world = np.array(grid_to_world(waypoint_grid[0], waypoint_grid[1]))

distance_to_next_waypoint = np.linalg.norm(ee_pos[:2] - waypoint_world[:2])

target_x = clamp(waypoint_world[0], 0, limit)
target_y = clamp(waypoint_world[1], 0, limit)
p.setJointMotorControl2(gantry, 0, p.POSITION_CONTROL, targetPosition=target_x, force=100)
p.setJointMotorControl2(gantry, 1, p.POSITION_CONTROL, targetPosition=target_y, force=100)

magnet_threshold = 0.02
goal_threshold = 0.005
cluster_radius = 0.02
ee_speed = 0.001

all_links = links.copy()

# while not all(visited_goals):
#     # keys = p.getKeyboardEvents()
#     #
#     # if p.B3G_LEFT_ARROW in keys:
#     #     ee_target_x -= ee_speed
#     # if p.B3G_RIGHT_ARROW in keys:
#     #     ee_target_x += ee_speed
#     # if p.B3G_UP_ARROW in keys:
#     #     ee_target_y += ee_speed
#     # if p.B3G_DOWN_ARROW in keys:
#     #     ee_target_y -= ee_speed
#
#     # goal = goal_positions[current_goal_idx]
#     goal = goal_world
#
#     ee_state = p.getLinkState(gantry, 1)
#     ee_pos = np.array(ee_state[0])
#
#     if links:
#         positions = [p.getBasePositionAndOrientation(link_id)[0] for link_id in links]
#         avg_pos = np.mean(positions, axis=0)
#
#     start_grid = world_to_grid(ee_pos[0], ee_pos[1])
#     goal_grid = world_to_grid(goal[0], goal[1])
#
#     if not path or current_waypoint_idx >= len(path) or distance_to_next_waypoint < threshold*10:
#         path = astar.find_path(start_grid, goal_grid)
#         if not path:
#             break
#         current_waypoint_idx = 0
#
#     waypoint_grid = path[0]
#     if waypoint_grid[0] == start_grid[0] and waypoint_grid[1] == start_grid[1] and len(path) > 1:
#         waypoint_grid = path[1]
#     waypoint_world = np.array(grid_to_world(waypoint_grid[0], waypoint_grid[1]))
#
#     # print(start_grid, goal_grid, waypoint_grid, path)
#     # print(ee_pos, waypoint_world)
#     distance_to_next_waypoint = np.linalg.norm(ee_pos[:2] - waypoint_world[:2])
#
#     target_x = clamp(waypoint_world[0], 0, limit)
#     target_y = clamp(waypoint_world[1], 0, limit)
#     p.setJointMotorControl2(gantry, 0, p.POSITION_CONTROL, targetPosition=target_x, force=100)
#     p.setJointMotorControl2(gantry, 1, p.POSITION_CONTROL, targetPosition=target_y, force=100)
#
#     if distance_to_next_waypoint < threshold:
#         current_waypoint_idx += 1
#         if current_waypoint_idx >= len(path):
#             # visited_goals[current_goal_idx] = True
#             # current_goal_idx = (current_goal_idx + 1) % len(goal_positions)
#             current_waypoint_idx = 0
#             path = None
#             break
#
#     for link_id in links:
#         link_pos, _ = p.getBasePositionAndOrientation(link_id)
#         vec = [ee_pos[i] - link_pos[i] for i in range(3)]
#         distance = sum(v**2 for v in vec) ** 0.5
#
#         if distance > 0.0001:
#             direction = [v / distance for v in vec]
#             max_force = 0.4
#             falloff_distance = 0.1
#             force_magnitude = max_force * math.exp(- (distance / falloff_distance)**2)
#             force = [force_magnitude * d for d in direction]
#         else:
#             force = [0, 0, 0]
#
#         p.applyExternalForce(link_id, -1, force, [0, 0, 0], p.WORLD_FRAME)
#
#     p.stepSimulation()
#     time.sleep(1./240.)
uncollected_links = all_links.copy()

ee_state = p.getLinkState(gantry, 1)
ee_pos = np.array(ee_state[0])

clusters = cluster_links(uncollected_links, cluster_radius)
target_cluster = closest_cluster(ee_pos, clusters, astar, map)

prev_goal = [0, 0]
prev_start = [0, 0]
index = 0
last_update = 0
while not links_near_ee(p.getLinkState(gantry, 1)[0][:2], all_links, magnet_threshold):
    ee_state = p.getLinkState(gantry, 1)
    ee_pos = np.array(ee_state[0])

    pull_links_toward_ee(ee_pos, all_links)

    clusters = cluster_links(uncollected_links, cluster_radius)

    if not clusters:
        break

    if cluster_near_ee(ee_pos, target_cluster, magnet_threshold):
        for link in target_cluster:
            if link in uncollected_links:
                uncollected_links.remove(link)

        target_cluster = closest_cluster(ee_pos, clusters, astar, map)
        continue

    if target_cluster:
        centroid = np.clip(cluster_centroid(target_cluster), 0, 0.3048)
        ee_grid = world_to_grid(ee_pos[0], ee_pos[1])
        target_grid = world_to_grid(centroid[0], centroid[1])

        start = world_to_grid(ee_pos[0], ee_pos[1])
        goal = world_to_grid(centroid[0], centroid[1])

        if map[start[0], start[1]] == 1:
            start = find_nearest_free_cell(start, map)
        if map[goal[0], goal[1]] == 1:
            goal = find_nearest_free_cell(goal, map)

        if (goal[0] != prev_goal[0] or goal[1] != prev_goal[1]) or index >= len(path) - 3 or last_update >= 5:
            path = astar.find_path(start, goal)
            last_update = 0
            index = 0
        elif ee_grid[0] == next_waypoint[0] and ee_grid[1] == next_waypoint[1]:
            index = index + 1
        else:
            last_update += 1

        prev_goal = goal

        if path is None or len(path) == 1:
            next_waypoint = target_grid
        elif len(path) > 2:
            next_waypoint = path[2+index]
        elif len(path) > 1:
            next_waypoint = path[1]
        else:
            next_waypoint = path[0]

        waypoint_world = np.array(grid_to_world(next_waypoint[0], next_waypoint[1]))
        target_x = clamp(waypoint_world[0], 0, limit)
        target_y = clamp(waypoint_world[1], 0, limit)

        p.setJointMotorControl2(gantry, 0, p.POSITION_CONTROL, targetPosition=target_x, force=100, maxVelocity=0.1)
        p.setJointMotorControl2(gantry, 1, p.POSITION_CONTROL, targetPosition=target_y, force=100, maxVelocity=0.1)

        p.stepSimulation()
    # else:
    #     target_x = ee_pos[0]
    #     target_y = ee_pos[1]
    # time.sleep(1./240.)
index = 0
last_update = 0
prev_goal = [0, 0]
while True:
    ee_state = p.getLinkState(gantry, 1)
    ee_pos = np.array(ee_state[0])

    pull_links_toward_ee(ee_pos, all_links)

    ee_grid = world_to_grid(ee_pos[0], ee_pos[1])
    goal = world_to_grid(goal_world[0], goal_world[1])

    if (goal[0] != prev_goal[0] or goal[1] != prev_goal[1]) or index >= len(path) - 3 or last_update >= 10:
        path = astar.find_path(ee_grid, goal)
        last_update = 0
        index = 0
    elif ee_grid[0] == next_waypoint[0] and ee_grid[1] == next_waypoint[1]:
        index = index + 1
    else:
        last_update += 1

    prev_goal = goal

    if path is None or len(path) == 1:
        next_waypoint = target_grid
    elif len(path) > 2:
        next_waypoint = path[2 + index]
    elif len(path) > 1:
        next_waypoint = path[1]
    else:
        next_waypoint = path[0]

    waypoint_world = np.array(grid_to_world(next_waypoint[0], next_waypoint[1]))
    target_x = clamp(waypoint_world[0], 0, limit)
    target_y = clamp(waypoint_world[1], 0, limit)

    p.setJointMotorControl2(gantry, 0, p.POSITION_CONTROL, targetPosition=target_x, force=100, maxVelocity=0.5)
    p.setJointMotorControl2(gantry, 1, p.POSITION_CONTROL, targetPosition=target_y, force=100, maxVelocity=0.5)

    distance_to_goal = np.linalg.norm(ee_pos[:2] - goal_world[:2])
    if distance_to_goal < goal_threshold:
        break

    p.stepSimulation()
    time.sleep(1./240.)

p.stopStateLogging(video_id)
