import pybullet as p
import pybullet_data
import time
import random
import math

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
        link_id, -1,
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


num_links = 50  # number of floating links
link_size = 0.003
links = []

for _ in range(num_links):
    link_id = spawn_floating_sphere(link_size/2)
    links.append(link_id)

# ----------------------------
# Move EE in square
# ----------------------------
limit = one_foot
positions = [(0,0),(limit,0),(limit,limit),(0,limit)]

def clamp(val, lo, hi):
    return max(lo, min(val, hi))

#
ee_target_x = p.getJointState(gantry, 0)[0]
ee_target_y = p.getJointState(gantry, 1)[0]
ee_speed = 0.001

video_id = p.startStateLogging(
    p.STATE_LOGGING_VIDEO_MP4,
    "gantry_sim.mp4"
)

while True:
    keys = p.getKeyboardEvents()

    if p.B3G_LEFT_ARROW in keys:
        ee_target_x -= ee_speed
    if p.B3G_RIGHT_ARROW in keys:
        ee_target_x += ee_speed
    if p.B3G_UP_ARROW in keys:
        ee_target_y += ee_speed
    if p.B3G_DOWN_ARROW in keys:
        ee_target_y -= ee_speed

    # Clamp to gantry limits
    ee_target_x = clamp(ee_target_x, 0, limit)
    ee_target_y = clamp(ee_target_y, 0, limit)

    # Set joint positions
    p.setJointMotorControl2(gantry, 0, p.POSITION_CONTROL, targetPosition=ee_target_x, force=100)
    p.setJointMotorControl2(gantry, 1, p.POSITION_CONTROL, targetPosition=ee_target_y, force=100)

    p.stepSimulation()

    # Magnet code stays the same
    ee_state = p.getLinkState(gantry, 1)
    ee_pos = ee_state[0]

    for link_id in links:
        link_pos, _ = p.getBasePositionAndOrientation(link_id)
        vec = [ee_pos[i] - link_pos[i] for i in range(3)]
        distance = sum(v**2 for v in vec) ** 0.5

        if distance > 0.0001:
            direction = [v / distance for v in vec]
            max_force = 0.4
            falloff_distance = 0.1
            force_magnitude = max_force * math.exp(- (distance / falloff_distance)**2)
            force = [force_magnitude * d for d in direction]
        else:
            force = [0, 0, 0]

        p.applyExternalForce(link_id, -1, force, [0, 0, 0], p.WORLD_FRAME)

    time.sleep(1./240.)

p.stopStateLogging(video_id)
