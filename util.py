import logging
import sys
import cv2
import imutils
import numpy as np
from PIL import Image
import threading
from queue import Queue
import queue
import json
from datetime import datetime
import faulthandler
import math
import time
import path_planning
faulthandler.enable()

timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
filename = f"coord_log_{timestamp}.json"
filename2 = f"gant_log_{timestamp}.json"
vid_filename = f"video_log_{timestamp}.avi"



CAMERA_ID = 0

logger = logging.getLogger(__name__)

logger.setLevel(logging.INFO)
handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

calib = np.load("camera_calibration.npz")
CAMERA_MATRIX = calib["camera_matrix"]
DIST_COEFFS = calib["dist_coeffs"]

COORDINATES = []
coordinate_lock = threading.Lock()
WALLS = []
OBJECTS = []
TRAVERSING_MAZE = True
CALIBRATING = True
frame_queue = Queue(maxsize=1)

gantry_loc = [0, 0]
startY = 80
endY = 1000
startX = 100
endX = 1800
x_len = endX - startX
y_len = endY - startY

# m_x = 0.880
# b_x = 2.50
# m_y = 0.757
# b_y = -10.0
a = 0.8637257028391621
b = -0.032804896126967784 
tx = 10.699618897967925
c = 0.021941375266590568 
d = 0.7979861334949228 
ty = -20.55702736535999

def gantry_to_grid(x, y):
    gx = a*x + b*y + tx
    gy = c*x + d*y + ty
    return [gx, gy]

def grid_to_gantry(gx, gy):
    inv_mat = np.linalg.inv(np.array([[a, b], [c, d]]))
    x, y = inv_mat @ (np.array([gx, gy]) - np.array([tx, ty]))
    return [x, y]

def set_gantry(x, y):
    grid = gantry_to_grid(x, y)
    gantry_loc[0] = grid_to_camera(grid[0], grid[1])[0]
    gantry_loc[1] = grid_to_camera(grid[0], grid[1])[1]

def get_logger():
    return logger

def get_finished_walls():
    if len(WALLS) == 0:
        return None
    return WALLS

def get_finished_objects():
    if len(OBJECTS) == 0:
        return None
    return OBJECTS


def get_walls(frame):
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([100, 100, 0])
    upper_blue = np.array([160, 255, 255])
    blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
    # cv2.imshow("walls", cv2.resize(blue_mask, (1536, 864)))

    # img = Image.fromarray(blue_mask)
    resized_img = cv2.resize(blue_mask, (400, 200), interpolation=cv2.INTER_NEAREST)
    #cv2.imshow("walls", resized_img)

    return np.array(resized_img, dtype=np.int8)

def get_objects(
    frame,
    lower_green=(50, 50, 50),
    upper_green=(100, 255, 255),
    min_area=20,
    max_dist=200
):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, np.array(lower_green), np.array(upper_green))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    contours = [c for c in contours if cv2.contourArea(c) > min_area]

    if len(contours) == 0:
        print("no contours")
        return []

    centroids = []

    for cnt in contours:
        M = cv2.moments(cnt)

        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centroids.append((cx, cy))

    if len(centroids) == 0:
        print("no centroids")
        return []

    centroids = np.array(centroids)
    mean_center = np.mean(centroids, axis=0)

    print(len(centroids))

    filtered = []

    for (cx, cy) in centroids:
        dist = np.linalg.norm([cx - mean_center[0], cy - mean_center[1]])

        # if dist < max_dist:
        filtered.append(camera_to_grid(cx, cy))

    # print(filtered)

    return filtered

def camera_to_grid(cX, cY):
    return [(x_len-cX)*400/(x_len),(y_len - cY)*200/(y_len)]

def grid_to_camera(cX, cY):
    return [x_len - (cX*x_len/(400)),y_len - (cY*y_len/(200))]

def coord_log(coords):
    with open(filename, "a") as f:
        json.dump(coords, f)
        f.write("\n")

def gant_log(coord):
    with open(filename2, "a") as f:
        json.dump(coord, f)
        f.write("\n")

def prompt_for_filenames():
    global filename, filename2, vid_filename

    user_input = input("Enter a base filename (or press Enter to keep default): ").strip()
    if user_input:
        filename = f"{user_input}_coord_log.json"
        filename2 = f"{user_input}_gant_log.json"
        vid_filename = f"{user_input}_video_log.avi"
    else:
        pass
    
def locate_bots():
    logger.info("initializing camera")
    cam = cv2.VideoCapture(CAMERA_ID, cv2.CAP_ANY)

    # Get the default frame width and height
    frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
    # frame_width = 2000
    # frame_height = 1100

    cam.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

    # cam.set(cv2.CAP_PROP_FPS, 30)

    crop_w = endX - startX
    crop_h = endY - startY

    logger.info("Finished Initializing")
    first_loop = True
    global TRAVERSING_MAZE

    out = None

    frame_time = 1/30

    frame_count = 0
    start = time.time()
    

    while TRAVERSING_MAZE:
        # cam_fps = cam.get(cv2.CAP_PROP_FPS)
        # logger.info(f"Camera FPS setting: {cam_fps}")
        ret, frame = cam.read()

        frame_count += 1

        if not TRAVERSING_MAZE:
            break

        if not ret or frame is None:
            continue

        frame = cv2.undistort(frame, CAMERA_MATRIX, DIST_COEFFS)

        frame = cv2.flip(frame, -1)

        frame = frame[startY:endY, startX:endX]

        if out is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            out = cv2.VideoWriter(vid_filename, fourcc, 16.0, (crop_w, crop_h))
            if not out.isOpened():
                break

        if first_loop:
            global WALLS
            global OBJECTS
            WALLS = get_walls(frame)
            OBJECTS = get_objects(frame)

            first_loop = False

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 23, 73)

        cnts, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        coordinates = []
        for c in cnts:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
                continue
            coordinates.append(camera_to_grid(cX, cY))

        coord_log(coordinates)
        gant_log(gantry_loc)

        coordinate_lock.acquire()
        global COORDINATES
        # global OBJECTS
        COORDINATES = coordinates
        OBJECTS = get_objects(frame)
        coordinate_lock.release()

        # cv2.drawContours(frame, cnts, -1, (0, 0, 255), 3)

        # cv2.circle(frame, (int(gantry_loc[0]), int(gantry_loc[1])), 20, (0, 0, 255), 2) 

        out.write(frame)

        if frame_count % 30 == 0:
            elapsed = time.time() - start
            # logger.info(f"Pipeline FPS: {frame_count / elapsed:.2f}")

        try:
            frame_queue.put_nowait(frame)
        except queue.Full:
            frame_queue.get_nowait()
            frame_queue.put_nowait(frame)


    logger.info("Releasing")
    cam.release()
    if out:
        out.release()
    logger.info("Released camera and writer")

    

def get_coordinates():
    coordinate_lock.acquire()
    coordinates = COORDINATES
    coordinate_lock.release()
    return coordinates

def calibrate_visuals():
    # Something that would be cool is some detection software that can automatically generate the slopes and intercepts to calibrate
    logger.info("initializing camera")
    cam = cv2.VideoCapture(CAMERA_ID, cv2.CAP_ANY)


    # Get the default frame width and height
    frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_width = 1920
    frame_height = 1080
    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # out = cv2.VideoWriter('output.mp4', fourcc, 30.0, (frame_width, frame_height))


    cam.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

    global CALIBRATING

    logger.info("Finished Initializing")
    while CALIBRATING:
        ret, frame = cam.read()

        if frame is None:
            continue

        frame = cv2.undistort(frame, CAMERA_MATRIX, DIST_COEFFS)

        frame = cv2.flip(frame, -1)
        # out.write(frame)
        frame = frame[startY:endY, startX:endX]
        origin = grid_to_camera(60, 170)
        corner = grid_to_camera(350, 30)
        corner1 = grid_to_camera(350, 170)
        corner2 = grid_to_camera(60, 30)
        center = grid_to_camera(200, 100)
        
        # print(origin)
        # print(corner)
        cv2.circle(frame, (int(origin[0]), int(origin[1])), 20, (0, 0, 255), 2)
        cv2.circle(frame, (int(corner[0]), int(corner[1])), 20, (0, 0, 255), 2)
        cv2.circle(frame, (int(corner1[0]), int(corner1[1])), 20, (0, 255, 0), 2)
        cv2.circle(frame, (int(corner2[0]), int(corner2[1])), 20, (255, 0, 0), 2)
        cv2.circle(frame, (int(center[0]), int(center[1])), 20, (0, 255, 255), 2)
        cv2.circle(frame, (int(gantry_loc[0]), int(gantry_loc[1])), 20, (0, 0, 255), 2)

        extra_grid_points = [
            (120, 50),
            (280, 150),
            (200, 30),
            (100, 120),
            (300, 80),
            (150, 170)
        ]

        for gx, gy in extra_grid_points:
            px, py = grid_to_camera(gx, gy)
            cv2.circle(frame, (int(px), int(py)), 15, (255, 255, 0), 2)
        # print((int(gantry_loc[0]), int(gantry_loc[1])))
        # cv2.imshow("video2", cv2.resize(frame, (1536, 864)))
        # cv2.imshow("video", thresh)
        # cv2.imshow("video2", frame)
        try:
            frame_queue.put_nowait(frame)
        except queue.Full:
            frame_queue.get_nowait()
            frame_queue.put_nowait(frame)
        # if cv2.waitKey(1) == ord('q'):
        #     break

    # Release the capture and writer objects
    cam.release()
    # out.release()
    cv2.destroyAllWindows()


def collect_bots():
    waypoint = (0, 0)
    waypoint_grid = (0, 0)


    gantry_grid = camera_to_grid(gantry_loc[0], gantry_loc[1])

    min_dist = 10000

    closest_bot = []

    while get_coordinates() == []:
        pass

    bots = get_coordinates()

    for bot in bots:
        dist = math.dist(bot, gantry_grid)
        if dist < 50:
            continue
        else:
            if dist < min_dist:
                min_dist = dist
                closest_bot = bot

    waypoint_grid = closest_bot
    return waypoint_grid

def collect_objects():
    gantry_grid = camera_to_grid(gantry_loc[0], gantry_loc[1])

    min_dist = float("inf")
    closest_object = []

    while get_finished_objects() is None:
        pass

    objects = get_finished_objects()

    # print(len(objects))

    for obj in objects:
        dist = math.dist(obj, gantry_grid)

        if dist < 30:
            continue

        if dist < min_dist:
            min_dist = dist
            closest_object = obj

    return closest_object

def get_direction(p1, p2):
    dr = p2[0] - p1[0]
    dc = p2[1] - p1[1]

    dr = (dr > 0) - (dr < 0)
    dc = (dc > 0) - (dc < 0)

    direction_map = {
        ( 0, -1): "N",
        ( 1, -1): "NW",
        ( 1,  0): "W",
        ( 1,  1): "SW",
        ( 0,  1): "S",
        (-1,  1): "SE",
        (-1,  0): "E",
        (-1, -1): "NE",
    }

    return direction_map.get((dr, dc), "Same")

def get_cardinal_direction(p1, p2):
    dr = p2[0] - p1[0]
    dc = p2[1] - p1[1]

    if abs(dr) > abs(dc):
        return "W" if dr > 0 else "E"
    elif abs(dc) > abs(dr):
        return "S" if dc > 0 else "N"
    else:
        return "W" if dr > 0 else "E"

def direction_to_pattern(direction):
    pattern_map = {
        "N":  0,
        "S":  0,

        "E":  2,
        "W":  2,

        "NE": 3,
        "SW": 3,

        "NW": 1,
        "SE": 1,
    }

    return pattern_map.get(direction, None)

def cardinal_direction_to_pattern(direction):
    pattern_map = {
        "N":  0,

        "S":  2,

        "E":  1,

        "W":  3
    }

    return pattern_map.get(direction, None)

def get_average_direction(waypoints, index, lookahead=10):
    if index >= len(waypoints) - 1:
        return "Same"

    end_index = min(index + lookahead, len(waypoints) - 1)

    total_dx = 0.0
    total_dy = 0.0
    weight_sum = 0.0

    for i in range(index, end_index):
        p0 = waypoints[index]
        p1 = waypoints[i + 1]

        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]

        weight = (i - index + 1)

        total_dx += dx * weight
        total_dy += dy * weight
        weight_sum += weight

    if weight_sum == 0:
        return "Same"

    avg_dx = total_dx / weight_sum
    avg_dy = total_dy / weight_sum

    dx = (avg_dx > 0) - (avg_dx < 0)
    dy = (avg_dy > 0) - (avg_dy < 0)

    direction_map = {
        ( 0, -1): "N",
        ( 1, -1): "NW",
        ( 1,  0): "W",
        ( 1,  1): "SW",
        ( 0,  1): "S",
        (-1,  1): "SE",
        (-1,  0): "E",
        (-1, -1): "NE",
    }

    return direction_map.get((dx, dy), "Same")


def drawSAM():
    return [(275, 150), (300, 150), (300, 100), (275, 100), (275, 50), (300, 50), (250, 50), (225, 150), (200, 50), (175, 50), (150, 150), (125, 75), (100, 150), (75, 50)]

def navMaze():
    return [(110, 350), (110, 280), (60, 280), (60, 185), (140, 185), (140, 115), (90, 115), (90, 30)]