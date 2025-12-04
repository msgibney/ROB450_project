import logging
import sys
import cv2
import imutils
import numpy as np
from PIL import Image
import threading

CAMERA_ID = 4

logger = logging.getLogger(__name__)

logger.setLevel(logging.INFO)
handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

COORDINATES = []
coordinate_lock = threading.Lock()
WALLS = []

gantry_loc = [0, 0]
startY = 80
endY = 1000
startX = 100
endX = 1800
x_len = endX - startX
y_len = endY - startY

m_x = 0.836
b_x = 27.24
m_y = 0.85
b_y = -22.7

def grid_to_gantry(x, y):
    gantry_x = (x - b_x) / m_x
    gantry_y = (y - b_y) / m_y
    return [gantry_x, gantry_y]  

def gantry_to_grid(x, y):
    grid_x = m_x * x + b_x
    grid_y = m_y * y + b_y    
    return [grid_x, grid_y]

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


def get_walls(frame):
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([100, 100, 0])
    upper_blue = np.array([160, 255, 255])
    blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
    # cv2.imshow("walls", cv2.resize(blue_mask, (1536, 864)))

    # img = Image.fromarray(blue_mask)
    resized_img = cv2.resize(blue_mask, (400, 200), interpolation=cv2.INTER_NEAREST)
    cv2.imshow("walls", resized_img)

    return np.array(resized_img, dtype=np.int8)

def camera_to_grid(cX, cY):
    return [(x_len-cX)*400/(x_len),(y_len - cY)*200/(y_len)]

def grid_to_camera(cX, cY):
    return [x_len - (cX*x_len/(400)),y_len - (cY*y_len/(200))]
    
def locate_bots():
    logger.info("initializing camera")
    cam = cv2.VideoCapture(CAMERA_ID, cv2.CAP_ANY)

    # Get the default frame width and height
    frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_width = 1920
    frame_height = 1080

    cam.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

    # Define the codec and create VideoWriter object
    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # out = cv2.VideoWriter('output.mp4', fourcc, 30.0, (frame_width, frame_height))

    logger.info("Finished Initializing")
    first_loop = True
    # for i in range(300):
    while True:
        ret, frame = cam.read()

        if frame is None:
            continue
        
        frame = frame[startY:endY, startX:endX]
        if first_loop:
            global WALLS
            WALLS = get_walls(frame)
            first_loop = False

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 23, 73)
        cnts, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        coordinates = []
        for c in cnts:
            # print(len(cnts))
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
                continue
            coordinates.append(camera_to_grid(cX, cY))
            str_coor = [f"{num:.2f}" for num in coordinates[-1]]
            cv2.putText(frame, f"centroid: {str_coor}", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            
        coordinate_lock.acquire()
        global COORDINATES
        COORDINATES = coordinates
        coordinate_lock.release()
        cv2.circle(frame, (int(gantry_loc[0]), int(gantry_loc[1])), 20, (0, 0, 255), 2)

        cv2.drawContours(frame, cnts, -1, (0,0,255), 3)
        cv2.imshow("video2", cv2.resize(frame, (1536, 864)))
        # cv2.imshow("video", thresh)
        # cv2.imshow("video2", frame)
        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

    # Release the capture and writer objects
    cam.release()
    # out.release()
    cv2.destroyAllWindows()

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

    logger.info("Finished Initializing")
    for i in range(300):
        ret, frame = cam.read()

        if frame is None:
            continue
        # out.write(frame)
        frame = frame[startY:endY, startX:endX]
        origin = grid_to_camera(60, 30)
        corner = grid_to_camera(370, 170)
        
        # print(origin)
        # print(corner)
        cv2.circle(frame,(int(origin[0]),int(origin[1])), 20, (0,0,255), 2)
        cv2.circle(frame,(int(corner[0]),int(corner[1])), 20, (0,0,255), 2)
        cv2.circle(frame, (int(gantry_loc[0]), int(gantry_loc[1])), 20, (0, 0, 255), 2)
        # print((int(gantry_loc[0]), int(gantry_loc[1])))
        cv2.imshow("video2", cv2.resize(frame, (1536, 864)))
        # cv2.imshow("video", thresh)
        # cv2.imshow("video2", frame)
        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

    # Release the capture and writer objects
    cam.release()
    out.release()
    cv2.destroyAllWindows()