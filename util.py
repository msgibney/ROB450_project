import logging
import sys
import cv2
import imutils
import numpy as np
from PIL import Image
import threading

logger = logging.getLogger(__name__)

logger.setLevel(logging.DEBUG)
handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

COORDINATES = []
coordinate_lock = threading.Lock()
WALLS = []

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
    img = Image.fromarray(blue_mask)
    resized_img = img.resize((400, 200), Image.NEAREST)

    return np.array(resized_img)


def locate_bots():
    logger.info("initializing camera")
    cam = cv2.VideoCapture(0, cv2.CAP_ANY)

    # Get the default frame width and height
    frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_width = 1920
    frame_height = 1080

    cam.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # out = cv2.VideoWriter('output.mp4', fourcc, 30.0, (frame_width, frame_height))

    logger.info("Finished Initializing")
    first_loop = True
    # for i in range(300):
    while True:
        ret, frame = cam.read()

        startY = 30
        endY = 970
        startX = 90
        endX = 1800
        
        frame = frame[startY:endY, startX:endX]
        if first_loop:
            global WALLS
            WALLS = get_walls(frame)
            first_loop = False

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)[1]
        cnts, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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
            coordinates.append([cX,cY])
            cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
            cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        coordinate_lock.acquire()
        global COORDINATES
        COORDINATES = coordinates
        coordinate_lock.release()
        print("set new coordinates")
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