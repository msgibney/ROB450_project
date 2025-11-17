import cv2
import imutils

print("initializing camera")
cam = cv2.VideoCapture(0, cv2.CAP_ANY)

# Get the default frame width and height
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
frame_width = 1920
frame_height = 1080
print(frame_width)
print(frame_height)

cam.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)



# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# out = cv2.VideoWriter('output.mp4', fourcc, 30.0, (frame_width, frame_height))

print("Finished Initializing")

# for i in range(300):
while True:
    ret, frame = cam.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)[1]
    cnts, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #cnts = cnts[0] if imutils.is_cv2() else cnts[1]

    coordinates = []
    for c in cnts:
        # print(len(cnts))
        M = cv2.moments(c)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        #cv2.circle(image, (cX, cY), 1, (255, 255, 255), -1)
        coordinates.append([cX,cY])
        cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
        cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        # out.write(frame)

        # print([cX, cY])
    cv2.imshow("video", thresh)
    cv2.imshow("video2", frame)
    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Release the capture and writer objects
cam.release()
# out.release()
cv2.destroyAllWindows()

