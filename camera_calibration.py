import cv2
import numpy as np
import os

CHECKERBOARD = (9, 6)
SQUARE_SIZE = 25.0
NUM_IMAGES = 20
SAVE_DIR = "calib_images"
OUTPUT_FILE = "camera_calibration.npz"
CAMERA_ID = 0

os.makedirs(SAVE_DIR, exist_ok=True)

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []
imgpoints = []

cap = cv2.VideoCapture(CAMERA_ID)
img_count = 0

print("Press 'c' to capture an image when checkerboard is visible")
print("Press 'q' to finish capture and calibrate")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray,
        CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    display = frame.copy()

    if found:
        corners_refined = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            criteria=(
                cv2.TERM_CRITERIA_EPS +
                cv2.TERM_CRITERIA_MAX_ITER,
                30,
                0.001
            )
        )

        cv2.drawChessboardCorners(
            display,
            CHECKERBOARD,
            corners_refined,
            found
        )

    cv2.putText(
        display,
        f"Captured: {img_count}/{NUM_IMAGES}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2
    )

    cv2.putText(
        display,
        "Press 'c' to capture | 'q' to quit",
        (20, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2
    )

    cv2.imshow("Calibration Live View", display)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('c') and found:
        img_name = f"{SAVE_DIR}/img_{img_count:02d}.jpg"
        cv2.imwrite(img_name, frame)

        objpoints.append(objp)
        imgpoints.append(corners_refined)

        img_count += 1
        print(f"✔ Captured {img_name}")

        if img_count >= NUM_IMAGES:
            print("✅ Required number of images captured")
            break

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

if img_count < 5:
    raise RuntimeError("Not enough images for calibration")

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints,
    imgpoints,
    gray.shape[::-1],
    None,
    None
)

print("\nCalibration complete")
print("RMS reprojection error:", ret)
print("Camera matrix:\n", camera_matrix)
print("Distortion coefficients:\n", dist_coeffs)

np.savez(
    OUTPUT_FILE,
    camera_matrix=camera_matrix,
    dist_coeffs=dist_coeffs,
    rvecs=rvecs,
    tvecs=tvecs
)

print(f"\nCalibration saved to {OUTPUT_FILE}")