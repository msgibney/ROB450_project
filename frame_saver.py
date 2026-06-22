import cv2
import os

VIDEO_PATH = "draw_SAM_clean_video_log.avi"
SAVE_DIR = "saved_frames"

SAVE_DIR2 = os.path.join(
    SAVE_DIR,
    "draw_SAM_clean"
)

os.makedirs(SAVE_DIR, exist_ok=True)
os.makedirs(SAVE_DIR2, exist_ok=True)

cap = cv2.VideoCapture(VIDEO_PATH)

if not cap.isOpened():
    raise RuntimeError(f"Could not open {VIDEO_PATH}")

total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
current_frame = 0
saved_count = 0

print("Controls:")
print("  Right Arrow : Next frame")
print("  Left Arrow  : Previous frame")
print("  S           : Save current frame")
print("  Q or ESC    : Quit")

while True:
    current_frame = max(0, min(current_frame, total_frames - 1))

    cap.set(cv2.CAP_PROP_POS_FRAMES, current_frame)
    ret, frame = cap.read()

    if not ret:
        print("Could not read frame.")
        break

    display = frame.copy()

    cv2.putText(
        display,
        f"Frame {current_frame}/{total_frames-1}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2,
    )

    cv2.imshow("Video Frame Selector", display)

    key = cv2.waitKeyEx(0)

    if key == ord('d'):
        current_frame += 1

    elif key == ord('a'):
        current_frame -= 1


    elif key == ord('s') or key == ord('S'):
        filename = os.path.join(
            SAVE_DIR2,
            f"frame_{current_frame:06d}.png"
        )
        cv2.imwrite(filename, frame)
        saved_count += 1
        print(f"Saved: {filename}")

    elif key == ord('q') or key == ord('Q') or key == 27:
        break

cap.release()
cv2.destroyAllWindows()

print(f"Saved {saved_count} frame(s).")