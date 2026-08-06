import cv2
import numpy as np
import os

folder_path = "cluster_tests"
target_position = (635, 434)

num_robots_range = list(range(100, 201, 10))
speed_range = list(range(100, 1001, 50))

results = np.zeros((len(num_robots_range), len(speed_range)), dtype=int)

lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([179, 255, 255])

for i, num_robots in enumerate(num_robots_range):
    for j, speed in enumerate(speed_range):
        video_name = f"{num_robots}_microrobot_{speed}_speed_video_log.avi"
        video_path = os.path.join(folder_path, video_name)
        
        if not os.path.exists(video_path):
            print(f"Video not found: {video_name}, skipping.")
            continue

        cap = cv2.VideoCapture(video_path)
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        middle_frame_index = frame_count // 2

        ret, first_frame = cap.read()
        if not ret:
            print(f"Could not read first frame: {video_name}, skipping.")
            cap.release()
            continue

        hsv = cv2.cvtColor(first_frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = 10

        filtered_contours = []
        for cnt in contours:
            if cv2.contourArea(cnt) > min_area:
                filtered_contours.append(cnt)

        contours = filtered_contours

        centroids = []
        for cnt in contours:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centroids.append((cx, cy))

        centroids = np.array(centroids)

        mean_center = np.mean(centroids, axis=0)

        max_dist = 200
        filtered_contours = []
        for cnt, (cx, cy) in zip(contours, centroids):
            dist = np.linalg.norm([cx - mean_center[0], cy - mean_center[1]])
            if dist < max_dist:
                filtered_contours.append(cnt)

        contours = filtered_contours

        if len(contours) == 0:
            print(f"No microrobots detected in first frame: {video_name}")
            cap.release()
            continue

        all_points = np.vstack(contours).squeeze()
        (x, y), radius = cv2.minEnclosingCircle(all_points)
        radius = int(radius)

        cap.set(cv2.CAP_PROP_POS_FRAMES, middle_frame_index)
        ret, mid_frame = cap.read()
        if not ret:
            print(f"Could not read middle frame: {video_name}, skipping.")
            cap.release()
            continue

        hsv_mid = cv2.cvtColor(mid_frame, cv2.COLOR_BGR2HSV)
        mask1_mid = cv2.inRange(hsv_mid, lower_red1, upper_red1)
        mask2_mid = cv2.inRange(hsv_mid, lower_red2, upper_red2)
        mask_mid = cv2.bitwise_or(mask1_mid, mask2_mid)
        contours_mid, _ = cv2.findContours(mask_mid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        count_outside = 0
        for cnt in contours_mid:
            if cnt.shape[0] == 0:
                continue
            distances = np.sqrt((cnt[:,0,0] - target_position[0])**2 +
                                (cnt[:,0,1] - target_position[1])**2)
            if np.any(distances <= radius):
                count_outside += 1

        results[i, j] = count_outside
        print(f"Processed {video_name}: {count_outside} outside")

        cap.release()

np.save("microrobot_inside_counts.npy", results)
print("Results saved to microrobot_outside_counts.npy")