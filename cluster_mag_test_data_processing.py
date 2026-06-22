import cv2
import numpy as np
import os
from scipy.spatial import distance_matrix
from scipy.spatial import Delaunay


def extract_filtered_centroids(frame, lower_red1, upper_red1, lower_red2, upper_red2, min_area, max_dist):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]

    if len(contours) == 0:
        return []

    centroids = []
    for cnt in contours:
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centroids.append((cx, cy))

    if len(centroids) == 0:
        return []

    centroids = np.array(centroids)
    mean_center = np.mean(centroids, axis=0)

    filtered_centroids = []
    for (cx, cy) in centroids:
        dist = np.linalg.norm([cx - mean_center[0], cy - mean_center[1]])
        if dist < max_dist:
            filtered_centroids.append((cx, cy))

    return filtered_centroids

def extract_black_centroids(frame, min_area=20, max_dist=200, v_thresh=60):
    """
    Detect dark (black) objects instead of red ones.
    Uses brightness thresholding.
    """

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    v = hsv[:, :, 2]

    mask = cv2.inRange(v, 0, v_thresh)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) > min_area]

    if len(contours) == 0:
        return []

    centroids = []
    for cnt in contours:
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centroids.append((cx, cy))

    if len(centroids) == 0:
        return []

    centroids = np.array(centroids)
    mean_center = np.mean(centroids, axis=0)

    filtered = []
    for (cx, cy) in centroids:
        dist = np.linalg.norm([cx - mean_center[0], cy - mean_center[1]])
        if dist < max_dist:
            filtered.append((cx, cy))

    return filtered


def process_videos_radius(
    folder_path="magnet_strength_tests_copy",
    output_folder="radius_time_series_copy",
    num_robots_range=range(100, 201, 10),
    min_area=20,
    max_dist=200
):
    os.makedirs(output_folder, exist_ok=True)

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])

    for num_robots in num_robots_range:
        video_name = f"{num_robots}_microrobot_mag_test_video_log.avi"
        video_path = os.path.join(folder_path, video_name)

        if not os.path.exists(video_path):
            print(f"Video not found: {video_name}, skipping.")
            continue

        cap = cv2.VideoCapture(video_path)
        radii_over_time = []

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            centroids = extract_black_centroids(frame, min_area, max_dist)

            if len(centroids) == 0:
                radii_over_time.append(np.nan)
                continue

            all_points = np.array(centroids)
            (x, y), radius = cv2.minEnclosingCircle(all_points)

            radii_over_time.append(radius)

        cap.release()

        output_file = os.path.join(output_folder, f"radii_{num_robots}.npy")
        np.save(output_file, np.array(radii_over_time))

        print(f"[Radius] Processed {video_name}")


def average_knn_distance(centroids, k, expected_count, max_edge_pixels=50):
    if len(centroids) < 2:
        return np.nan, np.nan

    centroids = np.array(centroids)
    dists = distance_matrix(centroids, centroids)

    np.fill_diagonal(dists, np.inf)

    knn_distances = []
    for i in range(len(centroids)):
        nearest = np.sort(dists[i])[:k]
        knn_distances.extend(nearest)

    if len(knn_distances) == 0:
        return np.nan, np.nan

    knn_distances = np.array(knn_distances)

    # knn_distances = knn_distances[knn_distances <= max_edge_pixels]

    # if len(knn_distances) == 0:
    #     return np.nan, np.nan

    mean = np.mean(knn_distances)
    std = np.std(knn_distances)

    return mean, std


def process_videos_knn(
    folder_path="video_processing/photometricCalib_after",
    num_robots_range=range(100, 201, 10),
    min_area=20,
    max_dist=200,
    k=3
):
    output_folder = f"avg_dist_k{k}_contrasted"

    os.makedirs(output_folder, exist_ok=True)

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])

    for num_robots in num_robots_range:
        video_name = f"{num_robots}_microrobot_mag_test_video_log.avi"
        video_path = os.path.join(folder_path, video_name)

        if not os.path.exists(video_path):
            print(f"Video not found: {video_name}, skipping.")
            continue

        cap = cv2.VideoCapture(video_path)
        means = []
        stds = []

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            centroids = extract_black_centroids(frame, min_area, max_dist)

            if len(centroids) == 0:
                means.append(np.nan)
                stds.append(np.nan)
                continue

            avg_dist, std_dev = average_knn_distance(centroids, k, num_robots)
            means.append(avg_dist)
            stds.append(std_dev)


        cap.release()

        output_file = os.path.join(output_folder, f"radii_{num_robots}")
        np.savez(output_file, mean=means, std=stds)

        print(f"[k-NN] Processed {video_name}")

def process_videos_knn_corrected(
    folder_path="video_processing/photometricCalib_after",
    num_robots_range=range(100, 201, 10),
    min_area=20,
    max_dist=200,
    k=3
):
    output_folder = f"avg_dist_k{k}_contrasted"
    os.makedirs(output_folder, exist_ok=True)

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])

    contact_distance = 3.0

    for num_robots in num_robots_range:

        # video_name = f"{num_robots}_microrobot_mag_test_video_log.avi"
        video_name = f"{num_robots}_microrobot_mag_test_video_log_black_red.mp4"
        video_path = os.path.join(folder_path, video_name)

        if not os.path.exists(video_path):
            print(f"Video not found: {video_name}, skipping.")
            continue

        cap = cv2.VideoCapture(video_path)

        means = []
        stds = []
        occupancy_trace = []

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            centroids = extract_black_centroids(frame, min_area, max_dist)

            detected = len(centroids)

            if detected == 0:
                means.append(np.nan)
                stds.append(np.nan)
                occupancy_trace.append(0)
                continue

            avg_dist, std_dev = average_knn_distance(centroids, k, num_robots)

            means.append(avg_dist)
            stds.append(std_dev)

        cap.release()

        output_file = os.path.join(output_folder, f"radii_{num_robots}")

        np.savez(
            output_file,
            mean=means,
            std=stds,
            occupancy=occupancy_trace
        )

        print(f"[k-NN corrected] Processed {video_name}")


def voronoi_neighbor_distance(centroids, max_dist=100):

    if len(centroids) < 3:
        return np.nan, np.nan, [], None

    pts = np.array(centroids)

    pts = np.unique(pts, axis=0)

    if len(pts) < 3:
        return np.nan, np.nan, [], None

    try:
        tri = Delaunay(pts)
    except:
        return np.nan, np.nan, [], None

    edges = set()

    for simplex in tri.simplices:
        for i in range(3):
            a = simplex[i]
            b = simplex[(i + 1) % 3]
            edges.add(tuple(sorted((a, b))))

    edge_list = []
    dists = []

    MAX_EDGE_PX = 50

    for i, j in edges:

        dist = np.linalg.norm(pts[i] - pts[j])

        edge_list.append((i, j, dist))

        if dist <= MAX_EDGE_PX:
            dists.append(dist)

    dists = np.array(dists)

    if len(dists) == 0:
        return np.nan, np.nan, edge_list, pts

    return np.mean(dists), np.std(dists), edge_list, pts

def process_videos_voronoi(
    folder_path="video_processing/photometricCalib_after",
    output_folder="voronoi_dist_contrasted",
    num_robots_range=range(100, 201, 10),
    min_area=20,
    max_dist=200
):
    os.makedirs(output_folder, exist_ok=True)

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])

    for num_robots in num_robots_range:
        video_name = f"{num_robots}_microrobot_mag_test_video_log_black_red.mp4"
        video_path = os.path.join(folder_path, video_name)

        if not os.path.exists(video_path):
            print(f"Video not found: {video_name}, skipping.")
            continue

        cap = cv2.VideoCapture(video_path)

        means = []
        stds = []

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            centroids = extract_black_centroids(frame, min_area, max_dist)

            if len(centroids) == 0:
                means.append(np.nan)
                stds.append(np.nan)
                continue

            mean_val, std_val, edges, pts = voronoi_neighbor_distance(centroids, max_dist=max_dist)

            means.append(mean_val)
            stds.append(std_val)
            debug_frame = frame.copy()

            for p in pts:
                cv2.circle(
                    debug_frame,
                    tuple(np.int32(p)),
                    4,
                    (0, 255, 0),
                    -1
                )

            for i, j, dist in edges:

                p1 = tuple(np.int32(pts[i]))
                p2 = tuple(np.int32(pts[j]))

                color = (0, 0, 255) if dist >= max_dist else (255, 0, 0)

                cv2.line(
                    debug_frame,
                    p1,
                    p2,
                    color,
                    1
                )

            cv2.imshow("Voronoi Debug", debug_frame)

            key = cv2.waitKey(1)

            if key == 27:
                break

        cap.release()

        output_file = os.path.join(output_folder, f"voronoi_{num_robots}")

        # np.savez(output_file, mean=means, std=stds)

        print(f"[Voronoi] Processed {video_name}")

if __name__ == "__main__":
    # process_videos_radius()
    # process_videos_knn_corrected(k = 1)
    # process_videos_knn_corrected(k = 2)
    # process_videos_knn_corrected(k = 3)
    # process_videos_knn_corrected(k = 5)
    # process_videos_knn_corrected(k = 7)
    process_videos_voronoi()
    