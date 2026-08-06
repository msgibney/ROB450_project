import os
import numpy as np
import matplotlib.pyplot as plt
import math
import re
from scipy.ndimage import gaussian_filter1d


def plot_cluster_radius_time_series(
    folder="radius_time_series_copy",
    fps=16,
    interval=5,
    title="Cluster Radius Over Time as a Function of Field Strength",
    mm_per_pixel=3/15
):
    plt.figure()

    max_time = 0
    lines = []
    labels = []

    for file in sorted(
    os.listdir(folder),
    key=lambda f: int(f.replace("radii_", "").replace(".npy", ""))
    ):
        if not file.endswith(".npy"):
            continue

        data = np.load(os.path.join(folder, file))

        if len(data) == 0:
            continue

        data = data * mm_per_pixel

        time = np.arange(len(data)) / fps
        max_time = max(max_time, time[-1])

        num_robots = int(file.replace("radii_", "").replace(".npy", ""))

        line, = plt.plot(time, data, alpha=0.6)
        lines.append(line)
        labels.append(f"{num_robots} robots")
        num_lines = int(time[-1] // interval) + 1

        dot_times = []
        dot_values = []

        for k in range(num_lines):
            t = k * interval

            idx = np.argmin(np.abs(time - t))

            dot_times.append(time[idx])
            dot_values.append(data[idx])

        # plt.scatter(
        #     dot_times,
        #     dot_values,
        #     color=line.get_color(),
        #     s=15,
        #     zorder=3,
        #     alpha=0.8
        # )

    if len(lines) == 0:
        print("No valid .npy files found.")
        return

    num_lines = int(max_time // interval) + 1

    ymin, ymax = plt.ylim()
    padding = 0.1 * (ymax - ymin if ymax > ymin else 1)
    plt.ylim(ymin, ymax + padding)

    label_y = ymax + padding * 0.8

    for k in range(num_lines):
        t = k * interval
        strength = max(100 - 5 * k, 0)

        plt.axvline(x=t, linestyle='--', linewidth=0.8, alpha=0.5)

        plt.text(
            t,
            label_y,
            f"{strength}%",
            rotation=90,
            verticalalignment='top',
            horizontalalignment='right',
            fontsize=8,
            alpha=0.7
        )

    plt.xlabel("Time(s)")
    plt.ylabel("Cluster Radius(mm)")
    plt.title(title)

    plt.legend(
        lines,
        labels,
        fontsize=6,
        ncol=2,
        framealpha=0.5,
        loc='lower right'
    )

    plt.grid(False)
    plt.tight_layout()

    save_path = os.path.join(folder, "cluster_radius_plot.jpg")

    plt.savefig(save_path, format="jpg", bbox_inches="tight")
    # plt.savefig(save_path, format="svg", bbox_inches="tight")

    plt.show()


def plot_knn_distance_time_series(
    folder="avg_dist_k1",
    fps=16,
    interval=5,
    k=1,
    title="Average KNN Distance Over Time (All Experiments)",
    mm_per_pixel=3/15,
    show_std_combined=True
):

    output_folder = f"avg_dist_knn_{k}_neighbors"
    os.makedirs(output_folder, exist_ok=True)

    files = [
        f for f in os.listdir(folder)
        if f.endswith(".npz") or f.endswith(".npy")
    ]

    def extract_number(f):
        match = re.search(r"(\d+)", f)
        return int(match.group(1)) if match else -1

    files = sorted(files, key=extract_number)

    curves = []
    max_time = 0

    for file in files:
        path = os.path.join(folder, file)

        if file.endswith(".npz"):
            data = np.load(path)
            mean = data["mean"] * mm_per_pixel
            std = data["std"] * mm_per_pixel
        else:
            mean = np.load(path) * mm_per_pixel
            std = None

        time = np.arange(len(mean)) / fps
        max_time = max(max_time, time[-1])

        curves.append((time, mean, std, file))

    if not curves:
        print("No valid data found.")
        return

    n = len(curves)
    cols = 4
    rows = math.ceil(n / cols)

    fig, axes = plt.subplots(rows, cols, figsize=(16, 3 * rows), sharex=True, sharey=True)
    axes = axes.flatten()

    all_means = np.concatenate([c[1] for c in curves])
    y_min = np.nanmin(all_means)
    y_max = np.nanmax(all_means)
    padding = 0.1 * (y_max - y_min if y_max > y_min else 1)

    for idx, (time, mean, std, file) in enumerate(curves):

        ax = axes[idx]
        label = f"{''.join(filter(str.isdigit, file))} robots"

        ax.plot(time, mean, alpha=0.85)

        if std is not None:
            ax.fill_between(time, mean - std, mean + std, alpha=0.2)

        ax.set_title(label, fontsize=10)
        ax.set_ylim(y_min, y_max + padding)

        num_lines = int(max_time // interval) + 1
        label_y = y_max + padding * 0.8

        for i in range(num_lines):
            t = i * interval
            strength = max(100 - 5 * i, 0)

            ax.axvline(x=t, linestyle='--', linewidth=0.8, alpha=0.5)

            ax.text(
                t,
                label_y,
                f"{strength}%",
                rotation=90,
                verticalalignment='top',
                horizontalalignment='right',
                fontsize=7,
                alpha=0.7
            )

        ax.grid(False)

    for j in range(len(curves), len(axes)):
        fig.delaxes(axes[j])

    fig.suptitle(title, fontsize=16)
    fig.supxlabel("Time(s)")
    fig.supylabel("Average Distance(mm)")
    plt.tight_layout(rect=[0, 0, 1, 0.96])

    plt.savefig(os.path.join(output_folder, "all_individual_contrast.jpg"), dpi=300, bbox_inches="tight")
    plt.savefig(os.path.join(output_folder, "all_individual_contrast.svg"), format="svg", bbox_inches="tight")
    plt.show()

    plt.figure(figsize=(12, 6))

    for time, mean, std, file in curves:
        label = f"{''.join(filter(str.isdigit, file))} robots"

        plt.plot(time, mean, alpha=0.7, label=label)

        # if show_std_combined and std is not None:
        #     plt.fill_between(time, mean - std, mean + std, alpha=0.15)

    num_lines = int(max_time // interval) + 1
    ymin, ymax = plt.ylim()
    padding = 0.1 * (ymax - ymin if ymax > ymin else 1)
    plt.ylim(ymin, ymax + padding)
    label_y = ymax + padding * 0.8

    for i in range(num_lines):
        t = i * interval
        strength = max(100 - 5 * i, 0)

        plt.axvline(x=t, linestyle='--', linewidth=0.8, alpha=0.5)

        plt.text(
            t,
            label_y,
            f"{strength}%",
            rotation=90,
            verticalalignment='top',
            horizontalalignment='right',
            fontsize=8,
            alpha=0.7
        )

    plt.xlabel("Time(s)")
    plt.ylabel("Average Distance(mm)")
    plt.title(title)
    plt.legend(fontsize=8, ncol=2)
    plt.grid(False)
    plt.tight_layout()

    plt.savefig(os.path.join(output_folder, "combined_contrast.jpg"), dpi=300, bbox_inches="tight")
    plt.savefig(os.path.join(output_folder, "combined_contrast.svg"), format="svg", bbox_inches="tight")
    plt.show()

def plot_voronoi_distance_time_series(
    folder="avg_dist_k1",
    fps=16,
    interval=5,
    k=1,
    title="Average Voronoi Distance Over Time (All Experiments)",
    mm_per_pixel=3/15,
    show_std_combined=True
):

    output_folder = f"avg_dist_voronoi_neighbors"
    os.makedirs(output_folder, exist_ok=True)

    files = [
        f for f in os.listdir(folder)
        if f.endswith(".npz") or f.endswith(".npy")
    ]

    def extract_number(f):
        match = re.search(r"(\d+)", f)
        return int(match.group(1)) if match else -1

    files = sorted(files, key=extract_number)

    curves = []
    max_time = 0

    for file in files:
        path = os.path.join(folder, file)

        if file.endswith(".npz"):
            data = np.load(path)
            mean = data["mean"] * mm_per_pixel
            std = data["std"] * mm_per_pixel
        else:
            mean = np.load(path) * mm_per_pixel
            std = None

        time = np.arange(len(mean)) / fps
        max_time = max(max_time, time[-1])

        curves.append((time, mean, std, file))

    if not curves:
        print("No valid data found.")
        return

    n = len(curves)
    cols = 4
    rows = math.ceil(n / cols)

    fig, axes = plt.subplots(rows, cols, figsize=(16, 3 * rows), sharex=True, sharey=True)
    axes = axes.flatten()

    all_means = np.concatenate([c[1] for c in curves])
    y_min = np.nanmin(all_means)
    y_max = np.nanmax(all_means)
    padding = 0.1 * (y_max - y_min if y_max > y_min else 1)

    for idx, (time, mean, std, file) in enumerate(curves):

        ax = axes[idx]
        label = f"{''.join(filter(str.isdigit, file))} robots"

        # smooth_sigma = 3

        # mean_smooth = gaussian_filter1d(mean, sigma=smooth_sigma)

        # if std is not None:
        #     std_smooth = gaussian_filter1d(std, sigma=smooth_sigma)
        # else:
        #     std_smooth = None

        ax.plot(time, mean, alpha=0.85)

        if std is not None:
            ax.fill_between(time, mean - std, mean + std, alpha=0.2)
            

        ax.set_title(label, fontsize=10)
        ax.set_ylim(y_min, y_max + padding)

        num_lines = int(max_time // interval) + 1
        label_y = y_max + padding * 0.8

        for i in range(num_lines):
            t = i * interval
            strength = max(100 - 5 * i, 0)

            ax.axvline(x=t, linestyle='--', linewidth=0.8, alpha=0.5)

            ax.text(
                t,
                label_y,
                f"{strength}%",
                rotation=90,
                verticalalignment='top',
                horizontalalignment='right',
                fontsize=7,
                alpha=0.7
            )

        ax.grid(False)

    for j in range(len(curves), len(axes)):
        fig.delaxes(axes[j])

    fig.suptitle(title, fontsize=16)
    fig.supxlabel("Time(s)")
    fig.supylabel("Average Distance(mm)")
    plt.tight_layout(rect=[0, 0, 1, 0.96])

    plt.savefig(os.path.join(output_folder, "all_individual_contrast.jpg"), dpi=300, bbox_inches="tight")
    plt.savefig(os.path.join(output_folder, "all_individual_contrast.svg"), format="svg", bbox_inches="tight")
    plt.show()

    plt.figure(figsize=(12, 6))

    for time, mean, std, file in curves:
        label = f"{''.join(filter(str.isdigit, file))} robots"

        # smooth_sigma = 3

        # mean_smooth = gaussian_filter1d(mean, sigma=smooth_sigma)

        plt.plot(time, mean, alpha=0.7, label=label)

        # if show_std_combined and std is not None:
        #     plt.fill_between(time, mean - std, mean + std, alpha=0.15)

    num_lines = int(max_time // interval) + 1
    ymin, ymax = plt.ylim()
    padding = 0.1 * (ymax - ymin if ymax > ymin else 1)
    plt.ylim(ymin, ymax + padding)
    label_y = ymax + padding * 0.8

    for i in range(num_lines):
        t = i * interval
        strength = max(100 - 5 * i, 0)

        plt.axvline(x=t, linestyle='--', linewidth=0.8, alpha=0.5)

        plt.text(
            t,
            label_y,
            f"{strength}%",
            rotation=90,
            verticalalignment='top',
            horizontalalignment='right',
            fontsize=8,
            alpha=0.7
        )

    plt.xlabel("Time(s)")
    plt.ylabel("Average Distance(mm)")
    plt.title(title)
    plt.legend(fontsize=8, ncol=2)
    plt.grid(False)
    plt.tight_layout()

    plt.savefig(os.path.join(output_folder, "combined_contrast.jpg"), dpi=300, bbox_inches="tight")
    plt.savefig(os.path.join(output_folder, "combined_contrast.svg"), format="svg", bbox_inches="tight")
    plt.show()

# plot_knn_distance_time_series(folder="avg_dist_k1_corrected", k = 1, title="Average KNN(k = 1) Distance Over Time")
# plot_knn_distance_time_series(folder="avg_dist_k2_corrected", k = 2, title="Average KNN(k = 2) Distance Over Time")
# plot_knn_distance_time_series(folder="avg_dist_k3_corrected", k = 3, title="Average KNN(k = 3) Distance Over Time")
# plot_knn_distance_time_series(folder="avg_dist_k5_corrected", k = 5, title="Average KNN(k = 5) Distance Over Time")
# plot_knn_distance_time_series(folder="avg_dist_k7_corrected", k = 7, title="Average KNN(k = 7) Distance Over Time")
# plot_voronoi_distance_time_series(folder="voronoi_dist_contrasted", title="Average Voronoi Distance Over Time")

plot_cluster_radius_time_series()
