import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import RegularGridInterpolator
import os

data = np.load("microrobot_outside_counts.npy")

num_robots_range = np.array(list(range(100, 201, 10)))
speed_range = np.array(list(range(100, 1001, 50)))

interp = RegularGridInterpolator((num_robots_range, speed_range), data)

fine_num_robots = np.linspace(num_robots_range.min(), num_robots_range.max(), 200)
fine_speed = np.linspace(speed_range.min(), speed_range.max(), 400)

X, Y = np.meshgrid(fine_num_robots, fine_speed, indexing='ij')
points = np.stack([X.ravel(), Y.ravel()], axis=-1)

Z = interp(points).reshape(len(fine_num_robots), len(fine_speed))

plt.figure()
plt.imshow(Z, aspect='auto', origin='lower',
           extent=[fine_speed.min(), fine_speed.max(),
                   fine_num_robots.min(), fine_num_robots.max()])

plt.colorbar(label="Microrobots Outside Circle")
plt.xlabel("Speed(mm/min)")
plt.ylabel("Number of Microrobots")

plt.tight_layout()

output_folder = "speed_test_heatmaps"
os.makedirs(output_folder, exist_ok=True)

plt.savefig(os.path.join(output_folder, "outside_heatmap.jpg"), dpi=300, bbox_inches="tight")
plt.savefig(os.path.join(output_folder, "outside_heatmap.svg"), format="svg", bbox_inches="tight")

plt.show()