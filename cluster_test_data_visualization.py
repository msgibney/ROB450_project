import numpy as np
import matplotlib.pyplot as plt

data = np.load("microrobot_outside_counts.npy")

num_robots_range = list(range(100, 201, 10))
speed_range = list(range(100, 1001, 50))

plt.figure()

plt.imshow(data, aspect='auto', origin='lower')
plt.colorbar(label="Microrobots Outside Circle")

plt.xticks(ticks=range(len(speed_range)), labels=speed_range, rotation=45)
plt.yticks(ticks=range(len(num_robots_range)), labels=num_robots_range)

plt.xlabel("Speed")
plt.ylabel("Number of Microrobots")
plt.title("Microrobots Outside Circle Heatmap")

plt.tight_layout()
plt.show()