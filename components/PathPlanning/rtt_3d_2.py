import numpy as np
from src.rrt.rrt_star import RRTStar
from src.search_space.search_space import SearchSpace
from src.utilities.plotting import Plot

X_dimensions = np.array([(-25, 25), (-25, 25), (-25, 25)])  # dimensions of Search Space

# Obstacles = np.array(
#     [(20, 20, 20, 40, 40, 40), (20, 20, 60, 40, 40, 80), (20, 60, 20, 40, 80, 40), (60, 60, 20, 80, 80, 40),
#      (60, 20, 20, 80, 40, 40), (60, 20, 60, 80, 40, 80), (20, 60, 60, 40, 80, 80), (60, 60, 60, 80, 80, 80)])

ground_plane = (-25, -25, -25, 25, 25, 0)
obstacles = [(0.0, 0.0, 0.0, 2.2, 2.2, 2.2)]

cube_side = 2.2
center = (0, 0, 20)  # starting location
target = (-10, -15, cube_side / 2.0)  # goal location
boundary_distance = cube_side
boundary_height = cube_side * 2.5

target_obstacles = [
    # (target[0] - 2 * d, target[1] - 2 * d, 0, target[0] + 2 * d, target[1] - d, 2 * cube_side),
    # (target[0] - 2 * d, 10, 0.0, target[0] + 2 * d, 15, 2 * cube_side),
    # (target[0] - 2 * d, target[1] - 2 * d, 0.0, -25, 15, 2 * cube_side),
    # (target[0] + 2 * d, target[1] - 2 * d, 0.0, -5, 15, 2 * cube_side)

    # (target[0] - 2 * d, target[1] - 2 * d, 0, target[0] + 2 * d, target[1] - d, 2 * cube_side),
    # (target[0] - 2 * d, target[1] + 1 * d, 0.0, target[0] + 2 * d, target[1] + 2 * d, 2 * cube_side),
    # (target[0] - 2 * d, target[1] - 2 * d, 0.0, target[0] - 1 * d, target[1] + 2 * d, 2 * cube_side),
    # (target[0] + 2 * d, target[1] - 2 * d, 0.0, target[0] + 3 * d, target[1] + 2 * d, 2 * cube_side)

    (target[0] - 1 * boundary_distance, target[1] - 2 * boundary_distance, 0, target[0] + 1 * boundary_distance, target[1] - boundary_distance, boundary_height),
    (target[0] - 1 * boundary_distance, target[1] + 1 * boundary_distance, 0.0, target[0] + 1 * boundary_distance, target[1] + 2 * boundary_distance, boundary_height),
    (target[0] - 2 * boundary_distance, target[1] - 1 * boundary_distance, 0.0, target[0] - 1 * boundary_distance, target[1] + 1 * boundary_distance, boundary_height),
    (target[0] + 1 * boundary_distance, target[1] - 1 * boundary_distance, 0.0, target[0] + 2 * boundary_distance, target[1] + 1 * boundary_distance, boundary_height)
]
print(target_obstacles)

Obstacles = np.array([
    ground_plane,
    [obstacle for obstacle in obstacles][0],
    target_obstacles[0],
    target_obstacles[1],
    target_obstacles[2],
    target_obstacles[3]
    ])  # Each 3d bbox:  x1, y1, z1, x2, y2, z2

r = 1  # length of smallest edge to check for intersection with obstacles
# r = 5
Q = np.array([(r, r)])  # length of tree edges
# Q = np.array([(8, 4)])


max_samples = 512  # max number of samples to take before timing out
prc = 0.1  # probability of checking for a connection to goal
rewire_count = 32  # optional, number of nearby branches to rewire

X = SearchSpace(X_dimensions, Obstacles)

rrt = RRTStar(X, Q, center, target, max_samples, r, prc, rewire_count)
path = rrt.rrt_star()

print(path)

# from mpl_toolkits.mplot3d import axes3d
# import matplotlib.pyplot as plt
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# X, Y, Z = zip(*path)
# ax.plot(X, Y, Z)
# plt.show()

# plot
plot = Plot("rrt_star_3d")
plot.filename = "asdf.html"
plot.plot_tree(X, rrt.trees)
if path is not None:
    plot.plot_path(X, path)
plot.plot_obstacles(X, Obstacles)
plot.plot_start(X, center)
plot.plot_goal(X, target)
plot.draw(auto_open=True)
