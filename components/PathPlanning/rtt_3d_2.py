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

center = (0, 0, 20)  # starting location
target = (-20, 5, 1.5)  # goal location

Obstacles = np.array([
    # (2, 2, 2, 4, 4, 10),
    ground_plane,
    [obstacle for obstacle in obstacles][0]
    ])  # Each 3d bbox:  x1, y1, z1, x2, y2, z2


r = 5  # length of smallest edge to check for intersection with obstacles
Q = np.array([(r, r)])  # length of tree edges

Q = np.array([(8, 4)])  # length of tree edges
r = 1  # length of smallest edge to check for intersection with obstacles

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