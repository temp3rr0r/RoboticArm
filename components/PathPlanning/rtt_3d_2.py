import numpy as np
from src.rrt.rrt_star import RRTStar
from src.search_space.search_space import SearchSpace
from src.utilities.plotting import Plot

X_dimensions = np.array([(-25, 25), (-25, 25), (-25, 25)])  # dimensions of Search Space
ground_plane = (-25, -25, -25, 25, 25, 0)
target_side_size = 4.4
center = (0, 0, 20)  # starting location
fence_radius = target_side_size * 0.5
fence_height = target_side_size * 1.5

target = (-20, -15, target_side_size / 2.0)  # goal location

boundary_obstacles = [  # Each 3d bbox:  x1, y1, z1, x2, y2, z2
    ground_plane,
    (target[0] - 2 * fence_radius, target[1] - 2 * fence_radius, 0, target[0] + 2 * fence_radius, target[1] - fence_radius, fence_height),
    (target[0] - 2 * fence_radius, target[1] + 1 * fence_radius, 0.0, target[0] + 2 * fence_radius, target[1] + 2 * fence_radius, fence_height),
    (target[0] - 2 * fence_radius, target[1] - 2 * fence_radius, 0.0, target[0] - 1 * fence_radius, target[1] + 2 * fence_radius, fence_height),
    (target[0] + 1 * fence_radius, target[1] - 2 * fence_radius, 0.0, target[0] + 2 * fence_radius, target[1] + 2 * fence_radius, fence_height)
]
Obstacles = np.array(boundary_obstacles)

r = 1  # length of smallest edge to check for intersection with obstacles
# r = 5
# Q = np.array([(r, r)])  # length of tree edges
# Q = np.array([(8, 4)])
Q = np.array([(12, 4)])

max_samples = 1024  # max number of samples to take before timing out
prc = 0.1  # probability of checking for a connection to goal
rewire_count = 32  # optional, number of nearby branches to rewire

X = SearchSpace(X_dimensions, Obstacles)
rrt = RRTStar(X, Q, center, target, max_samples, r, prc, rewire_count)
path = rrt.rrt_star()

min_path_length = 5
max_path_length = 10

print("Path length: {}".format(len(path)))

while len(path) > max_path_length:
    path = rrt.rrt_star()

if len(path) < min_path_length:
    n = len(path)

    def function1(x):
        return n + (n - 1) * x  # y = n + (n - 1) * x

    from scipy.optimize import minimize, minimize_scalar
    result = minimize_scalar(function1, bounds=[min_path_length, max_path_length], method='bounded')
    print("result: {}".format(result))


if path is not None:
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
