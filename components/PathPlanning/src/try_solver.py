import numpy as np


def get_total_path_length(x, n):
    return n + (n - 1) * round(x, 0)  # y = n + (n - 1) * x


def get_additional_points(path, min_path_length=5, max_additional_points=10):
    n = len(path)
    additional_points = 0
    while get_total_path_length(additional_points, n) < min_path_length:
        additional_points = additional_points + 1
        if additional_points > max_additional_points:
            break
    return additional_points


if __name__ == '__main__':
    found_path = np.array([(0, 0, 20), (-20.12934917675124, -11.82221671426537, 11.660736179248794), (-20, -15, 2.2)])
    # found_path = np.array([(0, 0, 20), (-20.12934917675124, -11.82221671426537, 11.660736179248794)])

    if len(found_path) > 1:
        extra_points = get_additional_points(found_path)
        print("Additional points: {}".format(extra_points))
        print("get_total_path_length: {}".format(get_total_path_length(extra_points, len(found_path))))

        if extra_points > 0:
            new_path = []

            step_count = 2 + extra_points
            for i in range(len(found_path) - 1):
                step = np.array((found_path[i + 1] - found_path[i]) / float(step_count))

                position = found_path[i]
                for current_step in range(step_count):
                    position = position + current_step * step
                    new_path.append(position)

            print("New path: {}".format(new_path))
            for i in range(len(new_path)):
                sub_path = new_path[i]
                print(i, "--- ", sub_path)

