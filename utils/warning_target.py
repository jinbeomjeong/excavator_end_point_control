import numpy as np
from scipy.spatial import distance


class TargetDepth:
    def __init__(self, target_pos, target_depth):
        self.target_pos = target_pos
        self.target_depth = target_depth

    def get_target_depth(self, current_pos):
        depth = np.zeros(2)

        for i, pos in enumerate(current_pos):
            depth[i] = np.interp(pos, self.target_pos[:, i], self.target_depth)

        return np.min(depth)


def interp_2d_target(target_pos, step):
    x_output = np.array([])
    y_output = np.array([])

    for i in range(target_pos.shape[0]-1):
        x_inter = np.linspace(target_pos[i, 0], target_pos[i+1, 0], step, endpoint=False)
        y_inter = np.interp(x_inter, target_pos[:, 0], target_pos[:, 1])
        x_output = np.append(x_output, x_inter)
        y_output = np.append(y_output, y_inter)

    target_interp_pos = np.vstack((x_output, y_output)).T
    target_interp_pos = np.vstack((target_interp_pos, [target_pos[target_pos.shape[0]-1, 0],
                                                       target_pos[target_pos.shape[0]-1, 1]]))

    return target_interp_pos


def get_target_boundary(target_pos, radius, step=10):
    radius = radius/2
    bound_pos_x_max = []
    bound_pos_y_max = []
    bound_pos_x_min = []
    bound_pos_y_min = []

    for i in range((target_pos.shape[0]-1)):
        target_dir = np.arctan2(target_pos[i+1, 0]-target_pos[i, 0], target_pos[i+1, 1]-target_pos[i, 1])
        safety_dir_max = target_dir-(np.pi/2)
        safety_dir_min = target_dir+(np.pi/2)

        bound_pos_x_max.append((np.sin(safety_dir_max)*radius)+target_pos[i, 0])
        bound_pos_y_max.append((np.cos(safety_dir_max)*radius)+target_pos[i, 1])
        bound_pos_x_min.append((np.sin(safety_dir_min)*radius)+target_pos[i, 0])
        bound_pos_y_min.append((np.cos(safety_dir_min)*radius)+target_pos[i, 1])

    bound_pos_x_max.append((np.sin(safety_dir_max)*radius+target_pos[target_pos.shape[0]-1, 0]))
    bound_pos_y_max.append((np.cos(safety_dir_max)*radius+target_pos[target_pos.shape[0]-1, 1]))
    bound_pos_x_min.append((np.sin(safety_dir_min)*radius+target_pos[target_pos.shape[0]-1, 0]))
    bound_pos_y_min.append((np.cos(safety_dir_min)*radius+target_pos[target_pos.shape[0]-1, 1]))

    target_bound_max = np.array([bound_pos_x_max, bound_pos_y_max]).T
    target_bound_min = np.array([bound_pos_x_min, bound_pos_y_min]).T
    target_bound_max_inter = interp_2d_target(target_bound_max, step=step)
    target_bound_min_inter = interp_2d_target(target_bound_min, step=step)

    return np.stack([target_bound_max_inter, target_bound_min_inter])


def get_working_area(origin, radius):
    area_range = np.deg2rad(np.linspace(0, 360, 361))
    x_pos = np.sin(area_range) * radius + origin[0]
    y_pos = np.cos(area_range) * radius + origin[1]

    return x_pos, y_pos


def get_distance(origin, target):
    dis_arr = np.zeros(target.shape[0])

    for i, pos in enumerate(target):
        dis_arr[i] = distance.euclidean(origin, pos)

    return dis_arr


def get_end_pos(origin, y_distance, heading):
    heading = (heading+360) % 360
    x_pos = np.sin(np.deg2rad(heading)) * y_distance + origin[0]
    y_pos = np.cos(np.deg2rad(heading)) * y_distance + origin[1]

    return x_pos, y_pos


def get_safety_status(origin, end_point, target, radius):
    working_area_status = np.logical_or(get_distance(origin, target) < radius)
    end_point_status = np.logical_and(get_distance(origin, end_point) < 0.1)

    return working_area_status, end_point_status
