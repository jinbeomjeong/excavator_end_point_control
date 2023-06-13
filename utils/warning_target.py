import numpy as np
from scipy.spatial import distance
from numba import jit


class TargetDepth:
    def __init__(self, target_pos: np.ndarray, target_depth: np.ndarray):  # target_pos: np.ndarray of n*2
        self.target_pos = target_pos
        self.target_depth = target_depth
        self.depth = np.zeros(2, dtype=np.double)

    def get_target_depth(self, current_pos: np.ndarray):  # current_pos: np.ndarray of len 2
        for i, pos in enumerate(current_pos):
            self.depth[i] = np.interp(pos, self.target_pos[:, i], self.target_depth)

        return np.min(self.depth)


def interp_2d_target(target_pos: np.ndarray, step) -> np.ndarray:  # target_pos: np.ndarray of n*2
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


def get_target_boundary(target_pos: np.ndarray, radius: np.ndarray, step: int = 10) -> np.ndarray:
    # target_pos: np.ndarray of n by 2, radius np.ndarray of len n
    radius = np.divide(radius, 2)
    bound_pos_x_max = []
    bound_pos_y_max = []
    bound_pos_x_min = []
    bound_pos_y_min = []

    for i in range((target_pos.shape[0]-1)):
        target_dir = np.arctan2(target_pos[i+1, 0]-target_pos[i, 0], target_pos[i+1, 1]-target_pos[i, 1])
        safety_dir_max = target_dir-(np.pi/2)
        safety_dir_min = target_dir+(np.pi/2)

        bound_pos_x_max.append((np.sin(safety_dir_max)*radius[i])+target_pos[i, 0])
        bound_pos_y_max.append((np.cos(safety_dir_max)*radius[i])+target_pos[i, 1])
        bound_pos_x_min.append((np.sin(safety_dir_min)*radius[i])+target_pos[i, 0])
        bound_pos_y_min.append((np.cos(safety_dir_min)*radius[i])+target_pos[i, 1])

    # bound_pos_x_max.append((np.sin(safety_dir_max)*radius+target_pos[target_pos.shape[0]-1, 0]))
    # bound_pos_y_max.append((np.cos(safety_dir_max)*radius+target_pos[target_pos.shape[0]-1, 1]))
    # bound_pos_x_min.append((np.sin(safety_dir_min)*radius+target_pos[target_pos.shape[0]-1, 0]))
    # bound_pos_y_min.append((np.cos(safety_dir_min)*radius+target_pos[target_pos.shape[0]-1, 1]))

    target_bound_max = np.array([bound_pos_x_max, bound_pos_y_max]).T
    target_bound_min = np.array([bound_pos_x_min, bound_pos_y_min]).T
    target_bound_max_interp = interp_2d_target(target_bound_max, step=step)
    target_bound_min_interp = interp_2d_target(target_bound_min, step=step)

    return np.stack([target_bound_max_interp, target_bound_min_interp])  # output: np.ndarray of 3*n*2


def get_working_area(origin, radius):
    area_range = np.deg2rad(np.linspace(0, 360, 361))
    x_pos = np.sin(area_range) * radius + origin[0]
    y_pos = np.cos(area_range) * radius + origin[1]

    return x_pos, y_pos


def __get_distance(origin: np.ndarray, target: np.ndarray) -> np.ndarray:
    dis_arr = np.zeros(target.shape[0])

    for i, pos in enumerate(target):
        dis_arr[i] = distance.euclidean(origin, pos)

    return dis_arr


@jit(nopython=True, cache=True)
def __get_distance_opt(origin: np.ndarray, target: np.ndarray, threshold: float) -> [np.ndarray, np.bool_]:
    dis_arr = np.zeros(target.shape[0], dtype=np.float32)
    dis_bool = np.zeros(target.shape[0], dtype=np.uint8)

    for i, pos in enumerate(target):
        dis_arr[i] = np.sqrt(np.power(origin[0]-pos[0], 2) + np.power(origin[1]-pos[1], 2))
        dis_bool[i] = int(dis_arr[i] < threshold)

    in_work = np.any(dis_bool)

    return [dis_arr, in_work]


def get_end_pos(origin: [], y_distance, heading) -> [np.float32, np.float32]:
    heading = (heading+360) % 360
    x_pos = np.sin(np.deg2rad(heading)) * y_distance + origin[0]
    y_pos = np.cos(np.deg2rad(heading)) * y_distance + origin[1]

    return x_pos, y_pos


def get_safety_status(end_point: np.ndarray, target: np.ndarray) -> int:
    end_point_status = 1 if np.sum(__get_distance(end_point, target) < 0.01) else 0

    return end_point_status
