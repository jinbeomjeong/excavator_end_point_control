import numpy as np
from scipy.spatial import distance
from numba import jit
from typing import Tuple, List


est_depth_arr = np.zeros(2, dtype=np.double)
est_depth: float = 0.0
in_range_x_pos, in_range_y_pos = 0, 0
bucket_in_range = []
bucket_global_pos = np.zeros(shape=2, dtype=np.float64)


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


def interp_1d_target(target_position: np.ndarray, step: int = 1) -> np.ndarray:
    output_array = np.array([])

    for i in range(target_position.shape[0]-1):
        inter_arr = np.linspace(target_position[i], target_position[i+1], step, endpoint=False)
        output_array = np.append(output_array, inter_arr)

    output_array = np.append(output_array, target_position[target_position.shape[0]-1])

    return output_array


def est_pipe_depth_interpolate(target_position: np.ndarray, pipe_position: np.ndarray, pipe_depth: np.ndarray,
                               safety_length: float = 0.0) -> float:
    global est_depth_arr

    for i, pos in enumerate(target_position):
        est_depth_arr[i] = (np.interp(x=pos, xp=pipe_position[:, i], fp=pipe_depth))+safety_length

    return np.multiply(np.max(est_depth), 1000)


def est_pipe_depth_nearest(target_position: np.ndarray, pipe_position: np.ndarray, pipe_depth: np.ndarray,
                           safety_length: float = 0.0) -> float:
    global est_depth_arr

    for i, pos in enumerate(target_position):
        est_depth_arr[i] = pipe_depth[np.argmin(np.abs(pos-pipe_position[:, i]))]+safety_length

    return np.multiply(np.max(est_depth), 1000)


def est_pipe_depth_euclidean(target_position: np.ndarray, pipe_position: np.ndarray, pipe_depth: np.ndarray,
                             safety_length: float = 0.0) -> float:
    global est_depth

    pipe_pos_index_arr = np.arange(pipe_position.shape[0])
    distance_arr = get_distance(origin=target_position, target=pipe_position)
    est_depth = np.interp(x=np.argmin(distance_arr), xp=pipe_pos_index_arr, fp=pipe_depth)+safety_length

    return est_depth.item()


def get_target_boundary(target_pos: np.ndarray, radius: np.ndarray, step: int = 10) -> np.ndarray:
    # target_pos: np.ndarray of n by 2, radius np.ndarray of len n
    target_dir = np.arctan2(target_pos[:, 0], target_pos[:, 1])
    bound_dir_max = target_dir + (np.pi / 2)
    bound_dir_min = target_dir - (np.pi / 2)

    bound_pos_x_max = (np.sin(bound_dir_max)*radius)+target_pos[:, 0]
    bound_pos_y_max = (np.cos(bound_dir_max)*radius)+target_pos[:, 1]
    bound_pos_x_min = (np.sin(bound_dir_min)*radius)+target_pos[:, 0]
    bound_pos_y_min = (np.cos(bound_dir_min)*radius)+target_pos[:, 1]

    target_bound_max = np.array([bound_pos_x_max, bound_pos_y_max]).T
    target_bound_min = np.array([bound_pos_x_min, bound_pos_y_min]).T
    target_bound_max_interp = interp_2d_target(target_bound_max, step=step)
    target_bound_min_interp = interp_2d_target(target_bound_min, step=step)

    return np.stack([target_bound_max_interp, target_bound_min_interp])  # output: np.ndarray of 2*n*2


def recognition_in_range_boundary(input_pos: np.ndarray, target_pos_max: np.ndarray, target_pos_min: np.ndarray) -> int:
    global in_range_x_pos, in_range_y_pos, bucket_in_range
    bucket_in_range = []

    for i in range(target_pos_min.shape[0]):
        in_range_y_pos = np.all([np.min([target_pos_min[i, 0], target_pos_max[i, 0]]) < input_pos[0],
                                 input_pos[0] < np.max([target_pos_min[i, 0], target_pos_max[i, 0]])])
        in_range_x_pos = np.all([np.min([target_pos_min[i, 1], target_pos_max[i, 1]]) < input_pos[1],
                                 input_pos[1] < np.max([target_pos_min[i, 1], target_pos_max[i, 1]])])
        bucket_in_range.append(np.all([in_range_y_pos, in_range_x_pos]))

    return np.any(bucket_in_range)*1


def get_working_area(origin, radius):
    area_range = np.deg2rad(np.linspace(0, 360, 361))
    x_pos = np.sin(area_range) * radius + origin[0]
    y_pos = np.cos(area_range) * radius + origin[1]

    return x_pos, y_pos


def get_distance(origin: np.ndarray, target: np.ndarray) -> np.ndarray:
    dis_arr = np.zeros(target.shape[0], dtype=np.float64)

    for i, pos in enumerate(target):
        dis_arr[i] = distance.euclidean(origin, pos)

    return dis_arr


@jit(nopython=True, cache=True)
def get_distance_opt(origin: np.ndarray, target: np.ndarray, threshold: float) -> Tuple[np.ndarray, np.ndarray]:
    dis_arr = np.zeros(target.shape[0], dtype=np.float64)
    dis_bool = np.zeros(target.shape[0], dtype=np.uint8)

    for i, pos in enumerate(target):
        dis_arr[i] = np.sqrt(np.power(origin[0]-pos[0], 2) + np.power(origin[1]-pos[1], 2))
        dis_bool[i] = int(dis_arr[i] < threshold)

    in_work = np.any(dis_bool)

    return dis_arr, in_work


def get_bucket_global_pos(origin: np.ndarray, y_distance: np.ndarray, heading: np.ndarray) -> np.ndarray:
    global bucket_global_pos

    heading = (heading+360) % 360
    y_distance /= 1000
    bucket_global_pos[0] = np.cos(np.deg2rad(heading)) * y_distance + origin[0]
    bucket_global_pos[1] = np.sin(np.deg2rad(heading)) * y_distance + origin[1]

    return bucket_global_pos


def get_close_pipe_status(bucket_pos: np.ndarray, pipe_pos: np.ndarray) -> int:
    end_point_status = 1 if np.sum(__get_distance(bucket_pos, pipe_pos) < 0.01) else 0

    return end_point_status
