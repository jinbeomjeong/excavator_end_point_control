import numpy as np
import pandas as pd
from utils.warning_target import est_pipe_depth_euclidean, interp_2d_target, interp_1d_target

data = pd.read_csv(filepath_or_buffer='../data/pipe_info.csv', header=None).to_numpy()

origin_arr = np.array([126.86318003, 36.05227849])
pipe_coords_interp = interp_2d_target(data[:, 0:2], step=10)
depth_interp = interp_1d_target(data[:, 2], step=10)
depth = est_pipe_depth_euclidean(target_position=origin_arr, pipe_position=pipe_coords_interp, pipe_depth=depth_interp)
print(depth)
