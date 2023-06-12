import pandas as pd
import numpy as np
from utils.warning_target import TargetDepth, get_target_boundary, interp_2d_target


data = pd.read_csv('./data/pipe_info.csv', header=None)

xy_map = np.vstack((data[0], data[1])).T
est_pipe_depth = TargetDepth(target_pos=xy_map, target_depth=data[2].to_numpy())
est_depth = est_pipe_depth.get_target_depth([126.86273295, 36.051940])


inter_pos = get_target_boundary(target_pos=xy_map, radius=data[3].to_numpy(), step=10)


pd.DataFrame(inter_pos[0]).to_csv('./data/inter_pos.csv')
