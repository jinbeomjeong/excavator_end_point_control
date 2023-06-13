import pandas as pd
import numpy as np
from utils.calculate_gps_data import gps_unit_change
from utils.warning_target import TargetDepth, get_target_boundary, interp_2d_target


data = pd.read_csv('./data/pipe_info.csv', header=None)

xy_map = np.vstack((data[0], data[1])).T
xy_map_tm = np.zeros((xy_map.shape[0], 2), dtype=np.double)

for i, xy_pos in enumerate(xy_map):
    xy_map_tm[i, 0], xy_map_tm[i, 1] = gps_unit_change(latitude=xy_pos[0], longitude=xy_pos[1])

est_pipe_depth = TargetDepth(target_pos=xy_map_tm, target_depth=data[2].to_numpy())
est_depth = est_pipe_depth.get_target_depth(np.array([126.86273295, 36.051940]))


inter_pos = get_target_boundary(target_pos=xy_map, radius=data[3].to_numpy(), step=10)


pd.DataFrame(inter_pos[0]).to_csv('./data/inter_pos.csv')
