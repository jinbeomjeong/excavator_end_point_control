import numpy as np
import pandas as pd
from utils.calculate_gps_data import calculation_heading, gps_unit_change, est_gps_center


pipe_data = pd.read_csv('./data/pipe_info.csv', header=None).to_numpy()
pipe_depth = pipe_data[:, 2]
est_depth = np.zeros(2)
pipe_coords_tm = np.zeros((pipe_data.shape[0], 2), dtype=np.double)

for i, pipe_coord in enumerate(pipe_data):  # convert TM coordinate of pipe coordinate
    pipe_coords_tm[i, 1], pipe_coords_tm[i, 0] = gps_unit_change(latitude=pipe_coord[1], longitude=pipe_coord[0])

excavator_pos = np.array([283803.7205, 187403.5073])

for i, pos in enumerate(excavator_pos[0:2]):
    est_depth[i] = np.interp(pos, pipe_coords_tm[:, i], pipe_depth)

print(est_depth)
