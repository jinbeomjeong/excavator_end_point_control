import pandas as pd
import numpy as np
from utils.warning_target import recognition_in_range_boundary


pipe_boundary = pd.read_csv('./data/pipe_boundary_tm.csv', header=None).to_numpy()
pos = np.array([283724, 187280])

pipe_max, pipe_min = pipe_boundary[:, 0:2], pipe_boundary[:, 2:4]
state = recognition_in_range_boundary(input_pos=pos, target_pos_max=pipe_max, target_pos_min=pipe_min)

print(state)
