import time
import numpy as np
import pandas as pd
from utils.warning_target import get_target_boundary, get_distance, TargetDepth


data = pd.read_csv('../data/data.csv')
x = data['Y(E)']
y = data['X(N)']
z = data['Z(h)']

x = x[:64]
y = y[:64]
z = z[:64]

target = np.vstack((x, y)).T
depth_estimator = TargetDepth(target, z)
depth = depth_estimator.get_target_depth([10000, 20000])
print(depth)

pos = get_target_boundary(target, radius=0.5, step=5)
dis = get_distance([10000, 20000], pos[0])

