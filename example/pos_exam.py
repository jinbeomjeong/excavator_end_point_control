import time
import numpy as np
import pandas as pd
from utils.warning_target import get_target_boundary, get_distance, TargetDepth, get_end_pos
from utils.tcp_lib import TCPClient


radius: float = 1.0

np.set_printoptions(suppress=True)
data = pd.read_csv('../data/data.csv')
x = data['Y(E)'][:64]
y = data['X(N)'][:64]
z = data['Z(h)'][:64]

xy_map = np.vstack((x, y)).T
xz_map = np.vstack((x, z)).T

depth_estimator = TargetDepth(xy_map, z)
target_boundary = get_target_boundary(xy_map, radius=radius, step=5)

tcp_handle = TCPClient(address='localhost', port=6340)
tcp_handle.connect_to_server()
tcp_handle.send_msg(str(radius))
tcp_start_data = [xy_map, xz_map, target_boundary]

for data in tcp_start_data:
    tcp_handle.send_msg(np.array2string(data.flatten('C'), precision=1, separator=',').lstrip('[').rstrip(']'))

t0 = time.time()
while True:
    recv_data = np.fromstring(tcp_handle.receive_msg(), dtype=np.float32, sep=',')
    x_pos = recv_data[0]
    y_pos = recv_data[1]
    heading = recv_data[2]

    depth = depth_estimator.get_target_depth([x_pos, y_pos])
    tcp_handle.send_msg(str(depth))

    dis = get_distance([x_pos, y_pos], target_boundary[0])
    idx = np.argmin(dis)
