import pandas as pd
import numpy as np
from pyproj import Transformer

# 2097
transformer_grs80_to_wgs84 = Transformer.from_crs("EPSG:5186", "EPSG:4326")
transformer_wgs84_to_grs80 = Transformer.from_crs("EPSG:4326", "EPSG:5186")

data = pd.read_csv(filepath_or_buffer='../data/test_20231006-120121.csv')
x_pos = data['bucket_goal_x_pos(m)'].to_numpy()
y_pos = data['bucket_goal_y_pos(m)'].to_numpy()

x_pos_wgs = data['gps_1_latitude'].to_numpy()
y_pos_wgs = data['gps_1_longitude'].to_numpy()


# print(transformer_wgs84_to_grs80.transform(xx=x_pos_wgs, yy=y_pos_wgs))
conv_data = transformer_grs80_to_wgs84.transform(xx=x_pos, yy=y_pos)

conv_x = conv_data[0]
conv_y = conv_data[1]

print(transformer_wgs84_to_grs80.transform(xx=conv_x, yy=conv_y))