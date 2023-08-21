import numpy as np
import pandas as pd
from utils.end_point_kinematics import end_point_kinematics

data = pd.read_csv('data/arm_action_20230725-131749.csv')
boom_angle = data['boom_angle'].values
arm_angle = data['arm_angle'].values
bucket_angle = data['bucket_angle'].values

# calculate relative angle
relative_arm_angle = boom_angle - arm_angle
relative_bucket_angle = bucket_angle - relative_arm_angle - boom_angle

pass


# for boom, arm in data:
#     x_pos, z_pos, arm_rela, bucket_rela = end_point_kinematics(boom_angle=boom, arm_angle=arm, bucket_angle=0)
#     result.append(arm_rela)
#
# result = np.array(result)
# print(result)