import numpy as np


boom_length = 4400  # unit: mm
arm_length = 2107   # unit: mm
bucket_length = 1290  # unit: mm
height_offset = 1700  # unit: mm
length_offset = 150  # unit: mm

angle_t1 = 0
result_angle = 0
absolute_angle = np.zeros(shape=3, dtype=np.float64)  # 0: boom angle, 1: body angle, 2: bucket angle
local_x_pos = np.zeros(shape=4, dtype=np.float64)  # 0: boom x-axis, 1: arm x-axis, 2: bucket x-axis, 3: local x-axis
local_z_pos = np.zeros(shape=4, dtype=np.float64)  # 0: boom z-axis, 1: arm z-axis, 2: bucket z-axis, 3: local z-axis


def end_point_kinematics(body_rl_angle, boom_rl_angle, arm_rl_angle, bucket_rl_angle) -> [float, float]:
    # calculate absolute angle
    absolute_angle[0] = np.sum([boom_rl_angle, body_rl_angle])
    absolute_angle[1] = np.sum(([arm_rl_angle, boom_rl_angle]))
    absolute_angle[2] = np.sum([bucket_rl_angle, arm_rl_angle, boom_rl_angle])

    local_x_pos[0] = boom_length * np.cos(np.deg2rad(absolute_angle[0]))
    local_x_pos[1] = arm_length * np.cos(np.deg2rad(absolute_angle[1]))
    local_x_pos[2] = bucket_length * np.cos(np.deg2rad(absolute_angle[2]))
    local_x_pos[3] = np.sum(local_x_pos[0:3])+length_offset

    local_z_pos[0] = boom_length * np.sin(np.deg2rad(absolute_angle[0]))
    local_z_pos[1] = arm_length * np.sin(np.deg2rad(absolute_angle[1]))
    local_z_pos[2] = bucket_length * np.sin(np.deg2rad(absolute_angle[2]))
    local_z_pos[3] = np.sum(local_z_pos[0:3])+height_offset

    return local_x_pos[3], local_z_pos[3]


def convert_bucket_angle(angle) -> float:
    return np.rad2deg(np.arcsin(np.sin(np.deg2rad(angle))))


def angle_suppression_filter(angle) -> float:
    global angle_t1, result_angle

    if np.abs(angle-angle_t1) > 10:
        result_angle = angle_t1
    else:
        result_angle = angle

    angle_t1 = angle

    return result_angle


def bucket_link_calculator(bucket_link_angle):
    Lk3, Lk4, Lk7, Lk2, Lk6, Lk9 = 452, 420, 1423, 507.5, 350, 1759
    a3, a4 = arm_length, bucket_length
    delta_k1 = cos2ndraw(Lk9, Lk6, a3)
    delta_k7 = bucket_link_angle

    alpha_k2 = delta_k1 + delta_k7
    lk_link = cos2ndraw2(alpha_k2, Lk2, Lk6)
    alpha_k3 = cos2ndraw(Lk2, lk_link, Lk6)
    alpha_k4 = cos2ndraw(Lk3, lk_link, Lk4)
    delta_k3 = cos2ndraw(Lk7, Lk4, a4)
    bkt = 180 - (delta_k1 + alpha_k3 + alpha_k4 + delta_k3)

    return bkt


def cos2ndraw2(angle_a, side_b, side_c):
    side_a = np.sqrt(side_b ** 2 + side_c ** 2 - 2 * side_b * side_c * np.cos(np.radians(angle_a)))

    return side_a


def cos2ndraw(side_a, side_b, side_c):
    A_angle = np.arccos((side_b ** 2 + side_c ** 2 - side_a ** 2) / (2 * side_b * side_c))
    A_angle = np.degrees(A_angle)  # convert to degrees

    return A_angle
