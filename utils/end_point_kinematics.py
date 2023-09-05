import numpy as np
import math


boom_length = 4400  # unit: mm
arm_length = 2107   # unit: mm
bucket_length = 1290  # unit: mm
height_offset = 1700  # unit: mm
length_offset = 150  # unit: mm


def end_point_kinematics(boom_angle, arm_angle, bucket_angle):
    x_pos_boom = boom_length * np.cos(np.deg2rad(boom_angle))
    x_pos_arm = arm_length * np.cos(np.deg2rad(arm_angle))
    x_pos_bucket = bucket_length * np.cos(np.deg2rad(bucket_angle))
    x_pos = x_pos_boom + x_pos_arm + x_pos_bucket + length_offset

    z_pos_boom = boom_length * np.sin(np.deg2rad(boom_angle))
    z_pos_arm = arm_length * np.sin(np.deg2rad(arm_angle))
    z_pos_bucket = bucket_length * np.sin(np.deg2rad(bucket_angle))
    z_pos = z_pos_boom + z_pos_arm + z_pos_bucket + height_offset

    return x_pos, z_pos


theta4_1: float = 0
theta4_2: float = 0
cal_link_angle: float = 0

l1 = 351
l2 = 421
l3 = 457
l4 = 510


def convert_bucket_angle(angle) -> float:
    return np.rad2deg(np.arcsin(np.sin(np.deg2rad(angle))))


angle_t1 = 0
result_angle = 0


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