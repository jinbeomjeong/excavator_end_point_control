import numpy as np
import math


m_Boom_Length = 4400
m_Arm_Length = 2110 
m_Bucket_Length = 1260
m_Height = 1700


def end_point_kinematics(boom_angle: float, arm_angle: float, bucket_angle: float):
    arm_relative_angle = np.multiply(np.subtract(boom_angle, arm_angle), -1)
    bucket_relative_angle = np.multiply(np.subtract(arm_angle, bucket_angle), -1)

    x_pos_boom = m_Boom_Length * np.cos(np.deg2rad(boom_angle))
    x_pos_arm = m_Arm_Length * np.cos(np.deg2rad(arm_relative_angle))
    x_pos_bucket = m_Bucket_Length * np.cos(np.deg2rad(bucket_relative_angle))
    x_pos = x_pos_boom + x_pos_arm + x_pos_bucket

    z_pos_boom = m_Boom_Length * np.sin(np.deg2rad(boom_angle))
    z_pos_arm = m_Arm_Length * np.sin(np.deg2rad(arm_relative_angle))
    z_pos_bucket = m_Bucket_Length * np.sin(np.deg2rad(bucket_relative_angle))
    z_pos = z_pos_boom + z_pos_arm + z_pos_bucket + m_Height

    return x_pos, z_pos, arm_relative_angle, bucket_relative_angle


theta4_1: float = 0
theta4_2: float = 0

L1 = 348
L2 = 420
L3 = 450
L4 = 508

G = 0
F = 0
E = 0

g = 0
f = 0
e = 0


def link_calculator(theta2):
    G = L1 / L2
    F = L1 / L4
    E = (pow(L3, 2) - pow(L2, 2) - pow(L4, 2) - pow(L1, 2)) / (-2 * L2 * L4)
    g = E - (F * math.cos(theta2 * math.pi / 180)) - math.cos(theta2 * math.pi / 180) + G
    f = -2 * math.sin(theta2 * math.pi / 180)
    e = E - (F * math.cos(theta2 * math.pi / 180)) + math.cos(theta2 * math.pi / 180) - G

    theta4_1 = 2 * math.degrees(math.atan(((-1 * f) + math.sqrt(pow(f, 2) - (4 * e * g)))/(2 * e)))
    theta4_2 = 2 * math.degrees(math.atan(((-1 * f) - math.sqrt(pow(f, 2) - (4 * e * g)))/(2 * e)))

    return theta4_1, theta4_2


angle_t1 = 0
converted_angle = 0


def convert_bucket_angle(angle: float) -> float:
    global angle_t1, converted_angle

    if (270 <= angle <= 360) and (270 <= angle_t1 <= 360) and (angle-angle_t1 <= -20):
        converted_angle = angle+360

    else:
        converted_angle = angle

    if (0 <= angle <= 90) and (0 <= angle_t1 <= 90) and (angle-angle_t1 <= 20):
        converted_angle = angle-360

    angle_t1 = angle

    return converted_angle
