import numpy as np
import math


m_Boom_Length = 4400
m_Arm_Length = 2110 
m_Bucket_Length = 1260
m_Height = 1700


def end_point_kinematics(boom_angle, arm_angle, bucket_angle):
    x_pos = (m_Boom_Length * np.cos(np.deg2rad(boom_angle))) + \
            (m_Arm_Length * np.cos(np.deg2rad(boom_angle) + np.deg2rad(boom_angle-arm_angle))) + \
            (m_Bucket_Length * np.cos(np.deg2rad(boom_angle) + np.deg2rad(boom_angle-arm_angle) +
                                      np.deg2rad(arm_angle-bucket_angle)))

    z_pos = (m_Boom_Length * np.sin(np.deg2rad(boom_angle))) + \
            (m_Arm_Length * np.sin(np.deg2rad(boom_angle) + np.deg2rad(boom_angle-arm_angle))) + \
            (m_Bucket_Length * np.sin(np.deg2rad(boom_angle) + np.deg2rad(boom_angle-arm_angle) +
                                      np.deg2rad(arm_angle-bucket_angle))) + m_Height

    return x_pos, z_pos


def end_point_kinematics_test(boom_angle, arm_angle, bucket_angle):
    x_pos = (m_Boom_Length * np.cos(np.deg2rad(boom_angle))) + \
            (m_Arm_Length * np.cos(np.deg2rad(arm_angle))) + \
            (m_Bucket_Length * np.cos(np.deg2rad(bucket_angle)))

    z_pos = (m_Boom_Length * np.sin(np.deg2rad(boom_angle))) + \
            (m_Arm_Length * np.sin(np.deg2rad(arm_angle))) + \
            (m_Bucket_Length * np.sin(np.deg2rad(bucket_angle))) \
            + m_Height

    return x_pos, z_pos


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
