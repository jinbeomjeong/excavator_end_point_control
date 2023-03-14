import numpy as np
import math


m_Boom_Length = 4400
m_Arm_Length = 2110 
#m_Arm_Length = 0 

# m_Arm_Length = 0 
m_Bucket_Length = 1260
#m_Bucket_Length = 0

#m_Bucket_Length = 1264
m_Height = 1700


def end_point_kinematics(boom_angle, arm_angle, bucket_angle):
    x_pos = (m_Boom_Length * np.cos(np.deg2rad(boom_angle))) + \
            (m_Arm_Length * np.cos(np.deg2rad(boom_angle) + np.deg2rad(arm_angle))) + \
            (m_Bucket_Length * np.cos(np.deg2rad(boom_angle) + np.deg2rad(arm_angle) + np.deg2rad(bucket_angle)))

    z_pos = (m_Boom_Length * np.sin(np.deg2rad(boom_angle))) + \
            (m_Arm_Length * np.sin(np.deg2rad(boom_angle) + np.deg2rad(arm_angle))) + \
            (m_Bucket_Length * np.sin(np.deg2rad(boom_angle) + np.deg2rad(arm_angle) + np.deg2rad(bucket_angle))) \
            + m_Height

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

L1: int = 348
L2: int = 508
L3: int = 450
L4: int = 420

G: float = 0
F: float = 0
E: float = 0

g: float = 0
f: float = 0
e: float = 0

def Link_Calculator(theta2):
    G = L1/L2
    F = L1/L4
    E = (pow(L3, 2) - pow(L2, 2) - pow(L4, 2) - pow(L1, 2))/(-2 * L2 * L4)
    g = E - F * math.cos(theta2) - math.cos(theta2) + G
    f = -2 * math.sin(theta2)
    e = E - (F * math.cos(theta2)) + math.cos(theta2) - G

    theta4_1 = 2 * math.degrees(math.atan(((-1 * f) + math.sqrt(pow(f , 2) - (4 * e * g)))/(2 * e)))
    theta4_2 = 2 * math.degrees(math.atan(((-1 * f) - math.sqrt(pow(f , 2) - (4 * e * g)))/(2 * e)))

    return theta4_1, theta4_2