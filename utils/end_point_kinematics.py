import numpy as np


m_Boom_Length = 1350.019
m_Arm_Length = 703.452
m_Bucket_Length = 455.404
m_Height = 655


def end_point_kinematics(boom_angle, arm_angle, bucket_angle):
    x_pos = (m_Boom_Length * np.cos(np.deg2rad(boom_angle))) + \
            (m_Arm_Length * np.cos(np.deg2rad(boom_angle) + np.deg2rad(arm_angle))) + \
            (m_Bucket_Length * np.cos(np.deg2rad(boom_angle) + np.deg2rad(arm_angle) + np.deg2rad(bucket_angle)))

    z_pos = (m_Boom_Length * np.sin(np.deg2rad(boom_angle))) + \
            (m_Arm_Length * np.sin(np.deg2rad(boom_angle) + np.deg2rad(arm_angle))) + \
            (m_Bucket_Length * np.sin(np.deg2rad(boom_angle) + np.deg2rad(arm_angle) + np.deg2rad(bucket_angle))) \
            + m_Height

    return x_pos, z_pos
