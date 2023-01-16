import math


m_Boom_Length = 1350.019
m_Arm_Length = 703.452
m_Bucket_Length = 455.404
m_Height = 655
radian_conv = math.pi / 180.0


def end_point_kinematics(boom_angle, arm_angle, bucket_angle):
    x_pos = (m_Boom_Length * math.cos(boom_angle * radian_conv)) + \
            (m_Arm_Length * math.cos(boom_angle * radian_conv + arm_angle * radian_conv)) + \
            (m_Bucket_Length * math.cos(boom_angle * radian_conv + arm_angle * radian_conv + bucket_angle * radian_conv))

    z_pos = (m_Boom_Length * math.sin(boom_angle * radian_conv)) + \
            (m_Arm_Length * math.sin(boom_angle * radian_conv + arm_angle * radian_conv)) + \
            (m_Bucket_Length * math.sin(boom_angle * radian_conv + arm_angle * radian_conv + bucket_angle * radian_conv)) + m_Height

    #result = ('{0:.3f}').format(m_nZ)

    return x_pos, z_pos
