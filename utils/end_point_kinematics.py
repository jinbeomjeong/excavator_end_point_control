import numpy as np
import math


boom_length = 4400  # unit: mm
arm_length = 2110   # unit: mm
bucket_length = 1260  # unit: mm
height = 1700  # unit: mm


def end_point_kinematics_relative(boom_angle: float, arm_angle: float, bucket_angle: float):
    arm_relative_angle = np.multiply(np.subtract(boom_angle, arm_angle), -1)
    bucket_relative_angle = np.multiply(np.subtract(arm_angle, bucket_angle), -1)

    x_pos_boom = boom_length * np.cos(np.deg2rad(boom_angle))
    x_pos_arm = arm_length * np.cos(np.deg2rad(arm_relative_angle))
    x_pos_bucket = bucket_length * np.cos(np.deg2rad(bucket_relative_angle))
    x_pos = x_pos_boom + x_pos_arm

    z_pos_boom = boom_length * np.sin(np.deg2rad(boom_angle))
    z_pos_arm = arm_length * np.sin(np.deg2rad(arm_relative_angle))
    z_pos_bucket = bucket_length * np.sin(np.deg2rad(bucket_relative_angle))
    z_pos = z_pos_boom + z_pos_arm + height

    return x_pos, z_pos, arm_relative_angle, bucket_relative_angle


def end_point_kinematics_abs(boom_angle: float, arm_angle: float, bucket_angle: float):
    arm_relative_angle = np.multiply(np.subtract(boom_angle, arm_angle), -1)
    bucket_relative_angle = np.multiply(np.subtract(arm_angle, bucket_angle), -1)

    x_pos_boom = boom_length * np.cos(np.deg2rad(boom_angle))
    x_pos_arm = arm_length * np.cos(np.deg2rad(arm_angle))
    x_pos_bucket = bucket_length * np.cos(np.deg2rad(bucket_angle))
    x_pos = x_pos_boom + x_pos_arm + x_pos_bucket

    z_pos_boom = boom_length * np.sin(np.deg2rad(boom_angle))
    z_pos_arm = arm_length * np.sin(np.deg2rad(arm_angle))
    z_pos_bucket = bucket_length * np.sin(np.deg2rad(bucket_angle))
    z_pos = z_pos_boom + z_pos_arm + z_pos_bucket + height

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


def fcn_1(body_t, boom_t, arm_t):
    theta_BS_offset = -14.28
    theta_AS_offset = -7.44
    boom = boom_t - theta_BS_offset - body_t
    arm = arm_t - theta_AS_offset - boom - body_t
    return boom, arm


def fcn_2(body_t, boom_ang, arm_ang, bkt_link_t):
    Lk1, Lk3, Lk4, Lk5, Lk7, Lk2, Lk6, Lk8, Lk9 = 2294.95, 600.325, 515.6565, 2760, 1999.32, 660.7796, 474.3546, 879.2716, 2640.804
    a3, a4 = 3112.92, 1912.79
    theta_off3 = -91.0101
    delta_k1 = cos2ndraw(Lk9, Lk6, a3)
    delta_k7 = 180 + theta_off3 + bkt_link_t - arm_ang - boom_ang - body_t
    alpha_k2 = delta_k1 + delta_k7
    lk_link = cos2ndraw2(alpha_k2, Lk2, Lk6)
    alpha_k3 = cos2ndraw(Lk2, lk_link, Lk6)
    alpha_k4 = cos2ndraw(Lk3, lk_link, Lk4)
    delta_k3 = cos2ndraw(Lk7, Lk4, a4)
    bkt = 180 - (delta_k1 + alpha_k3 + alpha_k4 + delta_k3)
    return bkt, delta_k7, lk_link


def fwd_kin_30_fun(swing_angle: float, boom_angle: float, arm_angle: float, bucket_angle: float):
    a1, d4 = 120, 0
    Lk1, Lk3, Lk4, Lk5, Lk7, Lk2, Lk6, Lk8, Lk9 = 2294.95, 600.325, 515.6565, 2760, 1999.32, 660.7796, 474.3546, 879.2716, 2640.804
    a2, a3, a4 = 6246.35, 3112.92, 1912.79
    b1 = 30

    x = (a4*np.cos(np.radians(swing_angle))*np.cos(np.radians(boom_angle+arm_angle+bucket_angle))
         +a3*np.cos(np.radians(swing_angle))*np.cos(np.radians(boom_angle+arm_angle))
         +a2*np.cos(np.radians(swing_angle))*np.cos(np.radians(boom_angle))
         +a1*np.cos(np.radians(swing_angle))+d4*np.sin(np.radians(swing_angle))
         +b1*np.sin(np.radians(swing_angle)))

    y = (a4*np.sin(np.radians(swing_angle))*np.cos(np.radians(boom_angle+arm_angle+bucket_angle))
         +a3*np.sin(np.radians(swing_angle))*np.cos(np.radians(boom_angle+arm_angle))
         +a2*np.sin(np.radians(swing_angle))*np.cos(np.radians(boom_angle))
         +a1*np.sin(np.radians(swing_angle))-d4*np.cos(np.radians(swing_angle))
         -b1*np.cos(np.radians(swing_angle)))

    z = (a4*np.sin(np.radians(boom_angle+arm_angle+bucket_angle))
         +a3*np.sin(np.radians(boom_angle+arm_angle))
         +a2*np.sin(np.radians(boom_angle)))

    beta = 90
    alpha = np.rad2deg(np.arctan2(-np.cos(np.radians(swing_angle)), np.sin(np.radians(swing_angle))))
    gamma = np.rad2deg(np.arctan2(np.cos(np.radians(boom_angle+arm_angle+bucket_angle)),
                                  -np.sin(np.radians(boom_angle+arm_angle+bucket_angle))))
    position = [x, y, z]
    rotation = [alpha, beta, gamma]

    return position


def cos2ndraw2(angle_a, side_b, side_c):
    side_a = np.sqrt(side_b ** 2 + side_c ** 2 - 2 * side_b * side_c * np.cos(np.radians(angle_a)))
    return side_a


def cos2ndraw(side_a, side_b, side_c):
    A_angle = np.arccos((side_b ** 2 + side_c ** 2 - side_a ** 2) / (2 * side_b * side_c))
    A_angle = np.degrees(A_angle)  # convert to degrees
    return A_angle
