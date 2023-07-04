import numpy as np

angle_prv = np.zeros(3, dtype=np.double)
joystick_pos_prv = np.zeros(4, dtype=np.uint8)
excavator_pos_prv = np.zeros(2, dtype=np.double)
excavator_work_status: int = 0

boom_angle_threshold: float = 0.2
arm_angle_threshold: float = 0.2
bucket_angle_threshold: float = 0.2
joystick_position_threshold: int = 1
excavator_pos_threshold: float = 0.05


def recognition_excavator_work_status(boom_angle, arm_angle, bucket_angle, left_joystick_x_pos, left_joystick_y_pos,
                                      right_joystick_x_pos, right_joystick_y_pos, excavator_latitude, excavator_longitude):

    global angle_prv, joystick_pos_prv, excavator_pos_prv, excavator_work_status

    boom_derivative = np.abs(boom_angle-angle_prv[0])
    arm_derivative = np.abs(arm_angle-angle_prv[1])
    bucket_derivative = np.abs(bucket_angle-angle_prv[2])

    left_joystick_x_pos_derivative = np.abs(left_joystick_x_pos-joystick_pos_prv[0])
    left_joystick_y_pos_derivative = np.abs(left_joystick_y_pos-joystick_pos_prv[1])
    right_joystick_x_pos_derivative = np.abs(right_joystick_x_pos-joystick_pos_prv[2])
    right_joystick_y_pos_derivative = np.abs(right_joystick_y_pos-joystick_pos_prv[3])

    excavator_latitude_derivative = np.abs(excavator_latitude-excavator_pos_prv[0])
    excavator_longitude_derivative = np.abs(excavator_longitude-excavator_pos_prv[1])

    boom_move_recognition = boom_derivative <= boom_angle_threshold
    arm_move_recognition = arm_derivative <= arm_angle_threshold
    bucket_move_recognition = bucket_derivative <= bucket_angle_threshold

    left_joystick_x_pos_move_recognition = left_joystick_x_pos_derivative <= joystick_position_threshold
    left_joystick_y_pos_move_recognition = left_joystick_y_pos_derivative <= joystick_position_threshold
    right_joystick_x_pos_move_recognition = right_joystick_x_pos_derivative <= joystick_position_threshold
    right_joystick_y_pos_move_recognition = right_joystick_y_pos_derivative <= joystick_position_threshold

    excavator_latitude_move_recognition = excavator_latitude_derivative <= excavator_pos_threshold
    excavator_longitude_move_recognition = excavator_longitude_derivative <= excavator_pos_threshold

    if np.all([boom_move_recognition, arm_move_recognition, bucket_move_recognition,
               left_joystick_x_pos_move_recognition, left_joystick_y_pos_move_recognition,
               right_joystick_x_pos_move_recognition, right_joystick_y_pos_move_recognition,
               excavator_latitude_move_recognition, excavator_longitude_move_recognition]):

        excavator_work_status = 0  # 00: stand by

    elif np.any([excavator_latitude_move_recognition, excavator_longitude_move_recognition]):
        excavator_work_status = 1  # 01: movement state

    elif np.any([left_joystick_x_pos_move_recognition, left_joystick_y_pos_move_recognition,
                 right_joystick_x_pos_move_recognition, right_joystick_y_pos_move_recognition]):

        excavator_work_status = 2  # 02: working state

    for i, angle in enumerate([boom_angle, arm_angle, bucket_angle]):
        angle_prv[i] = angle

    for i, joystick_pos in enumerate([left_joystick_x_pos, left_joystick_y_pos, right_joystick_x_pos, right_joystick_y_pos]):
        joystick_pos_prv[i] = joystick_pos

    for i, excavator_pos in enumerate([excavator_latitude, excavator_longitude]):
        excavator_pos_prv[i] = excavator_pos

    return excavator_work_status
