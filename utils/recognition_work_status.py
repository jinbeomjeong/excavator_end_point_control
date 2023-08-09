import time
import numpy as np


movement_timer, work_timer = time.time(), time.time()
t0, movement_t0, work_t0 = 0.0, 0.0, 0.0
excavator_velocity: float = 0.0
angle_prv = np.zeros(shape=4, dtype=np.float64)  # 0: boom, 1: arm, 2: bucket, 3: swing
joystick_pos_prv = np.zeros(shape=4, dtype=np.uint8)
excavator_pos_prv = np.zeros(shape=2, dtype=np.float64)
movement_status: int = 0
work_status: int = 0

boom_angle_threshold: float = 0.1  # unit: degree
arm_angle_threshold: float = 0.1  # unit: degree
bucket_angle_threshold: float = 0.1  # unit: degree
joystick_position_threshold: int = 2  # unit: none
joystick_neutral_position: int = 75  # unit: none
excavator_velocity_threshold: float = 0.5  # unit: meter/second
heading_threshold: float = 1.0  # unit: degree


def recognition_excavator_work_status(boom_angle, arm_angle, bucket_angle, heading_angle,
                                      left_joystick_x_pos, left_joystick_y_pos, right_joystick_x_pos, right_joystick_y_pos,
                                      excavator_latitude, excavator_longitude, movement_time_threshold, work_time_threshold):
    # time threshold unit: second

    global angle_prv, joystick_pos_prv, excavator_pos_prv, movement_status, work_status, movement_timer, movement_t0, \
        work_timer, work_t0, t0, excavator_velocity

    boom_derivative = np.abs(boom_angle-angle_prv[0])
    arm_derivative = np.abs(arm_angle-angle_prv[1])
    bucket_derivative = np.abs(bucket_angle-angle_prv[2])
    heading_derivative = np.abs(heading_angle-angle_prv[3])

    excavator_latitude_derivative = np.abs(excavator_latitude-excavator_pos_prv[0])
    excavator_longitude_derivative = np.abs(excavator_longitude-excavator_pos_prv[1])
    excavator_velocity = (np.sqrt(np.power(excavator_longitude_derivative, 2)
                                  + np.power(excavator_latitude_derivative, 2)))/(time.time()-t0)

    left_joystick_x_pos_move_recognition = np.abs(left_joystick_x_pos - joystick_neutral_position) > 5  # swing recognition
    left_joystick_y_pos_move_recognition = np.abs(left_joystick_y_pos - joystick_neutral_position) > 5  # arm recognition
    right_joystick_x_pos_move_recognition = np.abs(right_joystick_x_pos - joystick_neutral_position) > 5  # boom recognition
    right_joystick_y_pos_move_recognition = np.abs(right_joystick_y_pos - joystick_neutral_position) > 5  # bucket recognition

    boom_move_recognition = np.all([boom_derivative >= boom_angle_threshold, right_joystick_y_pos_move_recognition])
    arm_move_recognition = np.all([arm_derivative >= arm_angle_threshold, left_joystick_y_pos_move_recognition])
    bucket_move_recognition = np.all([bucket_derivative >= bucket_angle_threshold, right_joystick_x_pos_move_recognition])
    swing_move_recognition = np.all([heading_derivative >= heading_threshold, left_joystick_x_pos_move_recognition])

    if np.all([excavator_velocity >= excavator_velocity_threshold, not swing_move_recognition]):  # movement recognition
        movement_status = 1  # 01: movement state
        movement_timer = 0.0
        movement_t0 = time.time()

    else:
        movement_timer = time.time() - movement_t0

        if movement_timer > movement_time_threshold:
            movement_status = 0
        else:
            movement_status = 1

    if np.any([boom_move_recognition, arm_move_recognition, bucket_move_recognition, swing_move_recognition]):  # working recognition
        work_status = 1  # 02: working state
        work_timer = 0.0
        work_t0 = time.time()

    else:
        work_timer = time.time() - work_t0

        if work_timer > work_time_threshold:
            work_status = 0
        else:
            work_status = 1

    # save previous values
    for i, angle in enumerate([boom_angle, arm_angle, bucket_angle, heading_angle]):
        angle_prv[i] = angle

    for i, joystick_pos in enumerate([left_joystick_x_pos, left_joystick_y_pos, right_joystick_x_pos, right_joystick_y_pos]):
        joystick_pos_prv[i] = joystick_pos

    for i, excavator_pos in enumerate([excavator_latitude, excavator_longitude]):
        excavator_pos_prv[i] = excavator_pos

    t0 = time.time()

    return movement_status, work_status
