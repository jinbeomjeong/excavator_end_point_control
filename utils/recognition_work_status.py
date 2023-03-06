boom_pitch_prv : int = 0
arm_pitch_prv : int = 0
bucket_pitch_prv : int = 0
left_joystick_x_pos_prv : int = 0
left_joystick_y_pos_prv : int = 0
right_joystick_x_pos_prv : int = 0
right_joystick_y_pos_prv : int = 0
gps_ch1_latitude_prv : int = 0
gps_ch1_longitude_prv : int = 0
gps_ch2_latitude_prv : int = 0
gps_ch2_longitude_prv : int = 0

def recognition_excavator_work_status(boom_pitch, arm_pitch, bucket_pitch,
                                              left_joystick_x_pos, left_joystick_y_pos,
                                              right_joystick_x_pos, right_joystick_y_pos,
                                              gps_ch1_latitude, gps_ch1_longitude,
                                              gps_ch2_latitude, gps_ch2_longitude):

    global boom_pitch_prv, arm_pitch_prv, bucket_pitch_prv, left_joystick_x_pos_prv, left_joystick_y_pos_prv, right_joystick_x_pos_prv, right_joystick_y_pos_prv
    global gps_ch1_latitude_prv, gps_ch1_longitude_prv, gps_ch2_latitude_prv, gps_ch2_longitude_prv

    excavator_work_status = 3 # 03: abnomaly state

    if(boom_pitch_prv == boom_pitch and arm_pitch_prv == arm_pitch and bucket_pitch_prv == bucket_pitch
       and left_joystick_x_pos_prv == left_joystick_x_pos and left_joystick_y_pos_prv == left_joystick_y_pos
       and right_joystick_x_pos_prv == right_joystick_x_pos and right_joystick_y_pos_prv == right_joystick_y_pos
       and right_joystick_y_pos_prv == right_joystick_y_pos and gps_ch1_latitude_prv == gps_ch1_latitude
       and gps_ch1_longitude_prv == gps_ch1_longitude and gps_ch2_latitude_prv == gps_ch2_latitude
       and gps_ch2_longitude_prv == gps_ch2_longitude ) :
        excavator_work_status = 0 # 00: stand by
    elif(gps_ch1_latitude_prv == gps_ch1_latitude and gps_ch1_longitude_prv == gps_ch1_longitude
          and gps_ch2_latitude_prv == gps_ch2_latitude and gps_ch2_longitude_prv == gps_ch2_longitude ) :
        excavator_work_status = 1 # 01: movement state
    else :
       excavator_work_status = 2 #  02: working state

    boom_pitch_prv = boom_pitch
    arm_pitch_prv = arm_pitch
    bucket_pitch_prv = bucket_pitch
    left_joystick_x_pos_prv = left_joystick_x_pos
    left_joystick_y_pos_prv = left_joystick_y_pos
    right_joystick_x_pos_prv = right_joystick_x_pos
    right_joystick_y_pos_prv = right_joystick_y_pos
    gps_ch1_latitude_prv = gps_ch1_latitude
    gps_ch1_longitude_prv = gps_ch1_longitude
    gps_ch2_latitude_prv = gps_ch2_latitude
    gps_ch2_longitude_prv = gps_ch2_longitude

    return excavator_work_status


