import can, threading, time
import numpy as np
from utils.can_msg_parser import SafetyControlMsgParser, InclinometerSensor
from utils.accessory_lib import system_info
from utils.calculate_gps_data import calculation_heading, gps_unit_change, est_gps_center
from typing import Final, List


can_channel_ic = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)
can_channel_cd = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS2', bitrate=250000)

boom_sensor = InclinometerSensor(arbitration_id=0x10FF5383)
arm_sensor = InclinometerSensor(arbitration_id=0x10FF5382)
bucket_sensor = InclinometerSensor(arbitration_id=0x10FF5381)
body_sensor = InclinometerSensor(arbitration_id=0x10FF5384)

safety_control_msg = SafetyControlMsgParser()

excavator_angle = np.zeros(shape=4, dtype=np.float64)
relative_angle = np.zeros(shape=3, dtype=np.float64)
angle_sensor_offset = np.array([268.5, 265.6, 386.9, 270], dtype=np.float64)  # offset angle of boom, arm, bucket and body
relative_angle_filtered = np.zeros(shape=3, dtype=np.float64)

excavator_pos = np.zeros(shape=3, dtype=np.float64)  # latitude, longitude, heading
gps_coords = np.zeros(shape=4, dtype=np.float64)  # gps_1_latitude, gps_1_longitude, gps_2_latitude, gps_2_longitude
gps_coords_tm = np.zeros(shape=4, dtype=np.float64) # converted from gps_coords
bucket_local_pos = np.zeros(shape=3, dtype=np.float64)  # x-axis, y-axis, z-axis


def can_com_ic_controller():
    for can_msg in can_channel_ic:
        boom_sensor.get_sensor_values(packet=can_msg)
        arm_sensor.get_sensor_values(packet=can_msg)
        bucket_sensor.get_sensor_values(packet=can_msg)
        body_sensor.get_sensor_values(packet=can_msg)


def can_com_cd_controller():
    for can_msg in can_channel_cd:
        safety_control_msg.get_cd_state(packet=can_msg)
        safety_control_msg.get_equip_pos(packet=can_msg)


can_com_ic_task = threading.Thread(target=can_com_ic_controller)
can_com_ic_task.daemon = True
can_com_ic_task.start()

can_com_cd_task = threading.Thread(target=can_com_cd_controller)
can_com_cd_task.daemon = True
can_com_cd_task.start()

system_info()

print('algorithm is running...')
start_time: Final[float] = time.time()

while True:
    elapsed_time = time.time() - start_time
    ic_state = safety_control_msg.read_variable('ic_state')
    cd_state_str = safety_control_msg.read_variable('cd_state')

    # read value of gps data
    for i, cal_pos in enumerate(safety_control_msg.read_variable('equipment_cal_pos_data')):
        gps_coords[i] = cal_pos

    # convert TM coordinate
    gps_coords_tm[1], gps_coords_tm[0] = gps_unit_change(latitude=gps_coords[0], longitude=gps_coords[1])  # TM coordinate of x pos, y pos
    gps_coords_tm[3], gps_coords_tm[2] = gps_unit_change(latitude=gps_coords[2], longitude=gps_coords[3])

    # estimation excavator center position
    excavator_pos[0], excavator_pos[1] = est_gps_center(gps_1_coord=gps_coords_tm[0:2], gps_2_coord=gps_coords_tm[2:4], offset=1.16)

    # calculation of excavator heading angle
    excavator_pos[2] = 360-(calculation_heading(gps_x_1=gps_coords_tm[1], gps_x_2=gps_coords_tm[3],
                                                gps_y_1=gps_coords_tm[0], gps_y_2=gps_coords_tm[2]))

    # read value and execution offset and execution low pass filter of angle sensor
    for i, angle_sensor_name in enumerate([boom_sensor, arm_sensor, bucket_sensor]):
        excavator_angle[i] = angle_sensor_name.read_sensor_value('y_axis')

    excavator_angle[3] = body_sensor.read_sensor_value('x_axis')

    for i, angle in enumerate(excavator_angle):
        excavator_angle[i] = np.subtract(angle, angle_sensor_offset[i])

    excavator_angle[3] = np.negative(excavator_angle[3])

    # calculate relative angle
    relative_angle[0] = excavator_angle[0] - excavator_angle[3]
    relative_angle[1] = excavator_angle[1] - relative_angle[0]
    relative_angle[2] = excavator_angle[2] - relative_angle[0] - relative_angle[1]

    # calculate low pass filter
    for i in range(len(excavator_angle[0:3])):
        relative_angle_filtered[i] = np.add(np.multiply(0.5, relative_angle_filtered[i]),
                                            np.multiply(0.5, relative_angle[i]))

    time.sleep(0.1)
