import time, threading, can, argparse
import numpy as np
import pandas as pd
from pyproj import Transformer
from utils.accessory_lib import system_info
from utils.predcit_api import predict_control_linear
from utils.calculate_gps_data import calculation_heading, gps_unit_change, est_gps_center
from utils.end_point_kinematics import end_point_kinematics, convert_bucket_angle, bucket_link_calculator
from utils.recognition_work_status import recognition_excavator_work_status as work_status
from utils.warning_target import (get_bucket_global_pos, interp_2d_target, interp_1d_target, est_pipe_depth_euclidean,
                                  get_target_boundary, recognition_in_range_boundary)
from utils.can_msg_parser import SafetyControlMsgParser, InclinometerSensor, JoystickMsgParser
from utils.tcp_lib import TCPClient
from typing import Final

parser = argparse.ArgumentParser()
parser.add_argument("--ip", type=str, default='')
parser.add_argument("--log_name", type=str, default='logging_data')
args = parser.parse_args()

can_channel_ic = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)
can_channel_cd = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS2', bitrate=250000)

boom_sensor = InclinometerSensor(arbitration_id=0x10FF5383)
arm_sensor = InclinometerSensor(arbitration_id=0x10FF5382)
bucket_sensor = InclinometerSensor(arbitration_id=0x10FF5381)
body_sensor = InclinometerSensor(arbitration_id=0x10FF5384)

joystick_msg = JoystickMsgParser()
safety_control_msg = SafetyControlMsgParser()

# initialize internal variable
msg_set_ic, msg_set_cd = [], []
control_state, safety_state, bucket_in_pipe, pipe_rela_dist = 0, 0, 0, 0
t0_1: float = 0.0
ic_state, cd_state, sc_state = 0, 0, 0
target_pipe_depth: int = -9999
excavator_work_state = np.zeros(shape=8, dtype=np.uint8)  # 1byte position: movement, 2byte position: work status
cd_query = 'none'
receive_state, done_state = True, True

pipe_coords, pipe_coords_tm = np.array([]), np.array([])
pipe_coords_tm_interp = np.array([])
pipe_boundary_coords_tm = np.array([])
pipe_safety_boundary_coords_tm = np.array([])
pipe_depth, pipe_diameter = np.array([]), np.array([])
pipe_depth_interp = np.array([])

gps_coords = np.zeros(shape=4, dtype=np.float64)  # gps_1_latitude, gps_1_longitude, gps_2_latitude, gps_2_longitude
gps_coords_tm = np.zeros(shape=4, dtype=np.float64)  # converted from gps_coords
bucket_local_pos = np.zeros(shape=3, dtype=np.float64)  # x-axis, y-axis, z-axis
bucket_global_pos = np.zeros(shape=2, dtype=np.float64)  # latitude, longitude

# left_x_pos, left_y_pos, right_x_pos, right_y_pos
# left_x_pos -> swing, left_y_pos -> arm, right_x_pos -> bucket, right_y -> boom
joystick_pos = np.zeros(shape=4, dtype=np.uint8)
excavator_angle = np.zeros(shape=4, dtype=np.float64)
comp_angle = np.zeros(shape=3, dtype=np.float64)  # boom, arm and bucket angle

angle_sensor_offset = np.array([267.9, 265.4, -87.9, 270.0], dtype=np.float64)  # offset angle of boom, arm, bucket and body
angle_sensor_offset.flags.writeable = False

relative_angle = np.zeros(shape=3, dtype=np.float64)  # relative angle of boom, arm, bucket
relative_angle_filtered = np.zeros(shape=3, dtype=np.float64)
absolute_angle_filtered = np.zeros(shape=3, dtype=np.float64)
excavator_pos = np.zeros(shape=3, dtype=np.float64)  # latitude, longitude, heading

elapsed_time: float = 0.0

# excavator_tm_latitude, excavator_tm_longitude, heading
# bucket_goal_x_pos, bucket_goal_y_pos, bucket_z_pos, target_depth
tcp_msg = np.zeros(shape=14, dtype=np.float64)

logging_data = pd.DataFrame()
logging_header = pd.DataFrame(columns=['time(sec)', 'boom_angle(deg)', 'arm_angle(deg)', 'bucket_angle(deg)',
                                       'body_angle(deg)', 'relative_boom_angle(deg)', 'relative_arm_angle(deg)',
                                       'relative_bucket_angle(deg)', 'joystick_left_x_pos', 'joystick_left_y_pos',
                                       'joystick_right_x_pos', 'joystick_right_y_pos',
                                       'excavator_global_x_pos(m)', 'excavator_global_y_pos(m)',
                                       'excavator_heading(deg)', 'bucket_goal_x_pos(m)', 'bucket_goal_y_pos(m)',
                                       'bucket_depth_pos(mm)', 'excavator_work_state', 'control_state',
                                       'target_pipe_depth(mm)', 'gps_1_latitude', 'gps_1_longitude',
                                       'gps_2_latitude', 'gps_2_longitude', 'gps_1_tm_latitude(m)', 'gps_1_tm_longitude(m)',
                                       'gps_2_tm_latitude(m)', 'gps_2_tm_longitude(m)'])

start_time_str = time.strftime('%Y%m%d-%H%M%S', time.localtime(time.time()))
logging_file_name = args.log_name + '_' + start_time_str
logging_file_path = './logging_data/' + logging_file_name + '.csv'
logging_header.to_csv(logging_file_path, mode='a', header=True)


def can_com_ic_controller():
    for can_msg in can_channel_ic:
        boom_sensor.get_sensor_values(packet=can_msg)
        arm_sensor.get_sensor_values(packet=can_msg)
        bucket_sensor.get_sensor_values(packet=can_msg)
        body_sensor.get_sensor_values(packet=can_msg)
        joystick_msg.get_joystick_values(packet=can_msg)
        safety_control_msg.get_ic_state(packet=can_msg)


def can_com_cd_controller():
    global cd_query, receive_state, done_state, pipe_coords, pipe_depth, pipe_coords_tm, pipe_coords_tm_interp, \
        pipe_depth_interp, pipe_diameter, pipe_boundary_coords_tm, pipe_safety_boundary_coords_tm

    for can_msg in can_channel_cd:
        safety_control_msg.get_cd_state(packet=can_msg)
        safety_control_msg.get_equip_pos(packet=can_msg)
        safety_control_msg.get_under_utils_query(packet=can_msg)
        safety_control_msg.get_work_area_information(packet=can_msg)
        cd_query = safety_control_msg.read_variable('cd_query')

        if cd_query == 'ready_to_receive':
            if receive_state:
                can_channel_cd.send(safety_control_msg.create_under_utils_query_msg(request_msg=2))
                print(safety_control_msg.read_variable('cd_query'))
                receive_state = False
                done_state = True

        elif cd_query == 'transmitting':
            print(cd_query)
            safety_control_msg.get_under_utils_data(packet=can_msg)

        elif cd_query == 'done':
            if done_state:
                pipe_coords = safety_control_msg.read_variable('under_utils_coord_data')  # pipe pos of latitude, longitude
                pipe_property = safety_control_msg.read_variable('under_utils_depth_data')  # pipe depth and diameter
                pipe_coords_tm = np.zeros((pipe_coords.shape[0], 2), dtype=np.double)
                pipe_depth, pipe_diameter = pipe_property[:, 0], pipe_property[:, 1]

                for i, pipe_coord in enumerate(pipe_coords):  # convert TM coordinate of pipe coordinate
                    pipe_coords_tm[i, 1], pipe_coords_tm[i, 0] = gps_unit_change(latitude=pipe_coord[0], longitude=pipe_coord[1])

                pipe_coords_tm_interp = interp_2d_target(pipe_coords_tm, step=10)
                pipe_depth_interp = interp_1d_target(pipe_depth, step=10)
                pipe_boundary_coords_tm = get_target_boundary(pipe_coords_tm, radius=pipe_diameter/2, step=10)
                safety_length = np.ones(shape=len(pipe_diameter))+(pipe_diameter/2)
                pipe_safety_boundary_coords_tm = get_target_boundary(pipe_coords_tm, radius=safety_length, step=10)
                print(safety_control_msg.read_variable('cd_query'))
                receive_state = True
                done_state = False


class TcpCom:
    def __init__(self, ip: str = ''):
        if len(args.ip) > 0:
            self.tcp_handle = TCPClient(address=ip, port=6340)
            self.tcp_handle.connect_to_server()

    def send_msg(self):
        self.tcp_handle.send_msg(np.array2string(tcp_msg.flatten('C'), precision=10, separator=',').lstrip('[').rstrip(']'))
        tcp_thread = threading.Timer(interval=0.05, function=self.send_msg, args=())
        tcp_thread.daemon = True
        tcp_thread.start()


# start thread for can communication ic controller
can_com_ic_task = threading.Thread(target=can_com_ic_controller)
can_com_ic_task.daemon = True
can_com_ic_task.start()

# start thread for can communication cd_controller
can_com_cd_task = threading.Thread(target=can_com_cd_controller)
can_com_cd_task.daemon = True
can_com_cd_task.start()

if len(args.ip) > 0:
    tcp_task = TcpCom(ip=args.ip)
    tcp_task.send_msg()


class SensorFilter:
    def __init__(self, kernel_size: int):
        self.__buffer = np.zeros(kernel_size)

    def point_by_median(self, input_value: float) -> float:
        self.__buffer = np.roll(self.__buffer, shift=1)
        self.__buffer[0] = input_value

        return np.median(self.__buffer)


boom_filter = SensorFilter(kernel_size=9)
arm_filter = SensorFilter(kernel_size=9)
bucket_filter = SensorFilter(kernel_size=9)
bucket_filter_2 = SensorFilter(kernel_size=5)

# convert gps coordinate
converter_grs80_to_wgs84 = Transformer.from_crs("EPSG:5186", "EPSG:4326")
converter_wgs84_to_grs80 = Transformer.from_crs("EPSG:4326", "EPSG:5186")

system_info()

# set stand-by state change of safety controller
'''
# waiting for integrated controller
while True:
    if safety_control_msg.read_variable('ic_state') == 'on': break
    time.sleep(0.1)
'''

start_time: Final[float] = time.time()
print('algorithm is running...')

while True:
    elapsed_time = time.time() - start_time
    ic_state = safety_control_msg.read_variable('ic_state')
    cd_state_str = safety_control_msg.read_variable('cd_state')

    # read value of gps data
    for i, cal_pos in enumerate(safety_control_msg.read_variable('equipment_cal_pos_data')):
        gps_coords[i] = cal_pos

    # convert TM coordinate
    gps_coords_tm[0], gps_coords_tm[1] = converter_wgs84_to_grs80.transform(xx=gps_coords[0], yy=gps_coords[1]) # TM coordinate of x pos, y pos
    gps_coords[2], gps_coords[3] = converter_wgs84_to_grs80.transform(xx=gps_coords[2], yy=gps_coords[3])

    # estimation excavator center position
    excavator_pos[0], excavator_pos[1] = est_gps_center(gps_1_coord=gps_coords_tm[0:2], gps_2_coord=gps_coords_tm[2:4], offset=1.16)

    # calculation of excavator heading angle
    excavator_pos[2] = 360-(calculation_heading(gps_x_1=gps_coords_tm[1], gps_x_2=gps_coords_tm[3],
                                                gps_y_1=gps_coords_tm[0], gps_y_2=gps_coords_tm[2]))

    # read value of angle sensor
    for i, angle_sensor_name in enumerate([boom_sensor, arm_sensor, bucket_sensor]):
        excavator_angle[i] = angle_sensor_name.read_sensor_value('y_axis')

    excavator_angle[3] = body_sensor.read_sensor_value('x_axis')

    # execute suppression bucket angle
    excavator_angle[2] = bucket_filter_2.point_by_median(input_value=excavator_angle[2])

    # offset of body angle value
    excavator_angle[3] = excavator_angle[3] - angle_sensor_offset[3]
    excavator_angle[3] = np.negative(excavator_angle[3])

    # calculate relative angle
    relative_angle[0] = excavator_angle[0] - angle_sensor_offset[0] - excavator_angle[3]
    relative_angle[1] = excavator_angle[1] - angle_sensor_offset[1] - relative_angle[0]
    relative_angle[2] = excavator_angle[2] - relative_angle[0] - relative_angle[1]
    relative_angle[2] = convert_bucket_angle(relative_angle[2]) - angle_sensor_offset[2]

    # calculate bucket 4 link angle
    relative_angle[2] = bucket_link_calculator(relative_angle[2])

    # calculate low pass filter for boom and arm angle
    for i in range(len(excavator_angle[0:3])):
        relative_angle_filtered[i] = np.add(np.multiply(0.5, relative_angle_filtered[i]),
                                            np.multiply(0.5, relative_angle[i]))

    # calculate absolute angle
    absolute_angle_filtered[0] = relative_angle_filtered[0] + excavator_angle[3]
    absolute_angle_filtered[1] = relative_angle_filtered[0] + relative_angle_filtered[1]
    absolute_angle_filtered[2] = relative_angle_filtered[0] + relative_angle_filtered[1] + relative_angle_filtered[2]

    # calculate median filter
    for i, sensor_filter in enumerate([boom_filter, arm_filter, bucket_filter]):
        comp_angle[i] = sensor_filter.point_by_median(input_value=relative_angle[i])

    # read value of joystick position
    for i, var_name in enumerate(['left_x_position', 'left_y_position', 'right_x_position', 'right_y_position']):
        joystick_pos[i] = joystick_msg.read_joystick_value(var_name)

    # recognition of excavator state
    excavator_work_state[7], excavator_work_state[6] = work_status(boom_angle=comp_angle[0], arm_angle=comp_angle[1],
                                                                   bucket_angle=comp_angle[2], heading_angle=excavator_pos[2],
                                                                   left_joystick_x_pos=joystick_pos[0],
                                                                   left_joystick_y_pos=joystick_pos[1],
                                                                   right_joystick_x_pos=joystick_pos[2],
                                                                   right_joystick_y_pos=joystick_pos[3],
                                                                   excavator_latitude=excavator_pos[0],
                                                                   excavator_longitude=excavator_pos[1],
                                                                   movement_time_threshold=5,
                                                                   work_time_threshold=3)

    # estimation of bucket position
    bucket_local_pos[1], bucket_local_pos[2] = end_point_kinematics(boom_angle=absolute_angle_filtered[0],
                                                                    arm_angle=absolute_angle_filtered[1],
                                                                    bucket_angle=absolute_angle_filtered[2])
    bucket_global_pos[0], bucket_global_pos[1] = get_bucket_global_pos(origin=excavator_pos[0:2],
                                                                       y_distance=bucket_local_pos[1],
                                                                       heading=excavator_pos[2])

    if pipe_coords_tm.shape[0] > 0:
        bucket_in_pipe = recognition_in_range_boundary(input_pos=bucket_global_pos,
                                                       target_pos_max=pipe_safety_boundary_coords_tm[0],
                                                       target_pos_min=pipe_safety_boundary_coords_tm[1])
    else:
        bucket_in_pipe = 0

    work_limit_state = safety_control_msg.read_variable('work_limit')  # enable work_limit_state = 'de-active'

    # get target pipe depth
    if bucket_in_pipe:
        target_pipe_depth = est_pipe_depth_euclidean(target_position=bucket_global_pos, pipe_position=pipe_coords_tm,
                                                     pipe_depth=np.negative(pipe_depth_interp), safety_length=0.1)
        pipe_rela_dist = np.subtract(target_pipe_depth, bucket_local_pos[2])
        pipe_rela_dist = np.abs(pipe_rela_dist)
        pipe_rela_dist = int(pipe_rela_dist / 10)

    else:
        target_pipe_depth = -9999  # normal excavating depth
        pipe_rela_dist = 9999  # invalid pipe relative distance

    if work_limit_state == 'active':
        target_pipe_depth = 100  # ground limit excavating depth(mm)

    # safety control of bucket position
    if control_state == 0:
        control_state, predict_signal = predict_control_linear(input_signal=bucket_local_pos[2], time_offset=1.0,
                                                               threshold_signal=target_pipe_depth, release_signal=500)

        t0_1 = time.time()

    if control_state == 1:
        el_time = np.subtract(time.time(), t0_1)

        if np.greater_equal(bucket_local_pos[2], target_pipe_depth+500) and np.greater(el_time, 2.0):
            control_state = 0
        else:
            control_state = 1

    rela_dist_state = 0

    if pipe_rela_dist >= 0:
        rela_dist_state = 0
    else:
        rela_dist_state = 1

    safety_state = 0
    if bucket_in_pipe:
        safety_state = 1
    if control_state:
        safety_state = 2

    # create and send message of safety controller
    sc_state_msg = safety_control_msg.create_sc_state_msg(sc_state=2, safety_state=safety_state,
                                                          excavator_state=np.packbits(excavator_work_state)[0])
    msg_set_ic.append(sc_state_msg)
    msg_set_ic.append(safety_control_msg.create_equip_origin_pos_msg(latitude=excavator_pos[0], longitude=excavator_pos[1]))
    msg_set_ic.append(safety_control_msg.create_equip_heading_msg(heading=excavator_pos[2]))
    msg_set_ic.append(safety_control_msg.create_comp_angle_msg(boom_angle=relative_angle[0], arm_angle=relative_angle[1],
                                                               bucket_angle=relative_angle[2]))
    msg_set_ic.append(safety_control_msg.create_target_relative_distance_msg(state=rela_dist_state, distance=pipe_rela_dist))
    msg_set_ic.append(safety_control_msg.create_cd_state_msg(state=cd_state,
                                                             work_limit=1 if work_limit_state == 'active' else 0))
    msg_set_ic.append(safety_control_msg.create_work_area_information_msg(
        area_number=safety_control_msg.read_variable('work_area_number'),
        admin_phone_number=safety_control_msg.read_variable('admin_phone_number')))

    for message_ic in msg_set_ic:
        can_channel_ic.send(message_ic)

    msg_set_ic = []

    # create and send message of communication device
    msg_set_cd.append(sc_state_msg)
    msg_set_cd.append(safety_control_msg.create_end_point_position_msg(x_axis=bucket_global_pos[0],
                                                                       y_axis=bucket_global_pos[1]))

    for message_cd in msg_set_cd:
        can_channel_cd.send(message_cd)

    msg_set_cd = []

    for i, data in enumerate([excavator_pos[0], excavator_pos[1], excavator_pos[2],
                              bucket_global_pos[0], bucket_global_pos[1], target_pipe_depth, bucket_local_pos[1],
                              bucket_local_pos[2], relative_angle_filtered[0], relative_angle_filtered[1],
                              relative_angle_filtered[2], safety_state, np.packbits(excavator_work_state)[0], bucket_in_pipe]):
        tcp_msg[i] = data

    logging_data = pd.DataFrame({'1': round(elapsed_time, 2), '2': round(excavator_angle[0], 2),
                                 '3': round(excavator_angle[1], 2), '4': round(excavator_angle[2], 2),
                                 '5': round(excavator_angle[3], 2), '6': round(relative_angle[0], 2),
                                 '7': round(relative_angle[1], 2), '8': round(relative_angle[2], 2),
                                 '9': joystick_pos[0], '10': joystick_pos[1],
                                 '11': joystick_pos[2], '12': joystick_pos[3], '13': round(excavator_pos[0], 3),
                                 '14': round(excavator_pos[1], 3), '15': round(excavator_pos[2], 1),
                                 '16': round(bucket_global_pos[0], 3), '17': round(bucket_global_pos[1], 3),
                                 '18': int(bucket_local_pos[2]), '19': np.packbits(excavator_work_state)[0],
                                 '20': control_state, '21': target_pipe_depth, '22': gps_coords[0], '23': gps_coords[1],
                                 '24': gps_coords[2], '25': gps_coords[3], '26': gps_coords_tm[0],
                                 '27': gps_coords_tm[1], '28': gps_coords_tm[2], '29': gps_coords_tm[3]}, index=[0])

    logging_data.to_csv(logging_file_path, mode='a', header=False)
    time.sleep(0.1)
