import time, threading, can, argparse
import numpy as np
import pandas as pd
from utils.accessory_lib import system_info
from utils.predcit_api import predict_control_linear
from utils.calculate_gps_data import calculation_heading, gps_unit_change, est_gps_center
from utils.end_point_kinematics import end_point_kinematics, link_calculator
from utils.recognition_work_status import recognition_excavator_work_status as work_status
from utils.warning_target import get_global_bucket_pos, get_safety_status, get_target_boundary, TargetDepth
from utils.can_msg_parser import SafetyControlMsgParser, InclinometerSensor, JoystickMsgParser
from utils.tcp_lib import TCPClient
from typing import Final, List


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
msg_set_ic = []
msg_get_cd = []
control_state: int = 0
target_pipe_depth: float = 0.0
t0_1: float = 0.0
sc_state: int = 1
safety_state: int = 0
excavator_work_state: int = 0
cd_query = 'none'
receive_state = True
done_state = True
pipe_boundary_coord: np.array([])
pipe_coords_tm = np.array([])
pipe_depth = np.zeros(0, dtype=np.double)
gps_coords = np.zeros(4, dtype=np.double)  # gps_1_latitude, gps_1_longitude, gps_2_latitude, gps_2_longitude
gps_coords_tm = np.zeros(4, dtype=np.double)  # tm coordinates for gps coordinates
bucket_local_pos = np.zeros(3, dtype=np.double)  # x-axis, y-axis, z-axis
bucket_global_pos = np.zeros(2, dtype=np.double)  # latitude, longitude

# left_x_pos, left_y_pos, right_x_pos, right_y_pos
# left_x_pos -> swing, left_y_pos -> arm, right_x_pos -> bucket, right_y -> boom
joystick_pos = np.zeros(4, dtype=np.uint8)

excavator_angle = np.zeros(4, dtype=np.double)  # boom, arm and bucket angle
pred_angle = np.zeros(4, dtype=np.double)  # predict of boom, arm and bucket angle
angle_sensor_offset = np.array([268.5, 265.6, 270, 0], dtype=np.double)  # offset angle of boom, arm, bucket and body
angle_sensor_offset.flags.writeable = False

excavator_pos = np.zeros(3, dtype=np.double)  # latitude, longitude, heading
est_pipe_depth = TargetDepth(target_pos=np.array([]), target_depth=np.array([]))

logging_data: pd.DataFrame = pd.DataFrame()
elapsed_time: float = 0.0

# excavator_tm_latitude, excavator_tm_longitude, heading
# bucket_goal_x_pos, bucket_goal_y_pos, bucket_z_pos, target_depth
tcp_msg = np.zeros(13, dtype=np.double)


def can_com_ic_controller():
    for can_msg in can_channel_ic:
        boom_sensor.get_sensor_values(packet=can_msg)
        arm_sensor.get_sensor_values(packet=can_msg)
        bucket_sensor.get_sensor_values(packet=can_msg)
        body_sensor.get_sensor_values(packet=can_msg)
        joystick_msg.get_joystick_values(packet=can_msg)
        safety_control_msg.get_ic_state(packet=can_msg)


def can_com_cd_controller():
    global cd_query, pipe_depth, est_pipe_depth, receive_state, done_state, pipe_boundary_coord, pipe_coords_tm
    for can_msg in can_channel_cd:
        safety_control_msg.get_cd_state(packet=can_msg)
        safety_control_msg.get_equip_pos(packet=can_msg)
        safety_control_msg.get_under_utils_query(packet=can_msg)
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
                pipe_coords_tm = np.zeros((pipe_coords.shape[0], 2), dtype=np.double)

                for i, pipe_coord in enumerate(pipe_coords):  # convert TM coordinate of pipe coordinate
                    pipe_coords_tm[i, 0], pipe_coords_tm[i, 1] = gps_unit_change(latitude=pipe_coord[0], longitude=pipe_coord[1])

                pipe_boundary_coord = get_target_boundary(pipe_coords_tm, radius=0.1, step=10)
                pipe_depth = safety_control_msg.read_variable('under_utils_depth_data')
                est_pipe_depth = TargetDepth(target_pos=pipe_coords_tm, target_depth=pipe_depth)
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
        tcp_thread = threading.Timer(interval=0.1, function=self.send_msg, args=())
        tcp_thread.daemon = True
        tcp_thread.start()


class Logging:
    def __init__(self, file_name: str = ''):
        self.logging_header = pd.DataFrame(columns=['time(sec)', 'boom_angle', 'arm_angle', 'bucket_angle', 'body_angle',
                                                    'joystick_left_x_pos', 'joystick_left_y_pos',
                                                    'joystick_right_x_pos', 'joystick_right_y_pos',
                                                    'excavator_global_x_pos', 'excavator_global_y_pos',
                                                    'excavator_heading', 'bucket_goal_x_pos', 'bucket_goal_y_pos',
                                                    'bucket_depth_pos', 'excavator_work_state', 'control_state',
                                                    'target_pipe_depth', 'gps_1_latitude', 'gps_1_longitude',
                                                    'gps_2_latitude', 'gps_2_longitude', 'gps_1_tm_latitude',
                                                    'gps_1_tm_longitude', 'gps_2_tm_latitude', 'gps_2_tm_longitude'])

        self.start_time_str = time.strftime('%Y%m%d-%H%M%S', time.localtime(time.time()))
        self.logging_file_name = file_name + '_' + self.start_time_str
        self.logging_file_path = './logging_data/' + self.logging_file_name + '.csv'
        self.logging_header.to_csv(self.logging_file_path, mode='a', header=True)

    def start_logging(self, period=0.1):
        global logging_data
        logging_data = pd.DataFrame({'1': round(elapsed_time, 2), '2': round(pred_angle[0], 2),
                                     '3': round(pred_angle[1], 2), '4': round(pred_angle[2], 2),
                                     '5': round(pred_angle[3], 2), '6': joystick_pos[0], '7': joystick_pos[1],
                                     '8': joystick_pos[2], '9': joystick_pos[3], '10': excavator_pos[0],
                                     '11': excavator_pos[1], '12': excavator_pos[2], '13': bucket_global_pos[0],
                                     '14': bucket_global_pos[1], '15': bucket_local_pos[2], '16': excavator_work_state,
                                     '17': control_state, '18': target_pipe_depth,
                                     '19': gps_coords[0], '20': gps_coords[1], '21': gps_coords[2], '22': gps_coords[3],
                                     '23': gps_coords_tm[0], '24': gps_coords_tm[1], '25': gps_coords_tm[2],
                                     '26': gps_coords_tm[3]}, index=[0])

        logging_data.to_csv(self.logging_file_path, mode='a', header=False)
        logging_thread = threading.Timer(period, self.start_logging, (period,))
        logging_thread.daemon = True
        logging_thread.start()


can_com_ic_task = threading.Thread(target=can_com_ic_controller)
can_com_ic_task.daemon = True
can_com_ic_task.start()

can_com_cd_task = threading.Thread(target=can_com_cd_controller)
can_com_cd_task.daemon = True
can_com_cd_task.start()

if len(args.log_name) > 0:
    logging_task = Logging(file_name=args.log_name)
    logging_task.start_logging(period=0.1)

if len(args.ip) > 0:
    tcp_task = TcpCom(ip=args.ip)
    tcp_task.send_msg()

# set stand-by state change of safety controller
system_info()

'''
# waiting for integrated controller
while True:
    if safety_control_msg.read_variable('ic_state') == 'on': break
    time.sleep(0.1)
'''

# create interpolated pipe data
start_time: Final[float] = time.time()
print('algorithm is running...')

while True:
    elapsed_time = time.time() - start_time

    # read value of gps data
    for i, cal_pos in enumerate(safety_control_msg.read_variable('equipment_cal_pos_data')):
        gps_coords[i] = cal_pos

    # convert TM coordinate
    gps_coords_tm[0], gps_coords_tm[1] = gps_unit_change(latitude=gps_coords[0], longitude=gps_coords[1])  # TM coordinate of x pos, y pos
    gps_coords_tm[2], gps_coords_tm[3] = gps_unit_change(latitude=gps_coords[2], longitude=gps_coords[3])

    excavator_pos[0], excavator_pos[1] = est_gps_center(gps_1_coord=gps_coords_tm[0:2], gps_2_coord=gps_coords_tm[2:4], offset=1.16)
    excavator_pos[2] = calculation_heading(gps_x_1=gps_coords_tm[1], gps_x_2=gps_coords_tm[3],
                                           gps_y_1=gps_coords_tm[0], gps_y_2=gps_coords_tm[2])

    # print(excavator_pos)
    # read value and execution offset and execution low pass filter of angle sensor
    for i, angle_sensor_name in enumerate([boom_sensor, arm_sensor, bucket_sensor, body_sensor]):
        excavator_angle[i] = np.subtract(angle_sensor_name.read_sensor_value('y_axis'), angle_sensor_offset[i])
        pred_angle[i] = np.add(np.multiply(0.5, pred_angle[i]), np.multiply(0.5, excavator_angle[i]))

    # read value of joystick position
    for i, var_name in enumerate(['left_x_position', 'left_y_position', 'right_x_position', 'right_y_position']):
        joystick_pos[i] = joystick_msg.read_joystick_value(var_name)

    # recognition of excavator state
    excavator_work_state = work_status(boom_pitch=pred_angle[0], arm_pitch=pred_angle[1], bucket_pitch=pred_angle[2],
                                       left_joystick_x_pos=joystick_pos[0], left_joystick_y_pos=joystick_pos[1],
                                       right_joystick_x_pos=joystick_pos[2], right_joystick_y_pos=joystick_pos[3],
                                       gps_ch1_latitude=gps_coords_tm[0], gps_ch1_longitude=gps_coords_tm[1],
                                       gps_ch2_latitude=gps_coords_tm[2], gps_ch2_longitude=gps_coords_tm[3])

    # estimation of bucket position
    bucket_local_pos[0], bucket_local_pos[2], arm_relative_angle, bucket_relative_angle = \
        end_point_kinematics(pred_angle[0], pred_angle[1], pred_angle[2])

    bucket_global_pos[0], bucket_global_pos[1] = get_global_bucket_pos(origin=[excavator_pos[0], excavator_pos[1]],
                                                                       y_distance=bucket_local_pos[0], heading=excavator_pos[2])
    # recognition to safety status of bucket position
    if pipe_coords_tm.shape[0] > 0:
        bucket_safety_status = get_safety_status(end_point=bucket_global_pos, target=pipe_coords_tm)
    else:
        bucket_safety_status = 0

    # get target pipe depth
    if safety_control_msg.read_variable('work_limit') == 'de-active':
        target_pipe_depth = 0
    else:
        target_pipe_depth = est_pipe_depth.get_target_depth(current_pos=bucket_global_pos)

    # safety control of bucket position
    if control_state == 0:
        control_state, predict_signal = predict_control_linear(input_signal=bucket_local_pos[2], time_offset=0.7,
                                                               threshold_signal=target_pipe_depth)

        t0_1 = time.time()

    if control_state == 1:
        el_time = np.subtract(time.time(), t0_1)

        if np.greater_equal(bucket_local_pos[2], np.multiply(target_pipe_depth, 1.5)) and np.greater(el_time, 2.0):
            control_state = 0
        else:
            control_state = 1

    if bucket_safety_status:
        safety_state = 1
    elif control_state:
        safety_state = 2
    else:
        safety_state = 0

    # create message of safety controller
    '''
    msg_set_ic.append(safety_control_msg.create_sc_state_msg(sc_state=2, safety_state=safety_state,
                                                             excavator_state=excavator_work_state))
    '''
    msg_set_ic.append(safety_control_msg.create_equip_origin_pos_msg(latitude=excavator_pos[0], longitude=excavator_pos[1]))
    msg_set_ic.append(safety_control_msg.create_equip_heading_msg(heading=excavator_pos[2]))
    msg_set_ic.append(safety_control_msg.create_comp_angle_msg(boom_angle=np.abs(pred_angle[0]),
                                                               arm_angle=np.abs(np.subtract(pred_angle[0], pred_angle[1])),
                                                               bucket_angle=np.abs(np.subtract(pred_angle[1], pred_angle[2]))))

    for message in msg_set_ic:
        can_channel_ic.send(message)

    msg_set_ic = []

    for i, data in enumerate([excavator_pos[0], excavator_pos[1], excavator_pos[2],
                              bucket_global_pos[0], bucket_global_pos[1], target_pipe_depth, bucket_local_pos[0],
                              bucket_local_pos[2], pred_angle[0], pred_angle[1], pred_angle[2],
                              safety_state, excavator_work_state]):
        tcp_msg[i] = data

    print(pred_angle)
    print(bucket_local_pos)
    time.sleep(0.1)
