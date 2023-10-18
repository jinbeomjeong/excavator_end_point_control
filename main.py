import time, threading, can, argparse
import numpy as np
import pandas as pd
from utils.accessory_lib import system_info
from utils.predcit_api import predict_control_linear
from utils.end_point_kinematics import end_point_kinematics, convert_bucket_angle, bucket_link_calculator
from utils.can_msg_parser import SafetyControlMsgParser, InclinometerSensor, JoystickMsgParser
from utils.tcp_lib import TCPClient
from typing import Final

np.set_printoptions(precision=9)
parser = argparse.ArgumentParser()
parser.add_argument("--ip", type=str, default='')
parser.add_argument("--log_name", type=str, default='logging_data')
args = parser.parse_args()

can_channel_ic = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)

boom_sensor = InclinometerSensor(arbitration_id=0x10FF5383)
arm_sensor = InclinometerSensor(arbitration_id=0x10FF5382)
bucket_sensor = InclinometerSensor(arbitration_id=0x10FF5381)
body_sensor = InclinometerSensor(arbitration_id=0x10FF5384)

joystick_msg = JoystickMsgParser()
safety_control_msg = SafetyControlMsgParser()

# initialize internal variable
control_state = 0
t0_1: float = 0.0
ic_state, sc_state = 0, 0
target_pipe_depth: int = -9999

bucket_local_pos = np.zeros(shape=3, dtype=np.float64)  # x-axis, y-axis, z-axis
bucket_global_pos = np.zeros(shape=2, dtype=np.float64)  # latitude, longitude

# left_x_pos, left_y_pos, right_x_pos, right_y_pos
# left_x_pos -> swing, left_y_pos -> arm, right_x_pos -> bucket, right_y -> boom
joystick_pos = np.zeros(shape=4, dtype=np.uint8)
excavator_angle = np.zeros(shape=4, dtype=np.float64)  # 0: boom angle, 1: body angle, 2: bucket angle, 3: body angle
relative_angle = np.zeros(shape=3, dtype=np.float64)  # relative angle of boom, arm, bucket
relative_angle_filtered = np.zeros(shape=3, dtype=np.float64)

angle_sensor_offset = np.array([267.9, 265.4, -87.9, 270.0], dtype=np.float64)  # offset angle of boom, arm, bucket and body
angle_sensor_offset.flags.writeable = False

elapsed_time: float = 0.0

# excavator_tm_latitude, excavator_tm_longitude, heading
# bucket_goal_x_pos, bucket_goal_y_pos, bucket_z_pos, target_depth
tcp_msg = np.zeros(shape=14, dtype=np.float64)

logging_data = pd.DataFrame()
logging_header = pd.DataFrame(columns=['time(sec)', 'boom_angle(deg)', 'arm_angle(deg)', 'bucket_angle(deg)',
                                       'body_angle(deg)', 'relative_boom_angle(deg)', 'relative_arm_angle(deg)',
                                       'relative_bucket_angle(deg)', 'joystick_left_x_pos', 'joystick_left_y_pos',
                                       'joystick_right_x_pos', 'joystick_right_y_pos', 'bucket_y_axis_pos(mm)',
                                       'bucket_z_axis_pos(mm)', 'target_pipe_depth(mm)', 'control_state'])

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

if len(args.ip) > 0:
    tcp_task = TcpCom(ip=args.ip)
    tcp_task.send_msg()


system_info()

start_time: Final[float] = time.time()
print('algorithm is running...')

while True:
    elapsed_time = time.time() - start_time
    ic_state = safety_control_msg.read_variable('ic_state')

    # read value of angle sensor
    for i, angle_sensor_name in enumerate([boom_sensor, arm_sensor, bucket_sensor]):
        excavator_angle[i] = angle_sensor_name.read_sensor_value('y_axis')

    excavator_angle[3] = body_sensor.read_sensor_value('x_axis')

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

    # read value of joystick position
    for i, var_name in enumerate(['left_x_position', 'left_y_position', 'right_x_position', 'right_y_position']):
        joystick_pos[i] = joystick_msg.read_joystick_value(var_name)

    # estimation of bucket position
    bucket_local_pos[1], bucket_local_pos[2] = end_point_kinematics(body_rl_angle=excavator_angle[3],
                                                                    boom_rl_angle=relative_angle_filtered[0],
                                                                    arm_rl_angle=relative_angle_filtered[1],
                                                                    bucket_rl_angle=relative_angle_filtered[2])

    work_limit_state = safety_control_msg.read_variable('work_limit')  # enable work_limit_state = 'de-active'

    # safety control of bucket position
    if control_state == 0:
        control_state, predict_signal = predict_control_linear(input_signal=bucket_local_pos[2].item(), time_offset=1.0,
                                                               threshold_signal=target_pipe_depth, release_signal=500)

        t0_1 = time.time()

    if control_state == 1:
        el_time = np.subtract(time.time(), t0_1)

        if np.greater_equal(bucket_local_pos[2], target_pipe_depth+1000) and np.greater(el_time, 2.0):
            control_state = 0
        else:
            control_state = 1

    for i, data in enumerate([bucket_local_pos[1], bucket_local_pos[2],
                              relative_angle_filtered[0], relative_angle_filtered[1], relative_angle_filtered[2],
                              target_pipe_depth]):
        tcp_msg[i] = data

    logging_data = pd.DataFrame({'1': round(elapsed_time, 2), '2': round(excavator_angle[0].item(), 2),
                                 '3': round(excavator_angle[1].item(), 2), '4': round(excavator_angle[2].item(), 2),
                                 '5': round(excavator_angle[3].item(), 2), '6': round(relative_angle[0].item(), 2),
                                 '7': round(relative_angle[1].item(), 2), '8': round(relative_angle[2].item(), 2),
                                 '9': joystick_pos[0], '10': joystick_pos[1],
                                 '11': joystick_pos[2], '12': joystick_pos[3], '13': int(bucket_local_pos[1]),
                                 '14': int(bucket_local_pos[2]), '15': target_pipe_depth}, index=[0])

    logging_data.to_csv(logging_file_path, mode='a', header=False)
    time.sleep(0.1)
