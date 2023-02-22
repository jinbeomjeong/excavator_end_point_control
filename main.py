import time, threading
from can.interfaces.pcan import PcanBus as pcan
from utils.predcit_api import predict_control
from utils.sensor import InclinometerSensor
from utils.end_point_kinematics import end_point_kinematics
from utils.udp_lib import UdpClientCom
from utils.can_msg_parser import MsgParser

boom_pitch: float = 0.0
arm_pitch: float = 0.0
bucket_pitch: float = 0.0
boom_pitch_pred: float = 0.0
arm_pitch_pred: float = 0.0
bucket_pitch_pred: float = 0.0


def acquisition_inclinometer():
    global boom_pitch, arm_pitch, bucket_pitch, boom_pitch_pred, arm_pitch_pred, bucket_pitch_pred
    bus = pcan(bitrate=250000)
    sensor_boom = InclinometerSensor(arbitration_id=0x10FF5384)
    sensor_arm = InclinometerSensor(arbitration_id=0x10FF5385)
    sensor_bucket = InclinometerSensor(arbitration_id=0x10FF5386)

    for message in bus:
        boom_pitch, boom_roll, boom_yaw, boom_temp = sensor_boom.payload_paser(packet=message)
        arm_pitch, arm_roll, arm_yaw, arm_temp = sensor_arm.payload_paser(packet=message)
        bucket_pitch, bucket_roll, bucket_yaw, bucket_temp = sensor_bucket.payload_paser(packet=message)


def heartbeat():
    t0 = time.time()

    while True:
        el_time = time.time() - t0

        if el_time % 2 == 0:
            print(time.time())
            print('signal')


acquire_thread = threading.Thread(target=acquisition_inclinometer)
acquire_thread.daemon = True
acquire_thread.start()

heart_beat_thread = threading.Timer(interval=0.1, function=heartbeat)
heart_beat_thread.daemon = True
heart_beat_thread.start()

udp_handle = UdpClientCom(address='192.168.137.1', port=6340)
can_msg_handle = MsgParser()

while True:
    boom_pitch_pred = (0.5*boom_pitch_pred)+(0.5*boom_pitch)
    arm_pitch_pred = (0.5*arm_pitch_pred)+(0.5*arm_pitch)
    bucket_pitch_pred = (0.5*bucket_pitch_pred)+(0.5*bucket_pitch)

    x_pos, z_pos = end_point_kinematics(boom_pitch_pred, arm_pitch_pred, bucket_pitch_pred)

    control_state, predict_signal = predict_control(input_signal=z_pos, time_offset=1, threshold_signal=1000)

    msg = [f'{boom_pitch:.1f}', f'{arm_pitch:.1f}', f'{bucket_pitch:.1f}', f'{boom_pitch_pred:.1f}',
           f'{arm_pitch_pred:.1f}', f'{bucket_pitch_pred:.1f}', f'{x_pos:.1f}', f'{z_pos:.1f}', f'{control_state}',
           f'{predict_signal:.1f}']

    udp_handle.send_msg(','.join(msg))
    can_msg_handle.send_estimated_position(control_flag=0, state_flag=0, x_pos=x_pos, y_pos=0, z_pos=z_pos)
    time.sleep(0.05)
