import time, threading
from can.interfaces.pcan import PcanBus as pcan
from utils.predcit_api import predict_control
from utils.sensor import InclinometerSensor
from utils.end_point_kinematics import end_point_kinematics
from utils.udp_lib import UdpClientCom


boom_x_axis = 0.0
arm_x_axis = 0.0
bucket_x_axis = 0.0
boom_x_axis_pred = 0.0
arm_x_axis_pred = 0.0
bucket_x_axis_pred = 0.0


def acquisition_inclinometer():
    global boom_x_axis, arm_x_axis, bucket_x_axis, boom_x_axis_pred, arm_x_axis_pred, bucket_x_axis_pred
    bus = pcan(bitrate=250000)
    sensor_boom = InclinometerSensor(arbitration_id=0x10FF5384)
    sensor_arm = InclinometerSensor(arbitration_id=0x10FF5385)
    sensor_bucket = InclinometerSensor(arbitration_id=0x10FF5386)

    for message in bus:
        boom_x_axis, boom_y_axis, boom_z_axis, boom_temp = sensor_boom.payload_paser(packet=message)
        arm_x_axis, arm_y_axis, arm_z_axis, arm_temp = sensor_arm.payload_paser(packet=message)
        bucket_x_axis, bucket_y_axis, bucket_z_axis, bucket_temp = sensor_bucket.payload_paser(packet=message)


acquire_thread = threading.Thread(target=acquisition_inclinometer)
acquire_thread.daemon = True
acquire_thread.start()

udp_handle = UdpClientCom(address='192.168.137.1', port=6340)

while True:
    boom_x_axis_pred = (0.5*boom_x_axis_pred)+(0.5*boom_x_axis)
    arm_x_axis_pred = (0.5*arm_x_axis_pred)+(0.5*arm_x_axis)
    bucket_x_axis_pred = (0.5*bucket_x_axis_pred)+(0.5*bucket_x_axis)

    x_pos, z_pos = end_point_kinematics(boom_x_axis_pred, arm_x_axis_pred, bucket_x_axis_pred)

    control_state, predict_signal = predict_control(input_signal=z_pos, time_offset=1, threshold_signal=1000)

    msg = [f'{boom_x_axis:.1f}', f'{arm_x_axis:.1f}', f'{bucket_x_axis:.1f}', f'{boom_x_axis_pred:.1f}',
           f'{arm_x_axis_pred:.1f}', f'{bucket_x_axis_pred:.1f}', f'{x_pos:.1f}', f'{z_pos:.1f}', f'{control_state}',
           f'{predict_signal:.1f}']

    udp_handle.send_msg(','.join(msg))

    time.sleep(0.05)
