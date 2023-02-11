import time
from can.interfaces.pcan import PcanBus as pcan
from utils.predcit_api import predict_control
from utils.sensor import InclinometerSensor
from utils.end_point_kinematics import end_point_kinematics
from utils.udp_lib import UdpClientCom


sensor_boom = InclinometerSensor(arbitration_id=0x10FF5384)
sensor_arm = InclinometerSensor(arbitration_id=0x10FF5385)
sensor_bucket = InclinometerSensor(arbitration_id=0x10FF5386)

udp_handle = UdpClientCom(addr='localhost', port=8080)

bus = pcan(bitrate=250000)
boom_x_axis_pred = 0
arm_x_axis_pred = 0
bucket_x_axis_pred = 0

for message in bus:
    boom_x_axis, boom_y_axis, boom_z_axis, boom_temp = sensor_boom.payload_paser(packet=message)
    arm_x_axis, arm_y_axis, arm_z_axis, arm_temp = sensor_arm.payload_paser(packet=message)
    bucket_x_axis, bucket_y_axis, bucket_z_axis, bucket_temp = sensor_bucket.payload_paser(packet=message)

    boom_x_axis_pred = (0.5*boom_x_axis_pred) + (0.5*boom_x_axis)
    arm_x_axis_pred = (0.5*arm_x_axis_pred) + (0.5*arm_x_axis)
    bucket_x_axis_pred = (0.5*bucket_x_axis_pred) + (0.5*bucket_x_axis)

    x_pos, z_pos = end_point_kinematics(boom_x_axis_pred, arm_x_axis_pred, bucket_x_axis_pred)
    control_state = predict_control(z_pos)

    msg = [f'{boom_x_axis:.1f}', f'{arm_x_axis:.1f}', f'{bucket_x_axis:.1f}', f'{boom_x_axis_pred}', f'{arm_x_axis_pred}',
           f'{bucket_x_axis_pred}', f'{x_pos:.1f}', f'{z_pos:.1f}', f'{control_state}']

    udp_handle.send_msg(','.join(msg))
    #time.sleep(0.01)
