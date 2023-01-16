import serial
from can.interfaces.pcan import PcanBus as pcan
from utils.predcit_api import InclinometerSensor, predict_control
from utils.end_point_kinematics import end_point_kinematics


serial_handle = serial.Serial(port='/dev/ttyUSB0', baudrate=9600)
serial_handle.open()

sensor_boom = InclinometerSensor(arbitration_id=0x10FF5385)
sensor_arm = InclinometerSensor(arbitration_id=0x10FF5386)
sensor_bucket = InclinometerSensor(arbitration_id=0x10FF5387)

bus = pcan(bitrate=250000)

for message in bus:
    t0 = message.timestamp
    boom_x_axis, boom_y_axis, boom_z_axis, boom_temp = sensor_boom.payload_paser(packet=message)
    arm_x_axis, arm_y_axis, arm_z_axis, arm_temp = sensor_arm.payload_paser(packet=message)
    bucket_x_axis, bucket_y_axis, bucket_z_axis, bucket_temp = sensor_bucket.payload_paser(packet=message)

    x_pos, z_pos = end_point_kinematics(boom_y_axis, arm_y_axis, bucket_y_axis)
    control_state = predict_control(z_pos)

    serial_msg = f'{boom_y_axis:.1f}' + ',' + f'{arm_y_axis:.1f}' + ',' + f'{bucket_y_axis:.1f}' + ',' \
                 + f'{x_pos:.1f}' + ',' + f'{z_pos:.1f}'
    serial_handle.write(serial_msg)
    print(serial_msg)
