from can.interfaces.pcan import PcanBus as pcan
from predcit_api import InclinometerSensor

sensor_boom = InclinometerSensor(arbitration_id=0x10FF5385)
sensor_arm = InclinometerSensor(arbitration_id=0x10FF5386)
sensor_bucket = InclinometerSensor(arbitration_id=0x10FF5387)

bus = pcan(bitrate=250000)

for message in bus:
    t0 = message.timestamp
    boom_x_axis, boom_y_axis, boom_z_axis, boom_temp = sensor_boom.payload_paser(packet=message)
    arm_x_axis, arm_y_axis, arm_z_axis, arm_temp = sensor_arm.payload_paser(packet=message)
    bucket_x_axis, bucket_y_axis, bucket_z_axis, bucket_temp = sensor_bucket.payload_paser(packet=message)

    print(boom_y_axis, arm_y_axis, bucket_y_axis)