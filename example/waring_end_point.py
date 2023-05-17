import sys, os, can, threading, time

import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from utils.can_msg_parser import SafetyControlMsgParser, JoystickMsgParser, InclinometerSensor
from utils.warning_target import get_end_pos, get_safety_status
from utils.end_point_kinematics import end_point_kinematics_test


can_ch1 = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)
can_ch2 = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS2', bitrate=250000)

boom_sensor = InclinometerSensor(arbitration_id=0x10FF5384)
arm_sensor = InclinometerSensor(arbitration_id=0x10FF5385)
bucket_sensor = InclinometerSensor(arbitration_id=0x10FF5386)

joystick_status = JoystickMsgParser()

safety_control_status = SafetyControlMsgParser()


def ic_controller_can():
    for message in can_ch1:
        boom_sensor.get_sensor_values(packet=message)
        arm_sensor.get_sensor_values(packet=message)
        bucket_sensor.get_sensor_values(packet=message)

        joystick_status.get_joystick_values(packet=message)


def internal_controllers_can():
    for message in can_ch2:
        safety_control_status.get_equipment_pos(packet=message)


ic_controller_thread = threading.Thread(target=ic_controller_can)
ic_controller_thread.daemon = True
ic_controller_thread.start()

internal_controllers_thread = threading.Thread(target=internal_controllers_can)
internal_controllers_thread.daemon = True
internal_controllers_thread.start()


def main() -> None:
    while True:
        boom_pitch = boom_sensor.read_sensor_value('x_axis')
        arm_pitch = arm_sensor.read_sensor_value('x_axis')
        bucket_pitch = bucket_sensor.read_sensor_value('x_axis')

        gps_ch1_latitude, gps_ch1_longitude, gps_ch2_latitude, gps_ch2_longitude = \
            safety_control_status.read_variable('equipment_cal_pos_data')

        local_bucket_x, local_bucket_z = end_point_kinematics_test(boom_pitch, arm_pitch, bucket_pitch)
        global_bucket_x, global_bucket_y = get_end_pos(origin=[gps_ch1_latitude, gps_ch1_longitude],
                                                       y_distance=local_bucket_x, heading=0)

        status = get_safety_status(np.array([global_bucket_x, global_bucket_y]), np.array([]))
        time.sleep(0.1)


if __name__ == '__main__':
    main()
