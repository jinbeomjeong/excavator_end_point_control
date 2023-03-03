import time, threading, can
import numpy as np
from utils.predcit_api import predict_control_quadratic
from utils.end_point_kinematics import end_point_kinematics
from utils.can_msg_parser import SafetyControlMsgParser, InclinometerSensor, JoystickMsgParser


can_ch1 = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)
can_ch2 = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS2', bitrate=250000)

boom_pitch_pred: float = 0.0
arm_pitch_pred: float = 0.0
bucket_pitch_pred: float = 0.0

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


ic_controller_thread = threading.Thread(target=ic_controller_can)
ic_controller_thread.daemon = True
ic_controller_thread.start()


def main() -> None:
    print('program is running...')
    bucket_pos: np.ndarray = np.zeros(3, dtype=np.float32)  # x-axis, y-axis, z-axis

    while True:
        boom_pitch = boom_sensor.read_sensor_value('x_axis')
        arm_pitch = arm_sensor.read_sensor_value('x_axis')
        bucket_pitch = bucket_sensor.read_sensor_value('x_axis')

        #boom_pitch_pred = (0.5*boom_pitch_pred)+(0.5*boom_pitch)
        #arm_pitch_pred = (0.5*arm_pitch_pred)+(0.5*arm_pitch)
        #bucket_pitch_pred = (0.5*bucket_pitch_pred)+(0.5*bucket_pitch)

        bucket_pos[0], bucket_pos[2] = end_point_kinematics(boom_pitch, arm_pitch, bucket_pitch)
        bucket_pos = np.clip(bucket_pos, -54.0, +54.0)

        control_state, predict_signal = predict_control_quadratic(input_signal=bucket_pos[2], time_offset=0.3,
                                                                  kernel_size=10, threshold_signal=1000)

        sc_msg = safety_control_status.create_estimated_position_can_msg(control_flag=control_state, state_flag=0,
                                                                         x_pos=bucket_pos[0], y_pos=bucket_pos[1],
                                                                         z_pos=bucket_pos[2])
        can_ch2.send(sc_msg)
        time.sleep(0.1)


if __name__ == '__main__':
    main()
