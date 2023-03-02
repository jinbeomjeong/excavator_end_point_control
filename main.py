import time, threading, can
from utils.predcit_api import predict_control
from utils.end_point_kinematics import end_point_kinematics
from utils.udp_lib import UdpClientCom
from utils.can_msg_parser import SafetyControlMsgParser, InclinometerSensor


boom_pitch_pred: float = 0.0
arm_pitch_pred: float = 0.0
bucket_pitch_pred: float = 0.0

boom_sensor = InclinometerSensor(arbitration_id=0x10FF5384)
arm_sensor = InclinometerSensor(arbitration_id=0x10FF5385)
bucket_sensor = InclinometerSensor(arbitration_id=0x10FF5386)


def acquisition_inclinometer():
    can_ch1 = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)

    for message in can_ch1:
        boom_sensor.get_sensor_values(packet=message)
        arm_sensor.get_sensor_values(packet=message)
        bucket_sensor.get_sensor_values(packet=message)


def joystick_status():
    can_ch2 = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS2', bitrate=250000)

    for message in can_ch2:
        pass


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

acquire_thread = threading.Thread(target=joystick_status)
acquire_thread.daemon = True
acquire_thread.start()

heart_beat_thread = threading.Timer(interval=0.1, function=heartbeat)
heart_beat_thread.daemon = True
heart_beat_thread.start()

udp_handle = UdpClientCom(address='192.168.137.1', port=6340)
can_msg_handle = SafetyControlMsgParser()

while True:
    boom_pitch = boom_sensor.read_sensor_value('x_axis')
    arm_pitch = arm_sensor.read_sensor_value('x_axis')
    bucket_pitch = bucket_sensor.read_sensor_value('x_axis')

    boom_pitch_pred = (0.5*boom_pitch_pred)+(0.5*boom_pitch)
    arm_pitch_pred = (0.5*arm_pitch_pred)+(0.5*arm_pitch)
    bucket_pitch_pred = (0.5*bucket_pitch_pred)+(0.5*bucket_pitch)

    x_pos, z_pos = end_point_kinematics(boom_pitch_pred, arm_pitch_pred, bucket_pitch_pred)

    control_state, predict_signal = predict_control(input_signal=z_pos, time_offset=1, threshold_signal=1000)

    msg = [f'{boom_pitch:.1f}', f'{arm_pitch:.1f}', f'{bucket_pitch:.1f}', f'{boom_pitch_pred:.1f}',
           f'{arm_pitch_pred:.1f}', f'{bucket_pitch_pred:.1f}', f'{x_pos:.1f}', f'{z_pos:.1f}', f'{control_state}',
           f'{predict_signal:.1f}']

    udp_handle.send_msg(','.join(msg))
    can_msg_handle.create_estimated_position_can_msg(control_flag=0, state_flag=0, x_pos=x_pos, y_pos=0, z_pos=z_pos)
    time.sleep(0.05)
