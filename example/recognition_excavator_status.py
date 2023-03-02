import sys, os, can, threading

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from utils.can_msg_parser import SafetyControlMsgParser, JoystickMsgParser, InclinometerSensor


can_ch1 = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)
can_ch2 = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS2', bitrate=250000)

boom_sensor = InclinometerSensor(arbitration_id=0x10FF5384)
arm_sensor = InclinometerSensor(arbitration_id=0x10FF5385)
bucket_sensor = InclinometerSensor(arbitration_id=0x10FF5386)

joystick_status = JoystickMsgParser()

safety_control_status = SafetyControlMsgParser()


def inclinometer_joystick():
    for message in can_ch1:
        can_ch1_id = message.arbitration_id
        can_ch1_payload = message.data

        boom_sensor.get_sensor_values(packet=message)
        arm_sensor.get_sensor_values(packet=message)
        bucket_sensor.get_sensor_values(packet=message)

        joystick_status.get_joystick_status(arbitration_id=can_ch1_id, payload=can_ch1_payload)


def safety_control():
    for message in can_ch2:
        can_ch2_id = message.arbitration_id
        can_ch2_payload = message.data

        safety_control_status.get_equipment_pos(arbitration_id=can_ch2_id, payload=can_ch2_payload)


sensor_thread = threading.Thread(target=inclinometer_joystick)
sensor_thread.daemon = True
sensor_thread.start()

joystick_thread = threading.Thread(target=safety_control)
joystick_thread.daemon = True
joystick_thread.start()

while True:
    boom_pitch = boom_sensor.read_sensor_value('x_axis')
    arm_pitch = arm_sensor.read_sensor_value('x_axis')
    bucket_pitch = bucket_sensor.read_sensor_value('x_axis')

    left_joystick_x_pos = joystick_status.read_joystick_value('left_x_position')
    left_joystick_y_pos = joystick_status.read_joystick_value('left_y_position')
    right_joystick_x_pos = joystick_status.read_joystick_value('right_x_position')
    right_joystick_y_pos = joystick_status.read_joystick_value('right_y_position')

    gps_ch1_latitude, gps_ch1_longitude, gps_ch2_latitude, gps_ch2_longitude = \
        safety_control_status.read_variable('equipment_cal_pos_data')

    '''
    put code here
    '''


