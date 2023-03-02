import sys, os, can, threading
import time

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from utils.can_msg_parser import SafetyControlMsgParser, JoystickMsgParser, InclinometerSensor


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

        left_joystick_x_pos = joystick_status.read_joystick_value('left_x_position')
        left_joystick_y_pos = joystick_status.read_joystick_value('left_y_position')
        right_joystick_x_pos = joystick_status.read_joystick_value('right_x_position')
        right_joystick_y_pos = joystick_status.read_joystick_value('right_y_position')

        gps_ch1_latitude, gps_ch1_longitude, gps_ch2_latitude, gps_ch2_longitude = \
            safety_control_status.read_variable('equipment_cal_pos_data')

        '''
        def recognition_excavator_work_status(boom_pitch, arm_pitch, bucket_pitch,
                                              left_joystick_x_pos, left_joystick_y_pos,
                                              right_joystick_x_pos, right_joystick_y_pos,
                                              gps_ch1_latitude, gps_ch1_longitude,
                                              gps_ch2_latitude, gps_ch2_longitude) -> int:
        
            put code here!

            legend of output value
            00: stand by
            01: movement state
            02: working state
            03: anomaly state
        
            return excavator_work_status
        '''

        excavator_work_status = recognition_excavator_work_status(boom_pitch, arm_pitch, bucket_pitch,
                                                                  left_joystick_x_pos, left_joystick_y_pos,
                                                                  right_joystick_x_pos, right_joystick_y_pos,
                                                                  gps_ch1_latitude, gps_ch1_longitude,
                                                                  gps_ch2_latitude, gps_ch2_longitude)

        time.sleep(0.1)


if __name__ == '__main__':
    main()
