import can
import numpy as np


np.set_printoptions(suppress=True)


class SafetyControlMsgParser:
    def __init__(self):
        self.__automatic_control_recv_id__ = 0x18FF1000  # IC -> SC
        self.__automatic_control_send_id__ = 0x18FF1003  # IC -> SC
        self.__under_utils_data_id__ = 0x18FF1004  # CD -> SC
        self.__equipment_coord_frame_left_id__ = 0x18FF5010  # CD -> IC/SC
        self.__equipment_coord_frame_right_id__ = 0x18FF5020  # CD -> IC/SC
        self.__equipment_coord_frame_left_altitude_id__ = 0x18FF5011  # CD -> IC/SC
        self.__equipment_coord_frame_right_altitude_id__ = 0x18FF5021

        self.__automatic_control__: str = 'none'
        self.__under_utils_data_transmit__: str = 'none'
        self.__under_utils_data_number__: int = 0
        self.__equipment_raw_pos__: np.ndarray = np.zeros(4, dtype=np.uint32)  # gps_1_y, gps_1_x, gps_2_y, gps_2_x
        self.__equipment_cal_pos__: np.ndarray = np.zeros(4, dtype=np.float32)
        self.__equipment_altitude__: np.ndarray = np.zeros(2, dtype=np.float32)  # gps_1_altitude, gps_2_altitude
        self.__bucket_position_data__: [int] = [0]*3  # x_pos, y_pos, z_pos

        self.__class__.get_under_utils_data.called = False

    def get_automatic_control(self, packet):
        if packet.arbitration_id == self.__automatic_control_recv_id__:
            payload = packet.data

            if payload[0] == 0:
                self.__automatic_control__ = 'off'
            elif payload[0] == 1:
                self.__automatic_control__ = 'stand by'
            elif payload[0] == 2:
                self.__automatic_control__ = 'on'

    def get_under_utils_status(self, packet):
        if packet.arbitration_id == self.__under_utils_data_id__:
            payload = packet.data

            if payload[0] == 0:
                self.__automatic_control__ = 'off'
            elif payload[0] == 1:
                self.__automatic_control__ = 'stand by'
            elif payload[0] == 2:
                self.__automatic_control__ = 'done'

            if payload[1] == 0:
                self.__under_utils_data_transmit__ = 'none'
            elif payload[1] == 1:
                self.__under_utils_data_transmit__ = 'transmitting'
            elif payload[1] == 2:
                self.__under_utils_data_transmit__ = 'done'

            self.__under_utils_data_number__ = payload[2]

    def get_under_utils_data(self, packet):
        arbitration_id = packet.arbitration_id
        payload = packet.data

        if not self.__class__.get_under_utils_data.called:
            self.__under_utils_coord_id__ = np.zeros(self.__under_utils_data_number__, dtype=np.uint32)
            self.__under_utils_depth_id__ = np.zeros(self.__under_utils_data_number__, dtype=np.uint32)
            self.__under_utils_coord_data__ = np.zeros((self.__under_utils_data_number__, 2), dtype=np.uint32)
            self.__under_utils_depth_data__ = np.zeros((self.__under_utils_data_number__, 2), dtype=np.float32)
            self.__class__.get_under_utils_data.called = True

        else:
            for i in range(self.__under_utils_data_number__):
                self.__under_utils_coord_id__[i] = int('0x18FF20' + str(i + 1) + '0', base=16)

            for i in range(self.__under_utils_data_number__):
                self.__under_utils_depth_id__[i] = int('0x18FF201' + str(i + 1), base=16)

            for coord_id in self.__under_utils_coord_id__:
                if arbitration_id == coord_id:
                    self.__under_utils_coord_data__[int(hex(arbitration_id)[8]) - 1, 0:2] = \
                        np.array([int.from_bytes(payload[4:8], byteorder='little'),
                                  int.from_bytes(payload[0:4], byteorder='little')], dtype=np.uint32)

            for depth_id in self.__under_utils_depth_id__:
                if arbitration_id == depth_id:
                    self.__under_utils_depth_data__[int(hex(arbitration_id)[9]), 2:4] = \
                        np.array(int.from_bytes(payload[0:2], byteorder='little'), payload[2])

    def get_equipment_pos(self, packet):
        arbitration_id = packet.arbitration_id
        payload = packet.data

        if arbitration_id == self.__equipment_coord_frame_left_id__:
            self.__equipment_raw_pos__[0:2] = np.array([int.from_bytes(payload[0:4], byteorder='little'),
                                                        int.from_bytes(payload[4:8], byteorder='little')])

        if arbitration_id == self.__equipment_coord_frame_right_id__:
            self.__equipment_raw_pos__[2:4] = np.array([int.from_bytes(payload[0:4], byteorder='little'),
                                                        int.from_bytes(payload[4:8], byteorder='little')])

        if arbitration_id == self.__equipment_coord_frame_left_altitude_id__:
            self.__equipment_altitude__[0] = int.from_bytes(payload[0:2], byteorder='little', signed=True)*0.01

        if arbitration_id == self.__equipment_coord_frame_left_altitude_id__:
            self.__equipment_altitude__[1] = int.from_bytes(payload[0:2], byteorder='little', signed=True)*0.01
            
        self.__equipment_cal_pos__[0] = 36+(self.__equipment_raw_pos__[0]*(10**-8))
        self.__equipment_cal_pos__[1] = 126+(self.__equipment_raw_pos__[1]*(10**-8))
        self.__equipment_cal_pos__[2] = 36+(self.__equipment_raw_pos__[2]*(10**-8))
        self.__equipment_cal_pos__[3] = 126+(self.__equipment_raw_pos__[3]*(10**-8))

    def create_estimated_position_can_msg(self, control_flag: int = 0, state_flag: int = 0, x_pos: float = 0.0, 
                                          y_pos: float = 0.0, z_pos: float = 0.0):

        for i, data in enumerate([x_pos, y_pos, z_pos]):
            self.__bucket_position_data__[i] = int((data*100)+5400)

        x_pos_msb, x_pos_lsb = self.__bucket_position_data__[0].to_bytes(2, byteorder='little')
        y_pos_msb, y_pos_lsb = self.__bucket_position_data__[1].to_bytes(2, byteorder='little')
        z_pos_msb, z_pos_lsb = self.__bucket_position_data__[2].to_bytes(2, byteorder='little')
        __send_data__ = [control_flag, state_flag, x_pos_msb, x_pos_lsb, y_pos_msb, y_pos_lsb, z_pos_msb, z_pos_lsb]

        send_msg = can.Message(arbitration_id=self.__automatic_control_send_id__, data=__send_data__,
                               is_extended_id=True)

        return send_msg

    def read_variable(self, parameter_name=''):
        if parameter_name == 'automatic_control':
            return self.__automatic_control__

        elif parameter_name == 'under_utils_data_transmit':
            return self.__under_utils_data_transmit__

        elif parameter_name == 'under_utils_data_number':
            return self.__under_utils_data_number__

        elif parameter_name == 'equipment_raw_pos':
            return self.__equipment_raw_pos__

        elif parameter_name == 'equipment_cal_pos_data':
            return self.__equipment_cal_pos__[0], self.__equipment_cal_pos__[1], self.__equipment_cal_pos__[2],\
                self.__equipment_cal_pos__[3]
                    
        elif parameter_name == 'equipment_altitude':
            return self.__equipment_altitude__
        
        elif parameter_name == 'under_utils_coord_data':
            return self.__under_utils_coord_data__


class JoystickMsgParser:
    def __init__(self):
        self.__left_x_position__: float = 0.0
        self.__left_y_position__: float = 0.0
        self.__right_x_position__: float = 0.0
        self.__right_y_position__: float = 0.0

        self.__left_joystick_can_id__ = 0x18FF6381
        self.__right_joystick_can_id__ = 0x18FF6382

    def get_joystick_values(self, packet):
        arbitration_id = packet.arbitration_id
        payload = packet.data

        if arbitration_id == self.__left_joystick_can_id__:
            self.__left_x_position__ = payload[1]
            self.__left_y_position__ = payload[3]

        if arbitration_id == self.__right_joystick_can_id__:
            self.__right_x_position__ = payload[1]
            self.__right_y_position__ = payload[3]

    def read_joystick_value(self, parameter_name='') -> float:
        if parameter_name == 'left_x_position':
            return self.__left_x_position__

        elif parameter_name == 'left_y_position':
            return self.__left_y_position__

        elif parameter_name == 'right_x_position':
            return self.__right_x_position__

        elif parameter_name == 'right_y_position':
            return self.__right_y_position__


class InclinometerSensor:
    def __init__(self, arbitration_id):
        self.__arbitration_id__ = arbitration_id
        self.__x_axis__: float = 0
        self.__y_axis__: float = 0
        self.__z_axis__: float = 0
        self.__temp__: float = 0

    def get_sensor_values(self, packet):
        if packet.arbitration_id == self.__arbitration_id__:
            msg_hex = packet.data.hex()

            if len(msg_hex) == 16:
                self.__x_axis__ = (float(int(msg_hex[0:4], base=16)) / 16384) * 90
                self.__x_axis__ = (float(int(msg_hex[4:8], base=16)) / 16384) * 90
                self.__z_axis__ = (float(int(msg_hex[8:12], base=16)) / 16384) * 90
                self.__temp__ = -273 + (int(msg_hex[12:16], base=16) / 18.9)

    def read_sensor_value(self, parameter_name='') -> float:
        if parameter_name == 'x_axis':
            return self.__x_axis__

        elif parameter_name == 'y_axis':
            return self.__y_axis__

        elif parameter_name == 'z_axis':
            return self.__z_axis__

        elif parameter_name == 'temp':
            return self.__temp__
