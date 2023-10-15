import can, time, cantools
import numpy as np
from typing import Final


class SafetyControlMsgParser:
    def __init__(self):
        self.__ic_state_id: Final[int] = 0x18FF1010
        self.__cd_state_id: Final[int] = 0x18FF1011
        self.__sc_state_id: Final[int] = 0x18FF1012
        self.__under_utils_query_cd_id: Final[int] = 0x18FF3010  # CD -> SC
        self.__under_utils_query_sc_id: Final[int] = 0x18FF3011  # SC -> CD
        self.__equipment_coord_frame_left_id: Final[int] = 0x18FF5010  # CD -> IC/SC
        self.__equipment_coord_frame_right_id: Final[int] = 0x18FF5020  # CD -> IC/SC
        self.__equipment_coord_frame_left_altitude_id: Final[int] = 0x18FF5011  # CD -> IC/SC
        self.__equipment_coord_frame_right_altitude_id: Final[int] = 0x18FF5021
        self.__compensation_angle_id: Final[int] = 0x18FF1013  # SC -> IC
        self.__equipment_origin_pos_id: Final[int] = 0x18FF1014  # SC -> IC, CD
        self.__equipment_origin_heading_id: Final[int] = 0x18FF1015  # SC -> IC, CD
        self.__target_relative_distance_id: Final[int] = 0x18FF1018  # SC -> IC
        self.__work_area_information_id: Final[int] = 0x18FF1017  # CD -> IC
        self.__work_area_information_query_id: Final[int] = 0x18FF3012  # SC -> CD
        self.__equipment_integer_position_id: Final[int] = 0x18FF1019
        self.__end_point_float_position_id: Final[int] = 0x18FF1020

        self.__ic_state: str = 'none'
        self.__cd_state: str = 'none'
        self.__work_limit: str = 'de-active'
        self.__cd_query: str = 'none'
        self.__n_of_under_utils: int = 0
        self.__equipment_raw_integer_pos: np.ndarray = np.zeros(shape=2, dtype=np.uint32)  # Latitude, Longitude
        self.__equipment_raw_float_pos: np.ndarray = np.zeros(shape=4, dtype=np.uint32)  # gps_1_y, gps_1_x, gps_2_y, gps_2_x
        self.__equipment_cal_pos: np.ndarray = np.zeros(shape=4, dtype=np.float64)
        self.__equipment_origin_pos: np.ndarray = np.zeros(shape=2, dtype=np.float64)  # longitude, latitude, heading
        self.__equipment_altitude: np.ndarray = np.zeros(shape=2, dtype=np.float64)  # gps_1_altitude, gps_2_altitude
        self.__compensation_angle_data = np.zeros(shape=3, dtype=np.float64)
        self.__work_area_number: int = 0
        self.__admin_phone_number = np.zeros(shape=3, dtype=np.uint16)
        self.__class__.get_under_utils_data.called = False
        self.__t0: Final[float] = time.time()

    def __time_elapsed(self) -> float:
        return time.time() - self.__t0

    def get_ic_state(self, packet=can.Message()):
        if packet.arbitration_id == self.__ic_state_id:
            payload = packet.data

            if payload[0] == 0:
                self.__ic_state = 'off'
            elif payload[0] == 1:
                self.__ic_state = 'stand_by'
            elif payload[0] == 2:
                self.__ic_state = 'on'

    def get_cd_state(self, packet=can.Message()):
        if packet.arbitration_id == self.__cd_state_id:
            payload = packet.data

            if payload[0] == 0:
                self.__cd_state = 'off'
            elif payload[0] == 1:
                self.__cd_state = 'stand_by'
            elif payload[0] == 2:
                self.__cd_state = 'on'

            if payload[1] == 0:
                self.__work_limit = 'de-active'
            elif payload[1] == 1:
                self.__work_limit = 'active'

    def get_under_utils_query(self, packet=can.Message()):
        if packet.arbitration_id == self.__under_utils_query_cd_id:
            payload = packet.data

            if payload[0] == 0:
                self.__cd_query = 'none'
            elif payload[0] == 1:
                self.__cd_query = 'ready_to_receive'
            elif payload[0] == 2:
                self.__cd_query = 'transmitting'
            elif payload[0] == 3:
                self.__cd_query = 'done'

            self.__n_of_under_utils = payload[1]

    def get_under_utils_data(self, packet=can.Message()):
        arbit_id = packet.arbitration_id
        payload = packet.data

        if not self.__class__.get_under_utils_data.called:
            self.__under_utils_coord_id = np.zeros(self.__n_of_under_utils, dtype=np.uint32)
            self.__under_utils_depth_id = np.zeros(self.__n_of_under_utils, dtype=np.uint32)
            self.__under_utils_coord_data = np.zeros(shape=(self.__n_of_under_utils, 2), dtype=np.float64)
            self.__under_utils_depth_dia_data = np.zeros(shape=(self.__n_of_under_utils, 2), dtype=np.float64)

            for i in range(self.__n_of_under_utils):
                self.__under_utils_coord_id[i] = int('0x18FF' + hex(i)[2:4].zfill(2) + '01', base=16)

            for i in range(self.__n_of_under_utils):
                self.__under_utils_depth_id[i] = int('0x18FF' + hex(i)[2:4].zfill(2) + '02', base=16)

            self.__class__.get_under_utils_data.called = True

        else:
            for coord_id in self.__under_utils_coord_id:
                if coord_id == arbit_id:
                    self.__under_utils_coord_data[int(hex(arbit_id)[6:8], base=16), 0:2] = \
                        np.array([36+(int.from_bytes(payload[4:8], byteorder='little')*(10 ** -8)),
                                  126+(int.from_bytes(payload[0:4], byteorder='little')*(10 ** -8))], dtype=np.float64)

            for depth_id in self.__under_utils_depth_id:
                if depth_id == arbit_id:
                    self.__under_utils_depth_dia_data[int(hex(arbit_id)[6:8], base=16), 0:2] = \
                        np.array([int.from_bytes(payload[0:2], byteorder='little')*0.01, payload[2]*0.1], dtype=np.float64)

    def get_equip_pos(self, packet=can.Message()):
        payload = packet.data

        if packet.arbitration_id == self.__equipment_coord_frame_left_id:
            self.__equipment_raw_float_pos[0:2] = np.array([int.from_bytes(payload[0:4], byteorder='little'),
                                                            int.from_bytes(payload[4:8], byteorder='little')])

        if packet.arbitration_id == self.__equipment_coord_frame_right_id:
            self.__equipment_raw_float_pos[2:4] = np.array([int.from_bytes(payload[0:4], byteorder='little'),
                                                            int.from_bytes(payload[4:8], byteorder='little')])

        if packet.arbitration_id == self.__equipment_integer_position_id:
            self.__equipment_raw_integer_pos[0] = payload[0]
            self.__equipment_raw_integer_pos[1] = int.from_bytes(payload[1:3], byteorder='little', signed=False)

        if packet.arbitration_id == self.__equipment_coord_frame_left_altitude_id:
            self.__equipment_altitude[0] = int.from_bytes(payload[0:2], byteorder='little', signed=True) * 0.01

        if packet.arbitration_id == self.__equipment_coord_frame_left_altitude_id:
            self.__equipment_altitude[1] = int.from_bytes(payload[0:2], byteorder='little', signed=True) * 0.01

        for i, int_val in enumerate([36, 126, 36, 126]):
            self.__equipment_cal_pos[i] = int_val+(self.__equipment_raw_float_pos[i]*(10**-8))

    def get_work_area_information(self, packet=can.Message()):
        if packet.arbitration_id == self.__work_area_information_id:
            payload = packet.data
            self.__work_area_number = int.from_bytes(payload[0:2], byteorder='little', signed=False)

            for i in range(3):
                self.__admin_phone_number[i] = int.from_bytes(payload[2*(i+1):(2*(i+1))+2], byteorder='little', signed=False)

    def create_sc_state_msg(self, sc_state: int = 0, safety_state: int = 0, excavator_state: int = 0) -> can.Message:
        payload = [sc_state, safety_state, excavator_state]
        send_msg = can.Message(timestamp=self.__time_elapsed(), arbitration_id=self.__sc_state_id, data=payload,
                               is_extended_id=True)

        return send_msg

    def create_comp_angle_msg(self, boom_angle: np.ndarray = 0.0, arm_angle: np.ndarray = 0.0,
                              bucket_angle: np.ndarray = 0.0) -> can.Message:

        self.__compensation_angle_data[0] = boom_angle
        self.__compensation_angle_data[1] = arm_angle
        self.__compensation_angle_data[2] = bucket_angle

        payload = bytes(0)

        for comp_angle in self.__compensation_angle_data:
            if comp_angle >= 360:
                comp_angle = 360
            elif comp_angle <= -360:
                comp_angle = -360

            comp_angle = (comp_angle*10)+3600
            payload += int(comp_angle).to_bytes(length=2, byteorder='little')

        send_msg = can.Message(timestamp=self.__time_elapsed(), arbitration_id=self.__compensation_angle_id,
                               data=payload, is_extended_id=True)

        return send_msg

    def create_equip_origin_pos_msg(self, latitude: np.ndarray = 0.0, longitude: np.ndarray = 0.0) -> can.Message:
        self.__equipment_origin_pos[0] = latitude * (10**-8)
        self.__equipment_origin_pos[1] = longitude * (10**-8)
        payload = bytes(0)

        for position in self.__equipment_origin_pos:
            payload += int(position).to_bytes(length=4, byteorder='little')

        send_msg = can.Message(timestamp=self.__time_elapsed(), arbitration_id=self.__equipment_origin_pos_id,
                               data=payload, is_extended_id=True)

        return send_msg

    def create_equip_heading_msg(self, heading: np.ndarray = 0.0) -> can.Message:
        payload = int(heading * 10).to_bytes(length=2, byteorder='little')
        send_msg = can.Message(timestamp=self.__time_elapsed(), arbitration_id=self.__equipment_origin_heading_id,
                               data=payload, is_extended_id=True)

        return send_msg

    def create_under_utils_query_msg(self, request_msg: int = 0) -> can.Message:
        payload = [request_msg]
        send_msg = can.Message(timestamp=self.__time_elapsed(), arbitration_id=self.__under_utils_query_sc_id,
                               data=payload, is_extended_id=True)

        return send_msg

    def create_ic_state_msg(self, ic_state: int = 0) -> can.Message:
        send_msg = can.Message(timestamp=self.__time_elapsed(), arbitration_id=self.__sc_state_id, data=[ic_state],
                               is_extended_id=True)
        return send_msg

    def create_target_relative_distance_msg(self, state: int = 0, distance: int = 0) -> can.Message:
        payload = bytes(0)
        payload += state.to_bytes(length=1, byteorder='little')
        payload += distance.to_bytes(length=2, byteorder='little')
        send_msg = can.Message(timestamp=self.__time_elapsed(), arbitration_id=self.__target_relative_distance_id,
                               data=payload, is_extended_id=True)

        return send_msg

    def create_cd_state_msg(self, state: int = 0, work_limit: int = 0) -> can.Message:
        payload = [state, work_limit]
        send_msg = can.Message(timestamp=self.__time_elapsed(), arbitration_id=self.__cd_state_id, data=payload,
                               is_extended_id=True)

        return send_msg

    def create_work_area_information_msg(self, area_number: int = 0,
                                         admin_phone_number=np.zeros(shape=3, dtype=np.uint16)) -> can.Message:
        payload = bytes(0)
        payload += area_number.to_bytes(length=2, byteorder='little')

        for number in admin_phone_number:
            payload += int(number).to_bytes(length=2, byteorder='little')

        send_msg = can.Message(timestamp=self.__time_elapsed(), arbitration_id=self.__work_area_information_id,
                               data=payload, is_extended_id=True)

        return send_msg

    def create_end_point_tm_position_msg(self, x_axis: np.ndarray = 0.0, y_axis: np.ndarray = 0.0) -> can.Message:
        payload = bytes(0)

        if x_axis <= 0: x_axis = 0
        if y_axis <= 0: y_axis = 0

        payload += int(x_axis*1000).to_bytes(length=4, byteorder='little')
        payload += int(y_axis*1000).to_bytes(length=4, byteorder='little')

        send_msg = can.Message(timestamp=self.__time_elapsed(), arbitration_id=self.__end_point_float_position_id,
                               data=payload, is_extended_id=True)

        return send_msg

    def create_end_point_float_position_msg(self, x_axis: np.ndarray, y_axis: np.ndarray) -> can.Message:
        x_axis_float, x_axis_integer = np.modf(x_axis)
        y_axis_float, y_axis_integer = np.modf(y_axis)

        payload = bytes(0)
        payload += int(round(x_axis_float[0]*(10**8), 8)).to_bytes(length=4, byteorder='little')
        payload += int(round(y_axis_float[0]*(10**8), 8)).to_bytes(length=4, byteorder='little')

        send_msg = can.Message(timestamp=self.__time_elapsed(), arbitration_id=self.__end_point_float_position_id,
                               data=payload, is_extended_id=True)

        return send_msg

    def read_variable(self, parameter_name: str = ''):
        if parameter_name == 'ic_state':
            return self.__ic_state

        elif parameter_name == 'cd_state':
            return self.__cd_state

        elif parameter_name == 'cd_query':
            return self.__cd_query

        elif parameter_name == 'n_of_under_utils':
            return self.__n_of_under_utils

        elif parameter_name == 'under_utils_coord_data':
            return self.__under_utils_coord_data

        elif parameter_name == 'under_utils_coord_id':
            return self.__under_utils_coord_id

        elif parameter_name == 'under_utils_depth_data':
            return self.__under_utils_depth_dia_data

        elif parameter_name == 'equipment_raw_pos':
            return self.__equipment_raw_float_pos

        elif parameter_name == 'equipment_cal_pos_data':
            return self.__equipment_cal_pos

        elif parameter_name == 'equipment_altitude':
            return self.__equipment_altitude

        elif parameter_name == 'work_limit':
            return self.__work_limit

        elif parameter_name == 'work_area_number':
            return self.__work_area_number

        elif parameter_name == 'admin_phone_number':
            return self.__admin_phone_number


class JoystickMsgParser:
    def __init__(self):
        self.__left_x_position: float = 0.0
        self.__left_y_position: float = 0.0
        self.__right_x_position: float = 0.0
        self.__right_y_position: float = 0.0

        self.__left_joystick_can_id = 0x18FF6381
        self.__right_joystick_can_id = 0x18FF6382

    def get_joystick_values(self, packet: can.Message):
        if packet.arbitration_id == self.__left_joystick_can_id:
            self.__left_x_position = packet.data[3]
            self.__left_y_position = packet.data[1]

        if packet.arbitration_id == self.__right_joystick_can_id:
            self.__right_x_position = packet.data[3]
            self.__right_y_position = packet.data[1]

    def read_joystick_value(self, parameter_name='') -> float:
        if parameter_name == 'left_x_position':
            return self.__left_x_position

        elif parameter_name == 'left_y_position':
            return self.__left_y_position

        elif parameter_name == 'right_x_position':
            return self.__right_x_position

        elif parameter_name == 'right_y_position':
            return self.__right_y_position


class MotorCanMsgParser:
    def __init__(self, dbc_file_path: str = ''):
        self.__motor_can_db = cantools.db.load_file(dbc_file_path)
        self.__motor_msg_set = []

        for msg_set_name in ['motor_1', 'motor_2', 'motor_3', 'motor_4']:
            self.__motor_msg_set.append(self.__motor_can_db.get_message_by_name(msg_set_name))

    def create_motor_can_msg(self, motor_index: int = 0, motor_state: int = 0) -> can.Message:
        can_msg = can.Message(arbitration_id=self.__motor_msg_set[motor_index].frame_id,
                              data=self.__motor_msg_set[motor_index].encode({'motor_' + str(motor_index): motor_state}),
                              is_extended_id=True)
        return can_msg


class InclinometerSensor:
    def __init__(self, arbitration_id: int = 0):
        self.__arbitration_id: Final[int] = arbitration_id
        self.__x_axis: float = 0
        self.__y_axis: float = 0
        self.__z_axis: float = 0
        self.__temp: float = 0

    def get_sensor_values(self, packet: can.Message):
        if packet.arbitration_id == self.__arbitration_id:
            msg_hex = packet.data.hex()

            if len(msg_hex) == 16:
                self.__x_axis = float(int(msg_hex[0:4], base=16)) * 0.01
                self.__y_axis = float(int(msg_hex[4:8], base=16)) * 0.01
                self.__z_axis = float(int(msg_hex[8:12], base=16)) * 0.01
                self.__temp = -273 + (int(msg_hex[12:16], base=16) / 18.9)

    def read_sensor_value(self, parameter_name: str = '') -> float:
        if parameter_name == 'x_axis':
            return self.__x_axis

        elif parameter_name == 'y_axis':
            return self.__y_axis

        elif parameter_name == 'z_axis':
            return self.__z_axis

        elif parameter_name == 'temp':
            return self.__temp
