import can
import numpy as np
from sklearn.preprocessing import minmax_scale


np.set_printoptions(suppress=True)


class MsgParser:
    def __init__(self):
        self.__automatic_control_recv_frame__ = 0x18FF1000  # IC -> SC
        self.__automatic_control_send_frame__ = 0x18FF1003  # IC -> SC
        self.__under_utils_data_frame__ = 0x18FF1004  # CD -> SC
        self.__equipment_coord_frame_left__ = 0x18FF5010  # CD -> IC/SC
        self.__equipment_coord_frame_right__ = 0x18FF5011  # CD -> IC/SC
        self.__equipment_coord_frame_altitude__ = 0x18FF5012  # CD -> IC/SC

        self.__automatic_control__: str = ''
        self.__under_utils_data_transmit__: str = ''
        self.__under_utils_data_number__: int = 0
        self.__equipment_pos__: np.ndarray = np.zeros(5)  # gps_1_x, gps_1_y, gps_2_x, gps_2_y

    def get_automatic_control(self, arbitration_id: int, payload: []) -> str:
        if arbitration_id == self.__automatic_control_recv_frame__:
            if payload[0] == 0:
                self.__automatic_control__ = 'off'
            elif payload[0] == 1:
                self.__automatic_control__ = 'stand by'
            elif payload[0] == 2:
                self.__automatic_control__ = 'on'

            return self.__automatic_control__

    def get_under_utils_data(self, arbitration_id: int, payload: []):
        if arbitration_id == self.__under_utils_data_frame__:
            if payload[1] == 0:
                self.__under_utils_data_transmit__ = 'none'
            elif payload[1] == 1:
                self.__under_utils_data_transmit__ = 'transmitting'
            elif payload[1] == 2:
                self.__under_utils_data_transmit__ = 'done'

            self.__under_utils_data_number__ = payload[2]

            return self.__under_utils_data_transmit__, self.__under_utils_data_number__

    def get_under_utils_info(self, arbitration_id: int, payload: []):
        __under_utils_coord_id__ = np.zeros(self.__under_utils_data_number__)
        __under_utils_depth_id__ = np.zeros(self.__under_utils_data_number__)
        __under_utils_data_set__ = np.zeros((self.__under_utils_data_number__, 4))

        for i in range(self.__under_utils_data_number__):
            __under_utils_coord_id__[i] = int('0x18FF20'+str(i)+'0')

        for i in range(self.__under_utils_data_number__):
            __under_utils_depth_id__[i] = int('0x18FF201'+str(i))

        if np.any(__under_utils_coord_id__ == arbitration_id):
            __under_utils_data_set__[int(str(arbitration_id)[8]), 0:2] = np.array(int(payload[4:8], base=16),
                                                                                  int(payload[0:4], base=16))

        if np.any(__under_utils_depth_id__ == arbitration_id):
            __under_utils_data_set__[int(str(arbitration_id)[9]), 2:4] = np.array(int(payload[0:2], base=16),
                                                                                  payload[2])

        return __under_utils_data_set__

    def get_equipment_pos(self, arbitration_id: int, payload: []):
        if arbitration_id == self.__equipment_coord_frame_left__:
            self.__equipment_pos__[0:2] = np.array(int(payload[4:8], base=16), int(payload[0:4], base=16))

        if arbitration_id == self.__equipment_coord_frame_right__:
            self.__equipment_pos__[2:4] = np.array(int(payload[4:8], base=16), int(payload[0:4], base=16))

        if arbitration_id == self.__equipment_coord_frame_altitude__:
            self.__equipment_pos__[4] = int(payload[0:2], base=16)

            return self.__equipment_pos__

    def create_estimated_position(self, control_flag: int = 0, state_flag: int = 0, x_pos: float = 0.0,
                                y_pos: float = 0.0, z_pos: float = 0.0):

        position_data = []
        for data in [x_pos, y_pos, z_pos]:
            position_data.append(int(data*100))

        x_pos_msb, x_pos_lsb = position_data[0].to_bytes(2, byteorder='little')
        y_pos_msb, y_pos_lsb = position_data[1].to_bytes(2, byteorder='little')
        z_pos_msb, z_pos_lsb = position_data[2].to_bytes(2, byteorder='little')
        __send_data__ = [control_flag, state_flag, x_pos_msb, x_pos_lsb, y_pos_msb, y_pos_lsb, z_pos_msb, z_pos_lsb]

        send_msg = can.Message(arbitration_id=self.__automatic_control_send_frame__, data=__send_data__,
                               is_extended_id=True)

        return send_msg
