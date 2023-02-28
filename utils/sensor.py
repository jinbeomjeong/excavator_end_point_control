
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
