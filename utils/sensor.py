
class InclinometerSensor:
    def __init__(self, arbitration_id):
        self.arbitration_id = arbitration_id
        self.x_axis = 0
        self.y_axis = 0
        self.z_axis = 0
        self.temp = 0

    def payload_paser(self, packet):
        if packet.arbitration_id == self.arbitration_id:
            msg_hex = packet.data.hex()

            if len(msg_hex) == 16:
                self.x_axis = (float(int(msg_hex[0:4], base=16)) / 16384) * 90
                self.y_axis = (float(int(msg_hex[4:8], base=16)) / 16384) * 90
                self.z_axis = (float(int(msg_hex[8:12], base=16)) / 16384) * 90
                self.temp = -273 + (int(msg_hex[12:16], base=16) / 18.9)

        return self.x_axis, self.y_axis, self.z_axis, self.temp
