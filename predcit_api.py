import time

t1 = 0
signal_t1 = 0


def predict_control(input_signal=0, time_offset=0.5, threshold_signal=0):
    t0 = time.time()
    signal_t0 = input_signal
    global t1
    global signal_t1

    derivative_time = t0 - t1
    derivative_signal = signal_t0-signal_t1

    predict_signal = (derivative_time / time_offset * derivative_signal) + input_signal

    control_state = 1 if predict_signal <= threshold_signal else 0

    signal_t1 = signal_t0
    t1 = t0

    return control_state


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
