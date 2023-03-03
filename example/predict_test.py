import time
import numpy as np
from utils.predcit_api import predict_control_quadratic
from utils.tcp_lib import TCPClient


tcp_handle = TCPClient(address='localhost', port=6340)
tcp_handle.connect_to_server()

while True:
    signal = np.fromstring(tcp_handle.receive_msg(), dtype=np.float32, sep=' ')[0]
    control_state, predict_signal = predict_control_quadratic(input_signal=signal, threshold_signal=1,
                                                              time_offset=0.3, kernel_size=10)

    tcp_handle.send_msg(str(control_state)+','+str(predict_signal))

