import can, time, threading, os, sys
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from utils.can_msg_parser import MsgParser


np.set_printoptions(suppress=True)
bus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)


def can_com():
    print('waiting for can messages...')
    for msg in bus:
        can_id = msg.arbitration_id
        can_payload = msg.data

        can_msg_parser.get_under_utils_status(can_id, can_payload)
        can_msg_parser.get_under_utils_data(can_id, can_payload)


can_msg_parser = MsgParser()
can_com_task = threading.Thread(target=can_com)
can_com_task.daemon = True
can_com_task.start()

i = 0
while True:
    if input() == 'coord data':
        print(i, can_msg_parser.read_variable('under_utils_coord_data'))

    i += 1
    time.sleep(0.5)

