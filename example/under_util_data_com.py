import time, sys, os, can

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from utils.can_msg_parser import SafetyControlMsgParser

can_ch1 = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)
safety_control_msg = SafetyControlMsgParser()

request_msg = safety_control_msg.create_request_under_utils(request_msg=1)
can_ch1.send(request_msg)

for msg in can_ch1:
    if msg.arbitration_id == 0x18FF1004:
        safety_control_msg.get_under_utils_status(msg)
        break

request_msg = safety_control_msg.create_request_under_utils(request_msg=2)
can_ch1.send(request_msg)

i = 0
for msg in can_ch1:
    safety_control_msg.get_under_utils_status(packet=msg)
    safety_control_msg.get_under_utils_data(packet=msg)

    if safety_control_msg.read_variable('under_utils_data_transmit') == 'done': break

id_hex = []
id_arr = safety_control_msg.read_variable('under_utils_coord_id')

for idx in id_arr:
    id_hex.append(hex(idx))

print(id_hex)
depth_arr = safety_control_msg.read_variable('under_utils_coord_data')
print(depth_arr.shape)
print(depth_arr)
