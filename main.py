import time, threading, can
import numpy as np
from pandas import DataFrame
from utils.predcit_api import predict_control_quadratic, predict_control_linear
from utils.end_point_kinematics import end_point_kinematics, end_point_kinematics_test, Link_Calculator
from utils.can_msg_parser import SafetyControlMsgParser, InclinometerSensor, JoystickMsgParser
from utils.tcp_lib import TCPClient


can_ch1 = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)
#can_ch2 = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS2', bitrate=250000)

boom_sensor = InclinometerSensor(arbitration_id=0x10FF5383)
arm_sensor = InclinometerSensor(arbitration_id=0x10FF5382)
bucket_sensor = InclinometerSensor(arbitration_id=0x10FF5381)
# body_sensor = InclinometerSensor(arbitration_id=0x10FF5380)

joystick_status = JoystickMsgParser()

safety_control_status = SafetyControlMsgParser()


def ic_controller_can():
    for message in can_ch1:
        boom_sensor.get_sensor_values(packet=message)
        arm_sensor.get_sensor_values(packet=message)
        bucket_sensor.get_sensor_values(packet=message)


ic_controller_thread = threading.Thread(target=ic_controller_can)
ic_controller_thread.daemon = True
ic_controller_thread.start()

        
def main() -> None:
    print('program is running...')
    boom_pitch_pred: float = 0.0
    arm_pitch_pred: float = 0.0
    bucket_pitch_pred: float = 0.0
    height_pred:flaot = 0.0
    
    control_state: int = 0
    threshold_height: float = 1.15 #1.15
    t0: float = 0.0
    el_time:float = 0.0
    
    elapsed_time:float = 0 
    bucket_pos: np.ndarray = np.zeros(3, dtype=np.float32)  # x-axis, y-axis, z-axis
    body_pitch: float = 1.2
    
    height_kernel = np.zeros(5, dtype=np.float32)

    #tcp_handle = TCPClient(address='192.168.137.1', port=6340)
    #tcp_handle.connect_to_server()
    
    class logging_file: 
        def __init__(self, logging_header, file_name='logging_data'): 
            start_time = time.strftime('%Y%m%d-%H%M%S', time.localtime(time.time())) 
            logging_file_name = file_name + '_' + start_time 
            self.logging_file_path = './logging_data/' + logging_file_name + '.csv' 
            logging_header.to_csv(self.logging_file_path, mode='a', header=True) 

        def start_logging(self, period=0.1): 
            #global boom_pitch_pred, arm_pitch_pred, bucket_pitch_pred, bucket_pos, control_state
            
            logging_data = DataFrame({'1': round(elapsed_time, 2), '2': round(boom_pitch_pred, 1),
                                      '3': round(arm_pitch_pred, 1), '4': round(bucket_pitch_pred, 1),
                                      '5': round(bucket_pos[2], 3), '6': control_state}, index=[0])
        
            logging_data.to_csv(self.logging_file_path, mode='a', header=False) 
            logging_thread = threading.Timer(period, self.start_logging, (period, )) 
            logging_thread.daemon = True 
            logging_thread.start() 
    
    logging_header = DataFrame(columns=['time(sec)', 'boom(deg)', 'arm(deg)', 'bucket(deg)', 'height(m)', 'control_sate'])
    logging_task = logging_file(logging_header, file_name='Logging_Data')
    logging_task.start_logging(period=0.1)

    while True:
        boom_pitch = boom_sensor.read_sensor_value('y_axis')
        arm_pitch = arm_sensor.read_sensor_value('y_axis')
        bucket_pitch = bucket_sensor.read_sensor_value('x_axis')

        # angle offset        
        boom_pitch = (boom_pitch-276.3)
        arm_pitch = (arm_pitch-270.0)
        bucket_pitch = (bucket_pitch-7.1)
        
        boom_pitch_pred = (0.5*boom_pitch_pred)+(0.5*boom_pitch)
        arm_pitch_pred = (0.5*arm_pitch_pred)+(0.5*arm_pitch)
        bucket_pitch_pred = (0.5*bucket_pitch_pred)+(0.5*bucket_pitch)
        height_pred = (0.5*height_pred)+(0.5*bucket_pos[2])

        #a_1, a_2 = Link_Calculator(bucket_pitch_pred)
        
        bucket_pos[0], bucket_pos[2] = end_point_kinematics_test(boom_pitch_pred, arm_pitch_pred, bucket_pitch_pred) 
        
        '''
        height_kernel = np.delete(height_kernel, 5-1)
        height_kernel = np.insert(height_kernel, 0, bucket_pos[2])
        height_cal_kernel = np.flip(height_kernel)
        height_median = np.median(height_cal_kernel)/1000
        '''
        
        for i, data in enumerate(bucket_pos):
            bucket_pos[i] = bucket_pos[i] / 1000.0
        
        # height_offset
        bucket_pos[2] = (bucket_pos[2]*0.9806)+0.0542
        bucket_pos[2] = (bucket_pos[2]*1.013)+0.0219


        if control_state == 0:
            control_state, predict_signal = predict_control_linear(input_signal=bucket_pos[2], time_offset=0.3, threshold_signal=threshold_height)   
            
            '''
            control_state, predict_signal = predict_control_quadratic(input_signal=bucket_pos[2], time_offset=0.3, threshold_signal=threshold_height, kernel_size=5)   
            '''
        if control_state == 1:            
            if bucket_pos[2] >= threshold_height*1.3:
                control_state = 0
            else: 
                control_state = 1
                
        print(f'{boom_pitch:.1f}, {arm_pitch:.1f}, {bucket_pitch:.1f}, {bucket_pos[2]:.3f}, {predict_signal:.3f}, {control_state}')
            
        bucket_pos = np.clip(bucket_pos, -54.0, +54.0)
        
        sc_msg = safety_control_status.create_estimated_position_can_msg(control_flag=control_state*2, state_flag=control_state*2,
                                                                         x_pos=bucket_pos[0], y_pos=bucket_pos[1], z_pos=bucket_pos[2])
        can_ch1.send(sc_msg)
        # tcp_handle.send_msg(str(boom_pitch_pred)+','+str(arm_pitch_pred)+','+str(bucket_pitch_pred)
                            #+','+str(bucket_pos[2])+','+str(con))
        
        elapsed_time = time.time()-t0
        time.sleep(0.05)


if __name__ == '__main__':
    main()
