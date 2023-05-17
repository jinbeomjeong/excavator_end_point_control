import time
import numpy as np
from scipy.optimize import curve_fit


np.set_printoptions(precision=3, suppress=True)


i: int = 0
t0: float = 0.0
t1: float = 0.0
signal_t1: float = 0.0
threshold_signal_prv: float = 0.0
ttc: float = 0.0
control_state_t1: int = 0
result_control_state: int = 0


def predict_control_ttc(input_signal_1: float = 0, input_signal_2: float = 0, time_offset: float = 0.5,
                        threshold_signal: float = 0) -> (int, float):
    
    global t1, signal_t1, ttc

    t0 = time.time()
    signal_t0 = input_signal_1

    derivative_time = t0 - t1
    derivative_signal = signal_t0-signal_t1
    
    vel_sig = derivative_signal/derivative_time
    target_dis = input_signal_1-threshold_signal
    
    if input_signal_2 <= threshold_signal+0.3:
        if (target_dis < 0) or (input_signal_2 <= threshold_signal):
            control_state = 1
        
        else:
            if abs(vel_sig) > 0:
                ttc = target_dis / abs(vel_sig)
            else:
                ttc = 100
        
            if ttc < time_offset:
                control_state = 1
            else: 
                control_state = 0
    else:
        control_state = 0
 
    signal_t1 = signal_t0
    t1 = t0
    
    return control_state, ttc


kernel: np.ndarray
time_scale: np.ndarray


def predict_model(input_value, a, b, c):
    return (a*(input_value**2))+(b*x)+c


def predict_control_quadratic(input_signal: float = 0, time_offset: float = 0.5,
                              threshold_signal: float = 0, kernel_size: int = 5) -> (int, float):

    global i, kernel, time_scale, t0, t1

    if i == 0:
        kernel = np.zeros(kernel_size, dtype=np.float32)
        time_scale = np.linspace(0, kernel_size, kernel_size, endpoint=False)

    t0 = time.time()
    dt = t0-t1
    time_arr = time_scale * dt

    kernel = np.delete(kernel, kernel_size-1)
    kernel = np.insert(kernel, 0, input_signal)
    cal_kernel = np.flip(kernel)
    p_opt = curve_fit(f=predict_model, xdata=time_arr, ydata=cal_kernel, method='dogbox')

    time_offset = time_arr[kernel_size-1]+time_offset
    predict_signal = (p_opt[0][0]*(time_offset**2))+(p_opt[0][1]*time_offset)+p_opt[0][2]

    control_state = 1 if predict_signal <= threshold_signal else 0
    
    '''
    predict_signal = np.sum(np.multiply(p_opt[0][0], np.power(time_offset, 2)),
                            np.multiply(p_opt[0][1], time_offset),
                            p_opt[0][2])
    '''

    i += 1
    t1 = t0

    return control_state, predict_signal


def predict_control_linear(input_signal: float = 0.0, time_offset: float = 0.5, threshold_signal: float = 0.0)\
        -> (int, float):
    
    global t0, t1, signal_t1, control_state_t1, result_control_state

    t0 = time.time()
    signal_t0 = input_signal

    derivative_time = t0-t1
    derivative_signal = signal_t0-signal_t1
    vel_signal = derivative_signal/derivative_time
    
    predict_signal = input_signal+(vel_signal*time_offset)
    
    if input_signal <= threshold_signal+0.2:
        if predict_signal <= threshold_signal-0.1 or input_signal <= threshold_signal:
            control_state = 1
        else:
            control_state = 0

    else:
        control_state = 0
        
    signal_t1 = signal_t0
    t1 = t0
    
    if control_state == 1 and control_state_t1 == 1:
        result_control_state = 1
    else: 
        result_control_state = 0
        
    control_state_t1 = control_state
    
    return control_state, predict_signal
