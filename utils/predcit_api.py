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


