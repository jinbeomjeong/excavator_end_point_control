
LPF_Beta = 0.5

def low_pass_filter(m_Boom_Angle, SmoothData):
    RawData = m_Boom_Angle
    NewSmoothData = (1 - LPF_Beta) * SmoothData + LPF_Beta * RawData

    return NewSmoothData