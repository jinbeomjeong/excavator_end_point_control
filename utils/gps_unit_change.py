import math

m_arScaleFactor = 1
m_arLonCenter = 2.21661859489632
m_arLatCenter = 0.663225115757845
m_arFalseNorthing = 500000.0
m_arFalseEasting = 200000.0

m_arMajor = 6378137.0
m_arMinor = 6356752.3142
temp = m_arMinor / m_arMajor

m_dSrcEs = 1.0 - temp * temp
m_dDstEs = 1.0 - temp * temp
m_dDstEsp = m_dDstEs / (1.0 - m_dDstEs)
m_dDstE0 = 1.0 - 0.25 * m_dDstEs * (1.0 + m_dDstEs / 16.0 * (3.0 + 1.25 * m_dDstEs))
m_dDstE1 = 0.375 * m_dDstEs * (1.0 + 0.25 * m_dDstEs * (1.0 + 0.46875 * m_dDstEs))
m_dDstE2 = 0.05859375 * m_dDstEs * m_dDstEs * (1.0 + 0.75 * m_dDstEs)
m_dDstE3 = m_dDstEs * m_dDstEs * m_dDstEs * (35.0 / 3072.0)
m_dDstMl0 = m_arMajor * (m_dDstE0 * m_arLatCenter - m_dDstE1 * math.sin(2.0 * m_arLatCenter) + m_dDstE2 * math.sin(4.0 * m_arLatCenter) - m_dDstE3 * math.sin(6.0 * m_arLatCenter))

m_dDstInd = 0.0

def gps_unit_change(E_d, N_d):

    lon = E_d * math.pi / 180
    lat = N_d * math.pi / 180


    delta_lon = lon - m_arLonCenter
    sin_phi = math.sin(lat)
    cos_phi = math.cos(lat)

    al = cos_phi * delta_lon
    als = al * al
    c = m_dDstEsp * cos_phi * cos_phi
    tq = math.tan(lat)
    t = tq * tq
    con = 1.0 - m_dDstEs * sin_phi * sin_phi
    n = m_arMajor / math.sqrt(con)
    ml = m_arMajor * (m_dDstE0 * lat - m_dDstE1 * math.sin(2.0 * lat) + m_dDstE2 * math.sin(4.0 * lat) - m_dDstE3 * math.sin(6.0 * lat))

    TM_x = m_arScaleFactor * n * al * (1.0 + als / 6.0 * (1.0 - t + c + als / 20.0 * (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * m_dDstEsp))) + m_arFalseEasting
    TM_y = m_arScaleFactor * (ml - m_dDstMl0 + n * tq * (als * (0.5 + als / 24.0 * (5.0 - t + 9.0 * c + 4.0 * c * c + als / 30.0 * (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * m_dDstEsp))))) + m_arFalseNorthing

    return TM_x, TM_y
