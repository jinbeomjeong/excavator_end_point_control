import math
import numpy as np
from typing import Tuple

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


def gps_unit_change(latitude: float, longitude: float) -> Tuple[float, float]:
    lon = np.deg2rad(longitude)
    lat = np.deg2rad(latitude)

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


def calculation_heading(gps_x_1, gps_x_2, gps_y_1, gps_y_2) -> float:  #GPS 2개 안테나를 이용한 Heading(yaw) 각도 산출
    theta = np.arctan2(gps_y_2-gps_y_1, gps_x_2-gps_x_1)
    theta = np.rad2deg(theta)+180

    return theta


def to_cartesian(lat, lon):
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    x = math.cos(lat_rad) * math.cos(lon_rad)
    y = math.cos(lat_rad) * math.sin(lon_rad)
    z = math.sin(lat_rad)

    return x, y, z


def to_geodetic(x, y, z):
    lon_rad = math.atan2(y, x)
    hyp = math.sqrt(x * x + y * y)
    lat_rad = math.atan2(z, hyp)

    lat = math.degrees(lat_rad)
    lon = math.degrees(lon_rad)

    return lat, lon


def find_center(coord1, coord2):
    x1, y1, z1 = to_cartesian(*coord1)
    x2, y2, z2 = to_cartesian(*coord2)

    center_x = (x1 + x2) / 2
    center_y = (y1 + y2) / 2
    center_z = (z1 + z2) / 2

    center = to_geodetic(center_x, center_y, center_z)

    return center


def est_gps_center(gps_1_coord: np.ndarray, gps_2_coord: np.ndarray, offset: float) -> Tuple[float, float]:
    # gps coord: ndarray of len 2
    diff_pos = (gps_1_coord-gps_2_coord)*-1
    slope = np.arctan2(diff_pos[0], diff_pos[1])
    est_latitude = gps_2_coord[0]-(np.sin(slope)*offset)
    est_longitude = gps_2_coord[1]-(np.cos(slope)*offset)

    return est_latitude, est_longitude
