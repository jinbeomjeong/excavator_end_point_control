import numpy as np
from typing import Tuple


def calculation_heading(gps_x_1, gps_x_2, gps_y_1, gps_y_2) -> float:  #GPS 2개 안테나를 이용한 Heading(yaw) 각도 산출
    theta = np.arctan2(gps_y_2-gps_y_1, gps_x_2-gps_x_1)
    theta = np.rad2deg(theta)+180

    return theta


def est_gps_center(gps_1_coord: np.ndarray, gps_2_coord: np.ndarray, offset: float) -> Tuple[float, float]:
    # gps coord: ndarray of len 2
    diff_pos = (gps_1_coord-gps_2_coord)*-1
    slope = np.arctan2(diff_pos[0], diff_pos[1])
    est_latitude = gps_2_coord[0]-(np.sin(slope)*offset)
    est_longitude = gps_2_coord[1]-(np.cos(slope)*offset)

    return est_latitude, est_longitude
