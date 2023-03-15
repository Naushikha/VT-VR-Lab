import math
import copy
import numpy as np

from .libUKF import UKF
from .utils import course2Rad


def iterate_x(x_in, timestep):
    """this function is based on the x_dot and can be nonlinear as needed"""
    ret = np.empty_like(x_in)
    ret[0:1, :] = x_in[0:1, :] + timestep * x_in[3:4, :] * np.sin(x_in[2:3, :])
    ret[1:2, :] = x_in[1:2, :] + timestep * x_in[3:4, :] * np.cos(x_in[2:3, :])
    ret[2:3, :] = x_in[2:3, :] + timestep * x_in[4:5, :]
    ret[3:4, :] = x_in[3:4, :] + timestep * x_in[5:6, :]
    ret[4:5, :] = x_in[4:5, :]
    ret[5:6, :] = x_in[5:6, :]
    return ret


def unscented_kalman(aisData):
    aisData = copy.deepcopy(aisData)
    aisData["course"] = course2Rad(aisData)  # in Radians
    estFreq = 60  # in Hertz
    h = 1 / estFreq
    T = np.arange(0, aisData["duration"], 1 / estFreq)
    aX = []
    aY = []
    aT = []
    k = 0
    # x, y, course, speed, yaw rate , acceleration
    q = np.diag([0.01, 0.01, 0.1, 0.1, 0, 0])
    # yaw rate, acceleration, course, speed
    r_matrix = np.diag([1, 1, 0.001, 0.01])
    state_estimator = UKF(
        6, q, np.zeros((6, 1)), 0.0001 * np.eye(6), 0.04, 0.0, 2.0, iterate_x
    )
    state_estimator.modStates(
        [
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
        ]
    )

    for t in T:
        if t >= aisData["time"][k]:
            measure = np.array(
                [0, 0, aisData["course"][k], aisData["speed"][k]]
            ).reshape(-1, 1)
            if k < len(aisData["time"]) - 1:
                k += 1
            else:
                continue
        state_estimator.predict(h)
        state_estimator.update(measure, r_matrix)
        estiList = state_estimator.get_state()
        aX.append(estiList[0][0])
        aY.append(estiList[1][0])
        aT.append(t)
    return [aX, aY, aT]
