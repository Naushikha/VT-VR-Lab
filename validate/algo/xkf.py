import math
import copy
import numpy as np
from .utils import computeFossenChi, wrapToPi, fixCourse


def exogenous_kalman(aisData):
    # First make a copy of AIS data since we are modifying it
    aisData = copy.deepcopy(aisData)
    estFreq = 60  # in Hertz
    h = 1 / estFreq
    T = np.arange(0, aisData["duration"], h)
    aX = []
    aY = []
    aT = []
    k = 0
    # aisData["course"] = computeFossenChi(aisData)  # in Radians
    aisData["course"] = fixCourse(aisData)  # in Radians

    # Initialization of kinematic observer: x = [x y U chi]

    x_prd = aisData["x"][0]
    y_prd = aisData["y"][0]
    U_prd = aisData["speed"][0]
    chi_prd = aisData["course"][0]
    a = 0
    r = 0

    K1 = 10
    K2 = 10
    K3 = 30
    K4 = 50

    # Initialization of LTV Kalman filter
    Q = np.diag([1, 1, 10, 10])
    R = np.eye(4)

    P = np.eye(4)
    x_hat = np.matrix([x_prd, y_prd, U_prd, chi_prd]).T

    T_a = 10  # acceleration time constant
    T_r = 50  # yaw rate time constant

    B = np.matrix(
        [
            [0, 0],
            [0, 0],
            [1, 0],
            [0, 1],
        ]
    )

    H = np.eye(4)

    for t in T:

        # Store simulation data: only x & y for plotting
        aX.append(x_hat.item(0))
        aY.append(x_hat.item(1))
        aT.append(t)

        # Measurements
        if t >= aisData["time"][k]:
            if k >= len(aisData["time"]) - 1:
                continue
            x_k = aisData["x"][k]
            y_k = aisData["y"][k]
            U_k = aisData["speed"][k]
            chi_k = aisData["course"][k]

            # estimate of acceleration and yaw rate for sample k > 2
            if k > 2:
                h1 = aisData["time"][k] - aisData["time"][k - 1]
                h2 = aisData["time"][k - 1] - aisData["time"][k - 2]
                alp = ((h1 + h2) / h1) ** 2

                if (
                    h1 + h2
                ) / 2 > 4:  # do not compute a and r if mean sampling time > 4s
                    a_c = 0
                    r_c = 0
                else:
                    a_c = (
                        (1 - alp) * aisData["speed"][k]
                        + alp * aisData["speed"][k - 1]
                        - aisData["speed"][k - 2]
                    ) / ((1 - alp) * h1 + h2)
                    r_c = (
                        (1 - alp) * aisData["course"][k]
                        + alp * aisData["course"][k - 1]
                        - aisData["course"][k - 2]
                    ) / ((1 - alp) * h1 + h2)
            else:  # zero for first two data points
                a_c = 0
                r_c = 0

            # max values (saturation) to avoid estimates using wildpoints
            r_max = np.pi / 180

            if r_c > r_max:
                r_c = r_max
            elif r < -r_max:
                r_c = -r_max

            # max values
            a_max = 1

            if a_c > a_max:
                a_c = a_max
            elif a_c < -a_max:
                a_c = -a_max

            # Corrector Kalman filter (update states if new measurement)
            z_k = np.matrix([x_k, y_k, U_k, chi_k]).T
            eps = z_k - H * x_hat
            eps.itemset(3, wrapToPi(eps.item(3)))
            K = P * H.T * np.matrix(H * P * H.T + R).I
            x_hat = x_hat + K * eps
            P = (np.eye(4) - K * H) * P * (np.eye(4) - K * H).T + K * R * K.T

            # Corrector kinematic observer (update states if new measurement)
            x_prd = x_prd + h * K1 * (x_k - x_prd)
            y_prd = y_prd + h * K2 * (y_k - y_prd)
            U_prd = U_prd + h * K3 * (U_k - U_prd)
            chi_prd = chi_prd + h * K4 * wrapToPi(chi_k - chi_prd)

            k += 1

        # Kalman filter model
        X_prd = np.matrix([x_prd, y_prd, U_prd, chi_prd]).T
        f_prd = np.matrix(
            [
                X_prd.item(2) * math.cos(X_prd.item(3)),
                X_prd.item(2) * math.sin(X_prd.item(3)),
                0,
                0,
            ]
        ).T

        F = np.matrix(
            [
                [
                    0,
                    0,
                    math.cos(X_prd.item(3)),
                    -X_prd.item(2) * math.sin(X_prd.item(3)),
                ],
                [
                    0,
                    0,
                    math.sin(X_prd.item(3)),
                    X_prd.item(2) * math.cos(X_prd.item(3)),
                ],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
            ]
        ).T

        PHI = np.eye(4) + h * F

        # Predictor Kalman filter (k+1)
        x_hat = x_hat + h * (f_prd + F * (x_hat - X_prd) + B * np.matrix([a, r]).T)
        x_hat.itemset(3, wrapToPi(x_hat.item(3)))
        P = PHI * P * PHI.T + Q

        # Predictor kinematic observer (k+1)
        x_prd = x_prd + h * U_k * math.cos(chi_prd)
        y_prd = y_prd + h * U_k * math.sin(chi_prd)
        U_prd = U_prd + h * a
        chi_prd = chi_prd + h * r
        chi_prd = wrapToPi(chi_prd)
        a = a + (a_c - a) / T_a
        r = r + (r_c - r) / T_r

    return [aX, aY, aT]
