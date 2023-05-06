import math
import copy
import numpy as np
from .utils import computeFossenChi, wrapToPi, fixCourse, rad2course
from .utils import calcTrajectoryError


def extended_kalman(aisData):
    # First make a copy of AIS data since we are modifying it
    aisData = copy.deepcopy(aisData)
    estFreq = 50  # in Hertz
    h = 1 / estFreq
    T = np.arange(0, aisData["duration"], h)
    aX = []
    aY = []
    aT = []
    aE = []
    aC = []
    k = 0
    aisData["course"] = computeFossenChi(aisData)  # in Radians
    # aisData["course"] = fixCourse(aisData)  # in Radians
    # Initialization of EKF: X = [x y U chi]
    Q = np.diag([0.01, 0.01, 0.1, 0.1])
    R = np.diag([0.001, 0.001, 0.001, 0.01])
    X_prd = np.matrix(
        [aisData["x"][0], aisData["y"][0], aisData["speed"][0], aisData["course"][0]]
    ).T
    P_prd = 0.1 * np.eye(4)

    for t in T:

        # Corrector with K=0 (no update)
        X_hat = X_prd
        P_hat = P_prd

        # Measurements
        if t >= aisData["time"][k]:
            if k >= len(aisData["time"]) - 1:
                continue
            calcTrajectoryError(aE, aX, aY, aisData["x"][k], aisData["y"][k])
            x_k = aisData["x"][k]
            y_k = aisData["y"][k]
            U_k = aisData["speed"][k]
            chi_k = aisData["course"][k]

            z_k = np.matrix([x_k, y_k, U_k, chi_k]).T
            eps = z_k - X_prd
            eps.itemset(3, wrapToPi(eps.item(3)))

            # Corrector
            K = P_prd * np.matrix(P_prd + R, dtype="float").I

            X_hat = X_prd + K * eps
            P_hat = (np.eye(4) - K) * P_prd * (np.eye(4) - K).T + K * R * K.T

            k += 1

        # Store simulation data: only x & y for plotting
        aX.append(X_prd.item(0))
        aY.append(X_prd.item(1))
        aC.append(rad2course(math.pi - X_prd.item(3)))
        aT.append(t)

        # Predictor (k+1)
        f_hat = np.matrix(
            [
                X_hat.item(2) * math.cos(X_hat.item(3)),
                X_hat.item(2) * math.sin(X_hat.item(3)),
                0,
                0,
            ]
        ).T
        A = np.matrix(
            [
                [
                    0,
                    0,
                    math.cos(X_hat.item(3)),
                    -X_hat.item(2) * math.sin(X_hat.item(3)),
                ],
                [
                    0,
                    0,
                    math.sin(X_hat.item(3)),
                    X_hat.item(2) * math.cos(X_hat.item(3)),
                ],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
            ]
        )
        PHI = np.eye(4) + A * h
        X_prd = X_hat + h * f_hat
        P_prd = PHI * P_hat * PHI.T + Q

    return [aX, aY, aC, aT, aE]
