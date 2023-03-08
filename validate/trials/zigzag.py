import math
import numpy as np
from .utils import getAISReportingIntervalBySpeed, getAISDataByDist


def zigZagTrial():
    trialTime = 60
    trialDistance = 100

    X = np.linspace(0, trialDistance, 1000)  # in meters
    Y = []
    for x in X:
        Y.append(math.sin(x / 15) * 20)

    trialTotalDistance = 0
    for i in range(len(X) - 1):
        fragDist = math.sqrt((Y[i + 1] - Y[i]) ** 2 + (X[i + 1] - X[i]) ** 2)
        trialTotalDistance += fragDist

    speed = trialTotalDistance / trialTime
    speedKnots = speed * 1.94384
    print("Maneuvering Trial: Zig Zag")
    print("Trial Total Distance:", trialTotalDistance, "m")
    print("Trial Time:", trialTime, "s")
    print("Trial Speed:", speedKnots, "knots")
    print("Reporting Interval:", getAISReportingIntervalBySpeed(speedKnots))

    aisT = np.arange(0, trialTime, getAISReportingIntervalBySpeed(
        speedKnots))  # in seconds
    aisX = []  # in meters
    aisY = []  # in meters
    aisC = []  # in degrees
    aisS = np.repeat(speed, len(aisT))  # in ms-1

    for reportingTime in aisT:
        tX, tY, tH = getAISDataByDist(X, Y, reportingTime * speed)
        aisX.append(tX)
        aisY.append(tY)
        aisC.append(tH)

    aisData = {
        "time": aisT,
        "x": aisX,
        "y": aisY,
        "course": aisC,
        "speed": aisS,
        "duration": trialTime,
    }

    return X, Y, aisData
