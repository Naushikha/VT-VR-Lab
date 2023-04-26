import math
import numpy as np
from .utils import getAISReportingIntervalBySpeed, getAISDataByDist


def zigZagTrial():
    trialTime = 60
    trialDistance = 300

    X = np.linspace(0, trialDistance, 3000)  # in meters
    Y = []
    for x in X:
        Y.append(math.sin(x / 45) * 60)

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
    print("Reporting Interval:", getAISReportingIntervalBySpeed(speedKnots), "s")

    aisT = np.arange(
        0, trialTime, getAISReportingIntervalBySpeed(speedKnots)
    )  # in seconds
    aisX = []  # in meters
    aisY = []  # in meters
    aisC = []  # in degrees
    aisS = np.repeat(speed, len(aisT))  # in ms-1

    for reportingTime in aisT:
        tX, tY, tH = getAISDataByDist(X, Y, reportingTime * speed)
        aisX.append(tX)
        aisY.append(tY)
        aisC.append(tH)

    # For accuracy calculations
    estFreq = 60  # Hertz
    nT = np.linspace(0, trialTime, trialTime * estFreq)  # in meters
    nX = []
    nY = []
    for tinyNT in nT:
        nnX, nnY, nnH = getAISDataByDist(X, Y, tinyNT * speed)
        nX.append(nnX)
        nY.append(nnY)


    aisData = {
        "time": aisT,
        "x": aisX,
        "y": aisY,
        "course": aisC,
        "speed": aisS,
        "duration": trialTime,
    }

    return nX, nY, nT, aisData
