import numpy as np
from .utils import getAISReportingIntervalBySpeed, getAISDataByDist


def circleTrial():
    trialTime = 60
    trialRadius = 100

    trialTotalDistance = 2 * np.pi * trialRadius
    speed = trialTotalDistance / trialTime
    speedKnots = speed * 1.94384
    reportingInterval = getAISReportingIntervalBySpeed(speedKnots)

    print("Maneuvering Trial: Circle")
    print("Trial Total Distance:", trialTotalDistance, "m")
    print("Trial Time:", trialTime, "s")
    print("Trial Speed:", speedKnots, "knots")
    print("Reporting Interval:", reportingInterval, "s")

    theta = np.linspace(0, 2 * np.pi, 1000)
    X = trialRadius * np.cos(theta)
    Y = trialRadius * np.sin(theta)

    aisT = np.arange(0, trialTime, reportingInterval)  # in seconds
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
