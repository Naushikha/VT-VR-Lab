import math
import numpy as np
from .utils import calcTrajectoryError


def rate_turn(aisData):
    estFreq = 60  # in Hertz
    h = 1 / estFreq
    T = np.arange(0, aisData["duration"], h)
    aX = []
    aY = []
    aT = []
    aE = []
    k = 0
    lSpeed = 0  # last known avail. speed
    lCourse = 0  # last known avail. course
    lRateOfTurn = 0
    tSinceL = 0  # time since last known
    firstAIS = True
    for t in T:
        if t >= aisData["time"][k]:
            if k >= len(aisData["time"]) - 1:
                continue
            calcTrajectoryError(aE, aX, aY, aisData["x"][k], aisData["y"][k])
            tDelta = tSinceL
            tSinceL = 0
            pCourse = lCourse  # Previous course
            aX.append(aisData["x"][k])
            aY.append(aisData["y"][k])
            aT.append(t)
            # print(aisData["x"][k], aisData["y"][k])
            lSpeed = aisData["speed"][k]
            lCourse = aisData["course"][k]
            if not firstAIS:
                try:
                    lRateOfTurn = (lCourse - pCourse) / tDelta
                except:
                    lRateOfTurn = 0
            if firstAIS:
                firstAIS = False
            k += 1
        # print(tSinceL)
        aX.append(
            aX[-1]
            + lSpeed * math.sin(math.radians(lCourse + lRateOfTurn * tSinceL)) * h
        )
        aY.append(
            aY[-1]
            + lSpeed * math.cos(math.radians(lCourse + lRateOfTurn * tSinceL)) * h
        )
        aT.append(t)
        tSinceL += h
    return [aX, aY, aT, aE]
