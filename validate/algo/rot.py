import math
import numpy as np


def rate_turn(aisData):
    estFreq = 60  # in Hertz
    h = 1 / estFreq
    aT = np.arange(0, aisData["duration"], h)
    aX = []
    aY = []
    k = 0
    lSpeed = 0  # last known avail. speed
    lCourse = 0  # last known avail. course
    lRateOfTurn = 0
    tSinceL = 0  # time since last known
    firstAIS = True
    for deltaAT in aT:
        if deltaAT >= aisData["time"][k]:
            tDelta = tSinceL
            tSinceL = 0
            pCourse = lCourse  # Previous course
            aX.append(aisData["x"][k])
            aY.append(aisData["y"][k])
            # print(aisData["x"][k], aisData["y"][k])
            lSpeed = aisData["speed"][k]
            lCourse = aisData["course"][k]
            if not firstAIS:
                lRateOfTurn = (lCourse - pCourse) / tDelta
            if firstAIS:
                firstAIS = False
            if k < len(aisData["time"]) - 1:
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
        tSinceL += h
    return [aX, aY]
