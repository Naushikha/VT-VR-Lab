import math
import numpy as np
from numpy import testing
from numpy.matrixlib.defmatrix import _convert_from_string


def projective_velocity_blending(aisData):
    estFreq = 60  # in Hertz
    h = 1 / estFreq
    print("h is ", h)
    aT = np.arange(0, aisData["duration"], h)
    aX = []
    aY = []
    posList = []  # all (x, y) positions
    k = 0
    lPos = np.array([0, 0])  # last known position
    lVelocity = np.array([0, 0])  # last known velocity
    lAccel = np.array([0, 0])  # last known acceleration

    oVelocity = np.array([0, 0])  # old velocity
    oPos = np.array([0, 0])  # old position

    bVelocity = np.array([0, 0])  # blended velocity

    tSinceL = 0  # Time since last update
    tDelta = 16  # Time between updates: recommended to keep constant

    for deltaAT in aT:
        if deltaAT >= aisData["time"][k]:
            # for pvb
            oPos = lPos
            oVelocity = lVelocity
            # tDelta = tSinceL
            tSinceL = 0
            lPos = np.array([aisData["x"][k], aisData["y"][k]])
            lCourse = aisData["course"][k]
            lSpeed = aisData["speed"][k]
            lAccelTmp = 0
            lVelocity = np.array([
                lSpeed * math.sin(math.radians(lCourse)), lSpeed * math.cos(math.radians(lCourse))])
            lAccel = np.array([lAccelTmp * math.sin(math.radians(lCourse)),
                               lAccelTmp * math.cos(math.radians(lCourse))])
            # for dr
            posList.append(lPos)
            if k < len(aisData["time"]) - 1:
                k += 1
            else:
                break
        # else:
            # # DR
            # nPos = posList[-1] + lVelocity * h + (1/2 * lAccel * h**2)
            # posList.append(nPos)
            # aX.append(nPos[0])
            # aY.append(nPos[1])
            # ###########################

        tSinceL += h
        print("tsincel is ", tSinceL)

        if (tDelta == 0):  # No updates
            continue

        tHat = tSinceL / tDelta

        # Projective velocity blending
        bVelocity = oVelocity + (lVelocity - oVelocity) * tHat
        posProj = oPos + bVelocity * tSinceL + (1/2 * lAccel * tSinceL**2)
        posLast = lPos + lVelocity * tSinceL + (1/2 * lAccel * tSinceL**2)
        posComb = posProj + (posLast - posProj) * tHat
        # print(posComb)
        posList.append(posComb)
        aX.append(posComb[0])
        aY.append(posComb[1])

    return [aX, aY]
