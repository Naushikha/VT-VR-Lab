import math
import numpy as np


def dead_reckoning(aisData):
    estFreq = 60  # in Hertz
    aT = np.arange(0, aisData["duration"], 1 / estFreq)
    aX = []
    aY = []
    k = 0
    lSpeed = 0  # latest avail. speed
    lCourse = 0  # latest avail. course
    for deltaAT in aT:
        if (deltaAT >= aisData["time"][k]):
            aX.append(aisData["x"][k])
            aY.append(aisData["y"][k])
            lSpeed = aisData["speed"][k]
            lCourse = aisData["course"][k]
            if (k < len(aisData["time"])-1):
                k += 1
            else:
                break
        else:
            aX.append(aX[-1] + lSpeed *
                      math.sin(math.radians(lCourse)) * (1 / estFreq))
            aY.append(aY[-1] + lSpeed *
                      math.cos(math.radians(lCourse)) * (1 / estFreq))
    return [aX, aY]
