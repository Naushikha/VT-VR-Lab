import math
import numpy as np


def dead_reckoning(aisData):
    estFreq = 60  # in Hertz
    T = np.arange(0, aisData["duration"], 1 / estFreq)
    aX = []
    aY = []
    aT = []
    k = 0
    lSpeed = 0  # latest avail. speed
    lCourse = 0  # latest avail. course
    for t in T:
        if t >= aisData["time"][k]:
            if k >= len(aisData["time"]) - 1:
                continue
            aX.append(aisData["x"][k])
            aY.append(aisData["y"][k])
            aT.append(t)
            lSpeed = aisData["speed"][k]
            lCourse = aisData["course"][k]
            k += 1
        else:
            aX.append(aX[-1] + lSpeed * math.sin(math.radians(lCourse)) * (1 / estFreq))
            aY.append(aY[-1] + lSpeed * math.cos(math.radians(lCourse)) * (1 / estFreq))
            aT.append(t)
    return [aX, aY, aT]
