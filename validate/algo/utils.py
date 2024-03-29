import math
import numpy as np

# Fossen;
# The AIS course angle measurement is often unreliable. In these cases, it is recommended to
# compute the course angle from the North - East positions(x(k-1), y(k-1)) and (x(k), y(k))


def computeFossenChi(aisData):
    prevX = 0
    prevY = 0
    correctedChi = []
    for i in range(len(aisData["time"])):
        newChi = math.atan2((aisData["y"][i] - prevY), (aisData["x"][i] - prevX))
        correctedChi.append(newChi)
        prevX = aisData["x"][i]
        prevY = aisData["y"][i]
    return correctedChi


def wrapToPi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi
    # https://stackoverflow.com/a/15927914


def fixCourse(aisData):
    correctedCourse = []
    for course in aisData["course"]:
        correctedCourse.append(-wrapToPi(math.radians(course) - np.pi / 2))
    return correctedCourse


def course2Rad(aisData):
    correctedCourse = []
    for course in aisData["course"]:
        correctedCourse.append(wrapToPi(math.radians(course)))
    return correctedCourse


def rad2course(a):
    deg = math.degrees(a) % 360
    if deg < 0:
        deg += 360
    return deg


def calcTrajectoryError(errList, aX, aY, aisX, aisY):
    try:
        errX = abs(aX[-1] - aisX)
        errY = abs(aY[-1] - aisY)
        errList.append([errX, errY])
    except:
        pass
