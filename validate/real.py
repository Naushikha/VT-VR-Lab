import time
import multiprocessing
import numpy as np
import matplotlib.pyplot as plt
from algo.dr import dead_reckoning
from algo.ekf import extended_kalman
from algo.xkf import exogenous_kalman
from algo.ukf import unscented_kalman
from algo.pvb import projective_velocity_blending
from algo.own import own_algo
from algo.rot import rate_turn

aisT = []  # in seconds
aisX = []  # in meters
aisY = []  # in meters
aisS = []  # in ms-1
aisC = []  # in degrees

trial = open("real/test2.csv", "r")
tZero = None
for line in trial:
    epoch, mmsi, x, y, speed, course = line.strip().split(",")
    if tZero is None:
        tZero = int(epoch)
        aisT.append(0)
    else:
        aisT.append(int(epoch) - tZero)
    aisX.append(float(x))
    aisY.append(float(y))
    aisS.append(float(speed) / 1.94384)
    aisC.append(float(course))

estFreq = 60  # Hertz
trialTime = aisT[-1]
T = np.linspace(0, trialTime, trialTime * estFreq)

aisData = {
    "time": aisT,
    "x": aisX,
    "y": aisY,
    "course": aisC,
    "speed": aisS,
    "duration": aisT[-1] - aisT[0],
}

fig, ax = plt.subplots(figsize=(14, 8))
graphList = []
algoList = []

ttaT = []
ttaX = []
ttaY = []


def task(Ti):
    realX = np.interp(Ti, aisT, aisX)
    realY = np.interp(Ti, aisT, aisY)
    algoX = np.interp(Ti, ttaT, ttaX)
    algoY = np.interp(Ti, ttaT, ttaY)
    XsumOfDiff = abs(algoX - realX)
    YsumOfDiff = abs(algoY - realY)
    return [XsumOfDiff, YsumOfDiff]


def calcAccuracy(algo, aX, aY, aT):
    global ttaT, ttaX, ttaY
    pool = multiprocessing.Pool()
    # https://stackoverflow.com/a/6723457
    XsumOfDiff = 0
    YsumOfDiff = 0
    # for i in range(len(T)):
    #     realX = np.interp(T[i], aisT, aisX)
    #     realY = np.interp(T[i], aisT, aisY)
    #     algoX = np.interp(T[i], aT, aX)
    #     algoY = np.interp(T[i], aT, aY)
    #     XsumOfDiff += abs(algoX - realX)
    #     YsumOfDiff += abs(algoY - realY)
    ttaT = aT
    ttaX = aX
    ttaY = aY
    with multiprocessing.Pool() as pool:
        # call the function for each item in parallel
        for result in pool.map(task, aisT):
            XsumOfDiff += result[0]
            YsumOfDiff += result[1]
    XmeanDiff = XsumOfDiff / len(aisT)
    YmeanDiff = YsumOfDiff / len(aisT)
    meanDiff = (XmeanDiff + YmeanDiff) / 2
    print(algo, " : Mean Difference : ", meanDiff)
    print(algo, " : X sum : ", XsumOfDiff)
    print(algo, " : Y sum : ", YsumOfDiff)
    print(algo, " : T : ", len(aisT))
    pool.close()


def plot_algo(algo="DR"):
    startTime = time.time()
    if algo == "DR":
        aX, aY, aT = dead_reckoning(aisData)
        algoList.append("Dead Reckoning")
    if algo == "EKF":
        aX, aY, aT = extended_kalman(aisData)
        algoList.append("Extended Kalman Filter")
    if algo == "XKF":
        aX, aY, aT = exogenous_kalman(aisData)
        algoList.append("Exogenous Kalman Filter")
    if algo == "UKF":
        aX, aY, aT = unscented_kalman(aisData)
        algoList.append("Unscented Kalman Filter")
    if algo == "PVB":
        aX, aY, aT = projective_velocity_blending(aisData)
        algoList.append("Projective Velocity Blending")
    if algo == "OWN":
        aX, aY, aT = own_algo(aisData)
        algoList.append("Own Algo")
    if algo == "ROT":
        aX, aY, aT = rate_turn(aisData)
        algoList.append("DR + Rate of Turn")
    endTime = time.time()
    (tmpGraph,) = ax.plot(aX, aY, "o:", markersize=1)
    graphList.append(tmpGraph)
    calcAccuracy(algo, aX, aY, aT)
    algoTime = endTime - startTime
    print(algo, " : Processing Time : ", algoTime, "s")


# (tmpGraph,) = ax.plot(X, Y)
# graphList.append(tmpGraph)
(tmpGraph,) = ax.plot(aisData["x"], aisData["y"], "ro-")
graphList.append(tmpGraph)
# plot_algo("DR")
# plot_algo("EKF")
# plot_algo("XKF")
# plot_algo("UKF")
# plot_algo("ROT")
# plot_algo("PVB")
plot_algo("OWN")

legendList = ["AIS Report"]
legendList.extend(algoList)

legend = plt.legend(legendList)
plt.title("AIS Vessel Motion Simulator")
plt.xlabel("X-Axis (m)")
plt.ylabel("Y-Axis (m)")

graphLegends = {}

i = 0
for legendEntry in legend.get_lines():  # Skip first two legends
    legendEntry.set_picker(True)
    legendEntry.set_pickradius(10)
    graphLegends[legendEntry] = graphList[i]
    i += 1


def onLegendPick(event):
    legend = event.artist
    isVisible = legend.get_visible()
    graphLegends[legend].set_visible(not isVisible)
    legend.set_visible(not isVisible)
    fig.canvas.draw()


plt.connect("pick_event", onLegendPick)
plt.show()
