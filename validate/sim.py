import time
import numpy as np
import matplotlib.pyplot as plt
from algo.dr import dead_reckoning
from algo.ekf import extended_kalman
from algo.xkf import exogenous_kalman
from algo.ukf import unscented_kalman
from algo.pvb import projective_velocity_blending
from algo.own import own_algo
from algo.rot import rate_turn

from trials.zigzag import zigZagTrial
from trials.circle import circleTrial

X, Y, T, aisData = zigZagTrial()
# X, Y, T, aisData = circleTrial()

fig, ax = plt.subplots(figsize=(14, 8))
graphList = []
algoList = []


def calcAccuracy(algo, aX, aY, aT):
    # https://stackoverflow.com/a/6723457
    XsumOfSq = 0
    XsumOfDiff = 0
    YsumOfSq = 0
    YsumOfDiff = 0
    aE = []
    for i in range(len(T)):
        algoX = np.interp(T[i], aT, aX)
        algoY = np.interp(T[i], aT, aY)
        XsumOfSq += (algoX - X[i]) ** 2
        YsumOfSq += (algoY - Y[i]) ** 2
        XsumOfDiff += abs(algoX - X[i])
        YsumOfDiff += abs(algoY - Y[i])
        aE.append((abs(algoX - X[i]) + abs(algoY - Y[i])) / 2)
    # print(algo, " : Sum of Squares : ", XsumOfSq)
    # print(algo, " : Sum of Differences : ", XsumOfDiff)
    ##########
    plt.figure(3)
    x = np.arange(len(aE))
    plt.plot(x, aE, label=algo)
    plt.legend(algoList)
    plt.figure(1)
    XmeanDiff = XsumOfDiff / len(T)
    YmeanDiff = YsumOfDiff / len(T)
    meanDiff = (XmeanDiff + YmeanDiff) / 2
    # print(algo, " : Mean Difference : ", meanDiff)
    print(meanDiff, end=",")
    # print(algo, " : X sum : ", XsumOfDiff)
    # print(algo, " : Y sum : ", YsumOfDiff)
    # print(algo, " : T : ", len(T))


def calcTrajectoryAccuracy(algo, aE):
    XsumOfDiff = 0
    YsumOfDiff = 0
    for e in aE:
        XsumOfDiff += e[0]
        YsumOfDiff += e[1]
    XmeanDiff = XsumOfDiff / len(aE)
    YmeanDiff = YsumOfDiff / len(aE)
    meanDiff = (XmeanDiff + YmeanDiff) / 2
    # print(algo, " : Mean Trajectory Difference : ", meanDiff)
    print(meanDiff, end=",")


def plot_algo(algo="DR", config=[]):
    startTime = time.time()
    if algo == "DR":
        aX, aY, aT, aE = dead_reckoning(aisData)
        algoList.append("Dead Reckoning")
    if algo == "EKF":
        aX, aY, aT, aE = extended_kalman(aisData)
        algoList.append("Extended Kalman Filter")
    if algo == "XKF":
        aX, aY, aT, aE = exogenous_kalman(aisData)
        algoList.append("Exogenous Kalman Filter")
    if algo == "UKF":
        aX, aY, aT, aE = unscented_kalman(aisData)
        algoList.append("Unscented Kalman Filter")
    if algo == "PVB":
        aX, aY, aT, aE = projective_velocity_blending(aisData)
        algoList.append("Projective Velocity Blending")
    if algo == "OWN":
        algo = f"AGB-{config[0]},{round(config[1], 2)}"
        aX, aY, aT, aE = own_algo(aisData, config)
        algoList.append(algo)
    if algo == "ROT":
        aX, aY, aT, aE = rate_turn(aisData)
        algoList.append("DR + Rate of Turn")
    endTime = time.time()
    print(algo, end=",")
    (tmpGraph,) = ax.plot(aX, aY, "o:", markersize=1)
    graphList.append(tmpGraph)
    calcAccuracy(algo, aX, aY, aT)
    calcTrajectoryAccuracy(algo, aE)
    algoTime = endTime - startTime
    # print(algo, " : Processing Time : ", algoTime, "s")
    print(algoTime)


(tmpGraph,) = ax.plot(X, Y)
graphList.append(tmpGraph)
(tmpGraph,) = ax.plot(aisData["x"], aisData["y"], "ro")
graphList.append(tmpGraph)
plot_algo("DR")
# plot_algo("EKF")
# plot_algo("XKF")
# plot_algo("UKF")
plot_algo("ROT")
# plot_algo("PVB")
interpolationTypeList = [
    "P1",
    "P2_Rot",
    "P2_Quad",
    "P2_Cubic",
    "P3_Quad",
    "P3_QuadCT",
    "P4_Cubic",
]
blendingPercentageList = np.arange(0, 1.01, 0.05)
for interpolationType in interpolationTypeList:
    for blendingPercentage in blendingPercentageList:
        config = [interpolationType, blendingPercentage]
        plot_algo("OWN", config)

legendList = ["True Path", "AIS Report"]
legendList.extend(algoList)

legend = plt.legend(legendList, loc="center left", bbox_to_anchor=(1, 0.5))
plt.title("AIS Vessel Motion Simulator")
plt.xlabel("X-Axis (m)")
plt.ylabel("Y-Axis (m)")
plt.axis("scaled")

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
