import math
import time
import numpy as np
import matplotlib.pyplot as plt
from algo.dr import dead_reckoning
from algo.ekf import extended_kalman
from algo.xkf import exogenous_kalman
from algo.ukf import unscented_kalman
from algo.pvb import projective_velocity_blending
from algo.agb import own_algo
from algo.rot import rate_turn

from trials.zigzag import zigZagTrial
from trials.circle import circleTrial

from trials.utils import sim2unreal

X, Y, T, aisData = zigZagTrial()
# X, Y, T, aisData = circleTrial()

# print(sim2unreal(aisData))
# quit()

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
    plt.figure(3)
    x = np.arange(len(aE))
    # Plot E-T Graph
    plt.plot(x, aE, label=algo)
    plt.xlabel("t (frames)")
    plt.ylabel("Absolute Error")
    plt.legend(algoList)
    plt.figure(1)
    X_MAE = XsumOfDiff / len(T)
    Y_MAE = YsumOfDiff / len(T)
    MAE = (X_MAE + Y_MAE) / 2
    RMSE = (math.sqrt(XsumOfSq / len(T)) + math.sqrt(YsumOfSq / len(T))) / 2
    print(RMSE, end=",")
    print(MAE, end=",")


def calcTeleportabilityScore(algo, aE):
    XsumOfDiff = 0
    YsumOfDiff = 0
    for e in aE:
        XsumOfDiff += e[0]
        YsumOfDiff += e[1]
    XmeanDiff = XsumOfDiff / len(aE)
    YmeanDiff = YsumOfDiff / len(aE)
    telepScore = (XmeanDiff + YmeanDiff) / 2
    print(telepScore, end=",")


def calcSmoothingScore(algo, aX, aY):
    distSum = 0
    n = len(aX)

    for i in range(n - 1):
        xDiff = aX[i + 1] - aX[i]
        yDiff = aY[i + 1] - aY[i]
        dist = math.sqrt(xDiff**2 + yDiff**2)
        distSum += dist
    smoothScore = (n - 1) / distSum
    print(smoothScore, end=",")


def plotXYCGraphs(aX, aY, aC, aT):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)

    ax1.plot(aT, aX, color="red")
    ax1.set_ylabel("x")

    ax2.plot(aT, aY, color="green")
    ax2.set_ylabel("y")

    ax3.plot(aT, aC, color="blue")
    ax3.set_xlabel("t")
    ax3.set_ylabel("Course")

    plt.figure(1)


def plot_algo(algo="DR", config=[]):
    startTime = time.time()
    if algo == "DR":
        aX, aY, aC, aT, aE = dead_reckoning(aisData)
        algoList.append("Dead Reckoning")
    if algo == "EKF":
        aX, aY, aC, aT, aE = extended_kalman(aisData)
        algoList.append("Extended Kalman Filter")
    if algo == "XKF":
        aX, aY, aC, aT, aE = exogenous_kalman(aisData)
        algoList.append("Exogenous Kalman Filter")
    if algo == "UKF":
        aX, aY, aC, aT, aE = unscented_kalman(aisData)
        algoList.append("Unscented Kalman Filter")
    if algo == "PVB":
        aX, aY, aC, aT, aE = projective_velocity_blending(aisData)
        algoList.append("Projective Velocity Blending")
    if algo == "AGB":
        algo = f"AGB-{config[0]},{round(config[1], 2)}"
        aX, aY, aC, aT, aE = own_algo(aisData, config)
        algoList.append(algo)
    if algo == "ROT":
        aX, aY, aC, aT, aE = rate_turn(aisData)
        algoList.append("DR + Rate of Turn")
    endTime = time.time()
    print(algo, end=",")
    (tmpGraph,) = ax.plot(aX, aY, "o:", markersize=1)
    graphList.append(tmpGraph)
    calcAccuracy(algo, aX, aY, aT)
    calcTeleportabilityScore(algo, aE)
    calcSmoothingScore(algo, aX, aY)
    algoTime = endTime - startTime
    print(algoTime)
    plotXYCGraphs(aX, aY, aC, aT)
    # ALGONAME, RMSE, MAE, TELEPSCORE, SMOOTHSCORE, TIME


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
blendingPercentageList = [0.5]  # np.arange(0, 1.01, 0.05)
for interpolationType in interpolationTypeList:
    for blendingPercentage in blendingPercentageList:
        config = [interpolationType, blendingPercentage]
        plot_algo("AGB", config)

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
