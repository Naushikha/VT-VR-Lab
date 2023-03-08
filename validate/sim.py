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

X, Y, aisData = zigZagTrial()
# X, Y, aisData = circleTrial()

fig, ax = plt.subplots(figsize=(14, 8))
graphList = []
algoList = []


def calcAccuracy(algo, aX, aY):
    # https://stackoverflow.com/a/6723457
    sumOfSq = 0
    sumOfDiff = 0
    for i in range(len(X)):
        algoY = np.interp(X[i], aX, aY)
        sumOfSq += (algoY - Y[i]) ** 2
        sumOfDiff += abs(algoY - Y[i])
    print(algo, " : Sum of Squares : ", sumOfSq)
    print(algo, " : Sum of Differences : ", sumOfDiff)
    print(algo, " : Mean Difference : ", sumOfDiff / len(X))


def plot_algo(algo="DR"):
    startTime = time.time()
    if algo == "DR":
        aX, aY = dead_reckoning(aisData)
        algoList.append("Dead Reckoning")
    if algo == "EKF":
        aX, aY = extended_kalman(aisData)
        algoList.append("Extended Kalman Filter")
    if algo == "XKF":
        aX, aY = exogenous_kalman(aisData)
        algoList.append("Exogenous Kalman Filter")
    if algo == "UKF":
        aX, aY = unscented_kalman(aisData)
        algoList.append("Unscented Kalman Filter")
    if algo == "PVB":
        aX, aY = projective_velocity_blending(aisData)
        algoList.append("Projective Velocity Blending")
    if algo == "OWN":
        aX, aY = own_algo(aisData)
        algoList.append("Own Algo")
    if algo == "ROT":
        aX, aY = rate_turn(aisData)
        algoList.append("DR + Rate of Turn")
    endTime = time.time()
    (tmpGraph,) = ax.plot(aX, aY, "o:", markersize=1)
    graphList.append(tmpGraph)
    calcAccuracy(algo, aX, aY)
    algoTime = endTime - startTime
    print(algo, " : Processing Time : ", algoTime)


(tmpGraph,) = ax.plot(X, Y)
graphList.append(tmpGraph)
(tmpGraph,) = ax.plot(aisData["x"], aisData["y"], "ro")
graphList.append(tmpGraph)
plot_algo("DR")
# plot_algo("EKF")
# plot_algo("XKF")
# plot_algo("UKF")
plot_algo("ROT")
plot_algo("PVB")
plot_algo("OWN")

legendList = ["True Path", "AIS Report"]
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
