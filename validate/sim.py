import math
import matplotlib.pyplot as plt
import numpy as np
from algo.dr import dead_reckoning
from algo.ekf import extended_kalman
from algo.xkf import exogenous_kalman

X = np.linspace(0, 100, 1000)  # in meters
Y = []

for x in X:
    Y.append(math.sin(x / 15) * 20)

totalDist = 0

for i in range(len(X) - 1):
    fragDist = math.sqrt((Y[i + 1] - Y[i]) ** 2 + (X[i + 1] - X[i]) ** 2)
    totalDist += fragDist


def getCourse(x1, y1, x2, y2):
    return math.degrees(math.atan2((x2 - x1), (y2 - y1)))


def getAISDataByDist(dist):
    tDist = 0
    for i in range(len(X) - 1):
        if tDist >= dist:
            return [X[i], Y[i], getCourse(X[i], Y[i], X[i + 1], Y[i + 1])]
        fragDist = math.sqrt((Y[i + 1] - Y[i]) ** 2 + (X[i + 1] - X[i]) ** 2)
        tDist += fragDist
    return [X[len(X) - 1], Y[len(Y) - 1], getCourse(X[i - 1], Y[i - 1], X[i], Y[i])]


def getAISReportingIntervalBySpeed(speed, vClass="A"):
    if vClass == "A":
        if speed == 0:  # anchored
            return 3 * 60  # 3 mins
        elif 0 < speed <= 14:  # 0-14 knots and changing course
            return 3.33  # 3.33 seconds
        elif 14 < speed <= 23:  # 14-23 knots and changing course
            return 2  # 2 seconds
        else:  # faster than 23 knots
            return 2  # 2 seconds
    elif vClass == "B":
        return  # TODO: Implement!
    # https://help.marinetraffic.com/hc/en-us/articles/217631867
    # https://www.nauticast.com/en/cms/about_ais
    # https://arundaleais.github.io/docs/ais/ais_reporting_rates.html


time = 60  # seconds
speed = totalDist / time
speedKnots = speed * 1.94384
print("Total Distance:", totalDist, "m")
print("Time:", time, "s")
print("Speed:", speedKnots, "knots")

print("Reporting Interval:", getAISReportingIntervalBySpeed(speedKnots))

aisT = np.arange(0, time, getAISReportingIntervalBySpeed(speedKnots))  # in seconds
aisX = []  # in meters
aisY = []  # in meters
aisC = []  # in degrees
aisS = np.repeat(speed, len(aisT))  # in ms-1

for reportingTime in aisT:
    tX, tY, tH = getAISDataByDist(reportingTime * speed)
    aisX.append(tX)
    aisY.append(tY)
    aisC.append(tH)

aisData = {
    "time": aisT,
    "x": aisX,
    "y": aisY,
    "course": aisC,
    "speed": aisS,
    "duration": time,
}

fig, ax = plt.subplots(figsize=(14, 8))
graphList = []
algoList = []


def plot_algo(algo="DR"):
    if algo == "DR":
        aX, aY = dead_reckoning(aisData)
        algoList.append("Dead Reckoning")
    if algo == "EKF":
        aX, aY = extended_kalman(aisData)
        algoList.append("Extended Kalman Filter")
    if algo == "XKF":
        aX, aY = exogenous_kalman(aisData)
        algoList.append("Exogenous Kalman Filter")
    tmpGraph, = ax.plot(aX, aY, "o:", markersize=1)
    graphList.append(tmpGraph)


tmpGraph, = ax.plot(X, Y)
graphList.append(tmpGraph)
tmpGraph, = ax.plot(aisX, aisY, "ro")
graphList.append(tmpGraph)
plot_algo("DR")
plot_algo("EKF")
plot_algo("XKF")

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


plt.connect('pick_event', onLegendPick)
plt.show()
