import math
import numpy as np


class AISReport:
    time = 0  # report time
    posX = 0  # longitude
    posY = 0  # latitude
    speed = 0  # knots
    course = 0  # degrees

    def __init__(self, time, posX, posY, speed, course):
        self.time = time
        self.posX = posX
        self.posY = posY
        self.speed = speed
        self.course = course


class VesselState:
    posX = 0  # m
    posY = 0  # m
    speed = 0  # ms-1
    course = 0  # radians

    def __init__(self, posX, posY, speed, course):
        self.posX = posX
        self.posY = posY
        self.speed = speed
        self.course = course


class P1:  # predicting with one AIS report
    state = VesselState(0, 0, 0, 0)

    def __init__(self, aisReports):
        self.state = VesselState(
            aisReports[0].posX,
            aisReports[0].posY,
            aisReports[0].speed,
            math.radians(aisReports[0].course),
        )

    def predict(self, tDelta):
        speedNext = self.state.speed  # no update to speed
        courseNext = self.state.course  # no update to course
        posXNext = (
            self.state.posX + self.state.speed * math.sin(self.state.course) * tDelta
        )
        posYNext = (
            self.state.posY + self.state.speed * math.cos(self.state.course) * tDelta
        )
        stateNext = VesselState(
            posXNext, posYNext, speedNext, courseNext
        )  # future state
        self.state = stateNext  # advance state
        return self.state


class P2_Rot:  # predicting with two AIS reports
    state = VesselState(0, 0, 0, 0)
    rateOfTurn = 0  # derive from two directions

    def __init__(self, aisReports):
        self.state = VesselState(
            aisReports[1].posX,
            aisReports[1].posY,
            aisReports[1].speed,
            math.radians(aisReports[1].course),
        )
        self.rateOfTurn = math.radians(
            (aisReports[1].course - aisReports[0].course)
            / (aisReports[1].time - aisReports[0].time)
        )

    def predict(self, tDelta):
        speedNext = self.state.speed  # no update to speed
        courseNext = (
            self.state.course + self.rateOfTurn * tDelta
        )  # utilize rate of turn
        posXNext = self.state.posX + self.state.speed * math.sin(courseNext) * tDelta
        posYNext = self.state.posY + self.state.speed * math.cos(courseNext) * tDelta
        stateNext = VesselState(
            posXNext, posYNext, speedNext, courseNext
        )  # future state
        self.state = stateNext  # advance state
        return self.state


def solveQuad(p1, p2, t1, t2):
    x1, y1 = p1
    x2, y2 = p2
    A = np.array([[x1**2, x1, 1], [x2**2, x2, 1], [2 * x1, 1, 0], [2 * x2, 1, 0]])
    b = np.array([y1, y2, t1, t2])
    x = np.linalg.lstsq(A, b, rcond=None)[0]
    a, b, c = x[0], x[1], x[2]
    return [a, b, c]


class P2_Quad:  # predicting with two AIS reports
    state = VesselState(0, 0, 0, 0)
    quadCoefX = []
    quadCoefY = []
    quadTime = 0

    def __init__(self, aisReports):
        self.state = VesselState(
            aisReports[1].posX,
            aisReports[1].posY,
            aisReports[1].speed,
            math.radians(aisReports[1].course),
        )
        self.quadTime = aisReports[1].time
        # Calculate tangents
        tanX1 = aisReports[0].speed * math.sin(math.radians(aisReports[0].course))
        tanX2 = aisReports[1].speed * math.sin(math.radians(aisReports[1].course))
        tanY1 = aisReports[0].speed * math.cos(math.radians(aisReports[0].course))
        tanY2 = aisReports[1].speed * math.cos(math.radians(aisReports[1].course))
        self.quadCoefX = solveQuad(
            [aisReports[0].time, aisReports[0].posX],
            [aisReports[1].time, aisReports[1].posX],
            tanX1,
            tanX2,
        )
        self.quadCoefY = solveQuad(
            [aisReports[0].time, aisReports[0].posY],
            [aisReports[1].time, aisReports[1].posY],
            tanY1,
            tanY2,
        )

    def predict(self, tDelta):
        self.quadTime += tDelta
        posXNext = (
            self.quadCoefX[0] * self.quadTime**2
            + self.quadCoefX[1] * self.quadTime
            + self.quadCoefX[2]
        )
        posYNext = (
            self.quadCoefY[0] * self.quadTime**2
            + self.quadCoefY[1] * self.quadTime
            + self.quadCoefY[2]
        )
        speedNext = self.state.speed  # no update to speed
        courseNext = 0
        stateNext = VesselState(posXNext, posYNext, speedNext, courseNext)
        self.state = stateNext
        return self.state


def solveCubic(p1, p2, t1, t2):
    x1, y1 = p1
    x2, y2 = p2
    A = np.array(
        [
            [x1**3, x1**2, x1, 1],
            [x2**3, x2**2, x2, 1],
            [3 * x1**2, 2 * x1, 1, 0],
            [3 * x2**2, 2 * x2, 1, 0],
        ]
    )
    b = np.array([y1, y2, t1, t2])
    x = np.linalg.solve(A, b)
    return [x[0], x[1], x[2], x[3]]


class P2_Cubic:  # predicting with two AIS reports
    state = VesselState(0, 0, 0, 0)
    cubicCoefX = []
    cubicCoefY = []
    cubicTime = 0

    def __init__(self, aisReports):
        self.state = VesselState(
            aisReports[1].posX,
            aisReports[1].posY,
            aisReports[1].speed,
            math.radians(aisReports[1].course),
        )
        self.cubicTime = aisReports[1].time
        # Calculate tangents
        tanX1 = aisReports[0].speed * math.sin(math.radians(aisReports[0].course))
        tanX2 = aisReports[1].speed * math.sin(math.radians(aisReports[1].course))
        tanY1 = aisReports[0].speed * math.cos(math.radians(aisReports[0].course))
        tanY2 = aisReports[1].speed * math.cos(math.radians(aisReports[1].course))
        self.cubicCoefX = solveCubic(
            [aisReports[0].time, aisReports[0].posX],
            [aisReports[1].time, aisReports[1].posX],
            tanX1,
            tanX2,
        )
        self.cubicCoefY = solveCubic(
            [aisReports[0].time, aisReports[0].posY],
            [aisReports[1].time, aisReports[1].posY],
            tanY1,
            tanY2,
        )

    def predict(self, tDelta):
        self.cubicTime += tDelta
        posXNext = (
            self.cubicCoefX[0] * self.cubicTime**3
            + self.cubicCoefX[1] * self.cubicTime**2
            + self.cubicCoefX[2] * self.cubicTime
            + self.cubicCoefX[3]
        )
        posYNext = (
            self.cubicCoefY[0] * self.cubicTime**3
            + self.cubicCoefY[1] * self.cubicTime**2
            + self.cubicCoefY[2] * self.cubicTime
            + self.cubicCoefY[3]
        )
        speedNext = self.state.speed  # no update to speed
        courseNext = 0
        stateNext = VesselState(posXNext, posYNext, speedNext, courseNext)
        self.state = stateNext
        return self.state


def own_algo(aisData):
    estFreq = 60  # in Hertz
    h = 1 / estFreq
    T = np.arange(0, aisData["duration"], h)
    aX = []
    aY = []
    aT = []
    k = 0
    aisReports = []  # Max 3 reports
    vesselState = VesselState(0, 0, 0, 0)  # Store vessel state
    predictors = []  # Max 2 predictors: to blend (prev and current)
    reportingTime = 0
    tSinceLastReport = 0
    for t in T:
        if t >= aisData["time"][k]:
            aisReport = AISReport(
                aisData["time"][k],
                aisData["x"][k],
                aisData["y"][k],
                aisData["speed"][k],
                aisData["course"][k],
            )
            if len(aisReports) == 2:  # Only keep last 3 reports
                aisReports.pop(0)
            aisReports.append(aisReport)
            # Init predictors
            if len(predictors) == 2:  # Only keep 2 predictors
                predictors.pop(0)
            if len(aisReports) == 1:
                predictors.append(P1(aisReports))
            elif len(aisReports) == 2:
                predictors.append(P2_Quad(aisReports))
            reportingTime = tSinceLastReport
            tSinceLastReport = 0  # reset timer
            if k < len(aisData["time"]) - 1:
                k += 1
            else:
                continue
        stateFinal = None
        if len(predictors) == 2:
            stateOld = predictors[0].predict(h)
            stateNew = predictors[1].predict(h)
            # Blend between two
            blendEnd = 0.25
            blendWeight = tSinceLastReport / (reportingTime * blendEnd)
            if blendWeight > 1:
                blendWeight = 1
            posXFinal = stateOld.posX + (stateNew.posX - stateOld.posX) * blendWeight
            posYFinal = stateOld.posY + (stateNew.posY - stateOld.posY) * blendWeight
            speedFinal = (
                stateOld.speed + (stateNew.speed - stateOld.speed) * blendWeight
            )
            courseFinal = (
                stateOld.course + (stateNew.course - stateOld.course) * blendWeight
            )
            stateFinal = VesselState(posXFinal, posYFinal, speedFinal, courseFinal)
            # stateFinal = stateNew  # Override blending
        if len(predictors) == 1:
            stateNew = predictors[0].predict(h)
            stateFinal = stateNew
        if stateFinal is not None:
            vesselState = stateFinal
            # print(stateFinal.position)
            aX.append(stateFinal.posX)
            aY.append(stateFinal.posY)
            aT.append(t)
        tSinceLastReport += h
    return [aX, aY, aT]
