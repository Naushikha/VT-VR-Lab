import math
from typing import Counter
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


class P2:  # predicting with two AIS reports
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


def solveQuad(p1PosX, p1PosY, p2PosX, p2PosY, p3PosX, p3PosY):
    # http://www2.lawrence.edu/fast/GREGGJ/CMSC210/arithmetic/interpolation.html
    try:
        matForInv = np.matrix(
            [
                [p1PosX**2, p1PosX, 1],
                [p2PosX**2, p2PosX, 1],
                [p3PosX**2, p3PosX, 1],
            ]
        )
        matInv = np.linalg.inv(matForInv)
        matDep = np.matrix([p1PosY, p2PosY, p3PosY]).T
        quadraticCoef = matInv * matDep
        quadraticCoef = quadraticCoef.A1
        return quadraticCoef
    except:
        return None


def point2angle(x, y):
    # Calculate the angle in radians
    if x > 0 and y >= 0:
        # First quadrant
        angle = math.atan(y / x)
    elif x > 0 and y < 0:
        # Fourth quadrant
        angle = math.atan(y / x) + 2 * math.pi
    elif x < 0:
        # Second or third quadrant
        angle = math.atan(y / x) + math.pi
    elif x == 0 and y > 0:
        # Positive y-axis
        angle = math.pi / 2
    elif x == 0 and y < 0:
        # Negative y-axis
        angle = 3 * math.pi / 2
    else:
        # Origin
        angle = 0

    # Convert the angle to degrees
    return math.degrees(angle)


def transformPoint(x, y, angle):
    # Convert the angle to radians
    theta = math.radians(angle)

    # Apply the rotation transformation
    x1 = x * math.cos(theta) - y * math.sin(theta)
    y1 = x * math.sin(theta) + y * math.cos(theta)

    # Return the transformed point
    return x1, y1


class P3:  # predicting with three AIS reports
    state = VesselState(0, 0, 0, 0)
    quadraticType = "x"  # x or y
    quadraticCoef = []
    transAngle = 0
    transCoef = []
    transState = VesselState(0, 0, 0, 0)
    quadCoefX = []
    quadCoefY = []
    quadTime = 0

    def __init__(self, aisReports):
        self.state = VesselState(
            aisReports[2].posX,
            aisReports[2].posY,
            aisReports[2].speed,
            math.radians(aisReports[2].course),
        )
        print(
            "x:", aisReports[0].posX, aisReports[1].posX, aisReports[2].posX, sep=" , "
        )
        print(
            "y:", aisReports[0].posY, aisReports[1].posY, aisReports[2].posY, sep=" , "
        )
        print(
            "t:", aisReports[0].time, aisReports[1].time, aisReports[2].time, sep=" , "
        )
        quadCoefX = solveQuad(
            aisReports[0].time,
            aisReports[0].posX,
            aisReports[1].time,
            aisReports[1].posX,
            aisReports[2].time,
            aisReports[2].posX,
        )
        quadCoefY = solveQuad(
            aisReports[0].time,
            aisReports[0].posY,
            aisReports[1].time,
            aisReports[1].posY,
            aisReports[2].time,
            aisReports[2].posY,
        )
        self.quadCoefX = quadCoefX
        self.quadCoefY = quadCoefY
        self.quadTime = aisReports[2].time
        # Consider Point-1 as origin and calculate vector to Point-3: Vector[1->3]
        VecX = aisReports[2].posX - aisReports[0].posX
        VecY = aisReports[2].posY - aisReports[0].posY
        transAngle = point2angle(VecX, VecY)
        print("trans angle:", transAngle)
        self.transAngle = transAngle
        # Transform coordinates to coincide Vector[1->3] with Positive X-axis
        p1PosX, p1PosY = transformPoint(
            aisReports[0].posX, aisReports[0].posY, -transAngle
        )
        p2PosX, p2PosY = transformPoint(
            aisReports[1].posX, aisReports[1].posY, -transAngle
        )
        p3PosX, p3PosY = transformPoint(
            aisReports[2].posX, aisReports[2].posY, -transAngle
        )
        print("new x:", p1PosX, p2PosX, p3PosX, sep=" , ")
        print("new y:", p1PosY, p2PosY, p3PosY, sep=" , ")
        # Check if Point-2 lies in between Point-1 and Point-3
        if not (p1PosX < p2PosX and p2PosX < p3PosX):
            raise Exception("Point-2 is not between Point-1 and Point-3")
            # Quad prediction not possible fallback to prediction using 1 point
        coef = solveQuad(p1PosX, p1PosY, p2PosX, p2PosY, p3PosX, p3PosY)
        print(coef)
        self.transCoef = coef
        gradient = 2 * coef[0] * p3PosX + coef[1]
        transCourse = math.atan(gradient)  # is relative to the x-axis
        print("courseAIS courseQuad:", aisReports[2].course, math.degrees(transCourse))
        self.transState = VesselState(p3PosX, p3PosY, aisReports[2].speed, transCourse)
        print("----------------------------")

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
        posXNext = (
            self.transState.posX
            + self.transState.speed * math.cos(self.transState.course) * tDelta
        )
        # print(
        #     "speed course speed X:",
        #     self.transState.speed,
        #     self.transState.course,
        #     self.transState.speed * math.cos(self.transState.course),
        # )
        # ax2 + bx + c = 0
        posYNext = (
            self.transCoef[0] * posXNext**2
            + self.transCoef[1] * posXNext
            + self.transCoef[2]
        )
        speedNext = self.state.speed  # no update to speed
        gradient = 2 * self.transCoef[0] * posXNext + self.transCoef[1]
        courseNext = math.atan(gradient)
        transStateNext = VesselState(
            posXNext, posYNext, speedNext, courseNext
        )  # future state
        self.transState = transStateNext  # advance state
        # transform back
        posXNext, posYNext = transformPoint(posXNext, posYNext, self.transAngle)
        stateNext = VesselState(posXNext, posYNext, speedNext, courseNext)
        self.state = stateNext
        return self.state


def solveQuad2(p1, p2, t1, t2):
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
        self.quadCoefX = solveQuad2(
            [aisReports[0].time, aisReports[0].posX],
            [aisReports[1].time, aisReports[1].posX],
            tanX1,
            tanX2,
        )
        self.quadCoefY = solveQuad2(
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
    # Create the matrices
    A = np.array(
        [
            [x1**3, x1**2, x1, 1],
            [x2**3, x2**2, x2, 1],
            [3 * x1**2, 2 * x1, 1, 0],
            [3 * x2**2, 2 * x2, 1, 0],
        ]
    )
    b = np.array([y1, y2, t1, t2])
    # Solve the system of equations
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
            # elif len(aisReports) == 3:
            #     try:
            #         predictors.append(P3(aisReports))
            #     except:
            #         print("bad 3-pred!")
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
