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
            deg2rad(aisReports[0].course),
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
            deg2rad(aisReports[1].course),
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


def tryForQuadX(aisReports):
    # http://www2.lawrence.edu/fast/GREGGJ/CMSC210/arithmetic/interpolation.html
    try:
        matForInv = np.matrix(
            [
                [aisReports[0].posX ** 2, aisReports[0].posX, 1],
                [aisReports[1].posX ** 2, aisReports[1].posX, 1],
                [aisReports[2].posX ** 2, aisReports[2].posX, 1],
            ]
        )
        matInv = np.linalg.inv(matForInv)
        matDep = np.matrix(
            [
                aisReports[0].posY,
                aisReports[1].posY,
                aisReports[2].posY,
            ]
        ).T
        quadraticCoef = matInv * matDep
        quadraticCoef = quadraticCoef.A1
        return quadraticCoef
    except:
        return None


def tryForQuadY(aisReports):
    try:
        matForInv = np.matrix(
            [
                [aisReports[0].posY ** 2, aisReports[0].posY, 1],
                [aisReports[1].posY ** 2, aisReports[1].posY, 1],
                [aisReports[2].posY ** 2, aisReports[2].posY, 1],
            ]
        )
        matInv = np.linalg.inv(matForInv)
        matDep = np.matrix(
            [
                aisReports[0].posX,
                aisReports[1].posX,
                aisReports[2].posX,
            ]
        ).T
        quadraticCoef = matInv * matDep
        quadraticCoef = quadraticCoef.A1
        return quadraticCoef
    except:
        return None


def deg2rad(deg):
    return deg * math.pi / 180


def rad2deg(rad):
    return rad * 180 / math.pi


# Clamp course to 0-360
def normalizeCourse(course):
    course = course % 360
    if course < 0:
        course += 360
    return course


# Need quad type, coefs, and state (position, course) to determine course
def getQuadCourseByState(quadType, quadCoef, state):
    stateCourse = normalizeCourse(rad2deg(state.course))
    if quadType == "x":
        # m = 2ax + b
        gradient = 2 * quadCoef[0] * state.posX + quadCoef[1]
        courseXPlus = normalizeCourse(rad2deg(math.atan(gradient)))
        courseXMinus = normalizeCourse(courseXPlus + 180)
        diffXPlus = abs(courseXPlus - stateCourse)
        diffXMinus = abs(courseXMinus - stateCourse)
        print(
            "stateCourse courseXPlus courseXMinus",
            stateCourse,
            courseXPlus,
            courseXMinus,
        )
        if diffXMinus > diffXPlus:
            # print("picked courseXPlus")
            return courseXPlus
        else:
            # print("picked courseXMinus")
            return courseXMinus
    if quadType == "y":
        # m = 2ay + b
        gradient = 2 * quadCoef[0] * state.posY + quadCoef[1]
        courseYPlus = normalizeCourse(rad2deg(math.atan(gradient)))
        courseYMinus = normalizeCourse(courseYPlus + 180)
        diffYPlus = abs(courseYPlus - stateCourse)
        diffYMinus = abs(courseYMinus - stateCourse)
        print(
            "stateCourse courseYPlus courseYMinus",
            stateCourse,
            courseYPlus,
            courseYMinus,
        )
        if diffYMinus > diffYPlus:
            # print("picked courseYPlus")
            return courseYPlus
        else:
            # print("picked courseYMinus")
            return courseYMinus


class P3:  # predicting with three AIS reports
    state = VesselState(0, 0, 0, 0)
    quadraticType = "x"  # x or y
    quadraticCoef = []

    def __init__(self, aisReports):
        self.state = VesselState(
            aisReports[2].posX,
            aisReports[2].posY,
            aisReports[2].speed,
            deg2rad(aisReports[2].course),
        )
        print(
            "x:",
            aisReports[0].posX,
            ",",
            aisReports[1].posX,
            ",",
            aisReports[2].posX,
        )
        print(
            "y:",
            aisReports[0].posY,
            ",",
            aisReports[1].posY,
            ",",
            aisReports[2].posY,
        )
        coefX = tryForQuadX(aisReports)
        coefY = tryForQuadY(aisReports)
        if coefX is not None and coefY is not None:
            # Pick the best
            courseX = getQuadCourseByState("x", coefX, self.state)
            courseY = getQuadCourseByState("y", coefY, self.state)
            diffX = abs(courseX - aisReports[2].course)
            diffY = abs(courseY - aisReports[2].course)
            print("actual courseX courseY:", aisReports[2].course, courseX, courseY)
            if diffY > diffX:
                self.quadraticCoef = coefX
                self.quadraticType = "x"
            else:
                self.quadraticCoef = coefY
                self.quadraticType = "y"
        elif coefX is not None:
            self.quadraticCoef = coefX
            self.quadraticType = "x"
        elif coefY is not None:
            self.quadraticCoef = coefY
            self.quadraticType = "y"
        print("picked", self.quadraticType)
        print(self.quadraticCoef)
        print("----------------------------")

    def predict(self, tDelta):
        if self.quadraticType == "x":
            # m = 2ax + b
            # gradient = (
            #     2 * self.quadraticCoef[0] * self.state.posX
            #     + self.quadraticCoef[1]
            # )
            course = getQuadCourseByState("x", self.quadraticCoef, self.state)
            polyCourse = deg2rad(course)
            print("course:", course)
            tmpPosX = self.state.posX + self.state.speed * math.sin(polyCourse) * tDelta
            # ax2 + bx + c = 0
            tmpPosY = (
                self.quadraticCoef[0] * tmpPosX**2
                + self.quadraticCoef[1] * tmpPosX
                + self.quadraticCoef[2]
            )
            speedNext = self.state.speed  # no update to speed
            courseNext = polyCourse
            posXNext = tmpPosX
            posYNext = tmpPosY
            stateNext = VesselState(
                posXNext, posYNext, speedNext, courseNext
            )  # future state
            self.state = stateNext  # advance state
            return self.state
        if self.quadraticType == "y":
            # m = 2ay + b
            # gradient = (
            #     2 * self.quadraticCoef[0] * self.state.posY
            #     + self.quadraticCoef[1]
            # )
            course = getQuadCourseByState("y", self.quadraticCoef, self.state)
            polyCourse = deg2rad(course)
            print("course:", course)
            tmpPosY = self.state.posY + self.state.speed * math.cos(polyCourse) * tDelta
            # ay2 + by + c = 0
            tmpPosX = (
                self.quadraticCoef[0] * tmpPosY**2
                + self.quadraticCoef[1] * tmpPosY
                + self.quadraticCoef[2]
            )
            speedNext = self.state.speed  # no update to speed
            courseNext = polyCourse
            posXNext = tmpPosX
            posYNext = tmpPosY
            stateNext = VesselState(
                posXNext, posYNext, speedNext, courseNext
            )  # future state
            self.state = stateNext  # advance state
            return self.state


def own_algo(aisData):
    estFreq = 60  # in Hertz
    h = 1 / estFreq
    aT = np.arange(0, aisData["duration"], h)
    aX = []
    aY = []
    k = 0
    aisReports = []  # Max 3 reports
    vesselState = VesselState(0, 0, 0, 0)  # Store vessel state
    predictors = []  # Max 2 predictors: to blend (prev and current)
    reportingTime = 0
    tSinceLastReport = 0
    for deltaAT in aT:
        if deltaAT >= aisData["time"][k]:
            aisReport = AISReport(
                aisData["time"][k],
                aisData["x"][k],
                aisData["y"][k],
                aisData["speed"][k],
                aisData["course"][k],
            )
            if len(aisReports) == 3:  # Only keep last 3 reports
                aisReports.pop(0)
            aisReports.append(aisReport)
            # Init predictors
            if len(predictors) == 2:  # Only keep 2 predictors
                predictors.pop(0)
            if len(aisReports) == 1:
                predictors.append(P1(aisReports))
            elif len(aisReports) == 2:
                predictors.append(P2(aisReports))
            elif len(aisReports) == 3:
                try:
                    predictors.append(P3(aisReports))
                except:
                    print("bad 3-pred!")
            reportingTime = tSinceLastReport
            tSinceLastReport = 0  # reset timer
            if k < len(aisData["time"]) - 1:
                k += 1
            else:
                break
        stateFinal = None
        if len(predictors) == 2:
            stateOld = predictors[0].predict(h)
            stateNew = predictors[1].predict(h)
            # Blend between two
            blendWeight = tSinceLastReport / reportingTime
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
        tSinceLastReport += h
    return [aX, aY]
