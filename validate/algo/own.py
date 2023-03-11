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


def getReportPosition(aisReport):
    return np.array(aisReport.position)


def getReportVelocity(aisReport):
    return np.array(
        [
            aisReport.speed * math.sin(math.radians(aisReport.course)),
            aisReport.speed * math.cos(math.radians(aisReport.course)),
        ]
    )


def getGradient(quadraticCoef, posX):
    # ax^2 + bx + c
    x1 = posX - 0.01
    x2 = posX + 0.01
    y1 = quadraticCoef[0] * (x1**2) + quadraticCoef[1] * x1 + quadraticCoef[2]
    y2 = quadraticCoef[0] * (x2**2) + quadraticCoef[1] * x2 + quadraticCoef[2]
    gradient = (y2 - y1) / (x2 - x1)
    # print(y2, y1, x2, x1)
    return gradient


def getSpeedByVelocity(velocity):
    return math.sqrt(velocity[0] ** 2 + velocity[1] ** 2)


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
                [aisReports[0].position[0] ** 2, aisReports[0].position[0], 1],
                [aisReports[1].position[0] ** 2, aisReports[1].position[0], 1],
                [aisReports[2].position[0] ** 2, aisReports[2].position[0], 1],
            ]
        )
        matInv = np.linalg.inv(matForInv)
        matDep = np.matrix(
            [
                aisReports[0].position[1],
                aisReports[1].position[1],
                aisReports[2].position[1],
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
                [aisReports[0].position[1] ** 2, aisReports[0].position[1], 1],
                [aisReports[1].position[1] ** 2, aisReports[1].position[1], 1],
                [aisReports[2].position[1] ** 2, aisReports[2].position[1], 1],
            ]
        )
        matInv = np.linalg.inv(matForInv)
        matDep = np.matrix(
            [
                aisReports[0].position[0],
                aisReports[1].position[0],
                aisReports[2].position[0],
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


# Need quad type, coefs, and state (position and velocity) to determine course
def getQuadCourseByState(quadType, quadCoef, state):
    print("quad velocity", state.velocity)
    stateCourse = normalizeCourse(
        rad2deg(math.atan(state.velocity[0] / state.velocity[1]))
    )
    if quadType == "x":
        # m = 2ax + b
        gradient = 2 * quadCoef[0] * state.position[0] + quadCoef[1]
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
        gradient = 2 * quadCoef[0] * state.position[1] + quadCoef[1]
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
            print("picked courseYPlus")
            return courseYPlus
        else:
            print("picked courseYMinus")
            return courseYMinus


class P3:  # predicting with three AIS reports
    state = VesselState(0, 0, 0, 0)
    quadraticType = "x"  # x or y
    quadraticCoef = []

    def __init__(self, aisReports):
        self.state = VesselState(
            getReportPosition(aisReports[2]), getReportVelocity(aisReports[2])
        )
        print(
            "x:",
            aisReports[0].position[0],
            ",",
            aisReports[1].position[0],
            ",",
            aisReports[2].position[0],
        )
        print(
            "y:",
            aisReports[0].position[1],
            ",",
            aisReports[1].position[1],
            ",",
            aisReports[2].position[1],
        )
        coefX = tryForQuadX(aisReports)
        coefY = tryForQuadY(aisReports)
        if coefX is not None and coefY is not None:
            # Pick the best
            # m = 2ax + b
            # gradientX = 2 * coefX[0] * self.state.position[0] + coefX[1]
            # courseX = (math.pi / 2 - math.atan(gradientX)) * 180 / math.pi
            courseX = getQuadCourseByState("x", coefX, self.state)
            # m = 2ay + b
            # gradientY = 2 * coefY[0] * self.state.position[1] + coefY[1]
            # courseY = (math.pi * 2 - math.atan(gradientY)) * 180 / math.pi
            courseY = getQuadCourseByState("y", coefY, self.state)
            diffX = abs(courseX - aisReports[2].course)
            diffY = abs(courseY - aisReports[2].course)
            print("actual courseX courseY:", aisReports[2].course, courseX, courseY)
            # print(courseX, courseY, aisReports[2].course)
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

        # Set state
        self.state = VesselState(
            getReportPosition(aisReports[2]), getReportVelocity(aisReports[2])
        )

    def predict(self, tDelta):
        if self.quadraticType == "x":
            # m = 2ax + b
            # gradient = (
            #     2 * self.quadraticCoef[0] * self.state.position[0]
            #     + self.quadraticCoef[1]
            # )
            # polyCourse = math.pi / 2 - math.atan(gradient)  # * 180 / math.pi
            course = getQuadCourseByState("x", self.quadraticCoef, self.state)
            # print("course:", course)
            polyCourse = deg2rad(course)
            print("course:", course)
            tmpPosX = (
                self.state.position[0]
                + getSpeedByVelocity(self.state.velocity)
                * math.sin(polyCourse)
                * tDelta
            )
            # ax2 + bx + c = 0
            tmpPosY = (
                self.quadraticCoef[0] * tmpPosX**2
                + self.quadraticCoef[1] * tmpPosX
                + self.quadraticCoef[2]
            )
            self.state.position = np.array([tmpPosX, tmpPosY])

            theta = -polyCourse
            R = np.matrix(
                [
                    [math.cos(theta), -math.sin(theta)],
                    [math.sin(theta), math.cos(theta)],
                ]
            )  # Rotation matrix
            velocityNext = np.matmul(
                R, self.state.velocity
            ).A1  # https://stackoverflow.com/a/20765358/11436038
            speed = getSpeedByVelocity(velocityNext)
            velocityNext = [speed * math.sin(polyCourse), speed * math.cos(polyCourse)]
            posNext = np.array([tmpPosX, tmpPosY])
            stateNext = VesselState(posNext, velocityNext)  # future state
            self.state = stateNext  # advance state
            return self.state
        else:
            # m = 2ay + b
            # gradient = (
            #     2 * self.quadraticCoef[0] * self.state.position[1]
            #     + self.quadraticCoef[1]
            # )
            # polyCourse = math.pi / 2 - math.atan(gradient)  # * 180 / math.pi
            course = getQuadCourseByState("y", self.quadraticCoef, self.state)
            # print("course:", course)
            polyCourse = deg2rad(course)
            print("course:", course)
            tmpPosY = (
                self.state.position[1]
                + getSpeedByVelocity(self.state.velocity)
                * math.cos(polyCourse)
                * tDelta
            )
            # ay2 + by + c = 0
            tmpPosX = (
                self.quadraticCoef[0] * tmpPosY**2
                + self.quadraticCoef[1] * tmpPosY
                + self.quadraticCoef[2]
            )
            self.state.position = np.array([tmpPosX, tmpPosY])

            theta = -polyCourse
            R = np.matrix(
                [
                    [math.cos(theta), -math.sin(theta)],
                    [math.sin(theta), math.cos(theta)],
                ]
            )  # Rotation matrix
            velocityNext = np.matmul(
                R, self.state.velocity
            ).A1  # https://stackoverflow.com/a/20765358/11436038
            speed = getSpeedByVelocity(velocityNext)
            velocityNext = [speed * math.sin(polyCourse), speed * math.cos(polyCourse)]
            posNext = np.array([tmpPosX, tmpPosY])
            stateNext = VesselState(posNext, velocityNext)  # future state
            print("velocity:", velocityNext[0], velocityNext[1])
            print("speed:", getSpeedByVelocity(velocityNext))
            print("---")
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
            if len(aisReports) == 2:  # Only keep last 3 reports
                aisReports.pop(0)
            aisReports.append(aisReport)
            # Init predictors
            if len(predictors) == 2:  # Only keep 2 predictors
                predictors.pop(0)
            if len(aisReports) == 1:
                predictors.append(P1(aisReports))
            elif len(aisReports) == 2:
                predictors.append(P2(aisReports))
            # elif len(aisReports) == 3:
            #     try:
            #         predictors.append(P2(aisReports))
            #     except:
            #         print("bad 3-pred!")
            reportingTime = tSinceLastReport
            tSinceLastReport = 0  # reset timer
            print("-----")
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
