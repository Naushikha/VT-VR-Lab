import math
import numpy as np


class AISReport:
    time = 0  # report time
    position = [0, 0]  # latitude, longitude
    speed = 0  # knots
    course = 0  # degrees

    def __init__(self, time, position, speed, course):
        self.time = time
        self.position = position
        self.speed = speed
        self.course = course


class VesselState:
    position = np.array([0, 0])
    velocity = np.array([0, 0])

    def __init__(self, position, velocity):
        self.position = np.array(position)
        self.velocity = np.array(velocity)


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
    state = VesselState([0, 0], [0, 0])

    def __init__(self, aisReports):
        self.state = VesselState(
            getReportPosition(aisReports[0]), getReportVelocity(aisReports[0])
        )

    def predict(self, tDelta):
        velocityNext = self.state.velocity  # no update to velocity
        posNext = self.state.position + self.state.velocity * tDelta
        stateNext = VesselState(posNext, velocityNext)  # future state
        self.state = stateNext  # advance state
        return self.state


class P2:  # predicting with two AIS reports
    state = VesselState([0, 0], [0, 0])
    rateOfTurn = 0  # derive from two directions

    def __init__(self, aisReports):
        self.state = VesselState(
            getReportPosition(aisReports[1]), getReportVelocity(aisReports[1])
        )
        self.rateOfTurn = (aisReports[1].course - aisReports[0].course) / (
            aisReports[1].time - aisReports[0].time
        )

    def predict(self, tDelta):
        theta = -math.radians(
            self.rateOfTurn * tDelta
        )  # clockwise from +y axis is negative
        R = np.matrix(
            [[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]]
        )  # Rotation matrix
        velocityNext = np.matmul(
            R, self.state.velocity
        ).A1  # https://stackoverflow.com/a/20765358/11436038
        posNext = self.state.position + self.state.velocity * tDelta
        stateNext = VesselState(posNext, velocityNext)  # future state
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


class P3:  # predicting with three AIS reports
    state = VesselState([0, 0], [0, 0])
    quadraticType = "x"  # x or y
    quadraticCoef = []

    def __init__(self, aisReports):
        self.state = VesselState(
            getReportPosition(aisReports[2]), getReportVelocity(aisReports[2])
        )
        coefX = tryForQuadX(aisReports)
        coefY = tryForQuadY(aisReports)
        if coefX is not None and coefY is not None:
            # Pick the best
            gradientX = 2 * coefX[0] * self.state.position[0] + coefX[1]
            courseX = (math.pi / 2 - math.atan(gradientX)) * 180 / math.pi
            gradientY = 2 * coefY[0] * self.state.position[0] + coefY[1]
            courseY = (math.pi / 2 - math.atan(gradientY)) * 180 / math.pi
            diffX = abs(courseX - aisReports[2].course)
            diffY = abs(courseY - aisReports[2].course)
            print(courseX, courseY, aisReports[2].course)
            if diffY > diffX:
                self.quadraticCoef = coefX
                self.quadraticType = "x"
            else:
                self.quadraticCoef = coefY
                self.quadraticType = "y"
        elif coefX is not None:
            self.quadraticType = "x"
        elif coefY is not None:
            self.quadraticType = "y"
        print("picked", self.quadraticType)
        print(self.quadraticCoef)

        # Set state
        self.state = VesselState(
            getReportPosition(aisReports[2]), getReportVelocity(aisReports[2])
        )

    def predict(self, tDelta):
        if self.quadraticType == "x":
            # m = 2ax + b
            gradient = (
                2 * self.quadraticCoef[0] * self.state.position[0]
                + self.quadraticCoef[1]
            )
            polyCourse = math.pi / 2 - math.atan(gradient)  # * 180 / math.pi
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
            posNext = np.array([tmpPosX, tmpPosY])
            stateNext = VesselState(posNext, velocityNext)  # future state
            self.state = stateNext  # advance state
            return self.state
        else:
            # m = 2ay + b
            gradient = (
                2 * self.quadraticCoef[0] * self.state.position[1]
                + self.quadraticCoef[1]
            )
            polyCourse = math.pi / 2 - math.atan(gradient)  # * 180 / math.pi
            tmpPosY = (
                self.state.position[1]
                + getSpeedByVelocity(self.state.velocity)
                * math.sin(polyCourse)
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
            posNext = np.array([tmpPosX, tmpPosY])
            stateNext = VesselState(posNext, velocityNext)  # future state
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
    vesselState = VesselState([0, 0], [0, 0])  # Store vessel state
    predictors = []  # Max 2 predictors: to blend (prev and current)
    reportingTime = 0
    tSinceLastReport = 0
    for deltaAT in aT:
        if deltaAT >= aisData["time"][k]:
            aisReport = AISReport(
                aisData["time"][k],
                [aisData["x"][k], aisData["y"][k]],
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
            posFinal = (
                stateOld.position
                + (stateNew.position - stateOld.position) * blendWeight
            )
            velFinal = (
                stateOld.velocity
                + (stateNew.velocity - stateOld.velocity) * blendWeight
            )
            stateFinal = VesselState(posFinal, velFinal)
            # stateFinal = stateNew # Override blending
        if len(predictors) == 1:
            stateNew = predictors[0].predict(h)
            stateFinal = stateNew
        if stateFinal is not None:
            vesselState = stateFinal
            # print(stateFinal.position)
            aX.append(stateFinal.position[0])
            aY.append(stateFinal.position[1])
        tSinceLastReport += h
    return [aX, aY]
