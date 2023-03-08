import math


def getCourse(x1, y1, x2, y2):
    course = math.degrees(math.atan2((x2 - x1), (y2 - y1)))
    if (course < 0):  # course can return negative
        course += 360
    return course


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


def getAISDataByDist(X, Y, dist):
    tDist = 0
    for i in range(len(X) - 1):
        if tDist >= dist:
            return [X[i], Y[i], getCourse(X[i], Y[i], X[i + 1], Y[i + 1])]
        fragDist = math.sqrt((Y[i + 1] - Y[i]) **
                             2 + (X[i + 1] - X[i]) ** 2)
        tDist += fragDist
    return [X[len(X) - 1], Y[len(Y) - 1], getCourse(X[i - 1], Y[i - 1], X[i], Y[i])]
