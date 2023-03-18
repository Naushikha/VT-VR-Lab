from pyais import decode
from pprint import pprint
import utils, export

fileName = "data/2022-11-16.csv"

lineN = 0

AISMessages = []


def pushAISMesssage(msgDict, msgRecvTime):
    # print("Line Number ", lineN)
    # print(msgDict)
    msgDict["epoch_time"] = msgRecvTime
    AISMessages.append(msgDict)


multiPartMsg = []
multiPart = False

with open(fileName) as csvFile:
    for line in csvFile:
        lineN += 1
        line = line.rstrip()
        parts = line.split()  # "1668496136 !AIVDM...."
        msgRecvTime = parts[0]
        msg = parts[1]
        if not multiPart:
            try:
                decodedMsg = decode(msg)
                pushAISMesssage(decodedMsg.asdict(), msgRecvTime)
            except:
                # Failed: Message must be multipart.
                multiPartMsg.append(msg)
                multiPart = True
        else:
            try:
                multiPartMsg.append(msg)
                decodedMsg = decode(*multiPartMsg)
                multiPart = False
                multiPartMsg.clear()
                pushAISMesssage(decodedMsg.asdict(), msgRecvTime)
            except:
                # Failed: Need more parts for the multipart msg.
                multiPartMsg.append(msg)
                multiPart = True


def customFilter(msg):
    # if msg["heading"] == 511:
    #     return False
    if msg["mmsi"] == 636018145:
        return True
    return False


# Default filter (positional reports only) >> Custom filter
filteredAIS = utils.filter(AISMessages)
# filteredAIS = utils.filter(filteredAIS, customFilter)


# utils.printMapBounds(filteredAIS)
# export.writeCSVForUnreal(filteredAIS)
# export.writeCSVForOctave(filteredAIS)
# pprint(utils.getTimeDifferenceList(filteredAIS))
# pprint(utils.getVesselList(filteredAIS))

# originLat = 6.955879
# originLon = 79.844690
# testLat = 6.956607
# testLon = 79.84546
# print(utils.computeFlatX(testLon))
# print(utils.computeFlatY(testLat))
# print(utils.approximateFlatX(testLon))
# print(utils.approximateFlatY(testLat))

vessels = utils.getVesselList(filteredAIS)


vesselMMSI = 0


def vesselFilter(msg):
    # if msg["heading"] == 511:
    #     return False
    if msg["mmsi"] == vesselMMSI:
        return True
    return False


vesselReports = {}
vesselTimeDiffList = {}
vesselTrialRating = {}


def getTrialRating(timeDiffList):
    rating = 0
    for timeDiff in timeDiffList:
        if 0 <= timeDiff and timeDiff <= 60:
            rating += 1
    return rating


for vessel in vessels:
    vesselMMSI = vessel
    vesselAISReports = utils.filter(filteredAIS, vesselFilter)
    startTime = int(vesselAISReports[0]["epoch_time"])
    for vesselAISReport in vesselAISReports:
        vesselAISReport["epoch_time"] = int(vesselAISReport["epoch_time"]) - startTime
    timeDiffList = utils.getTimeDifferenceList(vesselAISReports)
    avgReportingInterval = sum(timeDiffList) / len(timeDiffList)
    vesselReports[vesselMMSI] = vesselAISReports
    vesselTimeDiffList[vesselMMSI] = timeDiffList
    vesselTrialRating[vesselMMSI] = getTrialRating(timeDiffList)
    # export.writeCSVForOctave(vesselAISReports, f"vessel_{vesselMMSI}")

sortedVesselTrialRating = sorted(vesselTrialRating.items(), key=lambda x: x[1])
pprint(sortedVesselTrialRating)

# References:
# https://pyais.readthedocs.io/en/latest/messages.html
# https://gpsd.gitlab.io/gpsd/AIVDM.html#_types_1_2_and_3_position_report_class_a
# https://www.navcen.uscg.gov/ais-messages
# https://stackoverflow.com/a/51636871/11436038
