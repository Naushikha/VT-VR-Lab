from pyais import decode
import csv

fileName = "data/2022-11-16.csv"

lineN = 0

# Filter msgs using type
filteredMsgs = []


def filterMsgs(msgDict, msgRecvTime):
    if msgDict["msg_type"] in [1, 2, 3]:
        if (
            msgDict["heading"] == 511
        ):  # Remove ships that do not transmit heading information
            return
        # print("Line Number ", lineN)
        # print(msgDict)
        msgDict["epoch_time"] = msgRecvTime
        filteredMsgs.append(msgDict)


def findMapBounds():
    minLon = float(min(filteredMsgs, key=lambda x: x["lon"])["lon"])
    maxLon = float(max(filteredMsgs, key=lambda x: x["lon"])["lon"])
    minLat = float(min(filteredMsgs, key=lambda x: x["lat"])["lat"])
    maxLat = float(max(filteredMsgs, key=lambda x: x["lat"])["lat"])
    print(f"from ({minLat}, {minLon}) to ({maxLat}, {maxLon})")


# Format msgs for Unreal: ID, TIMESTAMP, MMSI, LAT, LON, SPEED, COURSE, HEADING
def formatMsgsAsCSV():
    csvFields = [
        "ID",
        "Timestamp",
        "MMSI",
        "Latitude",
        "Longitude",
        "Speed",
        "Course",
        "Heading",
    ]
    csvData = []
    i = 0
    for msg in filteredMsgs:
        # TIMESTAMP, MMSI, LAT, LON, SPEED, COURSE, HEADING
        singleRecord = [
            i,
            msg["epoch_time"],
            msg["mmsi"],
            # computeFlatX(float(msg["LAT"])),  # i["LAT"],
            # computeFlatY(float(msg["LON"])),  # i["LON"],
            msg["lat"],
            msg["lon"],
            msg["speed"],
            msg["course"],
            msg["heading"],
        ]
        csvData.append(singleRecord)
        i += 1
    with open("out.csv", "w", newline="") as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(csvFields)
        writer.writerows(csvData)


multiPartMsg = []
multiPart = False

with open(fileName) as csvFile:
    for line in csvFile:
        lineN += 1
        line = line.rstrip()
        parts = line.split()
        msgRecvTime = parts[0]
        msg = parts[1]
        if not multiPart:
            try:
                decodedMsg = decode(msg)
                filterMsgs(decodedMsg.asdict(), msgRecvTime)
            except:
                # failed. must be multipart!
                multiPartMsg.append(msg)
                multiPart = True
        else:
            try:
                multiPartMsg.append(msg)
                decodedMsg = decode(*multiPartMsg)
                multiPart = False
                multiPartMsg.clear()
                filterMsgs(decodedMsg.asdict(), msgRecvTime)
            except:
                # failed. there must be more parts!
                multiPartMsg.append(msg)
                multiPart = True

# findMapBounds()
formatMsgsAsCSV()

# https://pyais.readthedocs.io/en/latest/messages.html
# https://gpsd.gitlab.io/gpsd/AIVDM.html#_types_1_2_and_3_position_report_class_a
# https://www.navcen.uscg.gov/ais-messages
# https://stackoverflow.com/a/51636871/11436038
