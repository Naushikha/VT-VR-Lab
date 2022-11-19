import csv
from datetime import datetime
import utils

defaultFileName = datetime.now().strftime("%Y_%m_%d-%H_%M")

# Format messages for Unreal: ID, Timestamp, MMSI, Latitude, Longitude, Speed, Course, Heading
def writeCSVForUnreal(msgDictList, fileName=defaultFileName):
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
    for msg in msgDictList:
        singleRecord = [
            i,
            msg["epoch_time"],
            msg["mmsi"],
            msg["lat"],
            msg["lon"],
            msg["speed"],
            msg["course"],
            msg["heading"],
        ]
        csvData.append(singleRecord)
        i += 1
    with open(f"out/{fileName}.unreal.csv", "w", newline="") as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(csvFields)
        writer.writerows(csvData)


# Format messages for Octave: Timestamp, MMSI, x, y, Speed, Course
def writeCSVForOctave(msgDictList, fileName=defaultFileName):
    csvData = []
    for msg in msgDictList:
        singleRecord = [
            msg["epoch_time"],
            msg["mmsi"],
            utils.approximateFlatX(float(msg["lon"])),
            utils.approximateFlatY(float(msg["lat"])),
            msg["speed"],
            msg["course"],
        ]
        csvData.append(singleRecord)
    with open(f"out/{fileName}.octave.csv", "w", newline="") as csvFile:
        writer = csv.writer(csvFile)
        writer.writerows(csvData)
