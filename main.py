import serial
from datetime import datetime
import time

c_port = serial.Serial("COM5", 38400)

aisMessage = ""  # Buffer message byte by byte


def storeAISMessage():
    global aisMessage
    aisMessage = aisMessage.strip()
    dateToday = datetime.now().strftime("%Y-%m-%d")
    fileName = "data/{}.csv".format(dateToday)
    csvFile = open(fileName, "a")
    epochTimeNow = int(time.time())
    aisRecord = "{} {}".format(epochTimeNow, aisMessage)
    csvFile.write(aisRecord + "\n")
    csvFile.close()
    aisMessage = ""  # Clear buffer
    print(aisRecord)


while True:
    aisMessageByte = c_port.read()  # Read a single byte
    aisMessage += aisMessageByte.decode("ascii")
    if "\r\n" in aisMessage:
        storeAISMessage()
