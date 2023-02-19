import math
import numpy as np


def own_algo(aisData):
    estFreq = 60  # in Hertz
    h = 1 / estFreq
    aT = np.arange(0, aisData["duration"], h)
    aX = []
    aY = []
    k = 0
    msgList = []
    vessel = {
        "position": [],
        "velocity": [],
        "rateOfTurn": 0,
        "poly": [],
    }  # Store vessel state
    tSinceLastAISMsg = 0
    for deltaAT in aT:
        if deltaAT >= aisData["time"][k]:
            aisMsg = {
                "x": aisData["x"][k],
                "y": aisData["y"][k],
                "speed": aisData["speed"][k],
                "course": aisData["course"][k],
                "rateOfTurn": None,
                "tSinceLastAIS": tSinceLastAISMsg,
            }
            if len(msgList) == 3:  # Only keep last 3 messages
                msgList.pop(0)
            msgList.append(aisMsg)
            tSinceLastAISMsg = 0  # reset timer
            if k < len(aisData["time"]) - 1:
                k += 1
        # Check available messages to predict
        if msgList:
            if len(msgList) == 1:
                if len(vessel["position"]):
                    # Use dead reckoning
                    vessel["position"] += vessel["velocity"] * h
                else:  # initialize position, velocity rate of turn (if avail.)
                    vessel["position"] = np.array([msgList[0]["x"], msgList[0]["y"]])
                    vessel["velocity"] = np.array(
                        [
                            msgList[0]["speed"]
                            * math.sin(math.radians(msgList[0]["course"])),
                            msgList[0]["speed"]
                            * math.cos(math.radians(msgList[0]["course"])),
                        ]
                    )
                    # vessel["rateOfTurn"] = msgList[0]["rot"]
                aX.append(vessel["position"][0])
                aY.append(vessel["position"][1])
            if len(msgList) == 2:
                # Calculate rate of turn
                if msgList[1]["rateOfTurn"] is None:
                    msgList[1]["rateOfTurn"] = (
                        msgList[1]["course"] - msgList[0]["course"]
                    ) / msgList[1]["tSinceLastAIS"]
                    vessel["rateOfTurn"] = msgList[1]["rateOfTurn"]  # No use for now
                vessel["velocity"] = np.array(
                    [
                        msgList[1]["speed"]
                        * math.sin(
                            math.radians(
                                msgList[1]["course"]
                                + msgList[1]["rateOfTurn"] * tSinceLastAISMsg
                            )
                        ),
                        msgList[1]["speed"]
                        * math.cos(
                            math.radians(
                                msgList[1]["course"]
                                + msgList[1]["rateOfTurn"] * tSinceLastAISMsg
                            )
                        ),
                    ]
                )
                # Use dead reckoning
                vessel["position"] += vessel["velocity"] * h
                aX.append(vessel["position"][0])
                aY.append(vessel["position"][1])
                # print("msg avail: ", len(msgList))
            if len(msgList) == 3:
                # Need to optimize polyfit: DO NOT DO THIS ALL THE TIME
                vessel["poly"] = np.polyfit(
                    [msgList[0]["x"], msgList[1]["x"], msgList[2]["x"]],
                    [msgList[0]["y"], msgList[1]["y"], msgList[2]["y"]],
                    2,
                )
                # m = 2ax + b
                gradient = (
                    2 * vessel["poly"][0] * vessel["position"][0] + vessel["poly"][1]
                )
                polyCourse = math.pi / 2 - math.atan(gradient)  # * 180 / math.pi
                print(polyCourse * 180 / math.pi)
                tmpPosX = (
                    vessel["position"][0]
                    + msgList[2]["speed"] * math.sin(polyCourse) * h
                )
                # ax2 + bx + c = 0
                tmpPosY = (
                    vessel["poly"][0] * tmpPosX**2
                    + vessel["poly"][1] * tmpPosX
                    + vessel["poly"][2]
                )
                vessel["position"] = np.array([tmpPosX, tmpPosY])
                aX.append(vessel["position"][0])
                aY.append(vessel["position"][1])
        tSinceLastAISMsg += h
    return [aX, aY]
